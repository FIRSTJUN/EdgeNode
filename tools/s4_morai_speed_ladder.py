#!/usr/bin/env python3
"""Run ordered MORAI-only S4 trials; stop the ladder on the first incomplete return.

Run after sourcing Humble and the EdgeNode overlay. Reuses the live four-node
configuration and saved Route Editor route. Leaves one stack running, disarmed.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters, ListParameters, GetParameters
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32
from nav_msgs.msg import Path as NavPath
from morai_ros2_msgs.msg import CtrlCmd, EgoVehicleStatus, CollisionData, ObjectStatusList, MultiEgoSetting
import yaml

NAMES = ('localization_node', 'planning_node', 'control_node', 'lidar_obstacle_node')
START = (137.17578363279785, 1371.1162325591251, -.20413285, 88.37166508500111)
STAGES = ('NORMAL', 'DETECT', 'AVOID_ENTRY', 'AVOID_HOLD', 'RETURN', 'NORMAL')


def processes():
    found = {}
    for line in subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True).splitlines():
        fields = line.strip().split(maxsplit=1)
        if len(fields) != 2:
            continue
        for name in NAMES:
            if '/install/' in fields[1] and f'/{name} --ros-args' in fields[1]:
                if name in found:
                    raise RuntimeError('duplicate node: ' + name)
                found[name] = int(fields[0])
    return found


def alive(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1][0] != 'Z'
    except FileNotFoundError:
        return False


class Trial(Node):
    def __init__(self, out, attach_first=False):
        super().__init__('s4_speed_ladder_supervisor')
        self.out = out
        self.attach_first = attach_first
        self.latest, self.received, self.global_path = {}, {}, []
        self.ego_message = None
        self.started = time.monotonic()
        self.stack = None
        self.owner = None
        self.rows = []
        self.route_ids = []
        self.phase_updates = []
        self.capture_phases = False
        self.events = None
        for topic, kind in [('/planning/avoidance_state', String), ('/planning/avoidance_stage', String),
                ('/planning/avoidance_debug', String), ('/planning/s4_sensor_state', String),
                ('/perception/lidar_status', String), ('/planning/state', String),
                ('/planning/map_match_status', String), ('/planning/drive_status', String),
                ('/planning/fsm_state', String), ('/planning/target_speed', Float32),
                ('/control/path_error', Float32), ('/ctrl_cmd', CtrlCmd),
                ('/ego_vehicle_status', EgoVehicleStatus), ('/collision_data', CollisionData),
                ('/object_status', ObjectStatusList), ('/planning/global_path', NavPath),
                ('/planning/local_path', NavPath), ('/planning/avoidance_return_path', NavPath)]:
            self.create_subscription(kind, topic, lambda m, t=topic: self.receive(t, m), qos_profile_sensor_data)
        self.setter = self.create_client(SetParameters, '/planning_node/set_parameters')
        self.parameter_names = self.create_client(ListParameters, '/planning_node/list_parameters')
        self.parameter_values = self.create_client(GetParameters, '/planning_node/get_parameters')
        self.resetter = self.create_client(Trigger, '/planning/reset_avoidance')
        self.pose_pub = self.create_publisher(MultiEgoSetting, '/multi_ego_setting', 1)

    def receive(self, topic, msg):
        self.received[topic] = time.monotonic()
        if isinstance(msg, EgoVehicleStatus):
            self.ego_message = msg
            value = dict(x=msg.position.x, y=msg.position.y, heading=msg.heading,
                         speed=math.hypot(msg.velocity.x, msg.velocity.y, msg.velocity.z)*3.6)
        elif isinstance(msg, CtrlCmd):
            value = dict(accel=msg.accel, brake=msg.brake, steer=msg.front_steer)
        elif isinstance(msg, CollisionData):
            value = len(msg.collision_object)
        elif isinstance(msg, ObjectStatusList):
            value = [dict(id=o.unique_id, x=o.position.x, y=o.position.y,
                          size=[o.size.x, o.size.y, o.size.z],
                          speed=math.hypot(o.velocity.x, o.velocity.y, o.velocity.z)*3.6)
                     for o in msg.npc_list+msg.obstacle_list]
        elif isinstance(msg, NavPath):
            if topic != '/planning/global_path':
                value = dict(count=len(msg.poses), stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                    first_xy=[msg.poses[0].pose.position.x,msg.poses[0].pose.position.y] if msg.poses else None,
                    last_xy=[msg.poses[-1].pose.position.x,msg.poses[-1].pose.position.y] if msg.poses else None)
                self.latest[topic] = value
                return
            # Same fixed global route is republished; retain only its first copy.
            if not self.global_path:
                self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            return
        elif isinstance(msg, String) and topic.endswith(('debug', 'sensor_state', 'lidar_status')):
            value = json.loads(msg.data)
            if topic.endswith('avoidance_debug'):value.setdefault('stage',value.get('state','UNKNOWN'))
        else:
            value = msg.data
        self.latest[topic] = value
        if self.events is not None and topic in ('/planning/avoidance_debug','/planning/target_speed',
                '/planning/s4_sensor_state','/perception/lidar_status'):
            self.events.write(json.dumps(dict(t=time.monotonic()-self.started,topic=topic,value=value))+'\n')
        if topic == '/planning/avoidance_debug' and self.capture_phases:
            if not self.phase_updates or self.phase_updates[-1][1]['stage'] != value['stage']:
                self.phase_updates.append((time.monotonic(), value, dict(self.latest)))

    def spin(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)

    def call(self, client, req):
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('service unavailable: ' + client.srv_name)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=4.)
        if not future.done() or future.result() is None:
            raise RuntimeError('service timeout: ' + client.srv_name)
        return future.result()

    def drive(self, enabled):
        result = self.call(self.setter, SetParameters.Request(parameters=[
            Parameter('enable_drive', value=enabled).to_parameter_msg()]))
        if not result.results[0].successful:
            raise RuntimeError(result.results[0].reason)

    def parameters(self):
        names = self.call(self.parameter_names, ListParameters.Request()).result.names
        values = self.call(self.parameter_values, GetParameters.Request(names=names)).values
        # Unset dynamic parameters are omitted, never serialized as string null.
        params = {name:parameter_value_to_python(value) for name,value in zip(names,values) if value.type}
        return {'/planning_node': {'ros__parameters': params}}

    def snapshot(self):
        now=time.monotonic()
        return dict(t=round(now-self.started, 3), values=dict(self.latest),
                    receipt_ages_s={t:round(now-received,4) for t,received in self.received.items()})

    def fresh(self):
        now = time.monotonic()
        return all(now-self.received.get(t, 0)<.6 for t in (
            '/ego_vehicle_status', '/ctrl_cmd', '/collision_data', '/planning/s4_sensor_state'))

    def ready(self):
        s = self.latest.get('/planning/s4_sensor_state', {})
        return (self.fresh() and s.get('ready') and not s.get('recovery_pending')
                and not s.get('latched_reason') and self.latest.get('/collision_data') == 0
                and self.latest.get('/planning/state') == 'GLOBAL_PATH_READY'
                and self.latest.get('/planning/map_match_status') == 'MATCHED'
                and self.count_publishers('/ctrl_cmd') == 1)

    def stopped(self):
        return self.latest.get('/ego_vehicle_status', {}).get('speed', 999) < .2

    def initialize(self):
        self.spin(.8)
        self.drive(False)
        self.spin(1.)
        if not self.fresh() or not self.stopped() or self.latest.get('/collision_data') != 0:
            raise RuntimeError('initial stationary sensor/collision check failed')
        current = processes()
        if len(current) != 4:
            raise RuntimeError('expected exactly four existing nodes')
        self.owner = int(Path(f"/proc/{current['planning_node']}/stat").read_text().split(') ', 1)[1].split()[1])
        owner_args = Path(f'/proc/{self.owner}/cmdline').read_bytes().decode().split('\0')[:-1]
        if not any(a.endswith(('s4_morai.launch.py', 'stack.launch.py')) for a in owner_args):
            raise RuntimeError('unrecognized launch owner')
        config = self.out/'config'
        config.mkdir()
        baseline = self.parameters()
        params = baseline.get('/planning_node', baseline.get('planning_node', {}))['ros__parameters']
        saved = yaml.safe_load(Path('/workspace/local_data/kcity_routes/kcity_demo_route.yaml').read_text())
        from edgenode_planning.dijkstra_planner import DijkstraPlanner
        planner = DijkstraPlanner(params['mgeo_dir'])
        waypoints = saved['waypoint_node_ids']
        full = planner.plan_route('A219BS010380', waypoints)
        current_link = 'A219BS010469'
        current_from = planner.links_by_id[current_link]['from_node_idx']
        self.route_ids = waypoints[waypoints.index(current_from)+1:]
        remaining = planner.plan_route(current_link, self.route_ids)
        if remaining['link_ids'] != full['link_ids'][full['link_ids'].index(current_link):]:
            raise RuntimeError('remaining route differs from the saved global path')
        if params.get('route_node_ids') not in (None, [], 'null', waypoints, self.route_ids):
            raise RuntimeError('active route does not match saved Route Editor route')
        (self.out/'route_resume.json').write_text(json.dumps(dict(saved_waypoints=waypoints,
            remaining_waypoints=self.route_ids, original_link_ids=full['link_ids'],
            remaining_link_ids=remaining['link_ids'], exact_saved_suffix=True), indent=2))
        (config/'baseline.yaml').write_text(yaml.safe_dump({'/planning_node': {'ros__parameters': params}}))
        manifest = []
        for name, pid in current.items():
            argv = Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')[:-1]
            for i, arg in enumerate(argv):
                if i and argv[i-1] == '--params-file':
                    target = config/f'{name}-{i}.yaml'
                    target.write_bytes(Path(arg).read_bytes())
                    argv[i] = str(target)
            if name == 'planning_node':
                argv = argv[:argv.index('--ros-args')]+['--ros-args', '--params-file', str(config/'stage.yaml')]
            manifest.append(dict(name=name, argv=argv))
        (config/'manifest.json').write_text(json.dumps(manifest))
        launch = """import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
    data=json.loads(Path(__file__).with_name('manifest.json').read_text())
    return LaunchDescription([ExecuteProcess(cmd=d['argv'],output='screen',additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}) for d in data])
"""
        (config/'stack.launch.py').write_text(launch)
        print('PREPARED', json.dumps(self.snapshot()), flush=True)

    def relocate(self):
        self.drive(False)
        self.spin(.8)
        if not self.stopped() or not self.fresh() or self.count_subscribers('/multi_ego_setting') != 1:
            raise RuntimeError('pose reset admission failed')
        m = self.ego_message
        x, y, z, yaw = START
        self.pose_pub.publish(MultiEgoSetting(number_of_ego_vehicle=1, camera_index=m.unique_id,
            ego_index=[m.unique_id], global_position_x=[x], global_position_y=[y], global_position_z=[z],
            global_roll=[0.], global_pitch=[0.], global_yaw=[yaw], velocity=[0.], gear=[4], ctrl_mode=[16]))
        self.spin(1.5)
        e = self.latest['/ego_vehicle_status']
        if math.hypot(e['x']-x, e['y']-y)>.2 or not self.stopped():
            raise RuntimeError('MORAI did not restore start pose')

    def restart_stack(self, speed, avoid):
        current = processes()
        self.drive(False)
        self.spin(.8)
        if not self.stopped():
            raise RuntimeError('not stationary before node restart')
        previous_session = self.latest.get('/planning/s4_sensor_state', {}).get('session_id')
        os.kill(self.owner, signal.SIGINT)
        end = time.monotonic()+7.
        while any(alive(p) for p in current.values()) and time.monotonic()<end:
            self.spin(.05)
        if any(alive(p) for p in current.values()):
            raise RuntimeError('old stack did not exit')
        end = time.monotonic()+4.
        while self.count_publishers('/ctrl_cmd') and time.monotonic()<end:
            self.spin(.05)
        if self.count_publishers('/ctrl_cmd'):
            raise RuntimeError('old control publisher still present')
        parameters = dict(enable_drive=False, route_node_ids=self.route_ids, cruise_speed_kmh=float(speed), curve_speed_kmh=float(speed),
                          sharp_curve_speed_kmh=float(speed), max_drive_speed_kmh=float(speed),
                          drive_duration_sec=240., avoidance_avoid_speed_kmh=float(avoid), avoidance_hold_duration_s=3.)
        # One typed parameter file: Humble's dump of an unset dynamic parameter
        # is YAML null, which can otherwise mask a later array override.
        merged = yaml.safe_load((self.out/'config/baseline.yaml').read_text())['/planning_node']['ros__parameters']
        merged.update(parameters)
        (self.out/'config/stage.yaml').write_text(yaml.safe_dump({'/planning_node': {'ros__parameters': merged}}))
        (self.out/f'{speed}kmh_parameters.json').write_text(json.dumps(parameters, indent=2))
        self.latest.pop('/planning/s4_sensor_state', None)
        self.global_path = []
        self.stack = subprocess.Popen(['ros2', 'launch', str(self.out/'config/stack.launch.py')],
            stdout=(self.out/f'{speed}kmh_stack.log').open('w'), stderr=subprocess.STDOUT, start_new_session=True)
        self.owner = self.stack.pid
        (self.out/'stack.pid').write_text(str(self.owner)+'\n')
        end = time.monotonic()+12.
        healthy_since = None
        startup_reset = None
        while time.monotonic()<end:
            self.spin(.1)
            if self.stack.poll() is not None:
                raise RuntimeError('replacement stack exited')
            status = self.latest.get('/planning/s4_sensor_state', {})
            if status.get('session_id') in (None, previous_session):
                continue  # Queued DDS samples from the stopped producer are not the new stack.
            if self.ready():
                break
            if status.get('latched_reason'):
                if status['latched_reason'] not in ('S4_SENSOR_SENSOR_FAULT_HISTORY', 'S4_SENSOR_RECOVERY_TIMEOUT'):
                    raise RuntimeError('startup safety latch: '+status['latched_reason'])
                producer = self.latest.get('/perception/lidar_status', {})
                healthy = self.fresh() and status.get('ready') and self.stopped() and self.latest.get('/collision_data') == 0
                healthy_since = (healthy_since or time.monotonic()) if healthy else None
                if healthy_since and time.monotonic()-healthy_since>.7:
                    log = (self.out/f'{speed}kmh_stack.log').read_text()
                    warnings = [line for line in log.splitlines() if '[python3-2]' in line and '[WARN]' in line]
                    if (producer.get('session_id') != status['session_id'] or
                            not ((producer.get('fault_seq') == 1 and warnings and all('Missing or stale localization' in line for line in warnings))
                                 or (status['latched_reason'] == 'S4_SENSOR_RECOVERY_TIMEOUT' and producer.get('fault_seq') == 0 and not warnings))):
                        raise RuntimeError('startup fault history cannot be attributed to localization initialization')
                    startup_reset = dict(reason=status['latched_reason'], producer=producer, warnings=warnings,
                                         action='explicit disarmed reset after 0.7s fresh stationary data')
                    result = self.call(self.resetter, Trigger.Request())
                    if not result.success:
                        raise RuntimeError('startup reset rejected: '+result.message)
        if startup_reset:
            (self.out/f'{speed}kmh_startup_reset.json').write_text(json.dumps(startup_reset, indent=2))
        if not self.ready():
            raise RuntimeError('startup not ready')
        result = self.call(self.resetter, Trigger.Request())
        if not result.success:
            raise RuntimeError('initial reset refused: '+result.message)
        applied_dump = self.parameters()
        applied = applied_dump.get('/planning_node', applied_dump.get('planning_node', {}))['ros__parameters']
        (self.out/f'{speed}kmh_applied.yaml').write_text(yaml.safe_dump(applied_dump))
        if any(applied.get(name) != value for name, value in parameters.items()):
            raise RuntimeError('live route/speed parameters differ from requested stage')
        self.spin(.4)
        if not self.ready() or not self.stopped():
            raise RuntimeError('not ready after initial reset')
        objects = self.latest.get('/object_status', [])
        if len(objects) != 1 or objects[0]['speed']>.1:
            raise RuntimeError('expected one stationary simulator obstacle')
        if math.hypot(objects[0]['x']-140.026474, objects[0]['y']-1399.841797)>.5:
            raise RuntimeError('obstacle moved from verified scene')

    def admit_attached_first_stage(self):
        # This option is an explicit operator reset of the stationary preparation,
        # never recovery of a failed driving attempt. No live fault may be hidden.
        end = time.monotonic()+3.
        good_since = None
        stamps = set()
        while time.monotonic()<end:
            self.spin(.1)
            sensor = self.latest.get('/planning/s4_sensor_state', {})
            producer = self.latest.get('/perception/lidar_status', {})
            if not (self.fresh() and sensor.get('ready') and self.stopped()
                    and self.latest.get('/collision_data') == 0):
                raise RuntimeError('attached preparation lacks fresh stationary data')
            reason = sensor.get('latched_reason', '')
            if reason not in ('', 'S4_SENSOR_RECOVERY_TIMEOUT') or producer.get('fault_seq') != 0:
                raise RuntimeError('attached preparation contains a real/unknown fault')
            good_since = good_since or time.monotonic()
            stamps.add(sensor.get('source_stamp_ns'))
            if len(stamps)>=3 and time.monotonic()-good_since>.7:
                break
        else:
            raise RuntimeError('attached fresh-frame confirmation timed out')
        parameters = yaml.safe_load(subprocess.check_output(['ros2','param','dump','/planning_node'], text=True))
        applied = parameters.get('/planning_node', parameters.get('planning_node', {}))['ros__parameters']
        required = dict(enable_drive=False, route_node_ids=self.route_ids, cruise_speed_kmh=3.,
                        max_drive_speed_kmh=3., avoidance_avoid_speed_kmh=3., avoidance_hold_duration_s=3.)
        if any(applied.get(k) != v for k,v in required.items()):
            raise RuntimeError('attached route/speed does not match the 3km/h preparation')
        (self.out/'3kmh_applied.yaml').write_text(yaml.safe_dump(parameters))
        result = self.call(self.resetter, Trigger.Request())
        if not result.success:
            raise RuntimeError('attached stationary reset rejected: '+result.message)
        (self.out/'3kmh_startup_reset.json').write_text(json.dumps(dict(reason=reason,
            producer=producer, action='explicit pre-drive reset after fresh-frame confirmation'), indent=2))
        self.spin(.5)
        if not self.ready() or self.latest.get('/planning/avoidance_stage') != 'NORMAL':
            raise RuntimeError('attached stage not ready after reset')
        objects = self.latest.get('/object_status', [])
        if len(objects)!=1 or objects[0]['speed']>.1:
            raise RuntimeError('attached scene lacks one stationary obstacle')

    def check(self, speed):
        if self.latest.get('/collision_data', 1):
            raise RuntimeError('simulator collision')
        if not self.fresh():
            now = time.monotonic()
            ages = {topic: round(now-self.received.get(topic, 0), 3) for topic in (
                '/ego_vehicle_status', '/ctrl_cmd', '/collision_data', '/planning/s4_sensor_state')}
            raise RuntimeError('stale input receipt ages (s): '+json.dumps(ages))
        if self.count_publishers('/ctrl_cmd') != 1:
            raise RuntimeError('duplicate/missing control publisher')
        sensor = self.latest.get('/planning/s4_sensor_state', {})
        if sensor.get('latched_reason'):
            raise RuntimeError(sensor['latched_reason'])
        cmd = self.latest['/ctrl_cmd']
        if not all(math.isfinite(v) for v in cmd.values()) or abs(cmd['steer'])>.700001:
            raise RuntimeError('invalid steering/control command')
        if self.latest['/ego_vehicle_status']['speed']>speed+1.:
            raise RuntimeError('speed overshoot limit')
        stage = self.latest.get('/planning/avoidance_stage')
        if stage in ('AVOID_ENTRY', 'AVOID_HOLD', 'RETURN') and self.latest.get('/control/path_error', 0.)>.5:
            raise RuntimeError('local path tracking error')
        if self.latest.get('/planning/avoidance_state') == 'BLOCKED':
            raise RuntimeError('planner blocked: '+self.latest.get('/planning/avoidance_debug', {}).get('reason', ''))

    def alignment(self):
        e = self.latest['/ego_vehicle_status']
        best = (math.inf, math.inf)
        for a, b in zip(self.global_path, self.global_path[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            norm = dx*dx+dy*dy
            if norm < 1e-12:
                continue
            fraction = max(0., min(1., ((e['x']-a[0])*dx+(e['y']-a[1])*dy)/norm))
            distance = math.hypot(e['x']-a[0]-fraction*dx, e['y']-a[1]-fraction*dy)
            heading = abs((e['heading']-math.degrees(math.atan2(dy, dx))+180.)%360.-180.)
            if distance<best[0]:
                best = (distance, heading)
        return dict(lateral_m=best[0], heading_deg=best[1])

    def run_stage(self, speed, avoid):
        row = dict(normal_target_kmh=speed, avoidance_cap_kmh=avoid, success=False, phases=[],
                   sensor_events=[], stop_events=[], peak_speed_kmh=0., hold_required_s=3., route_node_ids=self.route_ids)
        self.rows.append(row)
        try:
            self.relocate()
            if self.attach_first and speed == 3:
                self.admit_attached_first_stage()
            else:
                self.restart_stack(speed, avoid)
            row['start'] = self.snapshot()
            self.phase_updates = [(time.monotonic(), self.latest['/planning/avoidance_debug'], dict(self.latest))]
            self.capture_phases = True
            self.events = (self.out/f'{speed}kmh_events.jsonl').open('w')
            self.drive(True)
            started = time.monotonic()
            processed_phases = 0
            last_sensor = last_stop = None
            previous = self.latest['/ego_vehicle_status']
            travelled = 0.
            entry_distance = hold_started = returned_at = None
            next_log = 0.
            with (self.out/f'{speed}kmh_telemetry.jsonl').open('w') as stream:
                while time.monotonic()-started < 210.:
                    self.spin(.025)
                    now = time.monotonic()
                    e = self.latest['/ego_vehicle_status']
                    travelled += math.hypot(e['x']-previous['x'], e['y']-previous['y'])
                    previous = e
                    row['peak_speed_kmh'] = max(row['peak_speed_kmh'], e['speed'])
                    pending = self.phase_updates[processed_phases:]
                    processed_phases = len(self.phase_updates)
                    for phase_time, debug, values in pending:
                        stage = debug['stage']
                        phase_ego = values.get('/ego_vehicle_status', e)
                        phase = dict(t=round(max(0., phase_time-started), 3), stage=stage, travelled_m=travelled,
                                     ego=phase_ego, debug=debug)
                        row['phases'].append(phase)
                        print('PHASE', speed, json.dumps(phase), flush=True)
                        if stage == 'DETECT':
                            row['detection_center_distance_m'] = min(math.hypot(o['x']-phase_ego['x'], o['y']-phase_ego['y']) for o in values['/object_status'])
                            row['lidar_detection_body_gap_m'] = min(o['distance_m'] for o in debug['detected_obstacles'])
                            if debug.get('entry_commits') or debug.get('generated_transition_count'):
                                raise RuntimeError('DETECT generated a maneuver')
                        elif stage == 'AVOID_ENTRY':
                            entry_distance = travelled
                            if debug.get('return_commits'):
                                raise RuntimeError('early return commit')
                        elif stage == 'AVOID_HOLD':
                            hold_started = phase_time
                            row['entry_travel_m'] = travelled-entry_distance if entry_distance is not None else None
                            if debug.get('return_commits'):
                                raise RuntimeError('return committed before safe hold')
                        elif stage == 'RETURN':
                            row['hold_duration_actual_s'] = phase_time-hold_started if hold_started else None
                            row['hold_elapsed_on_return_s'] = debug.get('hold_elapsed_s')
                            row['surround_clearance_on_return_m'] = debug.get('surround_clearance_m')
                            if debug.get('hold_elapsed_s', 0.)<3.:
                                raise RuntimeError('RETURN before hold timer completed')
                        elif stage == 'NORMAL' and debug.get('return_commits', 0)>0:
                            returned_at = now
                            row['return_alignment'] = self.alignment()
                    self.check(speed)
                    sensor = self.latest['/planning/s4_sensor_state']
                    key = (sensor['ready'], sensor.get('recovery_pending'), sensor.get('reason'), sensor.get('latched_reason'))
                    if key != last_sensor:
                        row['sensor_events'].append(dict(t=now-started, sensor=sensor))
                        last_sensor = key
                    stop = (self.latest.get('/planning/avoidance_state'), self.latest.get('/planning/fsm_state'), self.latest.get('/planning/target_speed') == 0.)
                    if stop != last_stop:
                        row['stop_events'].append(dict(t=now-started, state=stop, speed_kmh=e['speed']))
                        last_stop = stop
                    if now>=next_log:
                        stream.write(json.dumps(self.snapshot())+'\n');stream.flush();next_log=now+.1
                    if returned_at is not None and now-returned_at>2. and e['speed']>=speed*.9:
                        alignment = self.alignment()
                        if alignment['lateral_m']>.30 or alignment['heading_deg']>8.:
                            raise RuntimeError('global path alignment not stable after return')
                        sequence = [p['stage'] for p in row['phases']]
                        if sequence != list(STAGES):
                            raise RuntimeError('incomplete/nonsequential four-stage execution: '+str(sequence))
                        row.update(success=True, reason='returned and normal speed recovered', final_alignment=alignment,
                                   normal_recovered_speed_kmh=e['speed'], travel_m=travelled)
                        break
                if not row['success']:
                    raise RuntimeError('return/normal speed recovery deadline exceeded')
        except Exception as exc:
            row['reason'] = str(exc)
            print('STOP', speed, str(exc), flush=True)
        finally:
            self.capture_phases = False
            try:
                self.drive(False)
                self.spin(1.8)
            except Exception as exc:
                row['disarm_error'] = str(exc)
            row['final'] = self.snapshot()
            if self.events is not None:
                self.events.close();self.events = None
            row['final_stopped'] = self.stopped()
            if not row['final_stopped'] or row.get('disarm_error'):
                row['success'] = False
            (self.out/f'{speed}kmh_result.json').write_text(json.dumps(row, indent=2))
            (self.out/'summary.json').write_text(json.dumps(dict(trials=self.rows, stack_pid=self.owner), indent=2))
            print('RESULT', speed, json.dumps({k:v for k,v in row.items() if k not in ('start','final','phases','stop_events','sensor_events','route_node_ids')}), flush=True)
        return row['success']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--attach-first', action='store_true', help='Use an already configured stationary 3km/h stack for the first trial')
    parser.add_argument('--speeds', type=int, nargs='+', choices=(3,5), default=[5],
                        help='MORAI trial speeds; default 5 km/h, maximum 5 km/h')
    args = parser.parse_args()
    if subprocess.check_output(['git', 'branch', '--show-current'], text=True).strip() != 's4-finish':
        raise RuntimeError('s4-finish required')
    out = Path(args.out).resolve();out.mkdir(parents=True, exist_ok=False)
    rclpy.init();trial = Trial(out, args.attach_first)
    try:
        trial.initialize()
        for speed in args.speeds:
            if not trial.run_stage(speed, speed):
                break
    finally:
        try:
            trial.drive(False);trial.spin(1.)
        finally:
            trial.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    main()
