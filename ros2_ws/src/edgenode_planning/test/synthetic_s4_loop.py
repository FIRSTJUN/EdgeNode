"""Stage 5A ONLY: explicit synthetic worlds, no ROS graph or approval files.

Production planner, gates, FSM and ControlNode.control_loop are invoked directly.
The pedal/steering plant is declared simulation physics, not a MORAI calibration.
Virtual compute latency and measured host CPU latency are deliberately separate.
"""
from collections import deque
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
from rclpy.time import Time
import yaml

from edgenode_control.control_node import ControlNode, PID
from edgenode_control.pure_pursuit import PurePursuit
from edgenode_planning.avoidance_geometry import DrivableArea, polygon_distance, swept_body
from edgenode_planning.lane_geometry import LaneNetwork, PolylineIndex, arc, sample
from edgenode_planning.lattice_path import control_local_path, curvatures
from edgenode_planning.local_path_planner import LocalPathPlanner
from edgenode_planning.planning_fsm import PlanningFSM
from edgenode_planning.sensor_readiness import SensorReadiness, footprint_digest
from edgenode_planning.s4_distance_safety import DistancePolicy, DistanceSafety, ObservationEvidence
from edgenode_planning.static_obstacle_planner import AvoidanceConfig, StaticObstaclePlanner, bicycle_step


@dataclass(frozen=True)
class Scene:
    name: str = 'synthetic_straight_3'
    experiment: str = 'A'  # B never injects an observation/map approval.
    initial_speed_kmh: float = 3.
    radius_m: float = 0.  # zero denotes straight
    side: str = 'left'
    obstacle_front_m: float = 55.
    obstacle_length_m: float = 4.
    obstacle_width_m: float = 1.
    obstacle_yaw_rad: float = 0.
    initial_lateral_m: float = 0.
    initial_yaw_rad: float = 0.
    road_end_m: float = 220.
    adjacent_end_m: float = 220.
    lane_width_m: float = 3.5
    raw_virtual_range_m: float = 60.
    usable_virtual_range_m: float = 50.
    sensor_delay_s: float = .05
    compute_latency_s: float = .02
    timing_mode: str = 'modeled'  # 'measured' advances virtual time by observed wall delay.
    command_delay_s: float = .1
    pedal_time_constant_s: float = .08
    steering_time_constant_s: float = .04
    accel_gain_mps2: float = 3.
    brake_gain_mps2: float = 5.
    drag_per_s: float = .04
    control_horizon_m: float = 20.
    duration_s: float = 155.
    post_return_s: float = 2.
    stop_hold_s: float = 1.
    fault: str = ''


class Road:
    def __init__(self, scene):
        self.scene = scene
        self.offset = scene.lane_width_m * (1 if scene.side == 'left' else -1)
        self.route_ids, links = [], {}
        cuts = [-20.] + [float(s) for s in range(40, int(scene.road_end_m), 50)] + [scene.road_end_m]
        for index, (start, end) in enumerate(zip(cuts, cuts[1:])):
            self.route_ids.append('SYN_O' + str(index))
            for label, offset in (('O', 0.), ('A', self.offset)):
                actual_end = min(end, scene.adjacent_end_m) if label == 'A' else end
                if actual_end <= start:
                    continue
                key = 'SYN_' + label + str(index)
                out = scene.side if label == 'O' else ('right' if scene.side == 'left' else 'left')
                points = [self.point(float(s), offset) for s in np.linspace(start, actual_end,
                          max(2, math.ceil((actual_end-start)/.5)+1))]
                links[key] = dict(idx=key, from_node_idx=label+str(index), to_node_idx=label+str(index+1),
                    points=points, max_speed=30., link_type='6', can_move_left_lane=False,
                    can_move_right_lane=False, left_lane_change_dst_link_idx=None,
                    right_lane_change_dst_link_idx=None)
                links[key]['can_move_' + out + '_lane'] = True
                links[key][out + '_lane_change_dst_link_idx'] = 'SYN_' + ('A' if label == 'O' else 'O') + str(index)
                if scene.experiment == 'B' or (scene.fault == 'permission_missing' and label == 'O'):
                    del links[key]['can_move_' + out + '_lane']
        self.network = LaneNetwork(SimpleNamespace(links_by_id=links,
            graph={'synthetic': [dict(link_id=k) for k in links]}), require_explicit_permission=True)
        self.reference = []
        for key in self.route_ids:
            self.reference.extend(links[key]['points'] if not self.reference else links[key]['points'][1:])
        self.index = PolylineIndex(self.reference)
        low, high = min(0., self.offset)-scene.lane_width_m/2, max(0., self.offset)+scene.lane_width_m/2
        if scene.fault == 'no_space':
            low, high = -scene.lane_width_m/2, scene.lane_width_m/2
        steps = np.linspace(-20., scene.road_end_m, math.ceil((scene.road_end_m+20)/2)+1)
        self.low = [self.point(float(s), low)[:2] for s in steps]
        self.high = [self.point(float(s), high)[:2] for s in steps]
        cells = ((self.low[0], self.low[-1], self.high[-1], self.high[0]),) if not scene.radius_m else tuple(
            (a, b, d, c) for a, b, c, d in zip(self.low, self.low[1:], self.high, self.high[1:]))
        self.area = DrivableArea(cells, 'SYNTHETIC_STAGE5A_EXPLICIT_EDGES_NOT_KCITY_NOT_DEPLOYABLE')
        boundary = np.asarray(self.low + list(reversed(self.high)), dtype=float)
        self.edges_a, self.edges_b = boundary, np.roll(boundary, -1, axis=0)
        self.original_hash = hashlib.sha256(json.dumps(self.reference).encode()).hexdigest()
        self.obstacles = [self.obstacle(scene.obstacle_front_m, scene.obstacle_length_m,
                                       scene.obstacle_width_m, scene.obstacle_yaw_rad)]
        if scene.fault == 'wall':
            self.obstacles = [self.obstacle(scene.obstacle_front_m, 4., 2*scene.lane_width_m+2., 0., self.offset/2)]

    def point(self, station, lateral):
        r = self.scene.radius_m
        theta = station/r if r else 0.
        x = r*math.sin(theta) if r else station
        y = r*(1-math.cos(theta)) if r else 0.
        return (x-lateral*math.sin(theta), y+lateral*math.cos(theta), 0.)

    def station(self, pose):
        r = self.scene.radius_m
        return r*math.atan2(pose[0], r-pose[1]) if r else pose[0]

    def obstacle(self, front, length, width, rotation, lateral=0.):
        center = self.point(front+length/2, lateral)
        angle = (front+length/2)/self.scene.radius_m if self.scene.radius_m else 0.
        angle += rotation
        result = []
        for x, y in ((-length/2, -width/2), (0., -width/2), (length/2, -width/2),
                     (length/2, 0.), (length/2, width/2), (0., width/2), (-length/2, width/2), (-length/2, 0.)):
            result.append((center[0]+x*math.cos(angle)-y*math.sin(angle),
                           center[1]+x*math.sin(angle)+y*math.cos(angle)))
        return tuple(result)

    def current_link(self, pose):
        progress = self.index.project(pose[:2])
        for key in self.route_ids:
            if progress <= self.index.project(self.network.links[key]['points'][-1][:2]) + 1e-8:
                return key
        return self.route_ids[-1]

    def margin(self, polygon, inside):
        # Exact distance between the outer scene boundary and swept polygon:
        # both endpoint-to-segment directions; internal cell seams excluded.
        p = np.asarray(polygon); q = np.roll(p, -1, axis=0)
        def distances(points, a, b):
            v = b-a; length = np.maximum((v*v).sum(axis=-1), 1e-20)
            w = points[:, None, :]-a[None, :, :]
            u = np.clip((w*v[None, :, :]).sum(axis=-1)/length[None, :], 0., 1.)
            return float(np.sqrt(((w-u[:, :, None]*v[None, :, :])**2).sum(axis=-1)).min())
        value = min(distances(p, self.edges_a, self.edges_b), distances(self.edges_a, p, q))
        return value if inside else -value


class Sink:
    def __init__(self):
        self.last = None
    def publish(self, value):
        self.last = value


class OfflineControl:
    """Actual production methods; no Node construction, DDS, rclpy.init or publish."""
    control_loop = ControlNode.control_loop
    publish_safe_stop = ControlNode.publish_safe_stop

    def __init__(self, values):
        self.values = values; self.ns = 10_000_000_000; self.last_control_ns = self.ns
        self.target_speed_kmh = self.current_speed_kmh = 0.
        self.latest_pose = None; self.local_path_points = []
        self.target_speed_stamp_ns = self.local_path_stamp_ns = None
        self.localization_stamp_ns = self.status_stamp_ns = None
        self.pure_pursuit = PurePursuit(values['wheelbase_m'], values['max_wheel_angle_rad'])
        self.speed_pid = PID(values['speed_kp'], values['speed_ki'], values['speed_kd'], values['speed_integral_limit'])
        for key in ('ctrl', 'lookahead_distance', 'path_error', 'target_left', 'target_forward', 'pure_pursuit_steering'):
            setattr(self, key + '_pub', Sink())

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])
    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=self.ns))
    def now_ns(self):
        return self.ns


class Vehicle:
    def __init__(self, scene, pose, config):
        self.scene = scene; self.pose = pose; self.v = scene.initial_speed_kmh/3.6
        self.wheel = math.atan(config.wheelbase_m/scene.radius_m) if scene.radius_m else 0.
        self.curvature = math.tan(self.wheel)/config.wheelbase_m
        self.accel = self.brake = 0.
        self.commands = deque(); self.applied = (0., 0., self.wheel)
        self.rate = 0.; self.acceleration = 0.; self.rate_limited_steps = 0

    def step(self, cmd, now, dt, config):
        wheel = cmd.front_steer/config.steering_sign*config.max_wheel_angle_rad
        self.commands.append((now+self.scene.command_delay_s, (cmd.accel, cmd.brake, wheel)))
        while self.commands and self.commands[0][0] <= now+1e-9:
            _, self.applied = self.commands.popleft()
        throttle, brake, target_wheel = self.applied
        alpha = 1-math.exp(-dt/self.scene.pedal_time_constant_s)
        self.accel += alpha*(throttle-self.accel); self.brake += alpha*(brake-self.brake)
        change = (target_wheel-self.wheel)*(1-math.exp(-dt/self.scene.steering_time_constant_s))
        bound = config.max_wheel_rate_radps*dt
        if abs(change) > bound: self.rate_limited_steps += 1
        change = max(-bound, min(bound, change)); self.wheel += change; self.rate = change/dt
        self.curvature = math.tan(self.wheel)/config.wheelbase_m
        self.acceleration = (self.scene.accel_gain_mps2*self.accel-self.scene.brake_gain_mps2*self.brake
                             -self.scene.drag_per_s*self.v)
        end_v = max(0., self.v+self.acceleration*dt)
        self.pose = bicycle_step(self.pose, self.curvature, (self.v+end_v)*.5*dt)
        self.v = end_v


def production_values():
    file = Path(__file__).resolve().parents[2]/'edgenode_control/config/control.yaml'
    return yaml.safe_load(file.read_text())['control_node']['ros__parameters']


def simulate(scene):
    if scene.timing_mode not in ('modeled','measured'):
        raise ValueError('unknown synthetic timing mode')
    values = production_values()
    fields = ('wheelbase_m', 'max_wheel_angle_rad', 'max_front_steer_normalized', 'steering_sign',
              'control_rate_hz', 'lookahead_min_m', 'lookahead_max_m', 'lookahead_preview_time_sec')
    config = AvoidanceConfig(**{key: values[key] for key in fields})
    policy = DistancePolicy(control_horizon_m=scene.control_horizon_m)
    road = Road(scene)
    planner = StaticObstaclePlanner(road.network,
        drivable_area=road.area if scene.experiment == 'A' and scene.fault != 'boundary_missing' else None,
        config=config)
    # Synthetic interfaces only. Operational JSON loaders still reject these.
    evidence = ObservationEvidence(scene.usable_virtual_range_m, 20., config.curvature_limit,
        tuple(road.route_ids), 'SYNTHETIC_VIRTUAL_SENSOR_NOT_REAL_LIDAR') if scene.experiment == 'A' else None
    safety = DistanceSafety(policy, config, planner.body, evidence)
    clock = [10_000_000_000]
    guard = SensorReadiness(steady_time_ns=lambda: clock[0])
    control = OfflineControl(values); fsm = PlanningFSM(); local = LocalPathPlanner()
    pose = (*road.point(0., scene.initial_lateral_m)[:2], scene.initial_yaw_rad)
    vehicle = Vehicle(scene, pose, config)
    dt = 1/config.control_rate_hz; period = .1
    rows, transitions, fsm_transitions, candidates, timings = [], [], [], [], []
    process_timings=[]
    planning_rows, rollout_failures, selected_geometry = [], [], []
    original_rollout = planner._rollout
    def audited_rollout(points, pose, speed, curvature, obstacles):
        reason = original_rollout(points, pose, speed, curvature, obstacles)
        if reason:
            try:
                available = arc(control_local_path(points, pose, policy.control_horizon_m))[-1]
            except ValueError:
                available = 0.
            rollout_failures.append(dict(state=planner.state, reason=reason, pose=pose,
                speed_kmh=speed, available_control_m=available, required_control_m=(
                    2*max(speed, config.avoid_speed_kmh)/3.6*config.rollout_seconds+
                    max(config.lookahead_min_m, min(config.lookahead_max_m,
                        max(speed, config.avoid_speed_kmh)/3.6*config.lookahead_preview_time_sec))+
                    2*config.tracking_limit_m)))
        return reason
    planner._rollout = audited_rollout
    pending = None; next_plan = 0.; next_sensor = 0.; seq = 0
    last_result = planner._result(); path = (); target = 0.; previous_index = previous_arc = None
    pending_stamp = None; fresh_stamps = set(); last_candidate_key = None
    blocked_at = blocked_station = None; blocked_pose = None; blocked_reason = blocked_phase = ''
    returned_at = entry_at = None; tracking_injected = False; stall_injected = False
    min_clearance = min_road = math.inf; max_tracking = max_yaw_error = max_wheel = max_rate = 0.
    max_requested_rate = max_avoid_speed = 0.
    max_speed = scene.initial_speed_kmh
    max_source_age = 0.; previous_request_wheel = vehicle.wheel
    stopped_at = None; cpu_overruns = 0; model_overruns = 0; validations = 0
    mission_stop_at = None; stop_started_at = stop_start_distance = None
    total_distance = 0.; return_fault_injected = False
    history = deque([(0., vehicle.pose)]); sensor_drop_at = None
    route_hash_before = road.original_hash

    def record_transition(now):
        if not transitions or transitions[-1]['state'] != planner.state:
            transitions.append(dict(t=now, state=planner.state, reason=planner.reason))

    def block(reason, now):
        nonlocal blocked_at, blocked_station, blocked_pose, blocked_reason, blocked_phase, path, target, last_result
        nonlocal stop_started_at, stop_start_distance
        if planner.state != 'BLOCKED':
            blocked_phase = planner.state
            last_result = planner.stop(reason)
        if blocked_at is None:
            blocked_at = now; blocked_station = road.station(vehicle.pose); blocked_pose = vehicle.pose
            blocked_reason = planner.reason
        if stop_started_at is None:
            stop_started_at = now; stop_start_distance = total_distance
        path = (); target = 0.
        control.local_path_points = []; control.target_speed_kmh = 0.
        control.local_path_stamp_ns = control.target_speed_stamp_ns = clock[0]
        record_transition(now)

    record_transition(0.)
    for tick in range(math.ceil(scene.duration_s/dt)):
        now = tick*dt; ns = 10_000_000_000+round(now*1e9); clock[0] = ns; control.ns = ns
        if returned_at is not None and now-returned_at >= scene.post_return_s and mission_stop_at is None:
            mission_stop_at = now
            stop_started_at = now; stop_start_distance = total_distance
            target = 0.; pending = None
        if mission_stop_at is not None:
            # A normal goal stop retains the verified original path so the
            # production explicit-stop branch and GOAL_REACHED FSM run together.
            # BLOCKED below still publishes an empty path and safe brake.
            path = control_local_path(road.reference, vehicle.pose, policy.control_horizon_m)
            control.local_path_points = path; control.target_speed_kmh = 0.
            control.local_path_stamp_ns = control.target_speed_stamp_ns = ns
        if scene.fault == 'return_obstruction' and planner.state == 'RETURN' and not return_fault_injected:
            # Occupy the reserved lane crossing itself. A nearer original-lane
            # obstacle may still be passed entirely in the adjacent lane.
            road.obstacles.append(road.obstacle(road.station(vehicle.pose)+14., 4., 1., 0.))
            return_fault_injected = True
        if scene.fault == 'boundary_lost_on_return' and planner.state == 'RETURN' and not return_fault_injected:
            planner.drivable_area = None; return_fault_injected = True
        # Producer and Planning share deterministic virtual time. Incoming data
        # are queued while a modeled Planning callback is busy, as in a single
        # threaded executor. No thread/ROS process/hardware is created.
        if pending is None and now+1e-9 >= next_sensor:
            next_sensor = now+.1
            source_t = max(0., now-scene.sensor_delay_s)
            while len(history)>2 and history[1][0] <= source_t: history.popleft()
            source_t, sensed_pose = history[0]
            drop = scene.fault == 'sensor_dropout' and entry_at is not None and now-entry_at >= 1.
            if not drop:
                groups = []
                usable = scene.usable_virtual_range_m if scene.experiment == 'A' else 21.19
                raw_range = scene.raw_virtual_range_m if scene.experiment == 'A' else 21.19
                for obstacle in road.obstacles:
                    projections = [(x-sensed_pose[0])*math.cos(sensed_pose[2])+(y-sensed_pose[1])*math.sin(sensed_pose[2]) for x,y in obstacle]
                    # Certified near-face trigger and full-body virtual extent
                    # must both fit the stated raw range. No future solids.
                    if min(projections) <= usable and max(projections) <= raw_range and max(projections)>0:
                        groups.append(obstacle)
                points = [p for group in groups for p in group]
                seq += 1; stamp = 10_000_000_000+round(source_t*1e9)
                guard.observation(stamp, points, ns)
                status = dict(schema_version=1, clock='ros', session_id='5'*32, status_seq=seq,
                    frame_seq=seq, fault_seq=0, state='READY_OBSTACLES' if groups else 'READY_EMPTY',
                    source_stamp_ns=stamp, frame_received_ns=ns, published_ns=ns,
                    cluster_count=len(groups), obstacle_digest=footprint_digest('map', stamp, points))
                event = guard.receive(json.dumps(status), ns)
                if event: block('S4_SENSOR_'+event, now)
            elif sensor_drop_at is None: sensor_drop_at = now
        if scene.fault == 'tracking_failure' and entry_at is not None and now-entry_at>1.5 and not tracking_injected:
            x,y,yaw = vehicle.pose
            vehicle.pose = (x-.65*math.sin(yaw), y+.65*math.cos(yaw), yaw)
            tracking_injected = True
        control.latest_pose = vehicle.pose; control.current_speed_kmh = vehicle.v*3.6
        control.localization_stamp_ns = control.status_stamp_ns = ns
        if pending is None:
            reason = guard.reason(ns)
            if reason: block('S4_SENSOR_'+reason, now)
        if pending is not None and now+1e-9 >= pending['ready_at']:
            job = pending; pending = None; last_result = job['result']
            if planner.state=='BLOCKED' and not blocked_phase: blocked_phase=job['phase_before']
            source_age = (ns-guard.matched[0]['source_stamp_ns'])/1e9 if guard.matched else 1e6
            max_source_age=max(max_source_age,source_age)
            sensor_reason = guard.reason(ns)
            job['timing_row'].update(completed_virtual_ns=ns,
                source_stamp_at_completion_ns=guard.matched[0]['source_stamp_ns'] if guard.matched else None,
                source_age_at_completion_s=source_age,freshness_reason_at_completion=sensor_reason,
                control_update_virtual_ns=ns)
            distance = safety.evaluate(vehicle.v*3.6, source_age, vehicle.curvature, road.route_ids)
            if sensor_reason: block('S4_SENSOR_'+sensor_reason, now)
            elif max(job['cpu_s'],job['model_s']) > policy.planning_budget_s:
                block('S4_DISTANCE_PLANNING_BUDGET_EXCEEDED', now)
            elif not distance['ready']: block('S4_DISTANCE_'+distance['reason'], now)
            elif planner.state == 'BLOCKED': block(planner.reason, now)
            else:
                if last_result['active']:
                    if job['new_validation'] or job['model_s']>.05:
                        pending_stamp = job['stamp']; fresh_stamps.clear()
                    if pending_stamp is not None:
                        if job['stamp']>pending_stamp: fresh_stamps.add(job['stamp'])
                        if len(fresh_stamps)<3:
                            path=(); target=0.; validations+=1
                        else:
                            pending_stamp=None; fresh_stamps.clear()
                    if pending_stamp is None:
                        path=control_local_path(last_result['points'],vehicle.pose,policy.control_horizon_m)
                        target=min(3.,last_result['speed_cap_kmh'] or 3.)
                else:
                    path=job['normal']; target=3.
                control.local_path_points=path; control.target_speed_kmh=target
                control.local_path_stamp_ns=control.target_speed_stamp_ns=ns
                if planner.state=='AVOID_ENTRY' and entry_at is None: entry_at=now
                if planner.state=='NORMAL' and any(r['state']=='RETURN' for r in transitions) and returned_at is None:
                    returned_at=now
            record_transition(now)
        if pending is None and planner.state != 'BLOCKED' and mission_stop_at is None and now+1e-9 >= next_plan:
            whole_started=time.perf_counter()
            process_started=time.process_time()
            host_started_ns=time.monotonic_ns()
            next_plan=now+period
            normal = local.extract(road.reference,*vehicle.pose[:2],horizon_m=20., previous_index=previous_index,
                previous_arc_m=previous_arc,resample_spacing_m=2.)
            previous_index,previous_arc=normal['nearest_index'],normal['progress_arc_m']
            preview=local.extract(road.reference,*vehicle.pose[:2],horizon_m=policy.preview_m,
                previous_index=previous_index,previous_arc_m=previous_arc,resample_spacing_m=2.)['points']
            source_age=(ns-guard.matched[0]['source_stamp_ns'])/1e9
            distance=safety.evaluate(vehicle.v*3.6,source_age,vehicle.curvature,road.route_ids)
            if not distance['ready']: block('S4_DISTANCE_'+distance['reason'],now)
            elif arc(preview)[-1] < distance['stopping_required_m']: block('S4_DISTANCE_MAP_STOPPING_SUPPORT',now)
            elif max(map(abs,curvatures(control_local_path(preview,vehicle.pose,distance['stopping_required_m']))),default=math.inf)>evidence.max_curvature:
                block('S4_DISTANCE_OBSERVATION_CURVATURE_SCOPE',now)
            else:
                groups=guard.groups()
                new=planner.needs_validation_stop(preview,groups)
                if new:
                    control.local_path_points=[];control.target_speed_kmh=0.
                    control.local_path_stamp_ns=control.target_speed_stamp_ns=ns
                    path=();target=0.
                phase_before=planner.state
                result=planner.plan(normal['points'],groups,vehicle.pose,speed_kmh=vehicle.v*3.6,
                    actual_curvature=vehicle.curvature,current_link_id=road.current_link(vehicle.pose),
                    route_link_ids=road.route_ids,observation_valid=True,control_horizon_m=policy.control_horizon_m,
                    hazard_reference=preview,entry_reserve_m=distance['entry_reserve_m'],
                    stopping_required_m=distance['stopping_required_m'],entry_observation_m=evidence.forward_range_m,
                    observation_curvature_limit=evidence.max_curvature)
                # Account for final window construction too; completion still
                # reprojects the same fixed geometry at the actual model pose.
                if result['active'] and not result['blocked']:
                    control_local_path(result['points'],vehicle.pose,policy.control_horizon_m)
                cpu=time.perf_counter()-whole_started  # legacy name: elapsed wall time
                process_cpu=time.process_time()-process_started
                process_timings.append(process_cpu)
                model=max(scene.compute_latency_s,cpu) if scene.timing_mode=='measured' else scene.compute_latency_s
                if scene.fault=='compute_timeout' and new and not stall_injected:
                    model=1.6;stall_injected=True
                timings.append(cpu);cpu_overruns+=int(cpu>policy.planning_budget_s);model_overruns+=int(model>policy.planning_budget_s)
                planning_rows.append(dict(t=now,state=planner.state,reason=planner.reason,cpu_s=cpu,
                    wall_s=cpu,process_cpu_s=process_cpu,timing_mode=scene.timing_mode,
                    source_stamp_at_start_ns=guard.matched[0]['source_stamp_ns'],
                    observation_received_virtual_ns=guard.matched[2],planning_start_virtual_ns=ns,
                    expected_completion_virtual_ns=ns+round(model*1e9),host_started_monotonic_ns=host_started_ns,
                    host_finished_monotonic_ns=time.monotonic_ns(),
                    modeled_s=model,source_age_s=source_age,new_validation=new,
                    speed_kmh=vehicle.v*3.6,stopping_required_m=distance['stopping_required_m']))
                key=tuple((c.side,c.transition_length_m,c.rejection) for c in planner.candidates)
                if key != last_candidate_key:
                    candidates.append(dict(t=now,measured_speed_kmh=vehicle.v*3.6,
                        rows=[dict(side=c.side,length_m=c.transition_length_m,rejection=c.rejection,
                                   entry_end_s=c.entry_end_s,return_start_s=c.return_start_s,return_end_s=c.return_end_s,
                                   return_control_reserve_m=getattr(c,'return_control_reserve_m',None),
                                   return_preview_extension_m=getattr(c,'return_preview_extension_m',None))
                              for c in planner.candidates]));last_candidate_key=key
                if planner.selected is not None and not selected_geometry:
                    selected_geometry.append(dict(t=now,side=planner.selected.side,points=planner.selected.points,
                        entry_path=planner.selected.entry_path,return_path=planner.selected.return_path,
                        entry_end_s=planner.selected.entry_end_s,return_start_s=planner.selected.return_start_s,
                        return_end_s=planner.selected.return_end_s,
                        return_control_reserve_m=getattr(planner.selected,'return_control_reserve_m',None),
                        return_preview_extension_m=getattr(planner.selected,'return_preview_extension_m',None)))
                pending=dict(result=result,normal=normal['points'],ready_at=now+model,cpu_s=cpu,model_s=model,
                             new_validation=new,stamp=ns,phase_before=phase_before,timing_row=planning_rows[-1])
                record_transition(now)
        fsm_state=fsm.update(localization_ready=True,map_match_ready=True,global_path_ready=True,
            local_path_ready=len(control.local_path_points)>=2,speed_ready=True,drive_status='ACTIVE',
            obstacle_avoid_requested=planner.state!='NORMAL',emergency_stop_requested=planner.state=='BLOCKED',
            goal_reached=mission_stop_at is not None)
        if not fsm_transitions or fsm_transitions[-1]['state']!=fsm_state:
            fsm_transitions.append(dict(t=now,state=fsm_state))
        if not fsm.motion_allowed: control.target_speed_kmh=0.
        control.control_loop(); cmd=control.ctrl_pub.last
        request=cmd.front_steer/config.steering_sign*config.max_wheel_angle_rad
        max_requested_rate=max(max_requested_rate,abs(request-previous_request_wheel)/dt);previous_request_wheel=request
        first=vehicle.pose;vehicle.step(cmd,now,dt,config);second=vehicle.pose
        total_distance += math.dist(first[:2],second[:2])
        sweep=swept_body(planner.body,first,second)
        inside=road.area.covers(sweep);margin=road.margin(sweep,inside)
        clearance=min(polygon_distance(sweep,o) for o in road.obstacles)
        min_road=min(min_road,margin);min_clearance=min(min_clearance,clearance)
        tracking=yaw_error=0.
        if len(control.local_path_points)>=2:
            idx=PolylineIndex(control.local_path_points); row=sample(control.local_path_points,idx.project(first[:2]))
            tracking=math.dist(first[:2],row[0][:2])
            yaw_error=abs(math.atan2(math.sin(first[2]-math.atan2(row[1][1],row[1][0])),math.cos(first[2]-math.atan2(row[1][1],row[1][0]))))
        max_tracking=max(max_tracking,tracking);max_yaw_error=max(max_yaw_error,yaw_error)
        max_wheel=max(max_wheel,abs(vehicle.wheel));max_rate=max(max_rate,abs(vehicle.rate));max_speed=max(max_speed,vehicle.v*3.6)
        if planner.state in ('AVOID_ENTRY','AVOID_HOLD','RETURN'): max_avoid_speed=max(max_avoid_speed,vehicle.v*3.6)
        rows.append(dict(t=now,x=second[0],y=second[1],yaw=second[2],state=planner.state,fsm=fsm_state,
            target_speed_kmh=control.target_speed_kmh,speed_kmh=vehicle.v*3.6,accel=cmd.accel,brake=cmd.brake,
            requested_wheel_rad=request,wheel_rad=vehicle.wheel,wheel_rate_radps=vehicle.rate,
            lateral_error_m=tracking,yaw_error_rad=yaw_error,obstacle_clearance_m=clearance,
            road_margin_m=margin,inside_road=inside,local_path_empty=not bool(control.local_path_points)))
        history.append((now+dt,vehicle.pose))
        if not inside or clearance<=config.obstacle_margin_m:
            block('MODEL_ROAD_DEPARTURE' if not inside else 'MODEL_BODY_COLLISION',now+dt)
            break
        if stop_started_at is not None and vehicle.v<.01:
            if stopped_at is None: stopped_at=now+dt
            if now+dt-stopped_at>=scene.stop_hold_s: break
    phases=[r['state'] for r in transitions]
    final_station=road.index.project(vehicle.pose[:2]); original=sample(road.reference,final_station)
    return_error=math.dist(vehicle.pose[:2],original[0][:2])
    return_heading=abs(math.atan2(math.sin(vehicle.pose[2]-math.atan2(original[1][1],original[1][0])),math.cos(vehicle.pose[2]-math.atan2(original[1][1],original[1][0]))))
    safe=min_clearance>config.obstacle_margin_m and min_road>=-1e-8
    full=phases==['NORMAL','AVOID_ENTRY','AVOID_HOLD','RETURN','NORMAL']
    success=bool(full and returned_at is not None and rows[-1]['t']-returned_at>=scene.post_return_s-dt
                 and return_error<=config.acquisition_lateral_m and return_heading<=config.acquisition_heading_rad
                 and safe and max_tracking<=config.tracking_limit_m)
    stop_safe=bool(stop_started_at is not None and stopped_at is not None and safe and
                   (rows[-1]['local_path_empty'] if blocked_at is not None else mission_stop_at is not None)
                   and rows[-1]['target_speed_kmh']==0. and rows[-1]['brake']>0.)
    success = success and stop_safe and mission_stop_at is not None
    assert hashlib.sha256(json.dumps(road.reference).encode()).hexdigest()==route_hash_before
    result=dict(schema='edgenode.synthetic.s4.stage5a.v1',data_kind='SYNTHETIC_NOT_DRIVING_APPROVAL',scene=asdict(scene),
        simulation_only=True,ros_nodes_started=False,production_enable_drive=False,controller='production ControlNode.control_loop + production PID + PP',
        vehicle_model=dict(type='no-slip rear-axle bicycle + delayed pedals + first-order rate-limited steering',
                           parameters=asdict(scene),geometry=asdict(planner.body)),
        distance_policy=asdict(policy),observation=dict(usable_range_m=evidence.forward_range_m if evidence else None,
            roi_ceiling_m=21.19 if scene.experiment=='B' else None,real_lidar_verified=False,
            synthetic_direct_injection=scene.experiment=='A',clock='deterministic virtual; host CPU recorded separately'),
        success=success,safe_model_stop=stop_safe,full_fsm_sequence=full,global_path_return=success,
        mission_stop_at_s=mission_stop_at,stopped_at_s=stopped_at,stop_hold_s=(
            rows[-1]['t']+dt-stopped_at if stopped_at is not None else 0.),
        blocked=blocked_at is not None,failure_phase=blocked_phase or ('INCOMPLETE_LIFECYCLE' if not success else ''),
        reason=blocked_reason or ('' if success else 'SIMULATION_DURATION'),transitions=transitions,planning_fsm=fsm_transitions,
        candidates=candidates,min_obstacle_clearance_m=min_clearance,min_road_margin_m=min_road,
        max_lateral_error_m=max_tracking,max_yaw_error_rad=max_yaw_error,max_wheel_rad=max_wheel,
        max_wheel_rate_radps=max_rate,max_requested_wheel_rate_radps=max_requested_rate,
        steering_rate_limited_steps=vehicle.rate_limited_steps,max_speed_kmh=max_speed,max_avoidance_speed_kmh=max_avoid_speed,
        return_error_m=return_error,return_heading_rad=return_heading,post_return_duration_s=(
            (mission_stop_at if mission_stop_at is not None else rows[-1]['t'])-returned_at
            if returned_at is not None else 0.),
        stop_time_s=stopped_at-stop_started_at if stopped_at is not None else None,
        stop_distance_m=total_distance-stop_start_distance if stop_start_distance is not None else None,
        blocked_pose=blocked_pose,final_pose=vehicle.pose,final_speed_kmh=vehicle.v*3.6,
        max_cpu_compute_s=max(timings,default=0.),cpu_budget_overruns=cpu_overruns,modeled_budget_overruns=model_overruns,
        max_process_cpu_compute_s=max(process_timings,default=0.),
        cpu_freshness_budget_exceedances=sum(t+scene.sensor_delay_s>guard.max_age_ns/1e9 for t in timings),
        max_observation_age_s=max_source_age,planning_cycles=len(timings),validating_cycles=validations,sensor_dropout_at_s=sensor_drop_at,
        global_reference_sha256=route_hash_before,rows=rows,planning_rows=planning_rows,
        selected_geometry=selected_geometry,rollout_failures=rollout_failures,
        world=dict(original=road.reference,road_low=road.low,road_high=road.high,obstacles=road.obstacles))
    return result
