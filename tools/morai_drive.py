#!/usr/bin/env python3
"""MORAI status/start/stop and supervised, recorded driving from a sourced shell."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from morai_ros2_msgs.msg import CollisionData, CtrlCmd, EgoVehicleStatus
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Float32, String


class Monitor(Node):
    def __init__(self, *, reliable_supervision=False):
        super().__init__('morai_drive_operator')
        self.values, self.received = {}, {}
        for topic, kind in [('/ego_vehicle_status', EgoVehicleStatus), ('/ctrl_cmd', CtrlCmd),
                ('/collision_data', CollisionData), ('/planning/target_speed', Float32),
                ('/control/path_error', Float32), ('/planning/current_link', String),
                ('/planning/fsm_state', String), ('/planning/drive_status', String),
                ('/planning/speed_mode', String), ('/planning/avoidance_debug', String),
                ('/planning/s4_sensor_state', String), ('/planning/map_match_status', String),
                ('/planning/local_path', NavPath)]:
            qos=qos_profile_sensor_data
            if reliable_supervision:
                from rclpy.qos import QoSProfile,ReliabilityPolicy
                publishers=self.get_publishers_info_by_topic(topic)
                reliable=publishers and all(p.qos_profile.reliability==ReliabilityPolicy.RELIABLE for p in publishers)
                qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(kind, topic, lambda m, t=topic: self.receive(t, m), qos)
        self.setter = self.create_client(SetParametersAtomically, '/planning_node/set_parameters_atomically')
        self.getter = self.create_client(GetParameters, '/planning_node/get_parameters')

    def receive(self, topic, msg):
        if isinstance(msg, EgoVehicleStatus):
            value = dict(speed_kmh=math.hypot(msg.velocity.x, msg.velocity.y, msg.velocity.z)*3.6,
                x=msg.position.x, y=msg.position.y, heading_deg=msg.heading,
                left_boundary_m=msg.distance_left_lane_boundary,
                right_boundary_m=msg.distance_right_lane_boundary)
        elif isinstance(msg, CtrlCmd):
            value = dict(accel=msg.accel, brake=msg.brake, steer=msg.front_steer)
        elif isinstance(msg, CollisionData):
            value = len(msg.collision_object)
        elif isinstance(msg, NavPath):
            value = dict(points=len(msg.poses))
        else:
            value = msg.data
            if isinstance(value, str):
                try: value = json.loads(value)
                except ValueError: pass
        self.values[topic] = value
        self.received[topic] = time.monotonic()

    def spin(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.03)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('ROS service unavailable: '+client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=4.)
        if not future.done() or future.result() is None:
            raise RuntimeError('ROS service timeout')
        return future.result()

    def parameters(self):
        names = ['enable_drive', 'cruise_speed_kmh', 'sharp_curve_speed_kmh',
                 'avoidance_avoid_speed_kmh', 'drive_duration_sec']
        return dict(zip(names, map(parameter_value_to_python,
            self.call(self.getter, GetParameters.Request(names=names)).values)))

    def set_parameters_remote(self, values):
        response = self.call(self.setter, SetParametersAtomically.Request(parameters=[
            Parameter(k, value=v).to_parameter_msg() for k, v in values.items()])).result
        if not response.successful:
            raise RuntimeError(response.reason)

    def fresh(self):
        topics = ('/ego_vehicle_status', '/ctrl_cmd', '/collision_data', '/planning/target_speed',
                  '/planning/s4_sensor_state', '/planning/fsm_state')
        return all(time.monotonic()-self.received.get(t, 0) <= .6 for t in topics)

    def stopped(self):
        v = self.values
        return (self.fresh() and v['/ego_vehicle_status']['speed_kmh'] < .2
            and v['/planning/target_speed'] == 0. and v['/ctrl_cmd']['accel'] == 0.
            and v['/ctrl_cmd']['brake'] >= .6)

    def stop(self):
        self.set_parameters_remote({'enable_drive': False})
        stable = None
        end = time.monotonic()+8.
        while time.monotonic() < end:
            self.spin(.05)
            stable = (stable or time.monotonic()) if self.stopped() else None
            if stable and time.monotonic()-stable >= .5:
                return True
        return False

    def snapshot(self):
        return dict(values=dict(self.values), ages_s={t:time.monotonic()-s
            for t,s in self.received.items()}, ctrl_publishers=self.count_publishers('/ctrl_cmd'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['status', 'watch', 'start', 'stop', 'run'])
    parser.add_argument('--normal-kmh', type=float)
    parser.add_argument('--sharp-kmh', type=float)
    parser.add_argument('--seconds', type=float, default=90.)
    parser.add_argument('--expect-return', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--record-bag', action='store_true')
    args = parser.parse_args()
    for speed in (args.normal_kmh, args.sharp_kmh):
        if speed is not None and (not math.isfinite(speed) or speed <= 0):
            parser.error('positive finite speed required')
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('positive finite duration required')
    if args.output:
        args.output.mkdir(parents=True, exist_ok=False)
    if args.record_bag and (args.command != 'run' or args.output is None):
        parser.error('--record-bag requires run and --output')
    rclpy.init(); node = Monitor(); armed = False; code = 0
    summary = {}; rows = []; started = time.monotonic(); original = None
    bag = bag_log = None
    def interrupt(signum, frame):
        raise KeyboardInterrupt()
    for sig in (signal.SIGTERM, signal.SIGINT): signal.signal(sig, interrupt)
    try:
        node.spin(5.0)
        if args.command == 'stop':
            summary['stop_confirmed'] = node.stop()
            if not summary['stop_confirmed']: raise RuntimeError('physical stop not confirmed')
        elif args.command in ('start', 'run'):
            original = node.parameters()
            if original['enable_drive'] or not node.stopped():
                raise RuntimeError('vehicle must be stopped and drive disabled')
            if node.count_publishers('/ctrl_cmd') != 1:
                raise RuntimeError('exactly one control publisher required')
            if args.record_bag:
                topics = ['/ego_vehicle_status', '/ctrl_cmd', '/localization/odometry',
                    '/control/path_error', '/control/lookahead_distance', '/collision_data',
                    '/lidar/points', '/Imu', '/gps', '/perception/lidar_obstacle_count',
                    '/perception/lidar_obstacle_centers', '/perception/lidar_diagnostics',
                    '/perception/lidar_status', '/planning/obstacles', '/planning/current_link',
                    '/perception/lidar_points_map_xy', '/perception/lidar_cluster_points_map_xy',
                    '/planning/map_match_status', '/planning/target_speed', '/planning/global_path',
                    '/planning/local_path', '/planning/avoidance_debug', '/planning/s4_sensor_state',
                    '/planning/s4_distance_state', '/planning/fsm_state', '/planning/drive_status',
                    '/planning/speed_mode', '/localization/grid_convergence_deg']
                bag_log = (args.output/'rosbag-console.log').open('w')
                bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(args.output/'bag'),
                    *topics], stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True)
                node.spin(2.)
                if bag.poll() is not None: raise RuntimeError('rosbag recorder failed to start')
            # Existing route preparation checks saved mission, sensor pairing,
            # collision state and READY before enabling. No temporary script.
            values = {}
            if args.normal_kmh is not None: values['cruise_speed_kmh'] = args.normal_kmh
            if args.sharp_kmh is not None: values['sharp_curve_speed_kmh'] = args.sharp_kmh
            if args.command == 'run': values['drive_duration_sec'] = args.seconds+5.
            if values: node.set_parameters_remote(values)
            prepare = Path(__file__).with_name('s4_morai_prepare.py')
            armed = True  # A failed/interrupting start must also disable motion.
            subprocess.run([sys.executable, str(prepare), '--saved-start-link', 'A219BS010380', '--start'], check=True)
            summary['started'] = True
            summary['requested_parameters'] = node.parameters()
            if args.command == 'run':
                initial_returns = node.values.get('/planning/avoidance_debug', {}).get('return_commits', 0)
                start = time.monotonic(); normal_since = None; tracking_since = None
                while time.monotonic()-start < args.seconds:
                    node.spin(.1); row = node.snapshot(); row['elapsed_s'] = time.monotonic()-start; rows.append(row)
                    if bag is not None and bag.poll() is not None:
                        raise RuntimeError('rosbag recorder exited during trial')
                    v = node.values; sensor = v.get('/planning/s4_sensor_state', {})
                    debug = v.get('/planning/avoidance_debug', {})
                    if not node.fresh(): raise RuntimeError('live input timeout')
                    if v.get('/collision_data') != 0: raise RuntimeError('collision')
                    if sensor.get('latched_reason'): raise RuntimeError('sensor latch: '+sensor['latched_reason'])
                    if debug.get('state') == 'BLOCKED': raise RuntimeError('S4 blocked: '+debug.get('reason', ''))
                    tracking=(v.get('/planning/target_speed',0.)>0. and
                              v.get('/planning/local_path',{}).get('points',0)>=2)
                    tracking_since=(tracking_since or time.monotonic()) if tracking else None
                    error_age=time.monotonic()-node.received.get('/control/path_error',0)
                    if tracking and time.monotonic()-tracking_since>.6 and error_age>.6:
                        raise RuntimeError('path error diagnostic timeout')
                    if tracking and error_age<=.6 and v.get('/control/path_error', math.inf) > 1.:
                        raise RuntimeError('path error exceeds 1m')
                    returned = debug.get('state') == 'NORMAL' and debug.get('return_commits', 0) > initial_returns
                    normal_since = (normal_since or time.monotonic()) if returned else None
                    if normal_since and time.monotonic()-normal_since > 1.:
                        summary['return_completed'] = True
                        if args.expect_return: break
                    if len(rows) % 50 == 0:
                        print(json.dumps(dict(t=round(row['elapsed_s'],1), ego=v.get('/ego_vehicle_status'),
                            stage=debug.get('state'), reason=debug.get('reason'))), flush=True)
                if args.expect_return and not summary.get('return_completed'):
                    raise RuntimeError('return not completed in observation window')
                summary['success'] = True
        elif args.command == 'watch':
            while time.monotonic()-started < args.seconds:
                node.spin(.5)
                print(json.dumps(node.snapshot()), flush=True)
        summary['parameters'] = node.parameters()
        if not node.fresh(): raise RuntimeError('live state unavailable/stale')
        if args.command == 'start': armed = False
    except (Exception, KeyboardInterrupt) as error:
        code = 1; summary.update(success=False, error=str(error) or 'interrupted')
    finally:
        if armed:
            try: summary['stop_confirmed'] = node.stop()
            except Exception as error: summary['stop_error'] = str(error)
            if not summary.get('stop_confirmed'):
                code = 1
            elif args.command == 'run' and original is not None:
                # A trial must not leave a short lease or temporary test targets.
                try:
                    node.set_parameters_remote({k:original[k] for k in (
                        'drive_duration_sec', 'cruise_speed_kmh', 'sharp_curve_speed_kmh')})
                except Exception as error:
                    summary['restore_error'] = str(error)
                    code = 1
        try: summary['final_parameters'] = node.parameters()
        except Exception as error: summary['parameter_readback_error'] = str(error)
        summary['final'] = node.snapshot()
        if bag is not None:
            if bag.poll() is None:
                os.killpg(bag.pid, signal.SIGINT)
                try: bag.wait(timeout=15.)
                except subprocess.TimeoutExpired:
                    bag.terminate(); bag.wait(timeout=5.)
            summary['rosbag_exit_code'] = bag.returncode
            summary['rosbag_metadata_present'] = (args.output/'bag'/'metadata.yaml').exists()
        if bag_log is not None: bag_log.close()
        if rows:
            summary['peak_speed_kmh'] = max(r['values']['/ego_vehicle_status']['speed_kmh'] for r in rows)
            summary['max_path_error_m'] = max(r['values'].get('/control/path_error',0.) for r in rows)
        if args.output:
            (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
            if rows: (args.output/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        print(json.dumps(summary), flush=True)
        node.destroy_node();rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
