#!/usr/bin/env python3
"""Read-only continuity and source-time audit of an already running S4 stack."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from morai_ros2_msgs.msg import EgoVehicleStatus, GPSMessage
from morai_drive import Monitor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 5.:
        parser.error('--seconds must be finite and at least 5')
    if args.output.exists():
        parser.error('output already exists; keep previous evidence')
    rclpy.init()
    node = Monitor(reliable_supervision=True)
    rows = {}
    sensor_states = []
    collecting = False

    def receive(topic, msg):
        if not collecting:
            return
        now = node.get_clock().now().nanoseconds
        if isinstance(msg, String):
            value = json.loads(msg.data)
            if topic == '/planning/s4_sensor_state':
                sensor_states.append(value)
            stamp = value.get('status', value).get('source_stamp_ns')
        else:
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        rows.setdefault(topic, []).append(dict(receipt_ns=now, steady_s=time.monotonic(), source_ns=stamp))

    for topic, kind in [('/lidar/points', PointCloud2), ('/gps', GPSMessage),
                        ('/Imu', Imu), ('/ego_vehicle_status', EgoVehicleStatus),
                        ('/localization/odometry', Odometry),
                        ('/perception/lidar_frame', String),
                        ('/planning/s4_sensor_state', String)]:
        node.create_subscription(kind, topic, lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data)
    node.create_subscription(String, '/perception/lidar_diagnostics',
                             lambda msg: node.receive('/perception/lidar_diagnostics', msg), 1)
    node.spin(3.)
    start = time.time_ns()
    collecting = True
    node.spin(args.seconds)
    collecting = False
    statistics = {}
    failures = []
    for topic in ['/lidar/points', '/gps', '/Imu', '/ego_vehicle_status',
                  '/localization/odometry', '/perception/lidar_frame', '/planning/s4_sensor_state']:
        values = rows.get(topic, [])
        stamps = [v['source_ns'] for v in values if v['source_ns']]
        gaps = [b['steady_s']-a['steady_s'] for a, b in zip(values, values[1:])]
        ages = [(v['receipt_ns']-v['source_ns'])/1e9 for v in values if v['source_ns']]
        regressions = sum(b < a for a, b in zip(stamps, stamps[1:]))
        statistics[topic] = dict(count=len(values), unique_source_stamps=len(set(stamps)),
            first_source_ns=stamps[0] if stamps else None, last_source_ns=stamps[-1] if stamps else None,
            max_receipt_gap_s=max(gaps, default=None), max_source_age_s=max(ages, default=None),
            min_source_age_s=min(ages, default=None), source_regressions=regressions)
        if (len(values) < 5 or len(set(stamps)) < 5 or regressions or
            not ages or max(ages) > .6 or min(ages) < -.05 or max(gaps, default=math.inf) > .6):
            failures.append(topic + ': discontinuous or stale source')
    parameters = node.parameters()
    snapshot = node.snapshot()
    publishers = {t: node.count_publishers(t) for t in
                  ('/ctrl_cmd', '/planning/local_path', '/perception/lidar_frame')}
    if not node.stopped() or parameters['enable_drive']:
        failures.append('drive must remain disabled and physically stopped')
    if any(v != 1 for v in publishers.values()):
        failures.append('expected one control/path/frame publisher')
    if not sensor_states or any(not s.get('ready') or s.get('latched_reason') for s in sensor_states):
        failures.append('sensor not continuously READY')
    data = dict(scope='READ_ONLY_NO_RESTART_NO_PARAMETER_CHANGE_NO_DRIVING',
        success=not failures, failures=failures, started_utc_ns=start, finished_utc_ns=time.time_ns(),
        duration_s=args.seconds, topics=statistics, sensor_states=sensor_states,
        parameters=parameters, publishers=publishers, snapshot=snapshot)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2)+'\n')
    print(json.dumps({k: v for k, v in data.items() if k not in ('sensor_states', 'snapshot')}, indent=2))
    print(args.output)
    node.destroy_node()
    rclpy.shutdown()
    return int(bool(failures))


if __name__ == '__main__':
    raise SystemExit(main())
