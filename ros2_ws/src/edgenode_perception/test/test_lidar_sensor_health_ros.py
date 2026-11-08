"""Actual LiDAR -> status/footprint DDS -> Planning STOP/reset, no Control node."""
from array import array
import json
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Float32, Header, String
from std_srvs.srv import Trigger

from edgenode_perception.lidar_obstacle_node import LidarObstacleNode
from edgenode_planning.planning_node import PlanningNode


def test_real_sensor_status_pairing_failure_timeout_restart_and_explicit_reset(tmp_path):
    directory = tmp_path / 'map'; directory.mkdir()
    (directory / 'node_set.json').write_text(json.dumps([
        dict(idx='A', point=[0., 0., 0.]), dict(idx='B', point=[100., 0., 0.])]))
    (directory / 'link_set.json').write_text(json.dumps([
        dict(idx='AB', from_node_idx='A', to_node_idx='B',
             points=[[float(x), 0., 0.] for x in range(101)], max_speed=30.)]))
    prefix = '/sensor_health_' + uuid.uuid4().hex[:8]
    args = ['--ros-args', '-r', '__ns:=' + prefix,
            '-p', 's4_sensor_validation_enabled:=true',
            '-p', 'mgeo_dir:=' + str(directory), '-p', 'goal_node_id:=B']
    topics = ('target_error', 'target_speed', 'state', 'current_link', 'map_match_status',
        'map_match_distance', 'map_match_heading_diff', 'global_path', 'local_path',
        'local_path_index', 'local_path_progress_m', 'recommended_speed', 'speed_mode',
        'speed_curvature', 'speed_max_curvature', 'drive_status', 'fsm_state',
        'runtime_cruise_speed', 'runtime_curve_speed', 'runtime_sharp_curve_speed',
        'runtime_max_drive_speed', 'obstacles', 'avoidance_state', 'avoidance_debug',
        'lattice_candidate_markers', 'original_lane_center', 'left_lane_center',
        'right_lane_center', 'avoidance_selected_path', 'avoidance_return_path',
        'reset_avoidance', 's4_sensor_state')
    for topic in topics:
        args += ['-r', '/planning/' + topic + ':=' + prefix + '/' + topic]
    for original, target in (
            ('/localization/odometry', 'odometry'), ('/lidar/points', 'points'),
            ('/perception/lidar_status', 'status'),
            ('/perception/lidar_obstacle_count', 'count'),
            ('/perception/nearest_obstacle_distance', 'nearest'),
            ('/perception/lidar_obstacle_centers', 'centers')):
        args += ['-r', original + ':=' + prefix + '/' + target]
    rclpy.init(args=args)
    executor = SingleThreadedExecutor(); nodes = []
    try:
        detector = LidarObstacleNode(); planner = PlanningNode(); probe = Node('probe')
        nodes.extend((detector, planner, probe))
        for item in nodes: executor.add_node(item)
        speeds, paths, statuses, footprints = [], [], [], []
        probe.create_subscription(Float32, prefix + '/target_speed', speeds.append, 10)
        probe.create_subscription(Path, prefix + '/local_path', paths.append, 10)
        probe.create_subscription(String, prefix + '/status',
                                  lambda msg: statuses.append(json.loads(msg.data)), 10)
        probe.create_subscription(PoseArray, prefix + '/obstacles', footprints.append, 10)
        lidar = probe.create_publisher(PointCloud2, prefix + '/points',
                                      QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        odom = probe.create_publisher(Odometry, prefix + '/odometry', 10)
        reset = probe.create_client(Trigger, prefix + '/reset_avoidance')
        pose = Odometry(); pose.header.frame_id = 'map'; pose.pose.pose.orientation.w = 1.

        def publish(kind='empty'):
            pose.header.stamp = probe.get_clock().now().to_msg(); odom.publish(pose)
            if kind is None: return
            points = [] if kind == 'empty' else [
                [14., 0., -.5], [14.2, 0., -.5], [14., .2, -.5],
                [14., 0., -.3], [14.1, .1, -.4]]
            cloud = create_cloud_xyz32(Header(frame_id='/map', stamp=pose.header.stamp), points)
            if kind == 'invalid': cloud.data = array('B', [0])
            lidar.publish(cloud)

        def until(predicate, kind=None, timeout=5.):
            deadline = time.monotonic() + timeout
            next_publish = 0.
            while time.monotonic() < deadline:
                if time.monotonic() >= next_publish:
                    publish(kind)
                    next_publish = time.monotonic() + .05
                executor.spin_once(timeout_sec=.01)
                if predicate(): return
            raise AssertionError('isolated sensor lifecycle timed out: ' +
                                 str((planner.avoidance_planner.state, statuses[-1:])))

        def recover_and_reset():
            until(lambda: not planner.sensor_guard.reason(probe.get_clock().now().nanoseconds), 'empty')
            assert planner.avoidance_planner.state == 'BLOCKED'
            until(reset.service_is_ready, 'empty')
            future = reset.call_async(Trigger.Request())
            until(future.done, 'empty'); assert future.result().success
            until(lambda: planner.avoidance_planner.state == 'NORMAL' and paths and paths[-1].poses, 'empty')

        until(lambda: speeds and paths and planner.avoidance_planner.state == 'BLOCKED')
        assert speeds[-1].data == 0. and not paths[-1].poses
        recover_and_reset()
        assert not planner.get_parameter('enable_drive').value

        until(lambda: statuses and statuses[-1]['state'] == 'READY_OBSTACLES'
              and footprints and len(footprints[-1].poses) == 8, 'obstacle')
        until(lambda: planner.avoidance_planner.state == 'BLOCKED', 'obstacle')
        recover_and_reset()

        until(lambda: statuses[-1]['state'] == 'INVALID_INPUT'
              and planner.avoidance_planner.state == 'BLOCKED'
              and paths and not paths[-1].poses, 'invalid')
        assert speeds[-1].data == 0. and not paths[-1].poses
        assert planner.avoidance_planner.reason.startswith('S4_SENSOR_')
        recover_and_reset()

        until(lambda: statuses[-1]['state'] == 'INPUT_TIMEOUT'
              and planner.avoidance_planner.state == 'BLOCKED', None)
        recover_and_reset()

        previous_session = detector.sensor_status.session_id
        executor.remove_node(detector); detector.destroy_node(); nodes.remove(detector)
        detector = LidarObstacleNode(); nodes.append(detector); executor.add_node(detector)
        until(lambda: planner.sensor_guard.session_id == detector.sensor_status.session_id
              and planner.avoidance_planner.state == 'BLOCKED', 'empty')
        assert detector.sensor_status.session_id != previous_session
        recover_and_reset()
        assert probe.count_publishers('/ctrl_cmd') == 0
        assert all(speed.data == 0. for speed in speeds)
    finally:
        executor.shutdown()
        for item in nodes: item.destroy_node()
        rclpy.shutdown()
