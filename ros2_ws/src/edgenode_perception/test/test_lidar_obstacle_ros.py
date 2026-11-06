"""Real ROS transport on unique topics; no vehicle commands or live Planning node."""

import math
import time
from types import SimpleNamespace
import uuid

import pytest
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Float32, Header, Int32

from edgenode_perception.lidar_obstacle_node import LidarObstacleNode
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from edgenode_planning.planning_fsm import PlanningFSM
from edgenode_planning.planning_node import PlanningNode


def test_best_effort_cloud_world_pose_planning_lattice_and_disabled_fsm():
    prefix = '/s4_lidar_test_' + uuid.uuid4().hex[:8]
    topics = {name: prefix + '/' + suffix for name, suffix in (
        ('lidar_topic', 'points'), ('localization_topic', 'odometry'),
        ('obstacles_topic', 'obstacles'), ('obstacle_count_topic', 'count'),
        ('nearest_distance_topic', 'nearest'), ('cluster_centers_topic', 'centers'))}
    args = ['--ros-args', '-r', '__ns:=' + prefix]
    for name, topic in topics.items():
        args.extend(['-p', f'{name}:={topic}'])
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    nodes = []

    def until(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), 'Isolated LiDAR ROS operation timed out'

    try:
        detector = LidarObstacleNode()
        nodes.append(detector)
        probe = Node('probe')
        nodes.append(probe)
        for node in nodes:
            executor.add_node(node)
        received, counts, distances = [], [], []
        probe.create_subscription(PoseArray, topics['obstacles_topic'], received.append, 10)
        probe.create_subscription(Int32, topics['obstacle_count_topic'], counts.append, 10)
        probe.create_subscription(Float32, topics['nearest_distance_topic'], distances.append, 10)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        lidar = probe.create_publisher(PointCloud2, topics['lidar_topic'], qos)
        odom = probe.create_publisher(Odometry, topics['localization_topic'], 10)
        until(lambda: lidar.get_subscription_count() == 1 and odom.get_subscription_count() == 1
              and detector.obstacles_pub.get_subscription_count() == 1
              and detector.count_pub.get_subscription_count() == 1
              and detector.distance_pub.get_subscription_count() == 1)
        scan = create_cloud_xyz32(Header(frame_id='/map'), [[14.06, 0.06, -0.44]] * 5)
        lidar.publish(scan)
        until(lambda: bool(received) and bool(counts) and bool(distances))
        assert not received[-1].poses
        assert counts[-1].data == 0
        assert math.isnan(distances[-1].data)

        pose = Odometry()
        pose.header.frame_id = 'map'
        pose.pose.pose.orientation.w = 1.0
        odom.publish(pose)
        until(lambda: detector.latest_pose is not None)
        lidar.publish(scan)
        until(lambda: bool(received[-1].poses) and counts[-1].data == 1
              and math.isfinite(distances[-1].data))
        result = received[-1]
        assert result.header.frame_id == 'map'
        assert result.poses[0].position.x == pytest.approx(15.25, abs=1e-6)
        assert result.poses[0].position.y == pytest.approx(0.06, abs=1e-6)
        assert result.poses[0].orientation.w == 1.0
        assert distances[-1].data == pytest.approx(math.hypot(14.06, 0.06))

        # Exercise the exact existing consumer and algorithm, without starting
        # a Planning node or publishing its paths/target speeds on live topics.
        consumer = SimpleNamespace(latest_obstacle_groups=[],obstacle_stamp_ns=None,
            get_clock=probe.get_clock,get_parameter=lambda name:SimpleNamespace(value=8))
        PlanningNode.obstacle_callback(consumer, result)
        assert len(result.poses) == 8
        assert len(consumer.latest_obstacle_groups) == 1
        assert len(consumer.latest_obstacle_groups[0]) == 8
        assert consumer.latest_obstacle_groups[0][0] == pytest.approx((15.25, 0.06), abs=1e-6)
        route = [[float(x), 0.0, 0.0] for x in range(31)]
        avoidance = StaticObstaclePlanner(SimpleNamespace(pair=lambda *args:None)).plan(
            route, consumer.latest_obstacle_groups, (0.0, 0.0, 0.0))
        # Repeated identical synthetic returns form a degenerate box. Detection
        # transport is valid; missing physical obstacle extent must fail closed.
        assert avoidance['blocked'] and not avoidance['points']
        fsm = PlanningFSM()
        assert fsm.update(localization_ready=True, map_match_ready=True,
                          global_path_ready=True, local_path_ready=True, speed_ready=True,
                          drive_status='DISABLED',
                          obstacle_avoid_requested=avoidance['active'],
                          emergency_stop_requested=avoidance['blocked']) == 'EMERGENCY_STOP'
        assert not fsm.motion_allowed

        old_count = len(received)
        lidar.publish(create_cloud_xyz32(Header(frame_id='/map'), []))
        until(lambda: len(received) > old_count and counts[-1].data == 0
              and math.isnan(distances[-1].data))
        assert not received[-1].poses
        PlanningNode.obstacle_callback(consumer, received[-1])
        assert not consumer.latest_obstacle_groups
    finally:
        executor.shutdown(timeout_sec=0)
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
