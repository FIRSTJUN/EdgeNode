"""Real ROS messages and detector callbacks, with transport mocked for unit tests."""

from array import array
import math
from pathlib import Path
import struct
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
import yaml

from edgenode_perception.lidar_obstacle_node import LidarObstacleNode, pointcloud_xyz
from edgenode_perception.lidar_obstacles import LidarObstacleConfig


def cloud(points, frame='/map'):
    return create_cloud_xyz32(Header(frame_id=frame, stamp=Time(seconds=7).to_msg()), points)


def npc_cloud():
    # Five asymmetrical returns check that a genuine arithmetic center is transformed.
    return cloud([[14, 0, -0.5], [14.2, 0, -0.5], [14, 0.2, -0.5],
                  [14, 0, -0.3], [14.1, 0.1, -0.4]])


def odometry(x=22.55, y=1115.61, yaw=0.0, frame='map'):
    msg = Odometry()
    msg.header.frame_id = frame
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


@pytest.fixture
def node(monkeypatch):
    parameters, publishers, subscriptions = {}, {}, []
    clock = SimpleNamespace(time_ns=10_000_000_000)
    logger = Mock()

    def declare(self, name, value, descriptor):
        assert descriptor.read_only
        parameters[name] = value

    def publish(self, message_type, topic, qos):
        pub = Mock()
        pub.message_type = message_type
        publishers[topic] = pub
        return pub

    def subscribe(self, message_type, topic, callback, qos):
        subscriptions.append((message_type, topic, callback, qos))
        return Mock()

    monkeypatch.setattr(Node, '__init__', Mock(return_value=None))
    monkeypatch.setattr(Node, 'declare_parameter', declare)
    monkeypatch.setattr(Node, 'get_parameter',
                        lambda self, name: SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(Node, 'create_publisher', publish)
    monkeypatch.setattr(Node, 'create_subscription', subscribe)
    monkeypatch.setattr(Node, 'get_clock', lambda self: SimpleNamespace(
        now=lambda: Time(nanoseconds=clock.time_ns)))
    monkeypatch.setattr(Node, 'get_logger', lambda self: logger)
    result = LidarObstacleNode()
    result.test_publishers = publishers
    result.test_subscriptions = subscriptions
    result.test_clock = clock
    result.test_logger = logger
    return result


def output(node, topic='/planning/obstacles'):
    return node.test_publishers[topic].publish.call_args.args[0]


def assert_empty(node):
    assert isinstance(output(node), PoseArray)
    assert output(node).header.frame_id == 'map'
    assert not output(node).poses
    assert not output(node, '/perception/lidar_obstacle_centers').poses
    assert output(node, '/perception/lidar_obstacle_count').data == 0
    assert math.isnan(output(node, '/perception/nearest_obstacle_distance').data)


def test_lidar_subscription_matches_morai_best_effort_volatile_keep_last(node):
    item = next(s for s in node.test_subscriptions if s[1] == '/lidar/points')
    assert item[0] == PointCloud2
    qos = item[3]
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 5
    assert any(s[0] == Odometry and s[1] == '/localization/odometry'
               for s in node.test_subscriptions)


def test_without_localization_publishes_empty_and_throttled_warning(node):
    node.lidar_callback(npc_cloud())
    assert_empty(node)
    assert node.test_logger.warning.call_args.kwargs['throttle_duration_sec'] == 5.0


def test_no_lidar_message_never_publishes_fake_detection(node):
    node.localization_callback(odometry())
    node.obstacles_pub.publish.assert_not_called()


def test_stale_localization_clears_previously_detected_obstacles(node):
    node.localization_callback(odometry())
    node.lidar_callback(npc_cloud())
    assert len(output(node).poses) == 8
    node.test_clock.time_ns += 601_000_000
    node.lidar_callback(npc_cloud())
    assert_empty(node)


def test_clock_regression_rejects_pose_from_future_receipt(node):
    node.localization_callback(odometry())
    node.test_clock.time_ns -= 1
    node.lidar_callback(npc_cloud())
    assert_empty(node)


def test_empty_cloud_clears_previous_detection(node):
    node.localization_callback(odometry())
    node.lidar_callback(npc_cloud())
    node.lidar_callback(PointCloud2())
    assert_empty(node)


@pytest.mark.parametrize('yaw,expected', [
    (0.0, (37.8, 1115.67)),
    (math.pi / 2, (22.49, 1130.86)),
])
def test_cluster_footprint_world_transform_and_centroid_diagnostics(node, yaw, expected):
    node.localization_callback(odometry(yaw=yaw))
    msg = npc_cloud()
    node.lidar_callback(msg)
    result = output(node)
    assert result.header.frame_id == 'map'
    assert result.header.stamp == msg.header.stamp
    assert len(result.poses) == 8
    if yaw == 0.0:
        footprint = [(37.74, 1115.61), (37.84, 1115.61), (37.94, 1115.61),
                     (37.94, 1115.71), (37.94, 1115.81), (37.84, 1115.81),
                     (37.74, 1115.81), (37.74, 1115.71)]
    else:
        footprint = [(22.55, 1130.80), (22.55, 1130.90), (22.55, 1131.00),
                     (22.45, 1131.00), (22.35, 1131.00), (22.35, 1130.90),
                     (22.35, 1130.80), (22.45, 1130.80)]
    np.testing.assert_allclose([(p.position.x, p.position.y) for p in result.poses],
                               footprint, rtol=0, atol=1e-6)
    centers = output(node, '/perception/lidar_obstacle_centers')
    assert len(centers.poses) == 1
    assert centers.header == result.header
    pose = centers.poses[0]
    assert (pose.position.x, pose.position.y) == pytest.approx(expected, abs=1e-6)
    assert pose.position.z == 0.0
    q = pose.orientation
    assert (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0)
    assert output(node, '/perception/lidar_obstacle_count').data == 1
    # Center=(14.06, .06, -.44); sensor-origin range, not world-origin range.
    assert output(node, '/perception/nearest_obstacle_distance').data == pytest.approx(
        math.hypot(14.06, 0.06))


def test_misleading_cloud_map_header_does_not_bypass_world_transform(node):
    node.localization_callback(odometry(100, 200))
    msg = npc_cloud()
    msg.header.frame_id = '/map'
    node.lidar_callback(msg)
    assert output(node).poses[0].position.x == pytest.approx(115.19)
    msg.header.frame_id = 'lidar'
    node.lidar_callback(msg)
    assert output(node).poses[0].position.x == pytest.approx(115.19)


def test_nonfinite_points_are_removed_even_when_is_dense_claims_true(node):
    node.localization_callback(odometry())
    points = [[6, 0, -0.5]] * 5 + [[math.nan, 0, 0], [8, math.inf, 0], [8, 0, math.inf]]
    msg = cloud(points)
    msg.is_dense = True
    node.lidar_callback(msg)
    assert len(output(node).poses) == 8
    assert output(node).poses[0].position.x == pytest.approx(29.74)


@pytest.mark.parametrize('kind', [
    'position_nan', 'quaternion_inf', 'zero_quaternion', 'wrong_frame',
])
def test_invalid_localization_clears_last_valid_pose(node, kind):
    node.localization_callback(odometry())
    msg = odometry()
    if kind == 'position_nan':
        msg.pose.pose.position.x = math.nan
    elif kind == 'quaternion_inf':
        msg.pose.pose.orientation.x = math.inf
    elif kind == 'zero_quaternion':
        msg.pose.pose.orientation.w = 0.0
    else:
        msg.header.frame_id = 'odom'
    node.localization_callback(msg)
    node.lidar_callback(npc_cloud())
    assert_empty(node)


def test_slash_map_odometry_and_scaled_quaternion_use_standard_yaw(node):
    msg = odometry(yaw=math.pi / 2, frame='/map')
    msg.pose.pose.orientation.z *= 2
    msg.pose.pose.orientation.w *= 2
    node.localization_callback(msg)
    node.lidar_callback(npc_cloud())
    centers = output(node, '/perception/lidar_obstacle_centers')
    assert centers.poses[0].position.y == pytest.approx(1130.86)


def test_malformed_cloud_is_empty_and_does_not_crash(node):
    node.localization_callback(odometry())
    msg = npc_cloud()
    msg.data = array('B', [0])
    node.lidar_callback(msg)
    assert_empty(node)


def test_pointcloud_reader_uses_field_names_offsets_and_ignores_intensity():
    msg = PointCloud2(height=1, width=1, point_step=20, row_step=20,
                      fields=[PointField(name='intensity', offset=0,
                                         datatype=PointField.UINT32, count=1),
                              PointField(name='z', offset=16, datatype=PointField.FLOAT32, count=1),
                              PointField(name='x', offset=4, datatype=PointField.FLOAT64, count=1),
                              PointField(name='y', offset=12,
                                         datatype=PointField.FLOAT32, count=1)],
                      data=array('B', struct.pack('<Idff', 7, 10.5, -2.0, -0.4)))
    np.testing.assert_allclose(pointcloud_xyz(msg), [[10.5, -2.0, -0.4]])


@pytest.mark.parametrize('bigendian', [False, True])
def test_organized_cloud_row_padding_and_byte_order_do_not_mutate_data(bigendian):
    fmt = '>fff' if bigendian else '<fff'
    data = array('B', struct.pack(fmt, 6, 1, 0) + b'pad!' + struct.pack(fmt, 8, -1, -0.5) + b'pad!')
    msg = PointCloud2(height=2, width=1, point_step=12, row_step=16,
                      is_bigendian=bigendian,
                      fields=[PointField(name=name, offset=i * 4,
                                         datatype=PointField.FLOAT32, count=1)
                              for i, name in enumerate(('x', 'y', 'z'))], data=data)
    before = bytes(msg.data)
    np.testing.assert_allclose(pointcloud_xyz(msg), [[6, 1, 0], [8, -1, -0.5]])
    assert bytes(msg.data) == before


@pytest.mark.parametrize('kind', ['missing_z', 'duplicate_x', 'vector_x', 'integer_x',
                                  'short_row', 'short_point', 'truncated'])
def test_bad_cloud_layout_is_rejected(kind):
    msg = npc_cloud()
    if kind == 'missing_z':
        msg.fields = msg.fields[:2]
    elif kind == 'duplicate_x':
        msg.fields[2].name = 'x'
    elif kind == 'vector_x':
        msg.fields[0].count = 2
    elif kind == 'integer_x':
        msg.fields[0].datatype = PointField.UINT32
    elif kind == 'short_row':
        msg.row_step = 1
    elif kind == 'short_point':
        msg.point_step = 4
    else:
        msg.data = msg.data[:-1]
    with pytest.raises(ValueError):
        pointcloud_xyz(msg)


def test_installed_baseline_config_matches_algorithm_defaults():
    root = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((root / 'config' / 'lidar_obstacle.yaml').read_text())[
        'lidar_obstacle_node']['ros__parameters']
    config = LidarObstacleConfig()
    for name in config.__dataclass_fields__:
        assert params[name] == getattr(config, name)


def test_two_cluster_count_and_nearest_are_independent_of_16_footprint_poses(node):
    node.localization_callback(odometry())
    msg = cloud([[10, 2, -0.5]] * 5 + [[15, -2, -0.5]] * 5)
    node.lidar_callback(msg)
    assert len(output(node).poses) == 16
    assert len(output(node, '/perception/lidar_obstacle_centers').poses) == 2
    assert output(node, '/perception/lidar_obstacle_count').data == 2
    assert output(node, '/perception/nearest_obstacle_distance').data == pytest.approx(
        math.hypot(10, 2))


@pytest.mark.parametrize('distance', [14.0, 10.0, 7.0, 5.0, 3.0])
def test_stationary_npc_distance_sequence_keeps_one_obstacle(node, distance):
    node.localization_callback(odometry())
    points = [[distance - 0.1, -0.3, -0.5], [distance - 0.1, 0.3, -0.5],
              [distance + 0.1, -0.3, -0.5], [distance + 0.1, 0.3, -0.5],
              [distance, 0.0, -0.5]]
    node.lidar_callback(cloud(points))
    assert output(node, '/perception/lidar_obstacle_count').data == 1
    assert len(output(node).poses) == 8
    assert len(output(node, '/perception/lidar_obstacle_centers').poses) == 1
    assert output(node, '/perception/nearest_obstacle_distance').data == pytest.approx(distance)
