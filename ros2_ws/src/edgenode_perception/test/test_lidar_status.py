"""Sensor success is separate from object absence and legacy output compatibility."""
from array import array
import json
from unittest.mock import patch

import pytest
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

from edgenode_perception.lidar_status import LidarStatus
from test_lidar_obstacle_node import node as node_fixture, npc_cloud, odometry, output


node = node_fixture


def fresh(node, stamp_ns=10_000_000_000):
    node.test_clock.time_ns = stamp_ns
    pose = odometry()
    pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    node.localization_callback(pose)
    cloud = npc_cloud()
    cloud.header.stamp = pose.header.stamp
    return cloud


def status(node):
    return json.loads(output(node, '/perception/lidar_status').data)


def test_startup_has_status_but_does_not_fabricate_obstacles(node):
    node.publish_sensor_status()
    assert status(node)['state'] == 'WAITING_FOR_INPUT'
    node.obstacles_pub.publish.assert_not_called()


def test_valid_empty_and_valid_obstacle_are_both_successful_distinct_frames(node):
    cloud = fresh(node)
    node.lidar_callback(cloud)
    first = status(node)
    assert first['state'] == 'READY_OBSTACLES' and first['cluster_count'] == 1
    assert len(output(node).poses) == 8 and len(first['obstacle_digest']) == 64
    fresh(node, 10_100_000_000)
    empty = create_cloud_xyz32(Header(frame_id='/map'), [])
    empty.header.stamp.sec = 10
    empty.header.stamp.nanosec = 100_000_000
    node.lidar_callback(empty)
    second = status(node)
    assert second['state'] == 'READY_EMPTY' and second['cluster_count'] == 0
    assert second['frame_seq'] > first['frame_seq']
    assert not output(node).poses


@pytest.mark.parametrize('damage', ['truncated', 'missing_fields', 'all_nan'])
def test_invalid_cloud_is_never_reported_as_ready_empty(node, damage):
    cloud = fresh(node)
    if damage == 'truncated':
        cloud.data = array('B', [0])
    elif damage == 'missing_fields':
        cloud = PointCloud2(header=cloud.header)
    else:
        cloud = create_cloud_xyz32(cloud.header, [[float('nan'), 0., 0.]] * 5)
    node.lidar_callback(cloud)
    assert status(node)['state'] == 'INVALID_INPUT'
    assert not output(node).poses  # The legacy PoseArray contract is preserved.


def test_clustering_failure_has_explicit_status(node):
    cloud = fresh(node)
    with patch('edgenode_perception.lidar_obstacle_node.detect_clusters', side_effect=RuntimeError('cluster fault')):
        node.lidar_callback(cloud)
    assert status(node)['state'] == 'PROCESSING_ERROR'
    assert not output(node).poses


@pytest.mark.parametrize('delta,expected', [
    (0, 'DUPLICATE_FRAME'), (-1, 'OUT_OF_ORDER'),
    (-1_000_000_000, 'STALE_INPUT'), (100_000_000, 'FUTURE_STAMP'),
])
def test_timestamp_faults_are_not_hidden_by_fresh_receipt(node, delta, expected):
    node.lidar_callback(fresh(node))
    cloud = npc_cloud()
    cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(10_000_000_000 + delta, 10**9)
    node.lidar_callback(cloud)
    assert status(node)['state'] == expected


def test_missing_and_stale_localization_are_unavailable(node):
    cloud = fresh(node)
    node.latest_pose = None
    node.lidar_callback(cloud)
    assert status(node)['state'] == 'LOCALIZATION_UNAVAILABLE'
    node.lidar_callback(fresh(node, 10_100_000_000))
    node.test_clock.time_ns += 601_000_000
    node.publish_sensor_status()
    assert status(node)['state'] == 'INPUT_TIMEOUT'


def test_heartbeat_cannot_refresh_frame_age_even_when_ros_clock_is_frozen():
    steady = [0]
    sensor = LidarStatus(steady_time_ns=lambda: steady[0])
    assert sensor.begin(10**9, 10**9)
    sensor.finish(0, 'a' * 64, 10**9)
    first = sensor.snapshot(10**9)
    steady[0] = 601_000_000
    second = sensor.snapshot(10**9)
    assert second['state'] == 'INPUT_TIMEOUT'
    assert second['frame_seq'] == first['frame_seq']
    assert second['status_seq'] > first['status_seq']


def test_clock_rewind_requires_producer_restart():
    sensor = LidarStatus()
    sensor.begin(2 * 10**9, 2 * 10**9)
    assert sensor.snapshot(10**9)['state'] == 'CLOCK_REGRESSION'
    sensor.begin(3 * 10**9, 3 * 10**9)
    assert sensor.snapshot(3 * 10**9)['state'] == 'CLOCK_REGRESSION'
    assert LidarStatus().session_id != sensor.session_id


def test_source_stamps_expire_without_refreshing_receipt(node):
    cloud = fresh(node)
    cloud.header.stamp.sec = 9
    cloud.header.stamp.nanosec = 450_000_000
    node.lidar_callback(cloud)
    assert status(node)['state'] == 'READY_OBSTACLES'
    node.test_clock.time_ns += 60_000_000
    node.publish_sensor_status()
    assert status(node)['state'] == 'STALE_INPUT'


def test_failure_counter_survives_immediate_successful_recovery(node):
    cloud = fresh(node); cloud.data = array('B', [0])
    node.lidar_callback(cloud)
    failure = status(node)
    node.lidar_callback(fresh(node, 10_100_000_000))
    recovered = status(node)
    assert failure['state'] == 'INVALID_INPUT' and failure['fault_seq'] > 0
    assert recovered['state'] == 'READY_OBSTACLES'
    assert recovered['fault_seq'] == failure['fault_seq']
