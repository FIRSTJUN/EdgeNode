import math

import pytest
from rclpy.time import Time
from test_lidar_obstacle_node import node,odometry,npc_cloud,output


def pose_at(node,seconds,x=0.,yaw=0.):
    msg=odometry(x,0.,yaw=yaw);msg.header.stamp=Time(seconds=seconds).to_msg()
    node.localization_callback(msg)


def test_lidar_uses_interpolated_source_time_pose_instead_of_latest_pose(node):
    node._time_aligned=True;node.test_clock.time_ns=7_100_000_000
    pose_at(node,7.,0.,0.);pose_at(node,7.1,1.,.2)
    msg=npc_cloud();msg.header.stamp=Time(seconds=7.05).to_msg();node.lidar_callback(msg)
    assert node.sensor_status.state=='READY_OBSTACLES'
    # Undo only the source-time transform; the cluster must remain at its
    # original local coordinates, regardless of the newer latest pose.
    center=output(node,'/perception/lidar_obstacle_centers').poses[0].position
    x,y=center.x-.5,center.y
    local=(x*math.cos(.1)+y*math.sin(.1),-x*math.sin(.1)+y*math.cos(.1))
    assert local==pytest.approx((14.06+node.config.lidar_offset_x_m,.06))


def test_cloud_waits_for_a_bracketing_pose_without_an_empty_ready_frame(node):
    node._time_aligned=True;node.test_clock.time_ns=7_050_000_000;pose_at(node,7.)
    msg=npc_cloud();msg.header.stamp=Time(seconds=7.05).to_msg();node.lidar_callback(msg)
    assert node._pending_cloud is not None
    assert node.test_publishers['/planning/obstacles'].publish.call_count==0
    node.test_clock.time_ns=7_070_000_000;pose_at(node,7.07,.07,.01)
    node.process_pending_cloud()
    assert node._pending_cloud is None and node.sensor_status.state=='READY_OBSTACLES'


def test_missing_pose_support_is_a_sensor_failure_and_never_latest_pose_fallback(node):
    node._time_aligned=True;node.test_clock.time_ns=7_050_000_000;pose_at(node,7.)
    msg=npc_cloud();msg.header.stamp=Time(seconds=7.05).to_msg();node.lidar_callback(msg)
    node.test_clock.time_ns=7_210_000_000;node.process_pending_cloud()
    assert node.sensor_status.state=='LOCALIZATION_UNAVAILABLE' and not output(node).poses


def test_delayed_bracketing_callback_within_budget_does_not_create_a_sensor_fault(node):
    node._time_aligned=True;node.test_clock.time_ns=7_050_000_000;pose_at(node,7.)
    msg=npc_cloud();msg.header.stamp=Time(seconds=7.05).to_msg();node.lidar_callback(msg)
    node.test_clock.time_ns=7_110_000_000;node.process_pending_cloud()
    assert node._pending_cloud is not None and node.sensor_status.fault_seq==0
    node.test_clock.time_ns=7_140_000_000;pose_at(node,7.08,.08)
    node.process_pending_cloud()
    assert node.sensor_status.state=='READY_OBSTACLES' and node._pending_cloud is None
    assert node.sensor_status.fault_seq==0


def test_cloud_leading_each_pose_is_not_starved_by_newer_cloud_arrival(node):
    # Reproduce 10 Hz LiDAR arriving just before the next bracketing pose.
    node._time_aligned=True;node.test_clock.time_ns=7_050_000_000;pose_at(node,7.)
    first=npc_cloud();first.header.stamp=Time(seconds=7.05).to_msg();node.lidar_callback(first)
    node.test_clock.time_ns=7_150_000_000
    second=npc_cloud();second.header.stamp=Time(seconds=7.15).to_msg();node.lidar_callback(second)
    assert node._pending_cloud[0].header.stamp==first.header.stamp and node._next_cloud is not None
    node.test_clock.time_ns=7_160_000_000;pose_at(node,7.15,.15)
    node.process_pending_cloud()
    assert node.sensor_status.state=='READY_OBSTACLES'
    assert node.sensor_status.source_stamp_ns==7_050_000_000
    node.process_pending_cloud()
    assert node.sensor_status.source_stamp_ns==7_150_000_000 and node.sensor_status.fault_seq==0


def test_measured_157ms_simulator_cadence_can_wait_for_real_bracket_without_extrapolation(node):
    node._time_aligned=True;node._source_pair_wait_ns=250_000_000
    node.test_clock.time_ns=7_001_000_000;pose_at(node,7.)
    msg=npc_cloud();msg.header.stamp=Time(seconds=7.001).to_msg();node.lidar_callback(msg)
    node.test_clock.time_ns=7_156_000_000;node.process_pending_cloud()
    assert node._pending_cloud is not None and node.sensor_status.fault_seq==0
    node.test_clock.time_ns=7_170_000_000;pose_at(node,7.157,.157)
    node.process_pending_cloud()
    assert node.sensor_status.state=='READY_OBSTACLES' and node.sensor_status.source_stamp_ns==7_001_000_000
