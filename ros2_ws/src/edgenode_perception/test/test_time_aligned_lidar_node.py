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
    assert node._pending_cloud is None and node.sensor_status.state=='READY_OBSTACLES'


def test_missing_pose_support_is_a_sensor_failure_and_never_latest_pose_fallback(node):
    node._time_aligned=True;node.test_clock.time_ns=7_050_000_000;pose_at(node,7.)
    msg=npc_cloud();msg.header.stamp=Time(seconds=7.05).to_msg();node.lidar_callback(msg)
    node.test_clock.time_ns=7_110_000_000;node.process_pending_cloud()
    assert node.sensor_status.state=='LOCALIZATION_UNAVAILABLE' and not output(node).poses
