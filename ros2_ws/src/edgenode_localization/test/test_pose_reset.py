import math
from unittest.mock import Mock,patch

import pytest
import rclpy
from morai_ros2_msgs.msg import GPSMessage
from edgenode_localization.pose_reset import PoseResetDetector
from edgenode_localization.localization_node import LocalizationNode


def test_single_outlier_does_not_confirm_and_repeated_stamp_cannot_count():
    detector=PoseResetDetector()
    assert detector.observe(0.,0.,1_000_000_000)=='NORMAL'
    assert detector.observe(100.,0.,1_100_000_000)=='PENDING'
    assert detector.observe(100.,0.,1_100_000_000)=='INVALID'
    assert detector.count==1
    assert detector.observe(.1,0.,1_200_000_000)=='CANCELLED'
    assert detector.candidate is None


def test_legitimate_50_kmh_travel_and_explicit_relocation():
    detector=PoseResetDetector()
    for i in range(20):assert detector.observe(i*1.389,0.,(10+i)*100_000_000)=='NORMAL'
    detector.expect(200.,100.)
    assert detector.observe(26.4,0.,3_000_000_000)=='PENDING'
    for i in range(3):state=detector.observe(200.+i*.01,100.,3_100_000_000+i*100_000_000)
    assert state=='CONFIRMED'


def test_confirmed_gps_jump_waits_for_stop_and_clears_velocity_history():
    rclpy.init(args=['--ros-args','-r','__ns:=/offline_pose_reset'])
    node=None
    try:
        node=LocalizationNode();node._reset_enabled=True
        node.ekf.initialize(0.,0.,0.,v=0.);node.latest_imu_yaw=.3
        node.reset_event_publisher=Mock();node.status_publisher=Mock()
        node.gps_odometry_publisher=Mock();node.gps_speed_publisher=Mock()
        node.gps_transformer=Mock()
        def gps(x,stamp):
            msg=GPSMessage(longitude=127.,latitude=37.,east_offset=1.,north_offset=1.)
            msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp,10**9)
            node.gps_transformer.transform.return_value=(x+1.,1.)
            node.gps_callback(msg)
        with patch.object(node,'_reset_stationary',return_value=False):
            gps(0.,1_000_000_000)
            for i in range(3):gps(100.,1_100_000_000+i*100_000_000)
            assert node.ekf.get_state()[0]<1.
        with patch.object(node,'_reset_stationary',return_value=True):gps(100.,1_400_000_000)
        x,y,yaw,v=node.ekf.get_state()
        assert (x,y,v)==(100.,0.,0.) and math.isclose(yaw,.3)
        assert node._previous_gps is None and node._last_imu_stamp_ns is None
        assert node.gps_speed_publisher.publish.call_count==0
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
