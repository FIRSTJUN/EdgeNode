from sensor_msgs.msg import Imu
from rclpy.time import Time
from test_lidar_obstacle_node import node,cloud,odometry,output,npc_cloud


def test_missing_or_unpaired_imu_never_publishes_ready_empty(node):
    node._gravity_ground=True;node.test_clock.time_ns=7_000_000_000
    pose=odometry();pose.header.stamp=Time(seconds=7).to_msg();node.localization_callback(pose)
    node.lidar_callback(cloud([[14.,0.,-.5]]*5))
    assert node.sensor_status.state=='WAITING_FOR_INPUT'
    assert node.test_publishers['/planning/obstacles'].publish.call_count==0
    node.test_clock.time_ns+=60_000_000;node.process_pending_cloud()
    assert node.sensor_status.state=='IMU_UNAVAILABLE'
    assert not output(node).poses
    # A fresh, timestamp-matched IMU/pose/cloud restores normal processing.
    node.test_clock.time_ns=7_100_000_000
    imu=Imu();imu.header.stamp=Time(seconds=7.1).to_msg();imu.orientation.w=1.
    node.imu_callback(imu)
    pose.header.stamp=imu.header.stamp;node.localization_callback(pose)
    msg=cloud([[14.,0.,-.5]]*5);msg.header.stamp=imu.header.stamp
    node.lidar_callback(msg)
    assert node.sensor_status.state=='READY_OBSTACLES' and len(output(node).poses)==8


def test_invalid_imu_and_duplicate_timestamps_do_not_refresh_attitude(node):
    imu=Imu();imu.header.stamp.sec=7;imu.orientation.w=1.
    node.imu_callback(imu);node.imu_callback(imu)
    assert len(node._attitudes)==1


def test_new_paired_cloud_supersedes_old_deferred_cloud(node):
    node._gravity_ground=True;node.test_clock.time_ns=7_000_000_000
    pose=odometry();pose.header.stamp=Time(seconds=7).to_msg();node.localization_callback(pose)
    node.lidar_callback(npc_cloud())
    assert node._pending_cloud is not None
    # A newer cloud has a usable attitude; the old cloud must never replay.
    node.test_clock.time_ns=7_100_000_000
    imu=Imu();imu.header.stamp=Time(seconds=7.1).to_msg();imu.orientation.w=1.
    node._attitudes.append((7_100_000_000,0.,0.))
    msg=npc_cloud();msg.header.stamp=imu.header.stamp
    node.lidar_callback(msg)
    assert node._pending_cloud is None
    node.test_clock.time_ns+=60_000_000;node.process_pending_cloud()
    assert node.sensor_status.last_stamp_ns==7_100_000_000
    assert node.sensor_status.state=='READY_OBSTACLES'
    assert node.sensor_status.fault_seq==0


def test_later_arriving_attitude_pairs_cloud_without_a_false_sensor_fault(node):
    node._gravity_ground=True;node.test_clock.time_ns=7_000_000_000
    pose=odometry();pose.header.stamp=Time(seconds=7).to_msg();node.localization_callback(pose)
    node.lidar_callback(npc_cloud())
    assert node._pending_cloud is not None
    node.test_clock.time_ns+=20_000_000
    imu=Imu();imu.header.stamp.sec=7;imu.orientation.w=1.;node.imu_callback(imu)
    assert node._pending_cloud is None and node.sensor_status.state=='READY_OBSTACLES'
    assert node.sensor_status.fault_seq==0
    imu.header.stamp.sec=8;imu.orientation_covariance[0]=-1.
    node.imu_callback(imu)
    assert len(node._attitudes)==1
