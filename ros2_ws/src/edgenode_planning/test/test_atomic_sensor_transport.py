"""MORAI legacy planner can consume complete latest frames without queue halves."""
import json
import time
import uuid
from types import SimpleNamespace
from unittest.mock import Mock
from unittest.mock import patch

import rclpy
from std_msgs.msg import String
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from edgenode_planning.planning_node import PlanningNode
from nav_msgs.msg import Odometry
from morai_ros2_msgs.msg import EgoVehicleStatus
from test_sensor_readiness import payload
from test_static_obstacle_node import PLANNING_TOPICS


def test_queued_pose_is_admitted_before_watchdog_and_real_pose_gap_still_stops(route_editor_map):
    prefix='/offline_pose_inbox_'+uuid.uuid4().hex[:8]
    args=['--ros-args','-r','__ns:='+prefix,'-p',f'mgeo_dir:={route_editor_map}',
          '-p','goal_node_id:=D','-p','s4_sensor_validation_enabled:=true',
          '-p','s4_atomic_sensor_frames:=true','-p','enable_drive:=false',
          '-p','avoidance_use_ego_status:=true','-r','/ego_vehicle_status:='+prefix+'/ego',
          '-r','/localization/odometry:='+prefix+'/odometry',
          '-r','/perception/lidar_frame:='+prefix+'/frame']
    for topic in PLANNING_TOPICS:args+=['-r',f'/planning/{topic}:={prefix}/{topic}']
    rclpy.init(args=args);node=producer=None
    try:
        with patch.object(PlanningNode,'create_timer'):node=PlanningNode()
        for name in list(node.__dict__):
            if name.endswith('_pub'):setattr(node,name,Mock())
        stamp=node.get_clock().now().nanoseconds;clock=SimpleNamespace(ns=stamp)
        node.get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=clock.ns))
        node.latest_pose=(0.,0.,0.);node.latest_pose_stamp_ns=stamp-650_000_000;node._s4_pose_valid=True
        assert not node.sensor_guard.frame(json.dumps(dict(schema_version=1,frame_id='map',
            status=payload(stamp=stamp),points=[])),stamp)
        assert node.sensor_reason(stamp)=='PLANNING_LOCALIZATION_STALE'
        producer=Node('pose_inbox_producer');publisher=producer.create_publisher(Odometry,prefix+'/odometry',1)
        ego_publisher=producer.create_publisher(EgoVehicleStatus,prefix+'/ego',1)
        deadline=time.monotonic()+3.
        while publisher.get_subscription_count()!=1 and time.monotonic()<deadline:rclpy.spin_once(producer,timeout_sec=.01)
        assert publisher.get_subscription_count()==1 and node.localization_sub.qos_profile.depth==1
        pose=Odometry();pose.header.frame_id='map';pose.header.stamp=Time(nanoseconds=stamp-10_000_000).to_msg()
        pose.pose.pose.orientation.w=1.;publisher.publish(pose)
        ego=EgoVehicleStatus();ego.header.stamp=pose.header.stamp;ego_publisher.publish(ego)
        assert publisher.wait_for_all_acked(Duration(seconds=2.))
        assert ego_publisher.wait_for_all_acked(Duration(seconds=2.))
        node.sensor_watchdog()
        assert node._pose_inbox_frames==1 and not node.sensor_reason(stamp)
        assert node._ego_inbox_frames==1
        assert node.latest_pose_stamp_ns==stamp-10_000_000
        # A fresh LiDAR frame cannot renew missing odometry.
        clock.ns+=650_000_000
        assert not node.sensor_guard.frame(json.dumps(dict(schema_version=1,frame_id='map',
            status=payload(stamp=clock.ns,seq=2,frame=2),points=[])),clock.ns)
        node.sensor_watchdog()
        assert node.sensor_reason(clock.ns)=='PLANNING_LOCALIZATION_STALE'
        assert node.latest_pose_stamp_ns==stamp-10_000_000 and node._pose_inbox_frames==1
        pose.header.stamp=Time(nanoseconds=clock.ns-10_000_000).to_msg();publisher.publish(pose)
        assert publisher.wait_for_all_acked(Duration(seconds=2.))
        node.sensor_watchdog()
        assert node.sensor_reason(clock.ns)=='EGO_STATUS_STALE'
    finally:
        if producer is not None:producer.destroy_node()
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_watchdog_admits_dds_queued_frame_before_stale_check_and_still_stops_on_real_gap(route_editor_map):
    prefix='/offline_atomic_inbox_'+uuid.uuid4().hex[:8]
    args=['--ros-args','-r','__ns:='+prefix,'-p',f'mgeo_dir:={route_editor_map}',
          '-p','goal_node_id:=D','-p','s4_sensor_validation_enabled:=true',
          '-p','s4_atomic_sensor_frames:=true','-p','enable_drive:=false',
          '-r','/perception/lidar_frame:='+prefix+'/frame',
          '-r','/localization/odometry:='+prefix+'/odometry',
          '-r','/planning/s4_sensor_state:='+prefix+'/sensor_state']
    for topic in PLANNING_TOPICS:args+=['-r',f'/planning/{topic}:={prefix}/{topic}']
    rclpy.init(args=args);node=producer=None
    try:
        with patch.object(PlanningNode,'create_timer'):node=PlanningNode()
        for name in list(node.__dict__):
            if name.endswith('_pub'):setattr(node,name,Mock())
        stamp=node.get_clock().now().nanoseconds;clock=SimpleNamespace(ns=stamp)
        node.get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=clock.ns))
        node.latest_pose=(0.,0.,0.);node.latest_pose_stamp_ns=stamp;node._s4_pose_valid=True
        def envelope(source,seq):
            return String(data=json.dumps(dict(schema_version=1,frame_id='map',
                status=payload(stamp=source,seq=seq,frame=seq),points=[])))
        assert not node.sensor_guard.frame(envelope(stamp,1).data,stamp)
        clock.ns=stamp+650_000_000;node.latest_pose_stamp_ns=clock.ns
        assert node.sensor_reason(clock.ns)  # Held Python state has expired.
        producer=Node('atomic_inbox_producer')
        publisher=producer.create_publisher(String,prefix+'/frame',1)
        deadline=time.monotonic()+3.
        while publisher.get_subscription_count()!=1 and time.monotonic()<deadline:
            rclpy.spin_once(producer,timeout_sec=.01)
        assert publisher.get_subscription_count()==1
        publisher.publish(envelope(clock.ns-10_000_000,2))
        assert publisher.wait_for_all_acked(Duration(seconds=2.))
        # The planner executor has NEVER spun. Only DDS has received the new
        # sample, reproducing a timer that runs before the subscription task.
        node.sensor_watchdog()
        assert node._sensor_inbox_frames==1
        assert not node.sensor_reason(clock.ns) and node._sensor_pause is None
        assert node.sensor_guard.matched[0]['source_stamp_ns']==clock.ns-10_000_000
        assert not node.target_speed_pub.publish.called
        # No queued frame is NOT an empty road and does not renew any clock.
        clock.ns+=700_000_000;node.latest_pose_stamp_ns=clock.ns
        node.sensor_watchdog()
        assert node._sensor_pause is not None
        assert node.target_speed_pub.publish.call_args.args[0].data==0.
        assert not node.local_path_pub.publish.call_args.args[0].poses
        heartbeat=payload(stamp=stamp+640_000_000,seq=3,frame=2,published=clock.ns)
        publisher.publish(String(data=json.dumps(dict(schema_version=1,frame_id='map',status=heartbeat,points=[]))))
        assert publisher.wait_for_all_acked(Duration(seconds=2.))
        node.sensor_watchdog()
        assert node._sensor_inbox_frames==2 and node.sensor_reason(clock.ns)
        assert node.sensor_guard.matched[0]['source_stamp_ns']==stamp+640_000_000
        assert node.target_speed_pub.publish.call_args.args[0].data==0.
    finally:
        if producer is not None:producer.destroy_node()
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_legacy_mode_atomic_transport_pairs_and_retains_source_deadline(route_editor_map):
    prefix='/offline_atomic_transport'
    args=['--ros-args','-r','__ns:='+prefix,'-p',f'mgeo_dir:={route_editor_map}',
          '-p','goal_node_id:=D','-p','s4_sensor_validation_enabled:=true',
          '-p','s4_atomic_sensor_frames:=true','-p','enable_drive:=false',
          '-r','/perception/lidar_frame:='+prefix+'/frame',
          '-r','/localization/odometry:='+prefix+'/odometry',
          '-r','/planning/s4_sensor_state:='+prefix+'/sensor_state']
    for topic in PLANNING_TOPICS:
        args+=['-r',f'/planning/{topic}:={prefix}/{topic}']
    rclpy.init(args=args)
    node=None
    try:
        with patch.object(PlanningNode,'create_timer'):
            node=PlanningNode()
        assert not node.avoidance_planner.config.continuous_lane_follow
        assert node.obstacle_sub is None
        assert node.sensor_status_sub.topic_name==prefix+'/frame'
        assert node.sensor_status_sub.qos_profile.depth==1
        stamp=node.get_clock().now().nanoseconds
        status=payload(stamp=stamp)
        message=String(data=json.dumps(dict(schema_version=1,frame_id='map',status=status,points=[])))
        # Admission itself has no motion side effects; localization gate is
        # exercised by the existing node/sensor tests.
        assert not node.sensor_guard.frame(message.data,stamp)
        assert not node.sensor_guard.reason(stamp)
        assert node.sensor_guard.reason(stamp+601_000_000)=='SENSOR_STATUS_TIMEOUT'
        # A recent heartbeat for the same source cannot extend its deadline.
        status.update(status_seq=2,published_ns=stamp+601_000_000)
        message.data=json.dumps(dict(schema_version=1,frame_id='map',status=status,points=[]))
        assert not node.sensor_guard.frame(message.data,stamp+601_000_000)
        assert node.sensor_guard.reason(stamp+601_000_000)=='OBSTACLE_FRAME_TIMEOUT'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
