"""Independent Phase C ROS node; tracking cannot publish vehicle commands."""
import json,math,uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from edgenode_planning.sensor_readiness import SensorReadiness,READY
from .tracker import TrafficTracker
from .ego_history import EgoHistory
from .provenance import implementation_sha256,manifest

class DynamicTrackingNode(Node):
    def __init__(self):
        super().__init__('dynamic_tracking_node')
        for key,value in [('mgeo_dir','/workspace/local_data/kcity_mgeo'),('hdmap_dir','/workspace/local_data/kcity_hdmap'),('measurement_std_m',.3),('acceleration_std_mps2',1.5),('max_missed_sec',1.2),('vehicle_length_prior_m',4.93),('vehicle_width_prior_m',2.12)]:self.declare_parameter(key,value)
        self.tracker=TrafficTracker(self.get_parameter('mgeo_dir').value,measurement_std_m=self.get_parameter('measurement_std_m').value,
            acceleration_std_mps2=self.get_parameter('acceleration_std_mps2').value,max_missed_s=self.get_parameter('max_missed_sec').value,
            vehicle_length_prior_m=self.get_parameter('vehicle_length_prior_m').value,vehicle_width_prior_m=self.get_parameter('vehicle_width_prior_m').value,
            hdmap_dir=self.get_parameter('hdmap_dir').value)
        self.ego=EgoHistory();self.guard=SensorReadiness(allow_transient_history=True);self.session=uuid.uuid4().hex
        self.pending=None;self.last_ego=None;self.last_source=None;self.reason='WAITING_FOR_LIDAR';self.errors=0
        self.pub=self.create_publisher(String,'/dynamic_tracking/tracks',1)
        self.create_subscription(String,'/perception/lidar_frame',self.frame,1)
        self.create_subscription(Odometry,'/localization/odometry',self.odom,qos_profile_sensor_data)
        self.create_service(Trigger,'/dynamic_tracking/reset',self.reset)
        self.create_timer(.1,self.publish)
    def reset(self,request,response):
        self.tracker.reset();self.session=uuid.uuid4().hex;self.pending=None;self.last_source=None;self.last_ego=None
        self.reason='TRACKER_RESET_WAITING_FOR_NEW_SOURCE';response.success=True;response.message='tracker reset; no vehicle control changed';return response
    def odom(self,msg):
        if msg.header.frame_id.lstrip('/')!='map':return
        p=msg.pose.pose;q=p.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if self.ego.add(stamp,(p.position.x,p.position.y,yaw),msg.twist.twist.linear.x):self.reset(None,Trigger.Response())
        self.process()
    def frame(self,msg):
        now=self.get_clock().now().nanoseconds;before=self.guard.session_id
        self.guard.frame(msg.data,now)
        if before is not None and self.guard.session_id!=before:self.reset(None,Trigger.Response())
        try:
            frame=json.loads(msg.data);stamp=frame['status']['source_stamp_ns']
            if frame['status']['state'] in READY and not self.guard.reason(now) and (self.last_source is None or stamp>self.last_source):self.pending=frame
        except (ValueError,KeyError,TypeError):self.reason='INVALID_TRACKING_FRAME';self.errors+=1;return
        self.process()
    def process(self):
        if self.pending is None:return
        frame=self.pending;stamp=frame['status']['source_stamp_ns'];now=self.get_clock().now().nanoseconds
        if not -.05<=(now-stamp)/1e9<=.6:self.pending=None;return
        ego=self.ego.sample(stamp)
        if ego is None:self.reason='SOURCE_EGO_POSE_UNPAIRED';return
        try:self.tracker.update(frame,ego[0])
        except (ValueError,KeyError,TypeError) as error:self.reason='INVALID_TRACKING_OBJECTS:'+str(error);self.errors+=1;self.pending=None;return
        self.pending=None;self.last_source=stamp;self.last_ego=ego;self.reason='';self.publish()
    def publish(self):
        self.process() if self.pending is not None else None
        now=self.get_clock().now().nanoseconds;reason=self.guard.reason(now) or self.reason
        if self.last_source is None:reason=reason or 'TRACKING_SOURCE_UNAVAILABLE'
        elif not -.05<=(now-self.last_source)/1e9<=.6:reason=reason or 'TRACKING_SOURCE_STALE'
        rows=self.tracker.rows(now,self.last_ego) if self.last_ego is not None else []
        if reason:
            for row in rows:row['usable']=False
        payload=dict(schema_version=1,mode='C_LIDAR_TRACKING_ONLY',tracker_session_id=self.session,ready=not reason,reason=reason,
            source_stamp_ns=self.last_source,published_ns=now,source_age_s=None if self.last_source is None else (now-self.last_source)/1e9,
            ego_source_pose=self.last_ego,tracks=rows,nontraffic_components=self.tracker.nontraffic,observation_models=self.tracker.observation_models,
            native_reference_evidence=self.tracker.native_reference_evidence,raw_collision_solids_unchanged=True,implementation_sha256=implementation_sha256(),files=manifest(),errors=self.errors,
            actor_api_used=False,vehicle_commands_published=False)
        self.pub.publish(String(data=json.dumps(payload,allow_nan=False,separators=(',',':'))))

def main(args=None):
    rclpy.init(args=args);node=DynamicTrackingNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
