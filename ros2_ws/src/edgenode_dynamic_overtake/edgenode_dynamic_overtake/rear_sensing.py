"""D-only raw rear evidence worker, separate from the planning callback.

Only real XYZ returns, source IMU and interpolated localization are admitted.
Binary transport retains float64 XYZ exactly; no Actor data or control output.
"""
import base64,json,math,time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from edgenode_dynamic_tracking.ego_history import EgoHistory
from edgenode_perception.lidar_obstacle_node import pointcloud_xyz
from edgenode_perception.lidar_obstacles import gravity_ground_heights
from .coverage import RaySupport
from .auxiliary_frame import AuxiliaryAdmission
from .ray_transport import RayFiles,read as read_ray_file,verify_inline
from .provenance import implementation_sha256

SENSORS=dict(main=dict(name='original_main_lidar',offset_x_m=1.19,offset_y_m=0.,minimum_belt_z=-.9,maximum_belt_z=-.4),roof=dict(name='d_roof_hdl64',offset_x_m=0.,offset_y_m=0.,minimum_belt_z=-1.99,maximum_belt_z=-1.49))

def decode_scene(value):
    sensors=[]
    for row in value.get('sensors',[]):
        if 'xyz_f64_le' in row:
            encoded=base64.b64decode(row['xyz_f64_le'],validate=True)
            if 'xyz_file' in row:verify_inline(encoded,row['source_ns'],row['xyz_file'])
            raw=np.frombuffer(encoded,dtype='<f8').reshape((-1,3))
        else:raw=read_ray_file(row['xyz_file'],row['source_ns'])
        sensors.append({k:v for k,v in row.items() if k not in ('xyz_f64_le','xyz_file','source_ns','ray_count')}|dict(ray=RaySupport(raw,row['source_ns']),ray_content_sha256=row.get('xyz_file',{}).get('sha256')))
    return dict(rear_sensors=sensors,auxiliary_ready=value.get('auxiliary_ready',False),auxiliary_source_ns=value.get('auxiliary_source_ns'),auxiliary_groups=value.get('auxiliary_groups',[]),evidence_published_ns=value.get('published_ns'),auxiliary_errors=value.get('errors',0),auxiliary_reason=value.get('auxiliary_reason'),auxiliary_status_state=value.get('auxiliary_status_state'))

class RearSensingNode(Node):
    def __init__(self):
        super().__init__('rear_sensing_node');self.ego=EgoHistory();self.attitudes=deque(maxlen=200);self.pending={};self.frames={};self.auxiliary=AuxiliaryAdmission();self.errors=0
        self.ray_files=RayFiles()
        self.archive_pub=self.create_publisher(String,'/dynamic_overtake/rear_ray_evidence',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pub=self.create_publisher(String,'/dynamic_overtake/rear_scene',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry,'/localization/odometry',self.odom,qos_profile_sensor_data)
        self.create_subscription(Imu,'/Imu',self.imu,QoSProfile(depth=50,reliability=ReliabilityPolicy.BEST_EFFORT))
        for kind,topic in [('main','/lidar/points'),('roof','/lidar_rear/points')]:self.create_subscription(PointCloud2,topic,lambda msg,k=kind:self.cloud(k,msg),QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.aux_sub=self.create_subscription(String,'/perception/rear_lidar_frame',self.rear_frame,1)
        self.create_timer(.02,self.process);self.create_timer(.1,self.publish)
    def odom(self,msg):
        if msg.header.frame_id.lstrip('/')!='map':return
        p=msg.pose.pose.position;q=msg.pose.pose.orientation;ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if self.ego.add(ns,(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))),msg.twist.twist.linear.x):self.pending.clear();self.frames.clear()
    def imu(self,msg):
        q=msg.orientation;norm=math.hypot(q.x,q.y,q.z,q.w);ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if ns<=0 or not math.isfinite(norm) or norm<1e-12 or msg.orientation_covariance[0]==-1:return
        if self.attitudes and ns<=self.attitudes[-1][0]:return
        x,y,z,w=(v/norm for v in (q.x,q.y,q.z,q.w));self.attitudes.append((ns,math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(max(-1.,min(1.,2*(w*y-z*x))))))
    def cloud(self,kind,msg):
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if not -.05<=(self.get_clock().now().nanoseconds-ns)/1e9<=.6:return
        if ns<=self.frames.get(kind,{}).get('source_ns',0):return
        # Keep recent unmatched frames: IMU/odometry can arrive after XYZ.
        self.pending[(kind,ns)]=msg
    def rear_frame(self,msg):
        if self.auxiliary.accept(msg.data,self.get_clock().now().nanoseconds):self.publish(archive=False,drain=False)
    def process(self):
        now=self.get_clock().now().nanoseconds
        for (kind,ns),msg in sorted(list(self.pending.items()),key=lambda row:row[0][1]):
            if not -.05<=(now-ns)/1e9<=.6:del self.pending[(kind,ns)];continue
            pose=self.ego.sample(ns);att=min(self.attitudes,key=lambda row:abs(row[0]-ns)) if self.attitudes else None
            if pose is None or att is None or abs(att[0]-ns)>50_000_000:continue
            try:
                xyz=gravity_ground_heights(pointcloud_xyz(msg),att[1],att[2]);meta=SENSORS[kind]
                # Discard only rays incapable of the below-sensor rear proof.
                xyz=xyz[np.isfinite(xyz).all(axis=1)&(xyz[:,0]<=0.)&(xyz[:,2]<=meta['maximum_belt_z'])]
                raw=np.asarray(xyz,dtype='<f8').tobytes()
                row=dict(meta,source_ns=ns,gravity_attitude_source_ns=att[0],gravity_roll_rad=att[1],gravity_pitch_rad=att[2],ego_source_pose=pose[0],ray_count=len(xyz),xyz_f64_le=base64.b64encode(raw).decode())
                if getattr(self,'ray_files',None) is not None:row['xyz_file']=self.ray_files.write(raw,ns,kind)
                if ns>=self.frames.get(kind,{}).get('source_ns',0):self.frames[kind]=row
            except (ValueError,TypeError,OSError):self.errors+=1
            del self.pending[(kind,ns)]
    def publish(self,archive=True,drain=True):
        # Same mutually exclusive group: consume a queued complete source
        # frame before constructing evidence, retaining its original clock.
        if drain:
            with self.aux_sub.handle:taken=self.aux_sub.handle.take_message(self.aux_sub.msg_type,self.aux_sub.raw)
            if taken is not None:self.auxiliary.accept(taken[0].data,self.get_clock().now().nanoseconds)
        now=self.get_clock().now().nanoseconds;sensors=[v for v in self.frames.values() if -.05<=(now-v['source_ns'])/1e9<=.6]
        auxiliary=self.auxiliary.snapshot(now);ns=auxiliary['source_ns']
        ready=bool(auxiliary['ready'] and any(row['name']=='d_roof_hdl64' for row in sensors));groups=auxiliary['groups']
        value=dict(implementation_sha256=implementation_sha256(),actor_api_used=False,published_ns=now,sensors=sensors,auxiliary_ready=ready,auxiliary_source_ns=ns,auxiliary_reason=auxiliary['reason'],auxiliary_status_state=auxiliary['status_state'],auxiliary_groups=groups,errors=self.errors,pending_frames=len(self.pending))
        small=dict(value,sensors=[{k:v for k,v in row.items() if k!='xyz_f64_le'} for row in sensors])
        self.pub.publish(String(data=json.dumps(small,allow_nan=False,separators=(',',':'))))
        if archive and self.archive_pub.get_subscription_count():self.archive_pub.publish(String(data=json.dumps(value,allow_nan=False,separators=(',',':'))))

def main(args=None):
    rclpy.init(args=args);node=RearSensingNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
