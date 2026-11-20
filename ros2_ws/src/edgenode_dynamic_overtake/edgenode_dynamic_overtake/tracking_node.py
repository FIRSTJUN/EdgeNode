"""Explicit D tracker adaptation with honest combined implementation identity."""
import json,math
from edgenode_dynamic_tracking.node import DynamicTrackingNode
from edgenode_dynamic_tracking.provenance import implementation_sha256 as c_sha
from edgenode_dynamic_tracking.vehicle_observation import nearest_native
from .road_kinematics import RoadObstacleTracking
from .view_tracker import ViewConsistentTracker
from .provenance import implementation_sha256,manifest
import rclpy

class TaggedPublisher:
    def __init__(self,publisher,filter_provider,scene_provider):self.publisher=publisher;self.filter_provider=filter_provider;self.scene_provider=scene_provider
    def publish(self,message):
        value=json.loads(message.data);value['base_tracking_files']=value.get('files',{});value['files']=manifest();value.update(base_tracking_implementation_sha256=c_sha(),implementation_sha256=implementation_sha256(),mode='D_ANISOTROPIC_LANE_FOLLOWING_TRACKING',process_assumption=dict(longitudinal_acceleration_std_mps2=1.5,lateral_acceleration_std_mps2=.15,isotropic_when_unaligned=True,visible_face_hypothesis='EGO_VIEW_CONSISTENT_D_PRIOR'))
        for track in value['tracks']:
            track['road_motion_conditioned']=self.filter_provider().process_modes.get(track['id'],False)
            original=self.filter_provider().tracks.get(track['id'])
            track['source_footprints_raw']=original.observation.footprints if original is not None else []
        value['observation_scene']=self.scene_provider()
        message.data=json.dumps(value,allow_nan=False,separators=(',',':'));self.publisher.publish(message)

class RoadTrackingNode(DynamicTrackingNode):
    def __init__(self):
        super().__init__()
        self.tracker=ViewConsistentTracker.from_existing(self.tracker)
        def heading(center):
            row=nearest_native(self.tracker.matcher,center)
            return math.radians(row[1]) if row is not None and row[0]<=2.2 else None
        self.tracker.filter=RoadObstacleTracking(self.get_parameter('measurement_std_m').value,self.get_parameter('acceleration_std_mps2').value,self.get_parameter('max_missed_sec').value,heading_provider=heading)
        self.scene={};original_update=self.tracker.update
        def update(envelope,ego):
            result=original_update(envelope,ego);self.scene=dict(source_ns=envelope['status']['source_stamp_ns'],points=envelope['points'])
            return result
        self.tracker.update=update
        self.pub=TaggedPublisher(self.pub,lambda:self.tracker.filter,lambda:self.scene)

def main(args=None):
    rclpy.init(args=args);node=RoadTrackingNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
