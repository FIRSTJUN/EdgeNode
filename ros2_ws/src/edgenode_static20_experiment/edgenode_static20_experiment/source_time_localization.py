"""Opt-in B localization; preserve original reset, frames and source headers."""
import json,math
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from morai_ros2_msgs.msg import EgoVehicleStatus
from edgenode_localization.localization_node import LocalizationNode
from .source_time_ekf import SourceTimeEKF
from .provenance import implementation_sha256

def source_ns(message):return message.header.stamp.sec*10**9+message.header.stamp.nanosec

class SourceTimeLocalizationNode(LocalizationNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('phase_b_source_history_sec',.8,ParameterDescriptor(read_only=True))
        self.declare_parameter('phase_b_vehicle_velocity_std_mps',.1,ParameterDescriptor(read_only=True))
        self.ekf=SourceTimeEKF(self.ekf,history_sec=self.get_parameter('phase_b_source_history_sec').value,
            source_velocity_std_mps=self.get_parameter('phase_b_vehicle_velocity_std_mps').value,
            max_predict_dt_sec=self.max_predict_dt_sec)
        self._source_gps_ns=None;self._source_imu_ns=None;self._source_velocity_ns=None
        self._pose_source_reason='WAITING_FOR_SOURCE_EPOCHS'
        self.vehicle_velocity_sub=self.create_subscription(EgoVehicleStatus,'/ego_vehicle_status',self.vehicle_velocity_callback,10)
        self.source_time_pub=self.create_publisher(String,'/localization/source_time_diagnostics',1)
        self.create_timer(1.,self.publish_source_time)

    def gps_callback(self,message):
        stamp=source_ns(message)
        if stamp<=0 or self._source_gps_ns is not None and stamp<=self._source_gps_ns:return
        self._source_gps_ns=stamp;self.ekf.begin('gps',stamp)
        try:return super().gps_callback(message)
        finally:self.ekf.end()

    def imu_callback(self,message):
        stamp=source_ns(message)
        if stamp<=0 or self._source_imu_ns is not None and stamp<=self._source_imu_ns:return
        self._source_imu_ns=stamp;self.ekf.begin('imu',stamp,gyro=message.angular_velocity.z)
        try:return super().imu_callback(message)
        finally:self.ekf.end()

    def vehicle_velocity_callback(self,message):
        stamp=source_ns(message);v=math.hypot(message.velocity.x,message.velocity.y)
        if (stamp<=0 or not math.isfinite(v) or not 0<=v<=60 or
            self._source_velocity_ns is not None and stamp<=self._source_velocity_ns):return
        self._source_velocity_ns=stamp;self.ekf.vehicle_velocity(stamp,v)

    def _commit_reset(self,x,y,stamp_ns):
        # The inherited service has already required stationary/disabled Ego
        # and fresh GPS/IMU. Preserve the real GPS epoch for that reset too.
        if self.ekf.context is not None:return super()._commit_reset(x,y,stamp_ns)
        self.ekf.begin('gps',stamp_ns)
        try:return super()._commit_reset(x,y,stamp_ns)
        finally:self.ekf.end()

    def _publish_ekf_odometry(self,stamp):
        ns=stamp.sec*10**9+stamp.nanosec;now=self.get_clock().now().nanoseconds
        # Never label a replayed/future state with a newer receipt time. GPS,
        # IMU and vehicle speed must ALL remain inside the existing .6s limit.
        ages=[None if v is None else (now-v)/1e9 for v in
            (getattr(self,'_valid_gps_stamp_ns',None),self._source_imu_ns,self._source_velocity_ns)]
        if any(v is None or not -.05<=v<=.6 for v in ages):
            self._pose_source_reason='SOURCE_EPOCH_STALE_OR_UNAVAILABLE';return
        if self.ekf.last_output_ns!=ns:
            self._pose_source_reason='FILTER_EPOCH_NOT_CURRENT_IMU';return
        self._pose_source_reason=''
        return super()._publish_ekf_odometry(stamp)

    def publish_source_time(self):
        self.source_time_pub.publish(String(data=json.dumps(dict(mode='B_SOURCE_TIME_GPS_IMU_VEHICLE_SPEED',
            implementation_sha256=implementation_sha256(),state_source_ns=self.ekf.last_output_ns,
            gps_source_ns=self._source_gps_ns,imu_source_ns=self._source_imu_ns,
            vehicle_speed_source_ns=self._source_velocity_ns,reason=self._pose_source_reason,
            history_events=len(self.ekf.events),dropped_before_history=self.ekf.dropped_before_history,
            actor_api_used=False),sort_keys=True)))

def main(args=None):
    rclpy.init(args=args);node=SourceTimeLocalizationNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
