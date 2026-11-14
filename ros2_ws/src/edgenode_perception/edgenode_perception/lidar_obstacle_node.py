"""Publish local MORAI LiDAR footprints through the existing world obstacle interface."""

from dataclasses import fields
from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, Imu
from sensor_msgs_py.point_cloud2 import dtype_from_fields
from std_msgs.msg import Float32, Int32, String

from edgenode_perception.lidar_status import LidarStatus, footprint_digest

from edgenode_perception.lidar_obstacles import (
    LidarObstacleConfig, detect_clusters, lidar_to_vehicle_xy, vehicle_to_world_xy,gravity_ground_heights,
)


def pointcloud_xyz(cloud):
    """Read named scalar XYZ with field offsets, byte order and padded row strides.

    The MORAI header may say /map while these numbers are sensor-local. Reading
    the payload never infers a transform from that misleading frame ID. The
    message buffer is not mutated, including for big-endian inputs.
    """
    xyz_fields = [field for field in cloud.fields if field.name in ('x', 'y', 'z')]
    if (len(xyz_fields) != 3 or {f.name for f in xyz_fields} != {'x', 'y', 'z'}
            or any(f.count != 1 or f.datatype not in (PointField.FLOAT32, PointField.FLOAT64)
                   for f in xyz_fields)):
        raise ValueError('PointCloud2 requires scalar floating-point x, y and z fields')
    if (cloud.point_step <= 0 or cloud.row_step < cloud.width * cloud.point_step
            or len(cloud.data) < cloud.row_step * cloud.height):
        raise ValueError('Invalid PointCloud2 point/row step or truncated data')
    if cloud.width == 0 or cloud.height == 0:
        if cloud.data or cloud.row_step or cloud.width:
            raise ValueError('Inconsistent empty PointCloud2 dimensions')
        return np.empty((0, 3), dtype=float)
    dtype = dtype_from_fields(xyz_fields, point_step=cloud.point_step)
    dtype = dtype.newbyteorder('>' if cloud.is_bigendian else '<')
    records = np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                         strides=(cloud.row_step, cloud.point_step))
    return np.stack([records[name] for name in ('x', 'y', 'z')], axis=-1).reshape(-1, 3)


class LidarObstacleNode(Node):
    """Static-obstacle candidate detection only; never enables or commands driving."""

    def __init__(self):
        super().__init__('lidar_obstacle_node')
        descriptor = ParameterDescriptor(read_only=True)
        defaults = LidarObstacleConfig()
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name), descriptor)
        self.config = LidarObstacleConfig(**{
            field.name: self.get_parameter(field.name).value for field in fields(defaults)
        })
        for name, default in (
            ('lidar_topic', '/lidar/points'),
            ('localization_topic', '/localization/odometry'),
            ('obstacles_topic', '/planning/obstacles'),
            ('obstacle_count_topic', '/perception/lidar_obstacle_count'),
            ('nearest_distance_topic', '/perception/nearest_obstacle_distance'),
            ('cluster_centers_topic', '/perception/lidar_obstacle_centers'),
            ('localization_timeout_sec', 0.6),
            ('sensor_status_topic', '/perception/lidar_status'),
            ('sensor_timeout_sec', .6),
            ('sensor_max_source_age_sec', .6),
            ('sensor_future_tolerance_sec', .05),
            ('sensor_status_period_sec', .1),
            ('gravity_ground_compensation',False),
        ):
            self.declare_parameter(name, default, descriptor)
        self.localization_timeout_sec = self.get_parameter('localization_timeout_sec').value
        if (isinstance(self.localization_timeout_sec, bool)
                or not math.isfinite(self.localization_timeout_sec)
                or self.localization_timeout_sec <= 0.0):
            raise ValueError('localization_timeout_sec must be finite and positive')
        self.latest_pose = None
        self._gravity_ground=self.get_parameter('gravity_ground_compensation').value
        self._attitudes=deque(maxlen=200)
        self._pending_cloud=None
        self.pose_received_ns = None
        self.pose_source_stamp_ns = None
        self.pose_stamp_ordered = False
        self.sensor_status = LidarStatus(
            self.get_parameter('sensor_timeout_sec').value,
            self.get_parameter('sensor_max_source_age_sec').value,
            self.get_parameter('sensor_future_tolerance_sec').value)
        period = self.get_parameter('sensor_status_period_sec').value
        if isinstance(period, bool) or not math.isfinite(period) or not 0 < period <= .2:
            raise ValueError('sensor_status_period_sec must be in (0, .2]')
        self.sensor_status_pub = self.create_publisher(
            String, self.get_parameter('sensor_status_topic').value, 10)
        self.diagnostics_pub=self.create_publisher(String,'/perception/lidar_diagnostics',10)
        self.sensor_status_timer = self.create_timer(period, self.publish_sensor_status,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.obstacles_pub = self.create_publisher(
            PoseArray, self.get_parameter('obstacles_topic').value, 10)
        self.count_pub = self.create_publisher(
            Int32, self.get_parameter('obstacle_count_topic').value, 10)
        self.distance_pub = self.create_publisher(
            Float32, self.get_parameter('nearest_distance_topic').value, 10)
        self.centers_pub = self.create_publisher(
            PoseArray, self.get_parameter('cluster_centers_topic').value, 10)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.lidar_sub = self.create_subscription(
            PointCloud2, self.get_parameter('lidar_topic').value,
            self.lidar_callback, sensor_qos)
        if self._gravity_ground:
            # Keep timestamp-matched attitudes while a cloud is processed.
            # Depth 1 can discard the matching IMU during a >100ms cloud job.
            imu_qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=50,
                reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Imu,'/Imu',self.imu_callback,imu_qos)
            self.create_timer(.02,self.process_pending_cloud,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.localization_sub = self.create_subscription(
            Odometry, self.get_parameter('localization_topic').value,
            self.localization_callback, 10)
        self.get_logger().info(
            'LiDAR obstacle detector ready: sensor-local XYZ despite MORAI /map header; '
            'waiting for LiDAR and map odometry; no drive commands')

    def imu_callback(self,msg):
        q=msg.orientation;stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        norm=math.hypot(q.x,q.y,q.z,q.w)
        if (stamp<=0 or not math.isfinite(norm) or norm<1e-12
                or msg.orientation_covariance[0]==-1 or (self._attitudes and stamp<=self._attitudes[-1][0])):return
        x,y,z,w=(v/norm for v in (q.x,q.y,q.z,q.w))
        roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch=math.asin(max(-1.,min(1.,2*(w*y-z*x))))
        self._attitudes.append((stamp,roll,pitch))
        self.process_pending_cloud()

    def process_pending_cloud(self):
        if self._pending_cloud is None:return
        msg,received=self._pending_cloud
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        paired=bool(self._attitudes) and min(abs(row[0]-stamp) for row in self._attitudes)<=50_000_000
        expired=self.get_clock().now().nanoseconds-received>=50_000_000
        if paired or expired:
            self._pending_cloud=None
            self.lidar_callback(msg,allow_defer=False)

    def localization_callback(self, msg):
        """Read the same ROS quaternion yaw convention as Planning/Control."""
        position, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.pose_stamp_ordered = stamp > 0 and (
            self.pose_source_stamp_ns is None or stamp > self.pose_source_stamp_ns)
        if self.pose_stamp_ordered:
            self.pose_source_stamp_ns = stamp
        values = (position.x, position.y, q.x, q.y, q.z, q.w)
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if (msg.header.frame_id.lstrip('/') != 'map'
                or not all(math.isfinite(value) for value in values)
                or not math.isfinite(norm) or norm < 1e-12):
            self.latest_pose = None
            self.pose_received_ns = None
            self.get_logger().warning(
                'Invalid localization pose or frame; map obstacles unavailable',
                throttle_duration_sec=5.0)
            return
        x, y, z, w = (value / norm for value in (q.x, q.y, q.z, q.w))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self.latest_pose = (position.x, position.y, yaw)
        # Receipt time avoids mixing MORAI header time with the ROS node clock.
        self.pose_received_ns = self.get_clock().now().nanoseconds

    def lidar_callback(self, msg, *, allow_defer=True):
        if self._gravity_ground and allow_defer:
            # KEEP_LAST(1): a newly received cloud supersedes a deferred cloud.
            # Otherwise an old cloud can run after this one when its IMU arrives,
            # manufacturing an OUT_OF_ORDER fault inside our own queue.
            self._pending_cloud=None
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            paired=bool(self._attitudes) and min(abs(row[0]-stamp) for row in self._attitudes)<=50_000_000
            if not paired:
                # Cross-topic arrival order is not a sensor fault. Wait at most
                # 50ms for its attitude without renewing the previous frame age.
                self._pending_cloud=(msg,self.get_clock().now().nanoseconds)
                return
        started=time.perf_counter()
        now = self.get_clock().now()
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.sensor_status.begin(stamp_ns, now.nanoseconds)
        age = ((now.nanoseconds - self.pose_received_ns) / 1e9
               if self.pose_received_ns is not None else math.inf)
        if self.latest_pose is None or not 0.0 <= age <= self.localization_timeout_sec:
            self.get_logger().warning(
                'Missing or stale localization; publishing empty map obstacles',
                throttle_duration_sec=5.0)
            self.publish_obstacles(msg.header.stamp, [])
            self.sensor_status.finish(0, '', now.nanoseconds, 'LOCALIZATION_UNAVAILABLE')
            self.publish_sensor_status()
            return
        failure = 'INVALID_INPUT'
        try:
            xyz = pointcloud_xyz(msg)
            if len(xyz) and not np.isfinite(xyz).all(axis=1).any():
                raise ValueError('cloud contains no finite XYZ returns')
            failure = 'PROCESSING_ERROR'
            attitude=None
            if self._gravity_ground:
                attitude=min(self._attitudes,key=lambda row:abs(row[0]-stamp_ns)) if self._attitudes else None
                if attitude is None or abs(attitude[0]-stamp_ns)>50_000_000:
                    self.publish_obstacles(msg.header.stamp,[])
                    self.sensor_status.finish(0,'',self.get_clock().now().nanoseconds,'IMU_UNAVAILABLE')
                    self.publish_sensor_status();return
                xyz=gravity_ground_heights(xyz,attitude[1],attitude[2])
            cluster_stats={}
            clusters = detect_clusters(xyz, self.config,diagnostics=cluster_stats)
        except Exception as error:
            self.get_logger().warning(
                f'Invalid LiDAR cloud; publishing empty map obstacles: {error}',
                throttle_duration_sec=5.0)
            self.publish_obstacles(msg.header.stamp, [])
            self.sensor_status.finish(0, '', self.get_clock().now().nanoseconds, failure)
            self.publish_sensor_status()
            return
        obstacles = self.publish_obstacles(msg.header.stamp, clusters)
        finite=xyz[np.isfinite(xyz).all(axis=1)]
        self.diagnostics_pub.publish(String(data=json.dumps(dict(
            source_stamp_ns=stamp_ns,raw_point_count=len(xyz),cluster_count=len(clusters),
            max_return_distance_m=float(np.max(np.linalg.norm(finite[:,:2],axis=1))) if len(finite) else 0.,
            max_cluster_distance_m=max((math.hypot(*c.center_xyz[:2]) for c in clusters),default=0.),
            roi_max_x_m=self.config.roi_max_x,
            ground_filter=cluster_stats,
            imu_pair_delta_ms=abs(attitude[0]-stamp_ns)/1e6 if attitude is not None else None,
            processing_ms=(time.perf_counter()-started)*1000.),allow_nan=False)))
        self.sensor_status.finish(len(clusters), footprint_digest('map', stamp_ns,
            [(p.position.x, p.position.y) for p in obstacles.poses]),
            self.get_clock().now().nanoseconds)
        self.publish_sensor_status()

    def publish_sensor_status(self):
        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - self.pose_received_ns) / 1e9 if self.pose_received_ns is not None else math.inf
        valid_pose = self.latest_pose is not None and 0 <= age <= self.localization_timeout_sec
        valid_pose = valid_pose and self.pose_stamp_ordered and (
            -self.sensor_status.future_ns <= now_ns - self.pose_source_stamp_ns
            <= int(self.localization_timeout_sec * 1e9))
        status = self.sensor_status.snapshot(now_ns, localization_valid=valid_pose)
        self.sensor_status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def world_pose(self, lidar_x, lidar_y):
        vx, vy = lidar_to_vehicle_xy(float(lidar_x), float(lidar_y), self.config)
        wx, wy = vehicle_to_world_xy(vx, vy, *self.latest_pose)
        pose = Pose()
        pose.position.x = float(wx)
        pose.position.y = float(wy)
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def publish_obstacles(self, stamp, clusters):
        obstacles = PoseArray()
        obstacles.header.stamp = stamp
        obstacles.header.frame_id = 'map'
        centers = PoseArray()
        centers.header = obstacles.header
        distances = []
        for cluster in clusters:
            for x, y in cluster.footprint_xy():
                obstacles.poses.append(self.world_pose(x, y))
            center = cluster.center_xyz
            centers.poses.append(self.world_pose(center[0], center[1]))
            # Preserve sensor-origin centroid XY range, independently of bounds.
            distances.append(math.hypot(float(center[0]), float(center[1])))
        self.obstacles_pub.publish(obstacles)
        self.centers_pub.publish(centers)
        self.count_pub.publish(Int32(data=len(clusters)))
        self.distance_pub.publish(Float32(data=min(distances) if distances else float('nan')))
        return obstacles


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarObstacleNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
