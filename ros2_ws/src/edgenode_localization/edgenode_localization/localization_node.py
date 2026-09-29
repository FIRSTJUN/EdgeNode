"""GPS local 위치와 IMU yaw/gyro를 융합하는 4-state EKF 노드."""

import math

import rclpy
from morai_ros2_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Transformer
from pyproj.exceptions import ProjError
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from edgenode_localization.ekf import EKF


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        for name, default in [
            ('gps_topic', '/gps'),
            ('imu_topic', '/Imu'),
            ('gps_odometry_topic', '/localization/gps_odometry'),
            ('odometry_topic', '/localization/odometry'),
            ('status_topic', '/localization/status'),
            ('utm_epsg', 32652),
            ('gps_frame_id', 'map'),
            ('odom_frame_id', 'map'),
            ('base_frame_id', 'base_link'),
            ('process_noise_x', 0.05),
            ('process_noise_y', 0.05),
            ('process_noise_yaw', 0.02),
            ('process_noise_v', 0.50),
            ('gps_position_std_m', 0.50),
            ('imu_yaw_std_deg', 2.0),
            ('initial_position_std_m', 1.0),
            ('initial_yaw_std_deg', 5.0),
            ('initial_velocity_std_mps', 2.0),
            ('max_predict_dt_sec', 0.2),
            ('fallback_east_offset', 360825.8208815998),
            ('fallback_north_offset', 4065896.298577933),
            ('status_publish_period_sec', 1.0),
            ('qos_depth', 10),
            ('sensor_qos_reliability', 'best_effort'),
        ]:
            self.declare_parameter(name, default)

        period = self.get_parameter('status_publish_period_sec').value
        depth = self.get_parameter('qos_depth').value
        reliability = self.get_parameter('sensor_qos_reliability').value
        reliability_options = {
            'best_effort': ReliabilityPolicy.BEST_EFFORT,
            'reliable': ReliabilityPolicy.RELIABLE,
        }
        if not math.isfinite(period) or period <= 0.0 or depth <= 0:
            raise ValueError('status_publish_period_sec와 qos_depth는 양수여야 합니다.')
        if reliability not in reliability_options:
            raise ValueError('sensor_qos_reliability는 best_effort 또는 reliable이어야 합니다.')

        self.fallback_east_offset = self.get_parameter('fallback_east_offset').value
        self.fallback_north_offset = self.get_parameter('fallback_north_offset').value
        if not all(math.isfinite(value) for value in (
            self.fallback_east_offset, self.fallback_north_offset,
        )):
            raise ValueError('fallback offset은 유한한 값이어야 합니다.')
        self.gps_frame_id = self.get_parameter('gps_frame_id').value
        self.gps_transformer = Transformer.from_crs(
            'EPSG:4326',
            f"EPSG:{self.get_parameter('utm_epsg').value}",
            always_xy=True,
        )

        self.max_predict_dt_sec = self.get_parameter('max_predict_dt_sec').value
        if not math.isfinite(self.max_predict_dt_sec) or self.max_predict_dt_sec <= 0.0:
            raise ValueError('max_predict_dt_sec는 유한한 양수여야 합니다.')
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.ekf = EKF(
            process_noise=tuple(self.get_parameter(name).value for name in (
                'process_noise_x', 'process_noise_y', 'process_noise_yaw', 'process_noise_v',
            )),
            gps_position_std_m=self.get_parameter('gps_position_std_m').value,
            imu_yaw_std_rad=math.radians(self.get_parameter('imu_yaw_std_deg').value),
            initial_position_std_m=self.get_parameter('initial_position_std_m').value,
            initial_yaw_std_rad=math.radians(self.get_parameter('initial_yaw_std_deg').value),
            initial_velocity_std_mps=self.get_parameter('initial_velocity_std_mps').value,
        )
        self.latest_gps_local = None
        self.latest_imu_yaw = None
        self.latest_gyro_z = None
        self._last_imu_stamp_ns = None
        self._last_imu_uses_header_stamp = None
        self.latest_gps = None
        self.latest_imu = None
        self.status = 'WAIT_FOR_SENSORS'

        sensor_qos = QoSProfile(
            depth=depth,
            reliability=reliability_options[reliability],
        )
        self.gps_subscription = self.create_subscription(
            GPSMessage, self.get_parameter('gps_topic').value,
            self.gps_callback, sensor_qos,
        )
        self.imu_subscription = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            self.imu_callback, sensor_qos,
        )

        self.gps_odometry_publisher = self.create_publisher(
            Odometry, self.get_parameter('gps_odometry_topic').value, depth,
        )
        self.odometry_publisher = self.create_publisher(
            Odometry, self.get_parameter('odometry_topic').value, depth,
        )
        self.status_publisher = self.create_publisher(
            String, self.get_parameter('status_topic').value, depth,
        )
        self.status_timer = self.create_timer(period, self.publish_status)
        self.get_logger().info('WAIT_FOR_SENSORS: GPS/IMU 수신 대기 중')

    def gps_callback(self, msg: GPSMessage):
        if self.latest_gps is None:
            self.get_logger().info('GPS 메시지 최초 수신')
        self.latest_gps = msg

        longitude, latitude = msg.longitude, msg.latitude
        if not (
            math.isfinite(longitude) and -180.0 <= longitude <= 180.0
            and math.isfinite(latitude) and -90.0 <= latitude <= 90.0
        ):
            self.get_logger().warning('유효하지 않은 GPS 경위도: GPS Odometry 발행 생략')
            return

        try:
            utm_easting, utm_northing = self.gps_transformer.transform(
                longitude, latitude, errcheck=True,
            )
        except ProjError as exc:
            self.get_logger().warning(f'GPS 좌표변환 실패: {exc}')
            return
        if not (math.isfinite(utm_easting) and math.isfinite(utm_northing)):
            self.get_logger().warning('유효하지 않은 UTM 좌표: GPS Odometry 발행 생략')
            return

        # C-track 원점은 UTM 52N의 유한한 양수 쌍이다.
        # 0/음수/NaN/Inf가 있으면 두 축 모두 설정값으로 대체한다.
        east_offset, north_offset = msg.east_offset, msg.north_offset
        if not all(math.isfinite(value) and value > 0.0 for value in (
            east_offset, north_offset,
        )):
            east_offset = self.fallback_east_offset
            north_offset = self.fallback_north_offset

        local_x = utm_easting - east_offset
        local_y = utm_northing - north_offset
        odometry = Odometry()
        odometry.header.stamp = msg.header.stamp
        odometry.header.frame_id = self.gps_frame_id
        odometry.pose.pose.position.x = local_x
        odometry.pose.pose.position.y = local_y
        # 고도는 원본 GPS 값만 복사하며 x/y 변환에는 사용하지 않는다.
        odometry.pose.pose.position.z = msg.altitude if math.isfinite(msg.altitude) else 0.0
        # orientation/twist는 미사용 기본값이며 유효한 자세/속도 추정값이 아니다.
        self.gps_odometry_publisher.publish(odometry)

        self.latest_gps_local = (local_x, local_y)
        if self.ekf.initialized:
            self.ekf.update_position(local_x, local_y)
        else:
            self._try_initialize_ekf()

    def imu_callback(self, msg: Imu):
        if self.latest_imu is None:
            self.get_logger().info('IMU 메시지 최초 수신')
        self.latest_imu = msg

        q = msg.orientation
        norm = math.hypot(q.x, q.y, q.z, q.w)
        gyro_z = msg.angular_velocity.z
        if (
            not math.isfinite(norm) or norm < 1e-12 or not math.isfinite(gyro_z)
            or msg.orientation_covariance[0] == -1.0
            or msg.angular_velocity_covariance[0] == -1.0
        ):
            self.get_logger().warning('유효하지 않은 IMU yaw/gyro: EKF 갱신 생략',
                                      throttle_duration_sec=5.0)
            return
        qx, qy, qz, qw = (value / norm for value in (q.x, q.y, q.z, q.w))
        imu_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        self.latest_imu_yaw = imu_yaw
        self.latest_gyro_z = gyro_z

        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        uses_header_stamp = stamp_ns > 0
        if not uses_header_stamp:
            # stamp가 없을 때만 노드 시각을 사용한다 (use_sim_time도 반영).
            stamp = self.get_clock().now().to_msg()
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        dt = None
        if (
            self._last_imu_stamp_ns is not None
            and uses_header_stamp == self._last_imu_uses_header_stamp
        ):
            dt = (stamp_ns - self._last_imu_stamp_ns) / 1e9
        # 중복/과거 stamp로 적분 기준을 되돌려 같은 구간을 중복 적분하지 않는다.
        # 시간 기준 전환 또는 긴 공백 후에는 현재 stamp를 새 기준으로 사용한다.
        if dt is None or dt > 0.0:
            self._last_imu_stamp_ns = stamp_ns
            self._last_imu_uses_header_stamp = uses_header_stamp

        if not self.ekf.initialized:
            if self._try_initialize_ekf():
                self._publish_ekf_odometry(stamp)
            return

        if dt is not None and 0.0 < dt <= self.max_predict_dt_sec:
            self.ekf.predict(gyro_z, dt)
        elif dt is not None:
            self.get_logger().warning(f'IMU dt={dt:.6f}s: EKF predict 생략',
                                      throttle_duration_sec=5.0)
        # 예측을 생략해도 유효한 quaternion yaw 측정은 반영한다.
        self.ekf.update_yaw(imu_yaw)
        self._publish_ekf_odometry(stamp)

    def _try_initialize_ekf(self):
        if self.ekf.initialized:
            return True
        if self.latest_gps_local is None or self.latest_imu_yaw is None:
            return False
        self.ekf.initialize(*self.latest_gps_local, self.latest_imu_yaw, v=0.0)
        self.status = 'EKF_READY'
        self.get_logger().info('EKF_READY: GPS 위치와 IMU yaw로 초기화 (v=0 m/s)')
        return True

    def _publish_ekf_odometry(self, stamp):
        x, y, yaw, velocity = self.ekf.get_state()
        covariance = self.ekf.get_covariance()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.odom_frame_id
        odometry.child_frame_id = self.base_frame_id
        odometry.pose.pose.position.x = float(x)
        odometry.pose.pose.position.y = float(y)
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation.x = 0.0
        odometry.pose.pose.orientation.y = 0.0
        odometry.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odometry.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odometry.twist.twist.linear.x = float(velocity)
        odometry.twist.twist.angular.z = self.latest_gyro_z

        # 미추정 축의 큰 분산은 출력용 자리표시자이며 필터 Q/R에 사용하지 않는다.
        for axis in range(6):
            odometry.pose.covariance[axis * 6 + axis] = 1e6
            odometry.twist.covariance[axis * 6 + axis] = 1e6
        # 상태 x/y/yaw -> ROS pose covariance x/y/rotation-Z. 교차항도 보존한다.
        for state_row, pose_row in enumerate((0, 1, 5)):
            for state_col, pose_col in enumerate((0, 1, 5)):
                odometry.pose.covariance[pose_row * 6 + pose_col] = float(
                    covariance[state_row, state_col]
                )
        odometry.twist.covariance[0] = float(covariance[3, 3])
        gyro_variance = self.latest_imu.angular_velocity_covariance[8]
        if math.isfinite(gyro_variance) and gyro_variance > 0.0:
            odometry.twist.covariance[35] = gyro_variance
        self.odometry_publisher.publish(odometry)

    def publish_status(self):
        self.status = 'EKF_READY' if self.ekf.initialized else 'WAIT_FOR_SENSORS'

        msg = String()
        msg.data = self.status
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LocalizationNode()
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
