import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from morai_ros2_msgs.msg import CtrlCmd, EgoVehicleStatus

from edgenode_control.dynamic_lookahead import calculate_lookahead_distance
from edgenode_control.pure_pursuit import PurePursuit
from edgenode_control.steering_command import to_morai_front_steer
from edgenode_control.steering_smoother import SteeringSmoother
from edgenode_control.target_smoothing_gate import TargetSmoothingGate


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class PID:
    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.integral = 0.0
        self.prev_error = None
        self.raw_derivative = 0.0
        self.filtered_derivative = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.raw_derivative = 0.0
        self.filtered_derivative = 0.0

    def update(self, error: float, dt: float,
               derivative_filter_time_constant_sec: float = 0.0, *, output_limits=None):
        """Filter only the error derivative; preserve P/I and the safety target.

        A zero time constant retains the legacy calculation exactly. The
        previous filtered derivative is the sole filter state and resets with
        the PID, so no delayed braking command survives a stop.
        """
        tau = derivative_filter_time_constant_sec
        try:
            valid = all(not isinstance(v, bool) and isinstance(v, (int, float))
                        and math.isfinite(v) for v in (error, dt, tau))
        except OverflowError:
            valid = False
        if not valid or not 0 <= tau <= 2.0:
            return None
        if output_limits is not None:
            if (not isinstance(output_limits,(tuple,list)) or len(output_limits)!=2
                    or not all(isinstance(v,(int,float)) and not isinstance(v,bool)
                               and math.isfinite(v) for v in output_limits)
                    or not output_limits[0]<0<output_limits[1]):
                return None
        if dt <= 1e-6:
            dt = 1e-3

        next_integral = clamp(self.integral+error*dt,-self.integral_limit,self.integral_limit)

        derivative = 0.0
        if self.prev_error is not None:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error
        self.raw_derivative = derivative
        self.filtered_derivative = (derivative if tau == 0.0 else
            self.filtered_derivative + dt / (tau + dt) * (derivative-self.filtered_derivative))
        if output_limits is not None:
            effort=self.kp*error+self.ki*next_integral+self.kd*self.filtered_derivative
            pushing=(effort>output_limits[1] and error>0 or effort<output_limits[0] and error<0)
            if pushing:
                # Freeze accumulation into saturation, but let opposite-sign
                # history unwind to zero rather than retain a throttle bias.
                if self.integral*error<0:
                    next_integral=math.copysign(max(0.,abs(self.integral)-abs(error*dt)),self.integral)
                else:
                    next_integral=self.integral
        self.integral=next_integral

        return (
            self.kp * error +
            self.ki * self.integral +
            self.kd * self.filtered_derivative
        )


class ControlNode(Node):
    """
    Pure Pursuit steering and PID throttle/brake for MORAI 2020 Hyundai Sonata.

    longl_cmd_type = 1 -> throttle/brake mode.
    front_steer is normalized [-1, 1] for MoraiCmdController in 26.R1.
    rear_steer stays at 0 for the Sonata baseline.
    """

    def __init__(self):
        super().__init__('control_node')

        for name, default in [
            ('target_error_topic', '/planning/target_error'),
            ('target_speed_topic', '/planning/target_speed'),
            ('local_path_topic', '/planning/local_path'),
            ('localization_topic', '/localization/odometry'),
            ('ego_status_topic', '/ego_vehicle_status'),
            ('ctrl_cmd_topic', '/ctrl_cmd'),
        ]:
            self.declare_parameter(name, default)

        for name, default in [
            ('control_rate_hz', 30.0),
            ('command_timeout_sec', 0.6),
            ('status_timeout_sec', 0.6),
            ('local_path_timeout_sec', 0.6),
            ('localization_timeout_sec', 0.6),
            ('wheelbase_m', 2.84),
            ('max_wheel_angle_rad', 0.610865),
            ('lookahead_distance_m', 2.5),
            ('lookahead_max_m', 10.0),
            ('lookahead_preview_time_sec', 0.45),
            ('steering_sign', -1.0),
            ('max_front_steer_normalized', 0.70),
            ('steering_filter_time_constant_sec', 0.0),
            ('steering_max_rate_per_sec', 0.0),
            ('target_smoothing_sigma_m', 0.0),
            ('target_smoothing_max_deviation_m', 0.10),
            ('target_smoothing_reversal_hold_m', 6.0),
            ('speed_kp', 0.14),
            ('speed_ki', 0.015),
            ('speed_kd', 0.02),
            ('speed_derivative_filter_time_constant_sec', 0.0),
            ('speed_pid_anti_windup_enabled', False),
            ('speed_integral_limit', 12.0),
            ('max_accel_cmd', 0.60),
            ('max_brake_cmd', 0.80),
        ]:
            self.declare_parameter(name, default)

        # Legacy configs supply the minimum at startup; an explicit new minimum
        # takes precedence. The legacy parameter no longer fixes the lookahead.
        self.declare_parameter(
            'lookahead_min_m', self.get_parameter('lookahead_distance_m').value)

        self.target_error = 0.0
        self.target_speed_kmh = 0.0
        self.current_speed_kmh = 0.0
        self.local_path_points = []
        self.latest_pose = None
        # None means never received; a receive time of zero is valid in ROS time.
        self.target_speed_stamp_ns = None
        self.local_path_stamp_ns = None
        self.localization_stamp_ns = None
        self.status_stamp_ns = None
        self.last_control_ns = self.get_clock().now().nanoseconds

        self.pure_pursuit = PurePursuit(
            wheelbase_m=self.get_parameter('wheelbase_m').value,
            max_wheel_angle_rad=self.get_parameter('max_wheel_angle_rad').value,
        )
        self.speed_pid = PID(
            float(self.get_parameter('speed_kp').value),
            float(self.get_parameter('speed_ki').value),
            float(self.get_parameter('speed_kd').value),
            float(self.get_parameter('speed_integral_limit').value),
        )
        self.steering_smoother = SteeringSmoother()
        self.target_smoothing_gate = TargetSmoothingGate()

        self.create_subscription(
            Float32, self.get_parameter('target_error_topic').value,
            self.target_error_cb, 10)
        self.create_subscription(
            Float32, self.get_parameter('target_speed_topic').value,
            self.target_speed_cb, 10)
        self.create_subscription(
            EgoVehicleStatus, self.get_parameter('ego_status_topic').value,
            self.status_cb, 10)
        self.create_subscription(
            Path, self.get_parameter('local_path_topic').value,
            self.local_path_cb, 10)
        self.create_subscription(
            Odometry, self.get_parameter('localization_topic').value,
            self.localization_cb, 10)

        self.ctrl_pub = self.create_publisher(
            CtrlCmd, self.get_parameter('ctrl_cmd_topic').value, 10)

        self.path_error_pub = self.create_publisher(
            Float32, '/control/path_error', 10)

        self.target_left_pub = self.create_publisher(
            Float32, '/control/target_left', 10)

        self.target_forward_pub = self.create_publisher(
            Float32, '/control/target_forward', 10)

        self.pure_pursuit_steering_pub = self.create_publisher(
            Float32, '/control/pure_pursuit_steering', 10)

        self.lookahead_distance_pub = self.create_publisher(
            Float32, '/control/lookahead_distance', 10)
        self.target_smoothing_offset_pub = self.create_publisher(
            Float32, '/control/target_smoothing_offset', 10)
        self.speed_pid_d_raw_pub = self.create_publisher(Float32, '/control/speed_pid_d_raw', 10)
        self.speed_pid_d_filtered_pub = self.create_publisher(Float32, '/control/speed_pid_d_filtered', 10)
        self.speed_pid_effort_pub = self.create_publisher(Float32, '/control/speed_pid_effort', 10)
        self.speed_pid_i_pub = self.create_publisher(Float32, '/control/speed_pid_i', 10)

        rate = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            'Control ready: Pure Pursuit steering + PID throttle/brake, '
            'CtrlCmd longl_cmd_type=1')

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def target_error_cb(self, msg: Float32):
        # Compatibility/debug only: this input never refreshes a watchdog.
        self.target_error = float(msg.data)

    def target_speed_cb(self, msg: Float32):
        speed = float(msg.data)
        self.target_speed_kmh = max(0.0, speed) if math.isfinite(speed) else speed
        self.target_speed_stamp_ns = self.now_ns()

    def local_path_cb(self, msg: Path):
        self.local_path_points = [
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in msg.poses
        ]
        self.local_path_stamp_ns = self.now_ns()

    def localization_cb(self, msg: Odometry):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.localization_stamp_ns = self.now_ns()
        if not all(math.isfinite(value) for value in
                   (position.x, position.y, q.x, q.y, q.z, q.w)):
            self.latest_pose = None
            return
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.latest_pose = (position.x, position.y, yaw)

    def status_cb(self, msg: EgoVehicleStatus):
        vx = float(msg.velocity.x)
        vy = float(msg.velocity.y)
        vz = float(msg.velocity.z)
        self.current_speed_kmh = math.sqrt(vx * vx + vy * vy + vz * vz) * 3.6
        self.status_stamp_ns = self.now_ns()

    def control_loop(self):
        now = self.now_ns()
        dt = (now - self.last_control_ns) / 1e9
        self.last_control_ns = now

        watchdogs = (
            (self.target_speed_stamp_ns, 'command_timeout_sec'),
            (self.local_path_stamp_ns, 'local_path_timeout_sec'),
            (self.localization_stamp_ns, 'localization_timeout_sec'),
            (self.status_stamp_ns, 'status_timeout_sec'),
        )
        for stamp, timeout_parameter in watchdogs:
            timeout = float(self.get_parameter(timeout_parameter).value)
            age = (now - stamp) / 1e9 if stamp is not None else math.inf
            if not math.isfinite(timeout) or timeout < 0 or not 0 <= age <= timeout:
                self.publish_safe_stop()
                self.speed_pid.reset()
                return

        if (self.latest_pose is None or len(self.local_path_points) < 2
                or not math.isfinite(self.target_speed_kmh)
                or not math.isfinite(self.current_speed_kmh)):
            self.publish_safe_stop()
            self.speed_pid.reset()
            return

        lookahead_distance_m = calculate_lookahead_distance(
            self.current_speed_kmh,
            lookahead_min_m=self.get_parameter('lookahead_min_m').value,
            lookahead_max_m=self.get_parameter('lookahead_max_m').value,
            lookahead_preview_time_sec=self.get_parameter('lookahead_preview_time_sec').value,
        )
        if lookahead_distance_m is None:
            self.publish_safe_stop()
            self.speed_pid.reset()
            return

        # Pure Pursuit is LEFT-positive; MORAI sign conversion happens here.
        sigma = self.get_parameter('target_smoothing_sigma_m').value
        result = self.pure_pursuit.compute(
            self.local_path_points, *self.latest_pose,
            lookahead_distance_m=lookahead_distance_m,
        )
        if result is None:
            self.publish_safe_stop()
            self.speed_pid.reset()
            return
        if sigma != 0.0:
            allow = self.target_smoothing_gate.update(result['steering_normalized'],
                self.current_speed_kmh/3.6*max(0.,min(dt,.1)),
                self.get_parameter('target_smoothing_reversal_hold_m').value)
            if allow is None:
                self.publish_safe_stop(); self.speed_pid.reset(); return
            if allow:
                result = self.pure_pursuit.compute(self.local_path_points, *self.latest_pose,
                    lookahead_distance_m=lookahead_distance_m,
                    target_smoothing_sigma_m=sigma,
                    target_smoothing_max_deviation_m=self.get_parameter('target_smoothing_max_deviation_m').value)
                if result is None:
                    self.publish_safe_stop(); self.speed_pid.reset(); return
        # Record the arc lookahead passed to Pure Pursuit, not target XY range.
        lookahead_msg = Float32()
        lookahead_msg.data = float(lookahead_distance_m)
        self.lookahead_distance_pub.publish(lookahead_msg)
        self.target_smoothing_offset_pub.publish(Float32(data=float(result['target_smoothing_offset_m'])))

        path_error_msg = Float32()
        path_error_msg.data = float(result['nearest_distance_m'])
        self.path_error_pub.publish(path_error_msg)

        target_left_msg = Float32()
        target_left_msg.data = float(result['target_left_m'])
        self.target_left_pub.publish(target_left_msg)

        target_forward_msg = Float32()
        target_forward_msg.data = float(result['target_forward_m'])
        self.target_forward_pub.publish(target_forward_msg)

        steering_msg = Float32()
        steering_msg.data = float(result['steering_normalized'])
        self.pure_pursuit_steering_pub.publish(steering_msg)
        front_steer = to_morai_front_steer(
            result['steering_normalized'],
            steering_sign=self.get_parameter('steering_sign').value,
            max_front_steer_normalized=self.get_parameter('max_front_steer_normalized').value,
        )
        if front_steer is None:
            self.publish_safe_stop()
            self.speed_pid.reset()
            return

        # Explicit stop clears PID history and overrides path-tracking steering.
        if self.target_speed_kmh <= 0.05:
            self.speed_pid.reset()
            accel = 0.0
            brake = 0.65
            front_steer = 0.0
            pedal = 0.0
            self.steering_smoother.reset()
            self.target_smoothing_gate.reset()
        else:
            front_steer = self.steering_smoother.update(
                front_steer, dt,
                self.get_parameter('steering_filter_time_constant_sec').value,
                self.get_parameter('steering_max_rate_per_sec').value,
                self.get_parameter('max_front_steer_normalized').value,
            )
            if front_steer is None:
                self.publish_safe_stop()
                self.speed_pid.reset()
                return
            # Longitudinal PID in km/h -> normalized pedal command.
            speed_error = self.target_speed_kmh - self.current_speed_kmh
            derivative_tau = self.get_parameter('speed_derivative_filter_time_constant_sec').value
            max_accel = float(self.get_parameter('max_accel_cmd').value)
            max_brake = float(self.get_parameter('max_brake_cmd').value)
            anti_windup = self.get_parameter('speed_pid_anti_windup_enabled').value
            if not isinstance(anti_windup,bool):
                self.publish_safe_stop()
                self.speed_pid.reset()
                return
            if anti_windup:
                pedal=self.speed_pid.update(speed_error,dt,derivative_tau,output_limits=(-max_brake,max_accel))
            else:
                pedal = (self.speed_pid.update(speed_error, dt) if derivative_tau == 0.0 else
                         self.speed_pid.update(speed_error, dt, derivative_tau))
            if pedal is None or not math.isfinite(pedal):
                self.publish_safe_stop()
                self.speed_pid.reset()
                return

            accel = clamp(pedal, 0.0, max_accel)
            brake = clamp(-pedal, 0.0, max_brake)

        cmd = CtrlCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.longl_cmd_type = 1
        cmd.accel = float(accel)
        cmd.brake = float(brake)
        cmd.front_steer = float(front_steer)
        cmd.rear_steer = 0.0
        cmd.velocity = 0.0
        cmd.acceleration = 0.0
        self.ctrl_pub.publish(cmd)
        self.speed_pid_d_raw_pub.publish(Float32(data=float(self.speed_pid.kd*self.speed_pid.raw_derivative)))
        self.speed_pid_d_filtered_pub.publish(Float32(data=float(self.speed_pid.kd*self.speed_pid.filtered_derivative)))
        self.speed_pid_effort_pub.publish(Float32(data=float(pedal)))
        self.speed_pid_i_pub.publish(Float32(data=float(self.speed_pid.ki*self.speed_pid.integral)))

    def publish_safe_stop(self):
        self.steering_smoother.reset()
        self.target_smoothing_gate.reset()
        self.speed_pid_d_raw_pub.publish(Float32(data=0.0))
        self.speed_pid_d_filtered_pub.publish(Float32(data=0.0))
        self.speed_pid_effort_pub.publish(Float32(data=0.0))
        self.speed_pid_i_pub.publish(Float32(data=0.0))
        cmd = CtrlCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.longl_cmd_type = 1
        cmd.accel = 0.0
        cmd.brake = 0.8
        cmd.front_steer = 0.0
        cmd.rear_steer = 0.0
        cmd.velocity = 0.0
        cmd.acceleration = 0.0
        self.ctrl_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_safe_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
