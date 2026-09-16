import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from morai_ros2_msgs.msg import CtrlCmd, EgoVehicleStatus


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

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 1e-6:
            dt = 1e-3

        self.integral += error * dt
        self.integral = clamp(
            self.integral, -self.integral_limit, self.integral_limit)

        derivative = 0.0
        if self.prev_error is not None:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )


class ControlNode(Node):
    """
    MORAI 26.R1 CtrlCmd baseline for ERP42.

    longl_cmd_type = 1 -> throttle/brake mode.
    front_steer is normalized [-1, 1] for MoraiCmdController in 26.R1.
    rear_steer stays at 0 for ERP42 baseline.
    """

    def __init__(self):
        super().__init__('control_node')

        for name, default in [
            ('target_error_topic', '/planning/target_error'),
            ('target_speed_topic', '/planning/target_speed'),
            ('ego_status_topic', '/ego_vehicle_status'),
            ('ctrl_cmd_topic', '/ctrl_cmd'),
        ]:
            self.declare_parameter(name, default)

        for name, default in [
            ('control_rate_hz', 30.0),
            ('command_timeout_sec', 0.6),
            ('status_timeout_sec', 0.6),
            ('steering_sign', -1.0),
            ('max_front_steer_normalized', 0.70),
            ('steering_kp', 0.80),
            ('steering_ki', 0.00),
            ('steering_kd', 0.12),
            ('steering_integral_limit', 0.50),
            ('speed_kp', 0.14),
            ('speed_ki', 0.015),
            ('speed_kd', 0.02),
            ('speed_integral_limit', 12.0),
            ('max_accel_cmd', 0.60),
            ('max_brake_cmd', 0.80),
        ]:
            self.declare_parameter(name, default)

        self.target_error = 0.0
        self.target_speed_kmh = 0.0
        self.current_speed_kmh = 0.0
        self.command_stamp_ns = 0
        self.status_stamp_ns = 0
        self.last_control_ns = self.get_clock().now().nanoseconds

        self.steer_pid = PID(
            float(self.get_parameter('steering_kp').value),
            float(self.get_parameter('steering_ki').value),
            float(self.get_parameter('steering_kd').value),
            float(self.get_parameter('steering_integral_limit').value),
        )
        self.speed_pid = PID(
            float(self.get_parameter('speed_kp').value),
            float(self.get_parameter('speed_ki').value),
            float(self.get_parameter('speed_kd').value),
            float(self.get_parameter('speed_integral_limit').value),
        )

        self.create_subscription(
            Float32, self.get_parameter('target_error_topic').value,
            self.target_error_cb, 10)
        self.create_subscription(
            Float32, self.get_parameter('target_speed_topic').value,
            self.target_speed_cb, 10)
        self.create_subscription(
            EgoVehicleStatus, self.get_parameter('ego_status_topic').value,
            self.status_cb, 10)

        self.ctrl_pub = self.create_publisher(
            CtrlCmd, self.get_parameter('ctrl_cmd_topic').value, 10)

        rate = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            'Control ready: PID steering + PID throttle/brake, CtrlCmd longl_cmd_type=1')

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def target_error_cb(self, msg: Float32):
        self.target_error = float(msg.data)
        self.command_stamp_ns = self.now_ns()

    def target_speed_cb(self, msg: Float32):
        self.target_speed_kmh = max(0.0, float(msg.data))
        self.command_stamp_ns = self.now_ns()

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

        command_timeout = float(self.get_parameter('command_timeout_sec').value)
        status_timeout = float(self.get_parameter('status_timeout_sec').value)

        command_age = (
            (now - self.command_stamp_ns) / 1e9
            if self.command_stamp_ns else float('inf')
        )
        status_age = (
            (now - self.status_stamp_ns) / 1e9
            if self.status_stamp_ns else float('inf')
        )

        if command_age > command_timeout or status_age > status_timeout:
            self.publish_safe_stop()
            self.steer_pid.reset()
            self.speed_pid.reset()
            return

        # Lateral PID
        steer_raw = self.steer_pid.update(self.target_error, dt)
        steer_sign = float(self.get_parameter('steering_sign').value)
        max_steer = float(self.get_parameter('max_front_steer_normalized').value)
        front_steer = clamp(steer_sign * steer_raw, -max_steer, max_steer)

        # Longitudinal PID in km/h -> normalized pedal command.
        speed_error = self.target_speed_kmh - self.current_speed_kmh
        pedal = self.speed_pid.update(speed_error, dt)

        max_accel = float(self.get_parameter('max_accel_cmd').value)
        max_brake = float(self.get_parameter('max_brake_cmd').value)

        accel = clamp(pedal, 0.0, max_accel)
        brake = clamp(-pedal, 0.0, max_brake)

        # Explicit stop command gets stronger braking near zero target speed.
        if self.target_speed_kmh <= 0.05:
            accel = 0.0
            brake = max(brake, 0.65)

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

    def publish_safe_stop(self):
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
