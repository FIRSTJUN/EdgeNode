import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

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
            ('steering_rate_limit', 1.5),
            ('steering_filter_sec', .10),
        ]:
            self.declare_parameter(name, default)

        self.target_error = 0.0
        self.target_speed_kmh = 0.0
        self.current_speed_kmh = 0.0
        self.error_stamp_ns = 0
        self.speed_stamp_ns = 0
        self.steering_stamp_ns = 0
        self.target_steering = 0.0
        self.last_steering = 0.0
        self.status_stamp_ns = 0
        self.last_control_ns = time.monotonic_ns()

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
            Float32, '/planning/target_steering', self.target_steering_cb, 10)
        self.create_subscription(
            EgoVehicleStatus, self.get_parameter('ego_status_topic').value,
            self.status_cb, qos_profile_sensor_data)

        self.ctrl_pub = self.create_publisher(
            CtrlCmd, self.get_parameter('ctrl_cmd_topic').value, 10)

        rate = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            'Control ready: map wheel steering + PID throttle/brake, CtrlCmd longl_cmd_type=1')

    def now_ns(self) -> int:
        return time.monotonic_ns()

    def target_error_cb(self, msg: Float32):
        value = float(msg.data)
        self.error_stamp_ns = self.now_ns() if math.isfinite(value) and -1 <= value <= 1 else 0
        self.target_error = value if self.error_stamp_ns else 0.0

    def target_speed_cb(self, msg: Float32):
        value = float(msg.data)
        self.speed_stamp_ns = self.now_ns() if math.isfinite(value) and 0 <= value <= 15 else 0
        if self.speed_stamp_ns and value < self.target_speed_kmh-.1:
            self.speed_pid.reset()
        self.target_speed_kmh = value if self.speed_stamp_ns else 0.0

    def target_steering_cb(self, msg: Float32):
        value = float(msg.data)
        self.steering_stamp_ns = self.now_ns() if math.isfinite(value) and abs(value) <= 1 else 0
        self.target_steering = value if self.steering_stamp_ns else 0.0

    def status_cb(self, msg: EgoVehicleStatus):
        vx = float(msg.velocity.x)
        vy = float(msg.velocity.y)
        vz = float(msg.velocity.z)
        self.current_speed_kmh = math.sqrt(vx * vx + vy * vy + vz * vz) * 3.6
        self.status_stamp_ns = self.now_ns() if math.isfinite(self.current_speed_kmh) else 0

    def control_loop(self):
        now = self.now_ns()
        dt = clamp((now - self.last_control_ns) / 1e9, .001, .1)
        self.last_control_ns = now

        command_timeout = float(self.get_parameter('command_timeout_sec').value)
        status_timeout = float(self.get_parameter('status_timeout_sec').value)

        stamps = (self.error_stamp_ns, self.speed_stamp_ns, self.steering_stamp_ns)
        command_age = max((now-stamp)/1e9 if stamp else float('inf') for stamp in stamps)
        status_age = (
            (now - self.status_stamp_ns) / 1e9
            if self.status_stamp_ns else float('inf')
        )

        if command_age > command_timeout or status_age > status_timeout:
            self.publish_safe_stop()
            self.steer_pid.reset()
            self.speed_pid.reset()
            return

        # Pursuit uses left-positive wheel angles. Live MORAI ERP42 was
        # measured right-positive, so convert at the actuator boundary.
        max_steer = float(self.get_parameter('max_front_steer_normalized').value)
        desired = clamp(float(self.get_parameter('steering_sign').value)*self.target_steering, -max_steer, max_steer)
        tau = max(0.0, float(self.get_parameter('steering_filter_sec').value))
        filtered = self.last_steering + dt/(tau+dt)*(desired-self.last_steering)
        step = max(.01, float(self.get_parameter('steering_rate_limit').value))*dt
        front_steer = clamp(filtered, self.last_steering-step, self.last_steering+step)
        self.last_steering = front_steer

        # Longitudinal PID in km/h -> normalized pedal command.
        speed_error = self.target_speed_kmh - self.current_speed_kmh
        pedal = self.speed_pid.update(speed_error, dt)

        max_accel = float(self.get_parameter('max_accel_cmd').value)
        max_brake = float(self.get_parameter('max_brake_cmd').value)

        # Conditional integration prevents windup during pedal saturation.
        if ((pedal > max_accel and speed_error > 0) or
                (pedal < -max_brake and speed_error < 0)):
            self.speed_pid.integral -= speed_error*dt
            self.speed_pid.integral = clamp(self.speed_pid.integral,
                -self.speed_pid.integral_limit, self.speed_pid.integral_limit)
        accel = clamp(pedal, 0.0, max_accel)
        brake = clamp(-pedal, 0.0, max_brake)

        # Stored integral must not keep accelerating above the requested speed.
        if self.current_speed_kmh > self.target_speed_kmh+.5:
            accel = 0.0
            brake = max(brake, min(max_brake, .08*(self.current_speed_kmh-self.target_speed_kmh)))

        # Explicit stop command gets stronger braking near zero target speed.
        if self.target_speed_kmh <= 0.05:
            accel = 0.0
            brake = max(brake, 0.75)
            self.speed_pid.reset()

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
        self.last_steering = 0.0
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
    # Keep the ROS context alive until the final braking command is published.
    # rclpy's default SIGINT handler shuts it down before the finally block.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None

    def request_stop(signum, frame):
        raise KeyboardInterrupt

    previous_handlers = {sig: signal.signal(sig, request_stop)
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        node = ControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None and rclpy.ok():
                node.publish_safe_stop()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    main()
