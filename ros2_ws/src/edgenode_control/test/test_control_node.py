"""Exercise real Control callbacks and commands with ROS publishers mocked."""

import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from morai_ros2_msgs.msg import CtrlCmd, EgoVehicleStatus
from rclpy.time import Time
from std_msgs.msg import Float32

from edgenode_control.control_node import ControlNode, PID
from edgenode_control.pure_pursuit import PurePursuit


class ControlNodeTest(unittest.TestCase):
    def setUp(self):
        self.now = 10_000_000_000
        self.parameters = {}
        self.publisher = Mock()
        self.clock = Mock()
        self.clock.now.side_effect = lambda: Time(nanoseconds=self.now)

        def declare_parameter(name, value):
            self.parameters[name] = value

        self.subscriptions = Mock()
        self.timer = Mock()
        node_methods = {
            '__init__': Mock(return_value=None),
            'declare_parameter': Mock(side_effect=declare_parameter),
            'get_parameter': Mock(side_effect=lambda name: SimpleNamespace(
                value=self.parameters[name],
            )),
            'get_clock': Mock(return_value=self.clock),
            'create_subscription': self.subscriptions,
            'create_publisher': Mock(return_value=self.publisher),
            'create_timer': self.timer,
            'get_logger': Mock(),
        }
        patcher = patch.multiple('edgenode_control.control_node.Node', **node_methods)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.node = ControlNode()
        self.initial_stamps = self.stamps()
        self.node.last_control_ns = self.now - 100_000_000
        # Spy on the actual algorithm, rather than supplying synthetic steering.
        self.node.pure_pursuit = Mock(wraps=self.node.pure_pursuit)
        self.feed_inputs()

    def stamps(self):
        return tuple(getattr(self.node, name + '_stamp_ns') for name in (
            'target_speed', 'local_path', 'localization', 'status',
        ))

    def set_path(self, points):
        msg = Path()
        # Receive time, not message header time, controls freshness.
        msg.header.stamp.sec = 1
        for x, y, z in points:
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            msg.poses.append(pose)
        self.node.local_path_cb(msg)

    def feed_inputs(self):
        self.node.target_speed_cb(Float32(data=0.0))
        self.set_path([[0, 1, 2], [10, 1, 3]])
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        odometry.header.stamp.sec = 1000
        self.node.localization_cb(odometry)
        self.node.status_cb(EgoVehicleStatus())

    def command(self):
        self.node.control_loop()
        command = self.publisher.publish.call_args.args[0]
        self.assertIsInstance(command, CtrlCmd)
        self.assertEqual(command.longl_cmd_type, 1)
        self.assertEqual(command.rear_steer, 0.0)
        self.assertEqual(command.velocity, 0.0)
        self.assertEqual(command.acceleration, 0.0)
        self.assertEqual(command.header.stamp, Time(nanoseconds=self.now).to_msg())
        return command

    def assert_safe_stop(self):
        self.node.speed_pid.integral = 10.0
        self.node.speed_pid.prev_error = 1.0
        command = self.command()
        self.assertEqual(command.accel, 0.0)
        self.assertEqual(command.brake, 0.8)
        self.assertEqual(command.front_steer, 0.0)
        self.assertEqual(self.node.speed_pid.integral, 0.0)
        self.assertIsNone(self.node.speed_pid.prev_error)

    def test_constructor_creates_new_subscriptions_and_default_parameters(self):
        self.subscriptions.assert_any_call(
            Path, '/planning/local_path', self.node.local_path_cb, 10,
        )
        self.subscriptions.assert_any_call(
            Odometry, '/localization/odometry', self.node.localization_cb, 10,
        )
        self.timer.assert_called_once_with(1.0 / 30.0, self.node.control_loop)
        self.assertEqual(self.initial_stamps, (None, None, None, None))
        self.assertEqual(self.parameters['wheelbase_m'], 1.04)
        self.assertEqual(self.parameters['max_wheel_angle_rad'], 0.49)
        self.assertEqual(self.parameters['lookahead_distance_m'], 2.5)
        self.assertEqual(self.parameters['steering_sign'], -1.0)
        self.assertEqual(self.parameters['max_front_steer_normalized'], 0.70)
        self.assertFalse(hasattr(self.node, 'steer_pid'))

    def test_left_target_produces_negative_morai_steering(self):
        command = self.command()
        # Inspect the actual sign through an independent real computation.
        expected = PurePursuit().compute(self.node.local_path_points, 0, 0, 0)
        self.assertGreater(expected['steering_normalized'], 0)
        self.assertAlmostEqual(command.front_steer, -expected['steering_normalized'])
        self.node.pure_pursuit.compute.assert_called_once_with(
            self.node.local_path_points, 0.0, 0.0, 0.0, lookahead_distance_m=2.5,
        )

    def test_right_target_produces_positive_morai_steering(self):
        self.set_path([[0, -1, 2], [10, -1, 3]])
        command = self.command()
        self.assertGreater(command.front_steer, 0.0)

    def test_front_steer_clamps_to_develop_safe_limit(self):
        for side in (-1, 1):
            with self.subTest(side=side):
                self.set_path([[0, 0, 0], [0.2, side, 0]])
                command = self.command()
                self.assertEqual(command.front_steer, -side * 0.70)
                self.assertEqual(command.accel, 0.0)
                self.assertGreaterEqual(command.brake, 0.65)

    def test_zero_target_speed_keeps_brakes_with_nonzero_steering(self):
        status = EgoVehicleStatus()
        status.velocity.x = 1.0
        self.node.status_cb(status)
        command = self.command()
        self.assertEqual(self.node.target_speed_kmh, 0.0)
        self.assertEqual(command.accel, 0.0)
        self.assertGreaterEqual(command.brake, 0.65)
        self.assertNotEqual(command.front_steer, 0.0)
        self.node.pure_pursuit.compute.assert_called_once()

    def test_stale_local_path_safe_stops(self):
        self.node.local_path_stamp_ns = self.now - 600_000_001
        self.assert_safe_stop()
        self.node.pure_pursuit.compute.assert_not_called()

    def test_stale_localization_safe_stops(self):
        self.node.localization_stamp_ns = self.now - 600_000_001
        self.assert_safe_stop()

    def test_stale_status_safe_stops(self):
        self.node.status_stamp_ns = self.now - 600_000_001
        self.assert_safe_stop()

    def test_stale_target_speed_safe_stops_despite_fresh_target_error(self):
        self.node.target_speed_stamp_ns = self.now - 600_000_001
        before = self.stamps()
        self.node.target_error_cb(Float32(data=100.0))
        self.assertEqual(self.stamps(), before)
        self.assert_safe_stop()

    def test_missing_inputs_safe_stop(self):
        for name in ('target_speed', 'local_path', 'localization', 'status'):
            with self.subTest(name=name):
                self.feed_inputs()
                setattr(self.node, name + '_stamp_ns', None)
                self.assert_safe_stop()

    def test_timeout_boundary_and_receive_time_zero_are_valid(self):
        self.now = 0
        self.feed_inputs()
        self.assertEqual(self.stamps(), (0, 0, 0, 0))
        self.node.last_control_ns = 0
        self.now = 600_000_000
        command = self.command()
        self.assertNotEqual(command.front_steer, 0.0)
        self.now += 1
        self.assert_safe_stop()

    def test_clock_reversal_and_invalid_timeouts_safe_stop(self):
        self.now -= 1
        self.assert_safe_stop()
        self.now += 1
        for timeout in (-1.0, math.nan, math.inf):
            with self.subTest(timeout=timeout):
                self.parameters['local_path_timeout_sec'] = timeout
                self.assert_safe_stop()

    def test_pure_pursuit_none_safe_stops(self):
        # Target is behind the vehicle, so the actual algorithm returns None.
        self.set_path([[0, 0, 0], [-10, 0, 0]])
        self.assert_safe_stop()
        self.node.pure_pursuit.compute.assert_called_once()

    def test_empty_short_and_invalid_path_safe_stop(self):
        for points in ([], [[0, 0, 0]], [[0, 0, 0], [10, 0, math.nan]]):
            with self.subTest(points=points):
                self.set_path(points)
                self.assert_safe_stop()

    def test_target_error_never_affects_steering_or_any_watchdog(self):
        expected = self.command().front_steer
        for error in (-100.0, 100.0, math.nan):
            with self.subTest(error=error):
                before = self.stamps()
                self.node.target_error_cb(Float32(data=error))
                self.assertEqual(self.stamps(), before)
                self.assertEqual(self.command().front_steer, expected)

    def test_callbacks_store_xyz_pose_yaw_and_independent_receive_times(self):
        before = self.stamps()
        self.now += 1
        self.set_path([[1, 2, 3], [4, 5, 6]])
        self.assertEqual(self.node.local_path_points, [[1, 2, 3], [4, 5, 6]])
        self.assertEqual(self.stamps(), (before[0], self.now, before[2], before[3]))
        self.now += 1
        odometry = Odometry()
        odometry.pose.pose.position.x = 12.0
        odometry.pose.pose.position.y = -3.0
        q = odometry.pose.pose.orientation
        # Quaternion for roll=30, pitch=-20, yaw=70 degrees.
        q.x, q.y = 0.30499790703769875, 0.00879977431167487
        q.z, q.w = 0.5824308212572792, 0.7534408929201135
        self.node.localization_cb(odometry)
        self.assertEqual(self.node.latest_pose[:2], (12.0, -3.0))
        self.assertAlmostEqual(math.degrees(self.node.latest_pose[2]), 70.0)
        self.assertEqual(self.node.localization_stamp_ns, self.now)
        self.assertEqual(self.node.target_speed_stamp_ns, before[0])
        self.assertEqual(self.node.status_stamp_ns, before[3])

    def test_nonfinite_pose_speed_and_status_safe_stop(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                self.feed_inputs()
                odometry = Odometry()
                odometry.pose.pose.orientation.w = value
                self.node.localization_cb(odometry)
                self.assertIsNone(self.node.latest_pose)
                self.assert_safe_stop()
                self.feed_inputs()
                self.node.target_speed_cb(Float32(data=value))
                self.assert_safe_stop()
                self.feed_inputs()
                status = EgoVehicleStatus()
                status.velocity.x = value
                self.node.status_cb(status)
                self.assert_safe_stop()

    def test_invalid_steering_conversion_safe_stops(self):
        self.parameters['steering_sign'] = math.nan
        self.assert_safe_stop()

    def test_longitudinal_pid_and_status_speed_calculation_are_unchanged(self):
        status = EgoVehicleStatus()
        status.velocity.x, status.velocity.y, status.velocity.z = 0.3, 0.4, 0.0
        self.node.status_cb(status)
        self.assertEqual(self.node.current_speed_kmh, 1.8)
        # Isolate PID output comparison without asking for positive target speed.
        reference = PID(0.14, 0.015, 0.02, 12.0)
        error = -self.node.current_speed_kmh
        expected_pedal = reference.update(error, 0.1)
        command = self.command()
        self.assertEqual(self.node.speed_pid.integral, reference.integral)
        self.assertEqual(self.node.speed_pid.prev_error, reference.prev_error)
        self.assertEqual(command.accel, 0.0)
        self.assertAlmostEqual(command.brake, max(min(-expected_pedal, 0.8), 0.65))


if __name__ == '__main__':
    unittest.main()
