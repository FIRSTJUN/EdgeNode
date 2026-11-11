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
        self.parameter_overrides = {}
        self.publisher = Mock()
        self.publishers = {'/ctrl_cmd': self.publisher}
        self.publisher_factory = Mock(side_effect=lambda msg_type, topic, depth:
                                      self.publishers.setdefault(topic, Mock()))
        self.clock = Mock()
        self.clock.now.side_effect = lambda: Time(nanoseconds=self.now)

        def declare_parameter(name, value):
            self.parameters[name] = self.parameter_overrides.get(name, value)

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
            'create_publisher': self.publisher_factory,
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
        with patch.object(self.node.speed_pid, 'update', wraps=self.node.speed_pid.update) as update:
            command = self.command()
            update.assert_not_called()
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
        self.assertEqual(self.parameters['wheelbase_m'], 2.84)
        self.assertEqual(self.parameters['max_wheel_angle_rad'], 0.610865)
        self.assertEqual(self.parameters['lookahead_distance_m'], 2.5)
        self.assertEqual(self.parameters['lookahead_min_m'], 2.5)
        self.assertEqual(self.parameters['lookahead_max_m'], 10.0)
        self.assertEqual(self.parameters['lookahead_preview_time_sec'], 0.45)
        self.publisher_factory.assert_any_call(
            Float32, '/control/lookahead_distance', 10,
        )
        self.assertEqual(self.parameters['steering_sign'], -1.0)
        self.assertEqual(self.parameters['max_front_steer_normalized'], 0.70)
        self.assertFalse(hasattr(self.node, 'steer_pid'))

    def test_shaping_never_delays_explicit_or_watchdog_stop(self):
        self.parameters.update(steering_filter_time_constant_sec=0.08,
                               steering_max_rate_per_sec=2.0)
        self.node.target_speed_cb(Float32(data=20.0))
        command = self.command()
        self.assertGreater(abs(command.front_steer), 0.0)
        self.assertLessEqual(abs(command.front_steer), 0.2)
        self.node.target_speed_cb(Float32(data=0.0))
        self.assertEqual(self.command().front_steer, 0.0)
        self.assertEqual(self.node.steering_smoother.command, 0.0)
        self.node.steering_smoother.command = 0.6
        self.node.status_stamp_ns = None
        self.assert_safe_stop()

    def test_spatial_smoothing_retains_raw_target_during_a_reversal(self):
        self.parameters['target_smoothing_sigma_m'] = .5
        self.node.target_speed_cb(Float32(data=20.0))
        self.command()
        self.assertEqual(self.node.pure_pursuit.compute.call_count, 2)
        self.node.pure_pursuit.compute.reset_mock()
        self.set_path([[0, -1, 2], [10, -1, 3]])
        self.command()
        self.assertEqual(self.node.pure_pursuit.compute.call_count, 1)
        self.assertGreater(self.node.target_smoothing_gate.remaining_m, 0.)
        self.assertEqual(self.publishers['/control/target_smoothing_offset'].publish.call_args.args[0].data, 0.)
        self.node.target_speed_cb(Float32(data=0.0))
        self.command()
        self.assertEqual(self.node.target_smoothing_gate.remaining_m, 0.)
        self.node.target_smoothing_gate.remaining_m = 3.
        self.node.status_stamp_ns = None
        self.assert_safe_stop()
        self.assertEqual(self.node.target_smoothing_gate.remaining_m, 0.)
        self.assertEqual(self.node.steering_smoother.command, 0.0)

    def test_speed_derivative_filter_preserves_target_and_resets_on_stop(self):
        self.parameters['speed_derivative_filter_time_constant_sec'] = .1
        self.node.target_speed_cb(Float32(data=20.0))
        self.command()
        self.now += 33_333_333
        self.node.target_speed_cb(Float32(data=19.5))
        self.command()
        self.assertEqual(self.node.target_speed_kmh,19.5)
        self.assertGreater(abs(self.node.speed_pid.raw_derivative),abs(self.node.speed_pid.filtered_derivative))
        self.node.target_speed_cb(Float32(data=0.0))
        self.assertEqual(self.command().brake,.65)
        self.assertEqual(self.node.speed_pid.raw_derivative,0.)
        self.assertEqual(self.node.speed_pid.filtered_derivative,0.)
        self.node.speed_pid.filtered_derivative=10.
        self.node.status_stamp_ns=None
        self.assert_safe_stop()
        self.assertEqual(self.node.speed_pid.filtered_derivative,0.)

    def test_speed_anti_windup_uses_real_actuator_limits_and_keeps_stop_priority(self):
        self.parameters['speed_pid_anti_windup_enabled']=True
        self.parameters['speed_derivative_filter_time_constant_sec']=.1
        self.node.target_speed_cb(Float32(data=60.))
        self.assertEqual(self.command().accel,.6)
        self.assertEqual(self.node.speed_pid.integral,0.)
        self.node.target_speed_cb(Float32(data=0.))
        self.assertEqual(self.command().brake,.65)
        self.assertEqual(self.node.speed_pid.integral,0.)
        self.node.speed_pid.filtered_derivative=10.
        self.node.status_stamp_ns=None
        self.assert_safe_stop()
        self.assertEqual(self.node.speed_pid.filtered_derivative,0.)

    def test_legacy_lookahead_parameter_supplies_minimum_at_startup(self):
        self.parameter_overrides['lookahead_distance_m'] = 3.0
        self.node = ControlNode()
        self.assertEqual(self.parameters['lookahead_min_m'], 3.0)
        self.feed_inputs()
        self.command()
        diagnostic = self.publishers['/control/lookahead_distance'].publish.call_args.args[0]
        self.assertEqual(diagnostic.data, 3.0)

    def test_explicit_minimum_overrides_legacy_lookahead_parameter(self):
        self.parameter_overrides.update(lookahead_distance_m=3.0, lookahead_min_m=2.5)
        self.node = ControlNode()
        self.assertEqual(self.parameters['lookahead_min_m'], 2.5)
        self.feed_inputs()
        self.command()
        diagnostic = self.publishers['/control/lookahead_distance'].publish.call_args.args[0]
        self.assertEqual(diagnostic.data, 2.5)

    def test_measured_speed_selects_and_publishes_actual_lookahead(self):
        # A constant target cannot account for the measured-speed sweep.
        self.node.target_speed_cb(Float32(data=50.0))
        self.set_path([[0, 0.25, 0], [20, 0.25, 0]])
        publisher = self.publishers['/control/lookahead_distance']
        for speed_kmh, expected_m in (
            (0.0, 2.5), (20.0, 2.5), (25.0, 3.125), (30.0, 3.75),
            (40.0, 5.0), (50.0, 6.25), (1000.0, 10.0),
        ):
            with self.subTest(speed_kmh=speed_kmh):
                status = EgoVehicleStatus()
                status.velocity.x = speed_kmh / 3.6
                self.node.status_cb(status)
                self.node.pure_pursuit.compute.reset_mock()
                publisher.reset_mock()
                self.command()
                self.node.pure_pursuit.compute.assert_called_once()
                call = self.node.pure_pursuit.compute.call_args
                self.assertEqual(call.args, (self.node.local_path_points, 0.0, 0.0, 0.0))
                self.assertAlmostEqual(
                    call.kwargs['lookahead_distance_m'], expected_m,
                )
                publisher.publish.assert_called_once()
                diagnostic = publisher.publish.call_args.args[0]
                self.assertIsInstance(diagnostic, Float32)
                self.assertAlmostEqual(diagnostic.data, expected_m)
                self.assertEqual(diagnostic.data, call.kwargs['lookahead_distance_m'])
                target = self.publishers['/control/target_forward'].publish.call_args.args[0]
                self.assertAlmostEqual(target.data, expected_m)

    def test_target_speed_changes_do_not_change_measured_speed_lookahead(self):
        status = EgoVehicleStatus()
        status.velocity.x = 50.0 / 3.6
        self.node.status_cb(status)
        for target_speed in (0.0, 20.0, 25.0, 50.0):
            with self.subTest(target_speed=target_speed):
                self.node.target_speed_cb(Float32(data=target_speed))
                self.command()
                self.assertEqual(
                    self.node.pure_pursuit.compute.call_args.kwargs['lookahead_distance_m'],
                    6.25,
                )

    def test_dynamic_lookahead_preserves_curve_target_formula_and_steering_sign(self):
        status = EgoVehicleStatus()
        status.velocity.x = 50.0 / 3.6
        self.node.status_cb(status)
        self.node.target_speed_cb(Float32(data=50.0))
        for side in (-1, 1):
            with self.subTest(side=side):
                self.set_path([[0, 0, 0], [2, 0, 0], [14, side * 5, 13]])
                command = self.command()
                # The second segment is 13 m in XY; advance 4.25 m along it.
                forward = 2.0 + 4.25 * 12.0 / 13.0
                left = side * 4.25 * 5.0 / 13.0
                curvature = 2.0 * left / (forward ** 2 + left ** 2)
                normalized = math.atan(2.84 * curvature) / 0.610865
                self.assertAlmostEqual(command.front_steer, -normalized)
                self.assertLess(abs(command.front_steer), 0.70)
                for topic, expected in (('target_left', left),
                                        ('target_forward', forward),
                                        ('pure_pursuit_steering', normalized)):
                    actual = self.publishers['/control/' + topic].publish.call_args.args[0].data
                    self.assertAlmostEqual(actual, expected)

    def test_diagnostic_retains_requested_arc_distance_near_path_end(self):
        status = EgoVehicleStatus()
        status.velocity.x = 50.0 / 3.6
        self.node.status_cb(status)
        self.node.target_speed_cb(Float32(data=50.0))
        self.set_path([[0, 0, 0], [1, 0, 0]])
        self.command()
        lookahead = self.publishers['/control/lookahead_distance'].publish.call_args.args[0]
        forward = self.publishers['/control/target_forward'].publish.call_args.args[0]
        self.assertEqual(lookahead.data, 6.25)
        self.assertEqual(forward.data, 1.0)

    def test_invalid_measured_speed_safe_stops_without_lookahead_publication(self):
        for speed_kmh in (-1.0, math.nan, math.inf, -math.inf):
            with self.subTest(speed_kmh=speed_kmh):
                self.node.current_speed_kmh = speed_kmh
                self.assert_safe_stop()
        self.node.pure_pursuit.compute.assert_not_called()
        self.publishers['/control/lookahead_distance'].publish.assert_not_called()

    def test_invalid_lookahead_configuration_uses_existing_safe_stop(self):
        defaults = self.parameters.copy()
        for parameter, value in (
            ('lookahead_min_m', 0.0), ('lookahead_min_m', math.nan),
            ('lookahead_max_m', 2.0), ('lookahead_max_m', math.inf),
            ('lookahead_preview_time_sec', -0.45),
            ('lookahead_preview_time_sec', math.nan),
        ):
            with self.subTest(parameter=parameter, value=value):
                self.parameters.update(defaults)
                self.parameters[parameter] = value
                self.assert_safe_stop()
        self.node.pure_pursuit.compute.assert_not_called()
        self.publishers['/control/lookahead_distance'].publish.assert_not_called()

    def test_explicit_stop_at_high_measured_speed_preserves_command_and_pid_reset(self):
        status = EgoVehicleStatus()
        status.velocity.x = 50.0 / 3.6
        self.node.status_cb(status)
        self.node.speed_pid.integral = 12.0
        self.node.speed_pid.prev_error = 20.0
        with patch.object(self.node.speed_pid, 'update', wraps=self.node.speed_pid.update) as update:
            command = self.command()
            update.assert_not_called()
        self.assertEqual(command.accel, 0.0)
        self.assertEqual(command.brake, 0.65)
        self.assertEqual(command.front_steer, 0.0)
        self.assertEqual(self.node.speed_pid.integral, 0.0)
        self.assertIsNone(self.node.speed_pid.prev_error)
        diagnostic = self.publishers['/control/lookahead_distance'].publish.call_args.args[0]
        self.assertEqual(diagnostic.data, 6.25)

    def test_left_target_produces_negative_morai_steering(self):
        self.node.target_speed_cb(Float32(data=1.0))
        # Keep the sign-conversion case below the Sonata steering limit;
        # saturated commands are checked separately for both turn directions.
        self.set_path([[0, 0.25, 2], [10, 0.25, 3]])
        command = self.command()
        # Inspect the actual sign through an independent real computation.
        expected = PurePursuit().compute(self.node.local_path_points, 0, 0, 0)
        self.assertGreater(expected['steering_normalized'], 0)
        self.assertLess(expected['steering_normalized'], 0.70)
        self.assertAlmostEqual(command.front_steer, -expected['steering_normalized'])
        self.node.pure_pursuit.compute.assert_called_once_with(
            self.node.local_path_points, 0.0, 0.0, 0.0, lookahead_distance_m=2.5,
        )

    def test_right_target_produces_positive_morai_steering(self):
        self.node.target_speed_cb(Float32(data=1.0))
        self.set_path([[0, -1, 2], [10, -1, 3]])
        command = self.command()
        self.assertGreater(command.front_steer, 0.0)

    def test_front_steer_clamps_to_configured_limit_while_driving(self):
        self.node.target_speed_cb(Float32(data=50.0))
        for side in (-1, 1):
            with self.subTest(side=side):
                self.set_path([[0, 0, 0], [0.2, side, 0]])
                command = self.command()
                self.assertEqual(command.front_steer, -side * 0.70)
                self.assertEqual(command.accel, 0.60)
                self.assertEqual(command.brake, 0.0)

    def test_zero_target_speed_clears_steering_while_braking(self):
        status = EgoVehicleStatus()
        status.velocity.x = 1.0
        self.node.status_cb(status)
        command = self.command()
        self.assertEqual(self.node.target_speed_kmh, 0.0)
        self.assertEqual(command.accel, 0.0)
        self.assertEqual(command.brake, 0.65)
        self.assertEqual(command.front_steer, 0.0)
        self.node.pure_pursuit.compute.assert_called_once()

    def test_explicit_stop_overrides_saturated_steering_on_first_cycle(self):
        # Reproduce the short endpoint target while the vehicle is still moving.
        for side in (-1, 1):
            for target in (0.0, 0.05):
                with self.subTest(side=side, target=target):
                    self.feed_inputs()
                    self.set_path([[0, 0, 0], [0.26, side * 0.02, 0]])
                    status = EgoVehicleStatus()
                    status.velocity.x = 20.5 / 3.6
                    self.node.status_cb(status)
                    self.node.target_speed_cb(Float32(data=50.0))
                    moving = self.command()
                    self.assertEqual(moving.front_steer, -side * 0.70)
                    self.assertEqual(moving.accel, 0.60)
                    self.assertEqual(moving.brake, 0.0)

                    self.node.target_speed_cb(Float32(data=target))
                    self.node.speed_pid.integral = 12.0
                    self.node.speed_pid.prev_error = 29.5
                    with patch.object(self.node.speed_pid, 'update',
                                      wraps=self.node.speed_pid.update) as update:
                        stopped = self.command()
                        update.assert_not_called()
                    self.assertEqual(stopped.accel, 0.0)
                    self.assertEqual(stopped.brake, 0.65)
                    self.assertEqual(stopped.front_steer, 0.0)
                    self.assertEqual(self.node.speed_pid.integral, 0.0)
                    self.assertIsNone(self.node.speed_pid.prev_error)

    def test_repeated_explicit_stop_never_updates_pid_and_clears_old_state(self):
        self.node.speed_pid.integral = -12.0
        self.node.speed_pid.prev_error = -3.6
        status = EgoVehicleStatus()
        status.velocity.x = 1.0
        with patch.object(self.node.speed_pid, 'update', wraps=self.node.speed_pid.update) as update:
            for _ in range(100):
                self.now += 100_000_000
                self.feed_inputs()
                self.node.status_cb(status)
                command = self.command()
                self.assertEqual(command.accel, 0.0)
                self.assertEqual(command.brake, 0.65)
                self.assertEqual(command.front_steer, 0.0)
                self.assertEqual(self.node.speed_pid.integral, 0.0)
                self.assertIsNone(self.node.speed_pid.prev_error)
            update.assert_not_called()

    def test_zero_to_one_kmh_restart_matches_fresh_pid_after_long_stop(self):
        status = EgoVehicleStatus()
        status.velocity.x = 1.0
        # Simulate coasting while a stop command persists, then settling to rest.
        for _ in range(100):
            self.now += 100_000_000
            self.feed_inputs()
            self.node.status_cb(status)
            self.command()
        self.now += 100_000_000
        self.feed_inputs()
        stopped = self.command()
        self.assertEqual(stopped.accel, 0.0)
        self.assertGreaterEqual(stopped.brake, 0.65)

        self.now += 500_000_000
        self.feed_inputs()
        self.node.target_speed_cb(Float32(data=1.0))
        with patch.object(self.node.speed_pid, 'update', wraps=self.node.speed_pid.update) as update:
            command = self.command()
            update.assert_called_once_with(1.0, 0.5)
        reference = PID(0.14, 0.015, 0.02, 12.0)
        expected_accel = reference.update(1.0, 0.5)
        self.assertAlmostEqual(command.accel, expected_accel)
        self.assertAlmostEqual(command.accel, 0.1475)
        self.assertGreater(command.accel, 0.14)
        self.assertEqual(command.brake, 0.0)
        self.assertEqual(self.node.speed_pid.integral, reference.integral)
        self.assertEqual(self.node.speed_pid.prev_error, reference.prev_error)

    def test_explicit_stop_threshold_precedes_pid_update(self):
        with patch.object(self.node.speed_pid, 'update', wraps=self.node.speed_pid.update) as update:
            for target in (0.0, 0.05):
                self.node.target_speed_cb(Float32(data=target))
                self.node.speed_pid.integral = -12.0
                self.node.speed_pid.prev_error = -1.0
                command = self.command()
                self.assertEqual(command.accel, 0.0)
                self.assertEqual(command.brake, 0.65)
                self.assertEqual(command.front_steer, 0.0)
                self.assertEqual(self.node.speed_pid.integral, 0.0)
                self.assertIsNone(self.node.speed_pid.prev_error)
            update.assert_not_called()
            self.now += 100_000_000
            self.feed_inputs()
            self.node.target_speed_cb(Float32(data=0.050001))
            command = self.command()
            update.assert_called_once()
            self.assertGreater(command.accel, 0.0)
            self.assertNotEqual(command.front_steer, 0.0)

    def test_stale_local_path_safe_stops(self):
        self.node.local_path_stamp_ns = self.now - 600_000_001
        self.assert_safe_stop()
        self.node.pure_pursuit.compute.assert_not_called()
        self.publishers['/control/lookahead_distance'].publish.assert_not_called()

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
        self.node.target_speed_cb(Float32(data=1.0))
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
        self.publishers['/control/lookahead_distance'].publish.assert_not_called()

    def test_empty_short_and_invalid_path_safe_stop(self):
        for points in ([], [[0, 0, 0]], [[0, 0, 0], [10, 0, math.nan]]):
            with self.subTest(points=points):
                self.set_path(points)
                self.assert_safe_stop()

    def test_target_error_never_affects_steering_or_any_watchdog(self):
        self.node.target_speed_cb(Float32(data=1.0))
        expected = self.command().front_steer
        self.assertNotEqual(expected, 0.0)
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
        # Verify the normal driving branch separately from explicit zero-speed stop.
        self.node.target_speed_cb(Float32(data=1.0))
        reference = PID(0.14, 0.015, 0.02, 12.0)
        error = 1.0 - self.node.current_speed_kmh
        expected_pedal = reference.update(error, 0.1)
        command = self.command()
        self.assertEqual(self.node.speed_pid.integral, reference.integral)
        self.assertEqual(self.node.speed_pid.prev_error, reference.prev_error)
        self.assertEqual(command.accel, 0.0)
        self.assertAlmostEqual(command.brake, min(-expected_pedal, 0.8))


if __name__ == '__main__':
    unittest.main()
