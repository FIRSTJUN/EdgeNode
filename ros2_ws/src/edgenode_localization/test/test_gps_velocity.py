"""Exercise GPS velocity gates with ROS messages and the production callback."""

import math
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from morai_ros2_msgs.msg import GPSMessage
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from edgenode_localization.ekf import EKF
from edgenode_localization.localization_node import LocalizationNode


class TestGPSVelocity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = LocalizationNode()
        self.addCleanup(self.node.destroy_node)
        self.node.gps_odometry_publisher = Mock()
        self.node.gps_speed_publisher = Mock()
        self.node.odometry_publisher = Mock()
        self.node.status_publisher = Mock()
        self.node.gps_transformer = Mock()

    def gps(self, x, y, stamp_ns, **fields):
        msg = GPSMessage(
            longitude=127.0, latitude=37.0, east_offset=1.0, north_offset=1.0,
        )
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        for name, value in fields.items():
            setattr(msg, name, value)
        self.node.gps_transformer.transform.return_value = (x + 1.0, y + 1.0)
        self.node.gps_callback(msg)
        return msg

    def speeds(self):
        return [call.args[0].data for call in self.node.gps_speed_publisher.publish.call_args_list]

    def test_distance_magnitude_header_time_and_callback_order(self):
        self.node.ekf.initialize(0.0, 0.0, math.pi)
        self.gps(0.0, 0.0, 1_000_000_000)
        self.assertEqual(self.speeds(), [])
        events = Mock()
        events.attach_mock(self.node.gps_odometry_publisher.publish, 'gps_odometry')
        events.attach_mock(self.node.gps_speed_publisher.publish, 'gps_speed')
        with patch.object(self.node.ekf, 'update_position', wraps=self.node.ekf.update_position) as pos:
            with patch.object(self.node.ekf, 'update_velocity', wraps=self.node.ekf.update_velocity) as vel:
                events.attach_mock(pos, 'position')
                events.attach_mock(vel, 'velocity')
                with patch.object(self.node, 'get_clock') as clock:
                    msg = self.gps(0.6, 0.8, 1_200_000_000)
                    clock.assert_not_called()
        self.assertEqual([call[0] for call in events.mock_calls], [
            'gps_odometry', 'position', 'gps_speed', 'velocity',
        ])
        self.assertAlmostEqual(self.speeds()[0], 5.0)
        self.assertGreater(self.node.ekf.get_state()[3], 0.0)
        published = self.node.gps_odometry_publisher.publish.call_args.args[0]
        self.assertEqual(published.header.stamp, msg.header.stamp)
        np.testing.assert_allclose(
            [published.pose.pose.position.x, published.pose.pose.position.y], [0.6, 0.8],
        )
        self.assertIsInstance(self.node.gps_speed_publisher.publish.call_args.args[0], Float32)

    def test_dt_and_speed_gate_boundaries(self):
        self.node.ekf.initialize(0.0, 0.0, 0.0)
        cases = [
            (100_000_000, 0.5, True), (500_000_000, 2.5, True),
            (99_999_999, 0.5, False), (500_000_001, 2.5, False),
            (0, 0.0, False), (-100_000_000, 0.5, False),
            (200_000_000, 0.0, True), (200_000_000, 3.0, True),
            (200_000_000, 3.001, False),
        ]
        for dt_ns, distance, accepted in cases:
            with self.subTest(dt_ns=dt_ns, distance=distance):
                self.node._previous_gps = None
                self.node.gps_speed_publisher.reset_mock()
                self.gps(0.0, 0.0, 2_000_000_000)
                with patch.object(self.node.ekf, 'update_velocity') as update:
                    self.gps(distance, 0.0, 2_000_000_000 + dt_ns)
                self.assertEqual(update.call_count, int(accepted))
                self.assertEqual(len(self.speeds()), int(accepted))
                self.assertEqual(self.node._previous_gps[2], 2_000_000_000 + dt_ns)
                if accepted:
                    self.assertAlmostEqual(self.speeds()[0], distance / (dt_ns / 1e9))

    def test_invalid_gps_does_not_replace_previous_valid_sample(self):
        self.node.ekf.initialize(0.0, 0.0, 0.0)
        self.gps(0.0, 0.0, 1_000_000_000)
        previous = self.node._previous_gps
        for fields in ({'longitude': math.nan}, {'latitude': math.inf}, {'latitude': 91.0}):
            self.gps(100.0, 0.0, 1_100_000_000, **fields)
            self.assertEqual(self.node._previous_gps, previous)
        self.gps(math.inf, 0.0, 1_100_000_000)
        self.assertEqual(self.node._previous_gps, previous)
        self.assertEqual(self.node.gps_odometry_publisher.publish.call_count, 1)
        self.gps(1.0, 0.0, 1_200_000_000)
        self.assertEqual(self.speeds(), [5.0])

    def test_nonfinite_derived_speed_is_rejected(self):
        self.node.ekf.initialize(0.0, 0.0, 0.0)
        self.gps(-1e308, 0.0, 1_000_000_000)
        with patch.object(self.node.ekf, 'update_position'), patch.object(
            self.node.ekf, 'update_velocity',
        ) as update:
            self.gps(1e308, 0.0, 1_200_000_000)
        update.assert_not_called()
        self.assertEqual(self.speeds(), [])

    def test_gap_and_outlier_rebase_to_current_valid_gps(self):
        self.gps(0.0, 0.0, 1_000_000_000)
        self.gps(10.0, 0.0, 2_000_000_000)
        self.gps(20.0, 0.0, 2_200_000_000)
        self.assertEqual(self.speeds(), [])
        self.gps(21.0, 0.0, 2_400_000_000)
        self.assertEqual(self.speeds(), [5.0])

    def test_clock_fallback_and_time_source_switch(self):
        with patch.object(self.node, 'get_clock') as get_clock:
            get_clock.return_value.now.side_effect = [
                Time(nanoseconds=1_000_000_000), Time(nanoseconds=1_200_000_000),
                Time(nanoseconds=1_800_000_000), Time(nanoseconds=2_000_000_000),
            ]
            self.gps(0.0, 0.0, 0)
            self.gps(1.0, 0.0, 0)
            self.gps(2.0, 0.0, 1_400_000_000)
            self.gps(3.0, 0.0, 1_600_000_000)
            self.gps(4.0, 0.0, 0)
            self.gps(5.0, 0.0, 0)
        self.assertEqual(self.speeds(), [5.0, 5.0, 5.0])

    def test_uninitialized_filter_only_publishes_debug_speed(self):
        with patch.object(self.node.ekf, 'update_velocity') as update:
            self.gps(0.0, 0.0, 1_000_000_000)
            self.gps(1.0, 0.0, 1_200_000_000)
        update.assert_not_called()
        self.assertFalse(self.node.ekf.initialized)
        self.assertEqual(self.speeds(), [5.0])

    def test_initialization_keeps_zero_velocity_for_either_sensor_order(self):
        for imu_first in (True, False):
            with self.subTest(imu_first=imu_first):
                self.node.ekf = EKF()
                self.node.latest_gps_local = None
                self.node.latest_imu_yaw = None
                self.node._previous_gps = None
                imu = Imu()
                imu.orientation.z = math.sin(0.15)
                imu.orientation.w = math.cos(0.15)
                imu.header.stamp.sec = 1
                if imu_first:
                    self.node.imu_callback(imu)
                self.gps(1.0, 2.0, 1_000_000_000)
                if not imu_first:
                    self.gps(2.0, 2.0, 1_200_000_000)
                    self.node.imu_callback(imu)
                np.testing.assert_allclose(
                    self.node.ekf.get_state(), [1.0 if imu_first else 2.0, 2.0, 0.3, 0.0],
                )

    def test_disabled_velocity_update_matches_v1(self):
        self.node.gps_velocity_update_enabled = False
        reference = EKF()
        reference.initialize(0.0, 0.0, 0.2)
        self.node.ekf.initialize(0.0, 0.0, 0.2)
        with patch.object(self.node.ekf, 'update_velocity') as update:
            for step in range(5):
                for ekf in (reference, self.node.ekf):
                    ekf.predict(0.1, 0.2)
                    ekf.update_yaw(0.2 + step * 0.02)
                reference.update_position(float(step), 0.0)
                self.gps(float(step), 0.0, 1_000_000_000 + step * 200_000_000)
                np.testing.assert_array_equal(self.node.ekf.get_state(), reference.get_state())
                np.testing.assert_array_equal(
                    self.node.ekf.get_covariance(), reference.get_covariance(),
                )
        update.assert_not_called()
        self.assertEqual(self.speeds(), [5.0] * 4)


if __name__ == '__main__':
    unittest.main()
