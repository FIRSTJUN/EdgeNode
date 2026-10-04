"""ROS-independent tests for MORAI steering sign and command limits."""

import math
import unittest

from edgenode_control.steering_command import to_morai_front_steer


class SteeringCommandTest(unittest.TestCase):
    def test_left_positive_becomes_negative_morai_command(self):
        self.assertEqual(to_morai_front_steer(0.25), -0.25)

    def test_right_negative_becomes_positive_morai_command(self):
        self.assertEqual(to_morai_front_steer(-0.25), 0.25)

    def test_safe_limit_clamps_both_directions(self):
        self.assertEqual(to_morai_front_steer(1.0), -0.70)
        self.assertEqual(to_morai_front_steer(-1.0), 0.70)

    def test_configured_sign_and_limit_are_applied(self):
        self.assertEqual(to_morai_front_steer(0.5, 1.0, 0.4), 0.4)

    def test_invalid_values_fail_closed(self):
        for value in (None, [], 'bad', True, math.nan, math.inf, -math.inf, 10 ** 400):
            for field in ('steering_normalized', 'steering_sign', 'max_front_steer_normalized'):
                arguments = {'steering_normalized': 0.2, field: value}
                with self.subTest(value=value, field=field):
                    self.assertIsNone(to_morai_front_steer(**arguments))
        for limit in (0.0, -0.7, 1.1):
            self.assertIsNone(to_morai_front_steer(0.2, max_front_steer_normalized=limit))
        self.assertIsNone(to_morai_front_steer(1e308, steering_sign=1e308))
        self.assertIsNone(to_morai_front_steer(10 ** 308, steering_sign=10 ** 308))


if __name__ == '__main__':
    unittest.main()
