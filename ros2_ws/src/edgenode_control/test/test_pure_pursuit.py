"""Offline tests for arc-length targets and left-positive Pure Pursuit."""

import math
import unittest

from edgenode_control.pure_pursuit import PurePursuit


class PurePursuitTest(unittest.TestCase):
    def setUp(self):
        self.planner = PurePursuit()
        self.straight = [[0, 0, 0], [5, 0, 0], [10, 0, 0]]

    def compute(self, points=None, x=0, y=0, yaw=0, lookahead=2.5):
        return self.planner.compute(
            self.straight if points is None else points, x, y, yaw, lookahead,
        )

    def test_centered_straight_path_has_zero_steering(self):
        result = self.compute()
        self.assertEqual(result['target_point'], [2.5, 0, 0])
        self.assertAlmostEqual(result['curvature'], 0.0)
        self.assertAlmostEqual(result['steering_angle_rad'], 0.0)
        self.assertAlmostEqual(result['steering_normalized'], 0.0)
        self.assertEqual(self.planner.wheelbase_m, 1.04)
        self.assertEqual(self.planner.max_wheel_angle_rad, 0.49)

    def test_left_curve_has_positive_steering_and_expected_formula(self):
        result = self.compute([[0, 0, 0], [2, 0, 0], [4, 2, 0]])
        dx, dy = 2 + 0.5 / math.sqrt(2), 0.5 / math.sqrt(2)
        curvature = 2 * dy / (dx * dx + dy * dy)
        angle = math.atan(1.04 * curvature)
        self.assertGreater(result['steering_angle_rad'], 0)
        self.assertAlmostEqual(result['curvature'], curvature)
        self.assertAlmostEqual(result['steering_angle_rad'], angle)
        self.assertAlmostEqual(result['steering_normalized'], angle / 0.49)

    def test_right_curve_has_negative_steering(self):
        left = self.compute([[0, 0, 0], [2, 0, 0], [4, 2, 0]])
        right = self.compute([[0, 0, 0], [2, 0, 0], [4, -2, 0]])
        self.assertLess(right['steering_angle_rad'], 0)
        self.assertAlmostEqual(right['steering_angle_rad'], -left['steering_angle_rad'])
        self.assertAlmostEqual(right['steering_normalized'], -left['steering_normalized'])

    def test_midpath_projection_defines_progress_and_next_target(self):
        result = self.compute(x=4.2, y=0.8)
        self.assertAlmostEqual(result['nearest_distance_m'], 0.8)
        self.assertAlmostEqual(result['current_arc_m'], 4.2)
        self.assertAlmostEqual(result['target_arc_m'], 6.7)
        self.assertAlmostEqual(result['target_point'][0], 6.7)
        self.assertAlmostEqual(result['target_forward_m'], 2.5)
        self.assertAlmostEqual(result['target_left_m'], -0.8)
        self.assertEqual(result['nearest_segment_index'], 0)
        self.assertEqual(result['target_segment_index'], 1)

    def test_interpolation_inside_segment_preserves_target_xyz(self):
        result = self.compute([[0, 0, 10], [10, 0, 50]])
        self.assertEqual(result['target_point'], [2.5, 0, 20])
        self.assertEqual(result['target_arc_m'], 2.5)

    def test_arc_length_target_follows_turn_instead_of_euclidean_radius(self):
        result = self.compute([[0, 0, 0], [2, 0, 0], [2, 10, 0]], x=1)
        self.assertEqual(result['current_arc_m'], 1.0)
        self.assertEqual(result['target_arc_m'], 3.5)
        self.assertEqual(result['target_point'], [2, 1.5, 0])
        self.assertAlmostEqual(result['target_distance_m'], math.hypot(1, 1.5))
        self.assertNotAlmostEqual(result['target_distance_m'], 2.5)
        self.assertAlmostEqual(result['curvature'], 3.0 / 3.25)

    def test_short_remaining_path_uses_last_point(self):
        result = self.compute([[0, 0, 1], [5, 0, 10]], x=4)
        self.assertEqual(result['target_point'], [5, 0, 10])
        self.assertEqual(result['current_arc_m'], 4.0)
        self.assertEqual(result['target_arc_m'], 5.0)
        self.assertEqual(result['target_distance_m'], 1.0)

    def test_projection_clamps_to_segment_endpoints(self):
        before = self.compute(x=-1)
        self.assertEqual(before['current_arc_m'], 0)
        self.assertEqual(before['nearest_distance_m'], 1)
        self.assertEqual(before['target_point'], [2.5, 0, 0])
        # Past the path end but facing back toward it, the endpoint is ahead.
        after = self.compute(x=11, yaw=math.pi)
        self.assertEqual(after['current_arc_m'], 10)
        self.assertEqual(after['nearest_distance_m'], 1)
        self.assertEqual(after['target_point'], [10, 0, 0])

    def test_nearest_projection_checks_all_segments(self):
        points = [[0, 0, 0], [10, 0, 0], [10, 10, 0]]
        result = self.compute(points, x=9, y=4, yaw=math.pi / 2)
        self.assertEqual(result['nearest_segment_index'], 1)
        self.assertEqual(result['nearest_distance_m'], 1)
        self.assertEqual(result['current_arc_m'], 14)
        self.assertEqual(result['target_arc_m'], 16.5)
        self.assertEqual(result['target_point'], [10, 6.5, 0])

    def test_vehicle_yaw_rotates_forward_and_left_correctly(self):
        result = self.compute([[0, 0, 0], [0, 2, 0], [-2, 4, 0]], yaw=math.pi / 2)
        self.assertAlmostEqual(result['target_forward_m'], 2 + 0.5 / math.sqrt(2))
        self.assertAlmostEqual(result['target_left_m'], 0.5 / math.sqrt(2))
        self.assertGreater(result['steering_angle_rad'], 0)

    def test_max_wheel_angle_clamps_both_signs(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                result = self.compute([[0, 0, 0], [0.2, direction, 0]])
                self.assertEqual(result['steering_angle_rad'], direction * 0.49)
                self.assertEqual(result['steering_normalized'], direction)

    def test_normalized_steering_stays_in_unit_interval(self):
        for offset in (-100, -2, -0.01, 0, 0.01, 2, 100):
            with self.subTest(offset=offset):
                result = self.compute([[0, offset, 0], [10, offset, 0]])
                self.assertIsNotNone(result)
                self.assertGreaterEqual(result['steering_normalized'], -1.0)
                self.assertLessEqual(result['steering_normalized'], 1.0)

    def test_malformed_empty_and_nonfinite_paths_fail_closed(self):
        invalid = (
            [], (), {}, 'bad', [[0, 0, 0]], [None, None],
            [[0, 0], [1, 0]], [[0, 0, 0, 0], [1, 0, 0]],
            [[0, 0, 0], ['1', 0, 0]], [[0, 0, 0], [True, 0, 0]],
            [[0, 0, 0], [1, 0, math.nan]], [[0, 0, 0], [1, 0, math.inf]],
            [[0, 0, 0], [-math.inf, 0, 0]], [[0, 0, 0], [10 ** 400, 0, 0]],
            [[0, 0, 0], [10, 0, 0], [20, 0, 'bad']],
        )
        self.assertIsNone(self.planner.compute(None, 0, 0, 0))
        for points in invalid:
            with self.subTest(points=points):
                self.assertIsNone(self.compute(points))

    def test_invalid_vehicle_pose_and_lookahead_fail_closed(self):
        for value in (None, [], 'bad', True, math.nan, math.inf, -math.inf, 10 ** 400):
            for field in ('x', 'y', 'yaw', 'lookahead'):
                with self.subTest(value=value, field=field):
                    self.assertIsNone(self.compute(**{field: value}))
        for lookahead in (0, -1):
            self.assertIsNone(self.compute(lookahead=lookahead))

    def test_invalid_vehicle_dimensions_fail_closed_without_constructor_exception(self):
        invalid = (0, -1, None, 'bad', True, [], math.nan, math.inf, -math.inf, 10 ** 400)
        for value in invalid:
            for field in ('wheelbase_m', 'max_wheel_angle_rad'):
                with self.subTest(value=value, field=field):
                    planner = PurePursuit(**{field: value})
                    self.assertIsNone(planner.compute(self.straight, 0, 0, 0))

    def test_duplicate_only_and_near_zero_paths_fail_closed(self):
        for points in (
            [[1, 2, 3], [1, 2, 4], [1, 2, 5]],
            [[0, 0, 0], [1e-12, 0, 0]],
        ):
            with self.subTest(points=points):
                self.assertIsNone(self.compute(points))

    def test_mixed_duplicates_do_not_prevent_forward_progress(self):
        points = [[0, 0, 0], [0, 0, 5], [5, 0, 10], [5, 0, 15]]
        result = self.compute(points)
        self.assertEqual(result['target_point'], [2.5, 0, 7.5])
        self.assertEqual(result['nearest_segment_index'], 1)
        self.assertEqual(result['target_segment_index'], 1)
        result = self.compute(points, x=4)
        self.assertEqual(result['target_point'], points[-1])

    def test_z_does_not_affect_xy_geometry_or_steering_and_input_is_preserved(self):
        flat = [[0, 1, 0], [10, 1, 0]]
        elevated = [[0, 1, -100], [10, 1, 500]]
        original = [point.copy() for point in elevated]
        a, b = self.compute(flat), self.compute(elevated)
        for key in a.keys() - {'target_point'}:
            self.assertEqual(a[key], b[key])
        self.assertEqual(b['target_point'], [2.5, 1, 50])
        b['target_point'][0] = 99
        self.assertEqual(elevated, original)

    def test_target_at_or_below_forward_threshold_fails_closed(self):
        for points, x, yaw in (
            (self.straight, 0, math.pi),
            (self.straight, 10, 0),
            ([[0, 0, 0], [0.1, 1, 0]], 0, 0),
            ([[0, 0, 0], [0.099, 1, 0]], 0, 0),
            ([[0, 0, 0], [0, 10, 0]], 0, 0),
        ):
            with self.subTest(points=points, x=x, yaw=yaw):
                self.assertIsNone(self.compute(points, x=x, yaw=yaw))
        self.assertIsNotNone(self.compute([[0, 0, 0], [0.101, 1, 0]]))

    def test_computed_geometry_overflow_fails_closed(self):
        for points, x, y, lookahead in (
            ([[-1e308, 0, 0], [1e308, 0, 0]], 0, 0, 2.5),
            ([[0, 0, 0], [1e308, 0, 0], [0, 0, 0]], 0, 0, 2.5),
            (self.straight, -1e308, -1e308, 2.5),
            ([[0, 0, 0], [1e200, 0, 0]], 0, 0, 1e200),
        ):
            with self.subTest(points=points, x=x, y=y, lookahead=lookahead):
                self.assertIsNone(self.compute(points, x=x, y=y, lookahead=lookahead))


if __name__ == '__main__':
    unittest.main()
