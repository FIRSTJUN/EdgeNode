"""Offline tests for preview curvature and robust three-level speed planning."""

import math
import unittest

from edgenode_planning.speed_planner import SpeedPlanner


def circular_path(radius, step_rad=0.1, point_count=6):
    return [[radius * math.sin(index * step_rad),
             radius * (1.0 - math.cos(index * step_rad)), index]
            for index in range(point_count)]


class SpeedPlannerTest(unittest.TestCase):
    def setUp(self):
        self.planner = SpeedPlanner()
        self.straight = [[index, 0, index * 10] for index in range(13)]

    def test_straight_path_is_cruise_with_zero_curvature(self):
        result = self.planner.plan(self.straight)
        self.assertEqual(result['speed_mode'], 'CRUISE')
        self.assertEqual(result['target_speed_kmh'], 1.0)
        self.assertAlmostEqual(result['representative_curvature'], 0.0)
        self.assertAlmostEqual(result['max_curvature'], 0.0)

    def test_gentle_curve_has_expected_curvature_and_speed(self):
        result = self.planner.plan(circular_path(20.0))
        self.assertAlmostEqual(result['representative_curvature'], 0.05)
        self.assertAlmostEqual(result['max_curvature'], 0.05)
        self.assertEqual(result['speed_mode'], 'CURVE')
        self.assertEqual(result['target_speed_kmh'], 0.7)

    def test_sharp_curve_has_expected_curvature_and_speed(self):
        result = self.planner.plan(circular_path(5.0))
        self.assertAlmostEqual(result['representative_curvature'], 0.2)
        self.assertEqual(result['speed_mode'], 'SHARP_CURVE')
        self.assertEqual(result['target_speed_kmh'], 0.5)

    def test_only_first_ten_metres_are_evaluated_with_interpolated_end(self):
        points = [[0, 0, 0], [4, 0, 0], [8, 0, 0], [12, 0, 0],
                  [12, 1, 0], [13, 1, 0]]
        result = self.planner.plan(points)
        self.assertEqual(result['evaluated_path_length_m'], 10.0)
        self.assertEqual(result['evaluated_point_count'], 4)
        self.assertEqual(result['max_curvature'], 0.0)
        self.assertEqual(result['speed_mode'], 'CRUISE')
        full_result = SpeedPlanner(preview_distance_m=100.0).plan(points)
        self.assertGreater(full_result['representative_curvature'], 0.10)
        self.assertEqual(full_result['speed_mode'], 'SHARP_CURVE')

    def test_preview_uses_accumulated_distance_through_a_turn(self):
        points = [[0, 0, 0], [3, 0, 0], [3, 4, 0], [6, 4, 0], [6, 8, 0]]
        result = SpeedPlanner(preview_distance_m=8.0).plan(points)
        self.assertEqual(result['evaluated_path_length_m'], 8.0)
        self.assertEqual(result['evaluated_point_count'], 4)
        # The last evaluated point is (4, 4), one metre along the third segment.
        self.assertAlmostEqual(result['max_curvature'], 2.0 / math.sqrt(17.0))

    def test_short_path_uses_whole_available_length(self):
        result = self.planner.plan([[0, 0, 1], [3, 4, 2], [4.8, 6.4, 3]])
        self.assertAlmostEqual(result['evaluated_path_length_m'], 8.0)
        self.assertEqual(result['evaluated_point_count'], 3)
        self.assertEqual(result['speed_mode'], 'CRUISE')

    def test_consecutive_duplicate_xy_points_are_ignored_even_with_different_z(self):
        points = [[0, 0, 1], [0, 0, 2], [3, 0, 3], [3, 0, 4], [6, 0, 5]]
        result = self.planner.plan(points)
        self.assertEqual(result['evaluated_point_count'], 3)
        self.assertEqual(result['evaluated_path_length_m'], 6.0)
        self.assertEqual(result['speed_mode'], 'CRUISE')

    def test_single_point_spike_does_not_make_representative_match_maximum(self):
        points = [[index * 0.25, 0, 0] for index in range(31)]
        points[15][1] = 1.0
        result = self.planner.plan(points)
        self.assertGreater(result['max_curvature'], 0.10)
        self.assertEqual(result['representative_curvature'], 0.0)
        self.assertEqual(result['speed_mode'], 'CRUISE')
        self.assertEqual(result['target_speed_kmh'], 1.0)

    def test_malformed_empty_and_nonfinite_paths_fail_closed(self):
        invalid = (
            None, [], (), {}, 'bad', [[0, 0, 0]], [[0, 0, 0], [1, 0, 0]],
            [None, None, None], [[0, 0], [1, 0], [2, 0]],
            [[0, 0, 0, 0], [1, 0, 0], [2, 0, 0]],
            [[0, 0, 0], [1, 0, 0], ['2', 0, 0]],
            [[0, 0, 0], [1, 0, 0], [True, 0, 0]],
            [[0, 0, 0], [1, 0, 0], [2, 0, math.nan]],
            [[0, 0, 0], [1, 0, 0], [math.inf, 0, 0]],
            [[0, 0, 0], [1, 0, 0], [2, -math.inf, 0]],
            [[0, 0, 0], [1, 0, 0], [2, 0, 10 ** 400]],
            # Validate malformed points even beyond the preview boundary.
            self.straight + [[20, 0, 'bad']],
        )
        for points in invalid:
            with self.subTest(points=points):
                self.assertIsNone(self.planner.plan(points))

    def test_duplicate_only_path_fails_closed(self):
        self.assertIsNone(self.planner.plan([[1, 2, 3], [1, 2, 4], [1, 2, 5]]))

    def test_short_geometry_and_missing_valid_triplets_fail_closed(self):
        for points in (
            [[0, 0, 0], [2e-7, 0, 0], [4e-7, 0, 0]],
            [[0, 0, 0], [1e-10, 0, 0], [1, 0, 0]],
            [[0, 0, 0], [1, 0, 0], [0, 0, 0]],
            # Three input points, but only two points inside the preview.
            [[0, 0, 0], [20, 0, 0], [40, 0, 0]],
        ):
            with self.subTest(points=points):
                self.assertIsNone(self.planner.plan(points))

    def test_invalid_parameter_types_and_nonfinite_values_fail_closed(self):
        fields = ('cruise_speed_kmh', 'curve_speed_kmh', 'sharp_curve_speed_kmh',
                  'preview_distance_m', 'curve_curvature_threshold', 'sharp_curvature_threshold')
        for field in fields:
            for value in (None, [], 'bad', True, math.nan, math.inf, -math.inf, 10 ** 400):
                with self.subTest(field=field, value=value):
                    planner = SpeedPlanner(**{field: value})
                    self.assertIsNone(planner.plan(self.straight))

    def test_negative_speeds_and_nonpositive_preview_fail_closed(self):
        for field in ('cruise_speed_kmh', 'curve_speed_kmh', 'sharp_curve_speed_kmh'):
            with self.subTest(field=field):
                self.assertIsNone(SpeedPlanner(**{field: -0.1}).plan(self.straight))
        for preview in (0.0, -1.0):
            self.assertIsNone(SpeedPlanner(preview_distance_m=preview).plan(self.straight))

    def test_invalid_threshold_order_fails_closed(self):
        for curve, sharp in ((-0.01, 0.1), (0.1, 0.1), (0.2, 0.1), (0, 0), (0.04, -0.1)):
            with self.subTest(curve=curve, sharp=sharp):
                planner = SpeedPlanner(curve_curvature_threshold=curve, sharp_curvature_threshold=sharp)
                self.assertIsNone(planner.plan(self.straight))

    def test_z_is_validated_but_never_used_for_geometry(self):
        flat = [[0, 0, 0], [3, 0, 0], [3, 4, 0]]
        elevated = [[0, 0, -1e100], [3, 0, 1e100], [3, 4, 500]]
        self.assertEqual(self.planner.plan(flat), self.planner.plan(elevated))

    def test_input_is_not_mutated(self):
        points = circular_path(20.0)
        original = [point.copy() for point in points]
        result = self.planner.plan(points)
        self.assertIsNotNone(result)
        self.assertEqual(points, original)
        result['target_speed_kmh'] = 99
        self.assertEqual(points, original)

    def test_threshold_boundaries_are_inclusive_for_slower_modes(self):
        points = [[0, 0, 0], [1, 0, 0], [1, 1, 0]]
        curvature = 2.0 / math.sqrt(2.0)
        planner = SpeedPlanner(curve_curvature_threshold=curvature,
                               sharp_curvature_threshold=2.0 * curvature)
        self.assertEqual(planner.plan(points)['speed_mode'], 'CURVE')
        planner = SpeedPlanner(curve_curvature_threshold=0.5 * curvature,
                               sharp_curvature_threshold=curvature)
        self.assertEqual(planner.plan(points)['speed_mode'], 'SHARP_CURVE')
        planner = SpeedPlanner(curve_curvature_threshold=math.nextafter(curvature, math.inf),
                               sharp_curvature_threshold=2.0 * curvature)
        self.assertEqual(planner.plan(points)['speed_mode'], 'CRUISE')

    def test_custom_speeds_preview_and_thresholds_are_used(self):
        planner = SpeedPlanner(cruise_speed_kmh=0.9, curve_speed_kmh=0.6,
                               sharp_curve_speed_kmh=0.3, preview_distance_m=5.0,
                               curve_curvature_threshold=0.03, sharp_curvature_threshold=0.08)
        for points, speed in ((self.straight, 0.9), (circular_path(20), 0.6),
                              (circular_path(5), 0.3)):
            with self.subTest(speed=speed):
                result = planner.plan(points)
                self.assertEqual(result['target_speed_kmh'], speed)
                self.assertLessEqual(result['evaluated_path_length_m'], 5.0)
                self.assertTrue(math.isfinite(result['target_speed_kmh']))
                self.assertGreaterEqual(result['target_speed_kmh'], 0.0)
        stopped = SpeedPlanner(cruise_speed_kmh=0, curve_speed_kmh=0, sharp_curve_speed_kmh=0)
        self.assertEqual(stopped.plan(self.straight)['target_speed_kmh'], 0.0)

    def test_left_and_right_curves_have_same_magnitude_and_speed(self):
        left = circular_path(20.0)
        right = [[x, -y, z] for x, y, z in left]
        self.assertEqual(self.planner.plan(left), self.planner.plan(right))

    def test_geometry_arithmetic_overflow_fails_closed(self):
        self.assertIsNone(self.planner.plan([[-1e308, 0, 0], [1e308, 0, 0], [1e308, 1, 0]]))
        planner = SpeedPlanner(preview_distance_m=1e201)
        self.assertIsNone(planner.plan([[0, 0, 0], [1e200, 0, 0], [1e200, 1e200, 0]]))


if __name__ == '__main__':
    unittest.main()
