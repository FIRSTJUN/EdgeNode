"""Offline tests for preview curvature and robust three-level speed planning."""

import math
import unittest

from edgenode_planning.speed_planner import SpeedPlanner


def circular_path(radius, step_rad=0.1, point_count=6):
    return [[radius * math.sin(index * step_rad),
             radius * (1.0 - math.cos(index * step_rad)), index]
            for index in range(point_count)]


def curvature_profile_path(profile):
    """Integrate exact circular/straight pieces: (curvature, length, spacing).

    Physical lengths and curvature are specified independently of vertex
    density, including dense short joints between sparsely sampled roads.
    """
    x, y, heading = 0.0, 0.0, 0.0
    points = [[x, y, 0.0]]
    for curvature, length, spacing in profile:
        count = max(1, math.ceil(length / spacing))
        step = length / count
        for _ in range(count):
            next_heading = heading + curvature * step
            if curvature == 0:
                x += step * math.cos(heading)
                y += step * math.sin(heading)
            else:
                x += (math.sin(next_heading) - math.sin(heading)) / curvature
                y += (math.cos(heading) - math.cos(next_heading)) / curvature
            heading = next_heading
            points.append([x, y, 0.0])
    return points


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
                  'preview_distance_m', 'curve_curvature_threshold', 'sharp_curvature_threshold',
                  'curvature_smoothing_window_m', 'curvature_persistence_m', 'curvature_hysteresis')
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


class StableCurvatureSpeedPlannerTest(unittest.TestCase):
    def setUp(self):
        self.planner = SpeedPlanner(cruise_speed_kmh=10.0, curve_speed_kmh=5.0,
                                    sharp_curve_speed_kmh=3.5)

    def test_nearly_straight_curvature_stays_at_ten_kmh(self):
        result = self.planner.plan(curvature_profile_path([(0.0005, 12, 0.5)]))
        self.assertEqual(result['speed_mode'], 'CRUISE')
        self.assertEqual(result['target_speed_kmh'], 10.0)
        self.assertAlmostEqual(result['representative_curvature'], 0.0005, places=6)

    def test_isolated_spike_keeps_raw_diagnostic_but_not_slow_mode(self):
        points = curvature_profile_path([(0.001, 4, 0.5), (0.125, 0.05, 0.01), (0.001, 6, 0.5)])
        result = self.planner.plan(points)
        self.assertAlmostEqual(result['max_curvature'], 0.125, places=6)
        self.assertLess(result['filtered_max_curvature'], 0.04)
        self.assertEqual(result['speed_mode'], 'CRUISE')
        self.assertEqual(result['target_speed_kmh'], 10.0)

    def test_short_dense_spikes_do_not_dominate_by_point_count(self):
        for curvature in (0.07, 0.10, 0.12):
            for length in (0.2, 0.5):
                with self.subTest(curvature=curvature, length=length):
                    points = curvature_profile_path([
                        (0.001, 4, 0.5), (curvature, length, 0.01), (0.001, 6, 0.5),
                    ])
                    result = self.planner.plan(points)
                    # Reproduce the old sample-count p80 failure, then verify
                    # the distance-based decision rejects the same geometry.
                    self.assertGreater(result['raw_representative_curvature'], 0.04)
                    self.assertLess(result['representative_curvature'], 0.04)
                    self.assertEqual(result['speed_mode'], 'CRUISE')

    def test_short_joint_decision_is_independent_of_vertex_density(self):
        for spacing in (0.01, 0.05, 0.1, 0.2):
            with self.subTest(spacing=spacing):
                points = curvature_profile_path([(0.001, 4, 0.5), (0.12, 0.4, spacing), (0.001, 6, 0.5)])
                result = self.planner.plan(points)
                self.assertEqual(result['speed_mode'], 'CRUISE')
                self.assertEqual(result['target_speed_kmh'], 10.0)

    def test_sustained_variable_curve_is_five_kmh(self):
        profile = [(0.001, 3, 0.5)]
        profile += [(k, 0.5, 0.1) for k in (0.055, 0.061, 0.073, 0.080, 0.076, 0.068)]
        profile += [(0.001, 5, 0.5)]
        result = self.planner.plan(curvature_profile_path(profile))
        self.assertEqual(result['speed_mode'], 'CURVE')
        self.assertEqual(result['target_speed_kmh'], 5.0)
        self.assertGreater(result['representative_curvature'], 0.04)
        self.assertLess(result['representative_curvature'], 0.10)

    def test_sustained_sharp_curve_is_three_point_five_kmh(self):
        result = self.planner.plan(curvature_profile_path([(0.001, 3, 0.5), (0.12, 4, 0.1), (0.001, 4, 0.5)]))
        self.assertEqual(result['speed_mode'], 'SHARP_CURVE')
        self.assertEqual(result['target_speed_kmh'], 3.5)
        self.assertAlmostEqual(result['representative_curvature'], 0.12, places=6)

    def test_sustained_curve_detection_is_independent_of_vertex_density(self):
        for spacing in (0.02, 0.1, 0.5):
            with self.subTest(spacing=spacing):
                result = self.planner.plan(curvature_profile_path([
                    (0.001, 3, 0.5), (0.06, 4, spacing), (0.001, 4, 0.5),
                ]))
                self.assertEqual(result['speed_mode'], 'CURVE')
                self.assertEqual(result['target_speed_kmh'], 5.0)

    def test_disconnected_short_spikes_cannot_accumulate_persistence(self):
        planner = SpeedPlanner(curvature_smoothing_window_m=0.1, curvature_persistence_m=2.0)
        profile = [(0.001, 1, 0.1)]
        for _ in range(4):
            profile += [(0.12, 0.5, 0.05), (0.001, 1, 0.1)]
        result = planner.plan(curvature_profile_path(profile))
        self.assertGreater(result['filtered_max_curvature'], 0.10)
        self.assertLess(result['representative_curvature'], 0.04)
        self.assertEqual(result['speed_mode'], 'CRUISE')

    def test_persistence_distance_is_configurable_and_not_a_sample_count(self):
        points = curvature_profile_path([(0.001, 3, 0.5), (0.12, 1, 0.05), (0.001, 6, 0.5)])
        for distance, mode in ((2.0, 'CRUISE'), (0.5, 'SHARP_CURVE')):
            with self.subTest(distance=distance):
                planner = SpeedPlanner(curvature_smoothing_window_m=0.1, curvature_persistence_m=distance)
                self.assertEqual(planner.plan(points)['speed_mode'], mode)

    def test_curve_hysteresis_prevents_repeated_boundary_switching(self):
        previous = None
        modes = []
        for curvature in (0.0395, 0.0405, 0.0395, 0.0405, 0.0395, 0.034):
            result = self.planner.plan(curvature_profile_path([(curvature, 12, 0.5)]), previous_speed_mode=previous)
            previous = result['speed_mode']
            modes.append(previous)
        self.assertEqual(modes, ['CRUISE', 'CURVE', 'CURVE', 'CURVE', 'CURVE', 'CRUISE'])

    def test_sharp_hysteresis_retains_then_releases_sharp_and_curve_modes(self):
        previous = None
        modes = []
        for curvature in (0.101, 0.099, 0.1005, 0.096, 0.094, 0.039, 0.034):
            result = self.planner.plan(curvature_profile_path([(curvature, 12, 0.5)]), previous_speed_mode=previous)
            previous = result['speed_mode']
            modes.append(previous)
        self.assertEqual(modes, ['SHARP_CURVE'] * 4 + ['CURVE', 'CURVE', 'CRUISE'])

    def test_hysteresis_can_be_disabled_and_planner_has_no_hidden_state(self):
        points = curvature_profile_path([(0.039, 12, 0.5)])
        self.assertEqual(self.planner.plan(points, previous_speed_mode='CURVE')['speed_mode'], 'CURVE')
        self.assertEqual(self.planner.plan(points)['speed_mode'], 'CRUISE')
        no_hysteresis = SpeedPlanner(curvature_hysteresis=0.0)
        self.assertEqual(no_hysteresis.plan(points, previous_speed_mode='CURVE')['speed_mode'], 'CRUISE')

    def test_short_preview_uses_all_remaining_distance_without_division_errors(self):
        for curvature, mode in ((0.0005, 'CRUISE'), (0.06, 'CURVE'), (0.12, 'SHARP_CURVE')):
            with self.subTest(curvature=curvature):
                result = self.planner.plan(curvature_profile_path([(curvature, 0.8, 0.1)]))
                self.assertEqual(result['speed_mode'], mode)
                self.assertLess(result['evaluated_path_length_m'], 2.0)
                self.assertTrue(math.isfinite(result['representative_curvature']))

    def test_duplicate_and_tiny_segments_do_not_corrupt_the_decision(self):
        points = [[0, 0, 0], [0, 0, 1], [1e-8, 1e-8, 0], [2e-8, 0, 0]]
        points += [[i * 0.5, 0, 0] for i in range(1, 25)]
        original = [point.copy() for point in points]
        result = self.planner.plan(points)
        self.assertTrue(math.isfinite(result['max_curvature']))
        self.assertEqual(result['speed_mode'], 'CRUISE')
        self.assertEqual(result['target_speed_kmh'], 10.0)
        self.assertEqual(result['representative_curvature'], 0.0)
        self.assertEqual(points, original)

    def test_signed_smoothing_has_symmetric_left_and_right_decisions(self):
        points = curvature_profile_path([(0.001, 3, 0.5), (0.06, 4, 0.1), (0.001, 4, 0.5)])
        left = self.planner.plan(points)
        right = self.planner.plan([[x, -y, z] for x, y, z in points])
        self.assertEqual(left['speed_mode'], right['speed_mode'])
        self.assertEqual(left['target_speed_kmh'], right['target_speed_kmh'])
        self.assertAlmostEqual(left['representative_curvature'], right['representative_curvature'])
        self.assertEqual(left['max_curvature'], right['max_curvature'])

    def test_invalid_spatial_parameters_and_previous_modes_fail_closed(self):
        points = curvature_profile_path([(0.06, 4, 0.5)])
        for field in ('curvature_smoothing_window_m', 'curvature_persistence_m'):
            for value in (0.0, -0.1):
                with self.subTest(field=field, value=value):
                    self.assertIsNone(SpeedPlanner(**{field: value}).plan(points))
        self.assertIsNone(SpeedPlanner(curvature_hysteresis=-0.1).plan(points))
        for previous in ('INVALID', True, 1, [], {}):
            with self.subTest(previous=previous):
                self.assertIsNone(self.planner.plan(points, previous_speed_mode=previous))


if __name__ == '__main__':
    unittest.main()
