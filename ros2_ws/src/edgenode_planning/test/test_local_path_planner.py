"""Offline tests for projected local paths and fail-closed validation."""

import math
import unittest

from edgenode_planning.local_path_planner import LocalPathPlanner
from edgenode_planning.speed_planner import SpeedPlanner


class LocalPathPlannerTest(unittest.TestCase):
    def setUp(self):
        self.planner = LocalPathPlanner()
        self.points = [[x, 0, x / 10.0] for x in range(0, 61, 5)]

    def assert_subdivision(self, result, vertices, length):
        for actual, expected in zip(result['points'][0], vertices[0]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(result['points'][-1], vertices[-1]):
            self.assertAlmostEqual(actual, expected)
        for vertex in vertices[1:-1]:
            self.assertIn(vertex, result['points'])
        distances = [math.dist(a[:2], b[:2])
                     for a, b in zip(result['points'], result['points'][1:])]
        self.assertTrue(all(0 < distance <= 2.0 + 1e-9 for distance in distances))
        self.assertAlmostEqual(sum(distances), length)
        self.assertAlmostEqual(result['path_length_m'], length)

    def test_projection_distance_and_points_behind_are_excluded(self):
        result = self.planner.extract(self.points, 12, 4)
        self.assertEqual(result['nearest_index'], 2)
        self.assertAlmostEqual(result['nearest_distance_m'], 4.0)
        self.assert_subdivision(result, [[12, 0, 1.2]] + self.points[3:7]
                                + [[32, 0, 3.2]], 20.0)

    def test_horizon_clips_inside_first_crossing_segment(self):
        result = self.planner.extract(self.points, 10, 0, horizon_m=12.0)
        self.assert_subdivision(result, self.points[2:5] + [[22, 0, 2.2]], 12.0)

    def test_horizon_uses_cumulative_xy_distance_on_turns(self):
        points = [[0, 0, 1], [3, 4, 100], [6, 0, -100], [9, 4, 2], [12, 0, 3]]
        result = self.planner.extract(points, 0, 0, horizon_m=12.0)
        self.assert_subdivision(result, points[:3] + [[7.2, 1.6, -59.2]], 12.0)

    def test_route_end_before_horizon_returns_only_remaining_points(self):
        result = self.planner.extract(self.points, 50, 0)
        self.assert_subdivision(result, self.points[10:], 10.0)

    def test_xyz_values_are_preserved_and_input_is_not_mutated(self):
        points = [[0, 0, -1], [3.5, 4.5, 100.25], [6, 7, -200.75]]
        original = [point.copy() for point in points]
        result = self.planner.extract(points, 3.5, 4.5)
        self.assertEqual(result['points'], [points[1], [4.75, 5.75, -50.25], points[2]])
        result['points'][0][0] = 999
        self.assertEqual(points, original)

    def test_single_point_and_nearest_at_last_point(self):
        for path in ([[1, 2, 3]], self.points):
            x, y, _ = path[-1]
            result = self.planner.extract(path, x, y)
            self.assertEqual(result['nearest_index'], len(path) - 1)
            self.assertEqual(result['points'], [path[-1]])
            self.assertEqual(result['nearest_distance_m'], 0.0)
            self.assertEqual(result['path_length_m'], 0.0)

    def test_equal_distances_choose_first_index(self):
        result = self.planner.extract([[0, 0, 1], [2, 0, 2]], 1, 0)
        self.assertEqual(result['nearest_index'], 0)

    def test_duplicate_xy_points_keep_first_z_and_continue_forward(self):
        points = [[0, 0, 1], [0, 0, 2], [5, 0, 3], [10, 0, 4]]
        result = self.planner.extract(points, 0, 0, horizon_m=5.0)
        self.assert_subdivision(result, [points[0], points[2]], 5.0)

    def test_empty_and_malformed_paths_fail_closed(self):
        invalid_paths = (
            None, [], (), 'bad', {}, [None], [1], [[0, 0]], [[0, 0, 0, 0]],
            [[0, 0, 0], ['1', 0, 0]], [[True, 0, 0]],
            [[0, 0, math.nan]], [[math.inf, 0, 0]], [[0, -math.inf, 0]],
            [[10 ** 400, 0, 0]], [[-1e308, 0, 0], [1e308, 0, 0]],
            # Invalid points must be rejected even past the requested horizon.
            [[0, 0, 0], [25, 0, 0], [30, 0, 'bad']],
        )
        for points in invalid_paths:
            with self.subTest(points=points):
                self.assertIsNone(self.planner.extract(points, 0, 0))

    def test_invalid_pose_and_horizon_fail_closed(self):
        for value in (None, 'bad', True, [], math.nan, math.inf, -math.inf, 10 ** 400):
            with self.subTest(value=value):
                self.assertIsNone(self.planner.extract(self.points, value, 0))
                self.assertIsNone(self.planner.extract(self.points, 0, value))
                self.assertIsNone(self.planner.extract(self.points, 0, 0, value))
        for horizon in (0, -1):
            with self.subTest(horizon=horizon):
                self.assertIsNone(self.planner.extract(self.points, 0, 0, horizon))

    def test_computed_distance_and_accumulated_length_overflow_fail_closed(self):
        self.assertIsNone(self.planner.extract([[1e308, 0, 0]], -1e308, 0))
        points = [[0, 0, 0], [1e308, 0, 0], [0, 0, 0]]
        self.assertIsNone(self.planner.extract(points, 0, 0, horizon_m=1.5e308))

    def test_progress_increases_sequentially_on_straight_path(self):
        points = [[x, 0, 0] for x in range(100)]
        previous = None
        for x in range(50):
            result = self.planner.extract(points, x, 0, previous_index=previous)
            self.assertEqual(result['nearest_index'], x)
            previous = result['nearest_index']

    def test_small_backwards_pose_noise_never_retreats_progress(self):
        points = [[x, 0, 0] for x in range(30)]
        previous = 10
        arc = 10.0
        for x in (9.4, 10.2, 9.2, 11.0, 10.3):
            result = self.planner.extract(points, x, 0, previous_index=previous,
                                          previous_arc_m=arc)
            self.assertGreaterEqual(result['nearest_index'], previous)
            self.assertGreaterEqual(result['progress_arc_m'], arc)
            self.assertAlmostEqual(result['nearest_distance_m'],
                                   abs(result['points'][0][0] - x))
            previous = result['nearest_index']
            arc = result['progress_arc_m']
        self.assertEqual(previous, 11)

    def test_crossing_geometry_does_not_select_distant_future_section(self):
        points = [[x, 0, 0] for x in range(51)]
        points += [[25, y, 0] for y in range(30, -31, -1)]
        # Full-path nearest picks a later crossing point at index 80.
        self.assertEqual(self.planner.extract(points, 25, 1)['nearest_index'], 80)
        result = self.planner.extract(points, 25, 1, previous_index=24,
                                      search_back=3, search_forward=5)
        self.assertEqual(result['nearest_index'], 25)
        self.assertEqual(result['points'][0], points[25])

    def test_per_cycle_advance_is_limited_even_after_large_pose_jump(self):
        points = [[x, 0, 0] for x in range(100)]
        previous = 10
        for _ in range(3):
            result = self.planner.extract(points, 90, 0, previous_index=previous,
                                          search_forward=4)
            self.assertEqual(result['nearest_index'], previous + 4)
            previous = result['nearest_index']

    def test_initialization_and_explicit_reset_allow_mid_route_start(self):
        points = [[x, 0, 0] for x in range(100)]
        self.assertEqual(self.planner.extract(points, 70, 0)['nearest_index'], 70)
        # A replacement, shorter path is initialized independently of old progress.
        result = self.planner.extract(self.points, 10, 0, previous_index=None)
        self.assertEqual(result['nearest_index'], 2)

    def test_progress_window_is_clipped_at_both_path_ends(self):
        for previous, x, expected in ((0, -10, 0), (11, 60, 12), (12, 59, 12)):
            result = self.planner.extract(self.points, x, 0, previous_index=previous)
            self.assertEqual(result['nearest_index'], expected)
            vertices = self.points[expected:expected + 5]
            self.assert_subdivision(result, vertices, vertices[-1][0] - vertices[0][0])

    def test_invalid_progress_and_windows_fail_closed(self):
        for previous in (-1, len(self.points), True, 1.5, '1', math.nan):
            with self.subTest(previous=previous):
                self.assertIsNone(self.planner.extract(self.points, 0, 0,
                                                      previous_index=previous))
        for window in (-1, True, 1.5, None, '10', math.inf):
            for name in ('search_back', 'search_forward'):
                with self.subTest(name=name, window=window):
                    self.assertIsNone(self.planner.extract(self.points, 0, 0,
                                                          **{name: window}))

    def test_zero_window_holds_index_and_tie_does_not_retreat(self):
        result = self.planner.extract(self.points, 60, 0, previous_index=3,
                                      search_back=0, search_forward=0)
        self.assertEqual(result['nearest_index'], 3)
        result = self.planner.extract([[0, 0, 0], [0, 0, 0], [1, 0, 0]],
                                      0, 0, previous_index=1)
        self.assertEqual(result['nearest_index'], 1)

    def test_long_first_segment_is_densified_and_clipped_to_horizon(self):
        points = [[0, 0, 0], [30, 0, 3], [40, 0, 4]]
        result = self.planner.extract(points, 0, 0)
        self.assertEqual(result['points'], [[x, 0, x / 10] for x in range(0, 21, 2)])
        self.assertEqual(result['path_length_m'], 20.0)
        self.assertEqual(result['nearest_index'], 0)
        self.assertEqual(points, [[0, 0, 0], [30, 0, 3], [40, 0, 4]])
        self.assertEqual(SpeedPlanner().plan(result['points'])['speed_mode'], 'CRUISE')

    def test_reported_mgeo_segment_supplies_valid_speed_preview(self):
        points = [[99.29995859932387, 1253.3509978000075, -0.5369995269775387],
                  [106.82295860990416, 1274.235997798387, -0.4940002593994137]]
        result = self.planner.extract(points, *points[0][:2])
        self.assertEqual(len(result['points']), 11)
        self.assertAlmostEqual(result['path_length_m'], 20.0)
        self.assertEqual(result['nearest_index'], 0)
        self.assertIsNotNone(SpeedPlanner().plan(result['points']))
        # Interpolation preserves the line; it does not invent a hidden curve.
        for p in result['points']:
            cross = ((p[0] - points[0][0]) * (points[1][1] - points[0][1])
                     - (p[1] - points[0][1]) * (points[1][0] - points[0][0]))
            self.assertAlmostEqual(cross, 0.0, places=9)

    def test_resampling_keeps_progress_in_original_global_indices(self):
        points = [[30 * i, 0, 0] for i in range(20)]
        previous = 2
        for x, expected in ((65, 2), (200, 6), (400, 10)):
            result = self.planner.extract(points, x, 0, previous_index=previous,
                                          search_back=3, search_forward=4)
            self.assertEqual(result['nearest_index'], expected)
            self.assertAlmostEqual(result['points'][0][0], min(x, 30 * (expected + 1)))
            self.assertGreaterEqual(len(result['points']), 3)
            previous = result['nearest_index']

    def test_dense_mgeo_points_are_preserved_without_downsampling(self):
        points = [[i * 0.5, 0, i] for i in range(61)]
        result = self.planner.extract(points, 0, 0)
        self.assertEqual(result['points'], points[:41])
        self.assertEqual(result['path_length_m'], 20.0)

    def test_corner_vertices_and_polyline_length_are_preserved(self):
        points = [[0, 0, 0], [3, 0, 3], [3, 4, 7], [8, 4, 12]]
        result = self.planner.extract(points, 0, 0, horizon_m=10.0)
        self.assertIn(points[1], result['points'])
        self.assertIn(points[2], result['points'])
        self.assertEqual(result['points'][-1], [6, 4, 10])
        self.assertAlmostEqual(result['path_length_m'], 10.0)
        for a, b in zip(result['points'], result['points'][1:]):
            self.assertLessEqual(math.dist(a[:2], b[:2]), 2.0)
            self.assertTrue(a[0] == b[0] or a[1] == b[1])

    def test_true_short_route_end_is_not_extended_or_padded(self):
        for points in ([[0, 0, 0]], [[0, 0, 0], [0.1, 0, 1]],
                       [[0, 0, 0], [0, 0, 1]]):
            with self.subTest(points=points):
                result = self.planner.extract(points, 0, 0)
                self.assertLessEqual(result['path_length_m'], 0.1)
                self.assertLess(len(result['points']), 3)
                self.assertIsNone(SpeedPlanner().plan(result['points']))

    def test_duplicate_xy_points_do_not_create_repeated_samples(self):
        result = self.planner.extract([[0, 0, 1], [0, 0, 2], [30, 0, 3],
                                       [30, 0, 4]], 0, 0)
        self.assertEqual(result['points'][0], [0, 0, 1])
        self.assertEqual(len(result['points']), 11)
        self.assertAlmostEqual(result['path_length_m'], 20.0)
        self.assertTrue(all(math.dist(a[:2], b[:2]) > 0
                            for a, b in zip(result['points'], result['points'][1:])))

    def test_resampling_preserves_existing_curve_speed_decisions(self):
        for radius, mode in ((20, 'CURVE'), (5, 'SHARP_CURVE')):
            points = [[radius * math.sin(i * 0.1),
                       radius * (1 - math.cos(i * 0.1)), i] for i in range(6)]
            with self.subTest(radius=radius):
                result = self.planner.extract(points, 0, 0)
                self.assertEqual(result['points'], points)
                self.assertEqual(SpeedPlanner().plan(result['points'])['speed_mode'], mode)

    def test_invalid_resample_spacing_fails_closed(self):
        for spacing in (0, -1, True, None, math.nan, math.inf, '0.5', 10 ** 400):
            with self.subTest(spacing=spacing):
                self.assertIsNone(self.planner.extract(self.points, 0, 0,
                                                      resample_spacing_m=spacing))

    def test_unbounded_resampling_request_fails_closed(self):
        self.assertIsNone(self.planner.extract([[0, 0, 0], [30, 0, 0]], 0, 0,
                                              resample_spacing_m=1e-300))


if __name__ == '__main__':
    unittest.main()
