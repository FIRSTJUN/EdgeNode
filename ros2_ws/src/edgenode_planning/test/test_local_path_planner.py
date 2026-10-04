"""Offline tests for nearest-point local paths and fail-closed validation."""

import math
import unittest

from edgenode_planning.local_path_planner import LocalPathPlanner


class LocalPathPlannerTest(unittest.TestCase):
    def setUp(self):
        self.planner = LocalPathPlanner()
        self.points = [[x, 0, x / 10.0] for x in range(0, 61, 5)]

    def test_nearest_index_distance_and_points_behind_are_excluded(self):
        result = self.planner.extract(self.points, 12, 4)
        self.assertEqual(result['nearest_index'], 2)
        self.assertAlmostEqual(result['nearest_distance_m'], math.hypot(2, 4))
        self.assertEqual(result['points'], self.points[2:7])
        self.assertEqual(result['path_length_m'], 20.0)

    def test_horizon_includes_first_point_crossing_it(self):
        result = self.planner.extract(self.points, 10, 0, horizon_m=12.0)
        self.assertEqual(result['points'], self.points[2:6])
        self.assertEqual(result['path_length_m'], 15.0)

    def test_horizon_uses_cumulative_xy_distance_on_turns(self):
        points = [[0, 0, 1], [3, 4, 100], [6, 0, -100], [9, 4, 2], [12, 0, 3]]
        result = self.planner.extract(points, 0, 0, horizon_m=12.0)
        self.assertEqual(result['points'], points[:4])
        self.assertEqual(result['path_length_m'], 15.0)

    def test_route_end_before_horizon_returns_only_remaining_points(self):
        result = self.planner.extract(self.points, 50, 0)
        self.assertEqual(result['points'], self.points[10:])
        self.assertEqual(result['path_length_m'], 10.0)

    def test_xyz_values_are_preserved_and_input_is_not_mutated(self):
        points = [[0, 0, -1], [3.5, 4.5, 100.25], [6, 7, -200.75]]
        original = [point.copy() for point in points]
        result = self.planner.extract(points, 3.5, 4.5)
        self.assertEqual(result['points'], points[1:])
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

    def test_duplicate_xy_points_keep_xyz_and_continue_forward(self):
        points = [[0, 0, 1], [0, 0, 2], [5, 0, 3], [10, 0, 4]]
        result = self.planner.extract(points, 0, 0, horizon_m=5.0)
        self.assertEqual(result['points'], points[:3])
        self.assertEqual(result['path_length_m'], 5.0)

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


if __name__ == '__main__':
    unittest.main()
