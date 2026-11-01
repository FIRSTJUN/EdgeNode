"""Continuous sparse-route progress and downstream path-tracking regressions."""

import math
import unittest

from edgenode_control.pure_pursuit import PurePursuit
from edgenode_planning.local_path_planner import LocalPathPlanner
from edgenode_planning.speed_planner import SpeedPlanner


class SegmentProjectionTest(unittest.TestCase):
    def setUp(self):
        self.planner = LocalPathPlanner()

    def test_sparse_segments_keep_horizon_ahead_after_twenty_metres(self):
        points = [[0, 0, 0], [40, 0, 4], [80, 0, 8]]
        previous = arc = None
        for x in (5, 15, 25, 35, 45):
            result = self.planner.extract(points, x, 0, previous_index=previous,
                                          previous_arc_m=arc)
            self.assertAlmostEqual(result['points'][0][0], x)
            self.assertAlmostEqual(result['points'][0][2], x / 10)
            self.assertAlmostEqual(result['points'][-1][0], x + 20)
            self.assertAlmostEqual(result['path_length_m'], 20)
            self.assertAlmostEqual(result['progress_arc_m'], x)
            self.assertEqual(result['progress_segment_index'], int(x // 40))
            self.assertGreaterEqual(len(result['points']), 3)
            self.assertIsNotNone(SpeedPlanner().plan(result['points']))
            target = PurePursuit().compute(result['points'], x, 0, 0)
            self.assertIsNotNone(target)
            self.assertAlmostEqual(target['target_forward_m'], 2.5)
            self.assertGreater(target['target_point'][0], x)
            previous, arc = result['nearest_index'], result['progress_arc_m']
        self.assertEqual(points, [[0, 0, 0], [40, 0, 4], [80, 0, 8]])

    def test_reported_85m_segment_moves_projection_without_changing_index(self):
        points = [[106.82295860990416, 1274.235997798387, 0],
                  [136.69395864993567, 1354.1159978010692, 0]]
        length = math.dist(points[0][:2], points[1][:2])
        result = self.planner.extract(points, 114.35674236921386, 1295.0808105281892,
                                      previous_index=0, previous_arc_m=20.0)
        self.assertEqual(result['nearest_index'], 0)
        self.assertAlmostEqual(result['progress_arc_m'], 22.1631294, places=4)
        self.assertAlmostEqual(result['progress_fraction'], result['progress_arc_m'] / length)
        self.assertAlmostEqual(math.dist(result['points'][0][:2], result['points'][-1][:2]), 20)
        self.assertAlmostEqual(result['nearest_distance_m'], 0.24456, places=3)

    def test_fraction_and_arc_never_retreat_under_pose_noise(self):
        points = [[0, 0, 0], [40, 0, 0], [80, 0, 0]]
        previous = arc = None
        for x, expected in ((25, 25), (24.8, 25), (26, 26), (23, 26)):
            result = self.planner.extract(points, x, 0, previous_index=previous,
                                          previous_arc_m=arc)
            self.assertEqual(result['nearest_index'], 0)
            self.assertAlmostEqual(result['progress_arc_m'], expected)
            self.assertAlmostEqual(result['points'][0][0], expected)
            previous, arc = result['nearest_index'], result['progress_arc_m']

    def test_dense_half_metre_path_starts_at_projection_and_keeps_vertices(self):
        points = [[i * 0.5, 0, i * 0.1] for i in range(100)]
        previous = arc = None
        for x in (0.13, 0.49, 0.51, 1.01):
            result = self.planner.extract(points, x, 0.2, previous_index=previous,
                                          previous_arc_m=arc)
            self.assertAlmostEqual(result['points'][0][0], x)
            self.assertAlmostEqual(result['points'][0][1], 0)
            self.assertAlmostEqual(result['points'][-1][0], x + 20)
            self.assertIn([1.5, 0, 0.30000000000000004], result['points'])
            previous, arc = result['nearest_index'], result['progress_arc_m']

    def test_crossing_outside_window_cannot_reacquire_future_leg(self):
        points = [[x, 0, 0] for x in range(51)]
        points += [[25, y, 0] for y in range(30, -31, -1)]
        result = self.planner.extract(points, 25.001, 0.01, previous_index=24,
                                      previous_arc_m=24.2, search_forward=5)
        self.assertEqual(result['progress_segment_index'], 25)
        self.assertAlmostEqual(result['progress_arc_m'], 25.001)

    def test_nearby_future_crossing_inside_window_does_not_skip_unpassed_segments(self):
        points = [[-5, 0, 0], [5, 0, 0], [5, 5, 0],
                  [0, 5, 0], [0, -5, 0], [10, -5, 0]]
        # A later segment is even closer, but the current segment is unfinished.
        for y in (0.0, 0.001):
            result = self.planner.extract(points, 0, y, previous_index=0,
                                          previous_arc_m=4.9)
            self.assertEqual(result['progress_segment_index'], 0)
            self.assertAlmostEqual(result['progress_arc_m'], 5)

    def test_adjacent_sharp_turn_can_advance_without_unrestricted_search(self):
        points = [[0, 0, 0], [5, 0, 0], [3, 4, 0], [3, 30, 0]]
        result = self.planner.extract(points, 4.5, 1.0, previous_index=0,
                                      previous_arc_m=4.9)
        self.assertEqual(result['progress_segment_index'], 1)
        self.assertGreater(result['progress_arc_m'], 5)

    def test_adjacent_parallel_return_leg_cannot_steal_progress(self):
        points = [[0, 0, 0], [85, 0, 0], [0, 0.001, 0]]
        result = self.planner.extract(points, 25, 0.001, previous_index=0,
                                      previous_arc_m=24.8)
        self.assertEqual(result['progress_segment_index'], 0)
        self.assertAlmostEqual(result['progress_arc_m'], 25.0)
        self.assertAlmostEqual(result['points'][0][1], 0.0)

    def test_zero_index_window_still_advances_inside_long_segment(self):
        result = self.planner.extract([[0, 0, 0], [40, 0, 0]], 25, 0,
                                      previous_index=0, previous_arc_m=10,
                                      search_back=0, search_forward=0)
        self.assertEqual(result['nearest_index'], 0)
        self.assertAlmostEqual(result['progress_fraction'], 0.625)
        self.assertAlmostEqual(result['points'][0][0], 25)

    def test_route_end_is_not_extended_and_short_geometry_stays_invalid(self):
        result = self.planner.extract([[0, 0, 0], [40, 0, 0]], 39.9, 0,
                                      previous_index=0, previous_arc_m=39.8)
        self.assertAlmostEqual(result['path_length_m'], 0.1)
        self.assertEqual(result['points'][-1], [40, 0, 0])
        self.assertIsNone(SpeedPlanner().plan(result['points']))
        ended = self.planner.extract([[0, 0, 0], [40, 0, 0]], 41, 0,
                                     previous_index=0, previous_arc_m=39.9)
        self.assertEqual(ended['points'], [[40, 0, 0]])
        self.assertEqual(ended['nearest_index'], 1)

    def test_invalid_or_inconsistent_arc_progress_fails_closed(self):
        points = [[0, 0, 0], [40, 0, 0], [80, 0, 0]]
        for arc in (-1, math.nan, math.inf, True, 81, 41):
            with self.subTest(arc=arc):
                self.assertIsNone(self.planner.extract(points, 20, 0, previous_index=0,
                                                      previous_arc_m=arc))


if __name__ == '__main__':
    unittest.main()
