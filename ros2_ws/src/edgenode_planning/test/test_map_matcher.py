"""Offline regression tests for matching geometry, thresholds and continuity."""

import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from edgenode_planning.map_matcher import (
    MapMatcher,
    angle_difference_deg,
    closest_point_on_link,
    point_to_segment,
)


def make_link(link_id, points, from_node='n0', to_node='n1', **extra):
    return {
        'idx': link_id,
        'points': points,
        'from_node_idx': from_node,
        'to_node_idx': to_node,
        **extra,
    }


class MapMatcherTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)

    def matcher(self, links=None, **parameters):
        if links is None:
            links = [make_link('A', [[0.0, 0.0], [10.0, 0.0]])]
        path = Path(self.temp_dir.name) / 'link_set.json'
        path.write_text(json.dumps(links), encoding='utf-8')
        return MapMatcher(self.temp_dir.name, **parameters)

    def connected_matcher(self):
        return self.matcher([
            make_link('A', [[0.0, 0.0], [10.0, 0.0]],
                      left_lane_change_dst_link_idx='L',
                      right_lane_change_dst_link_idx='R'),
            make_link('B', [[10.0, 0.0], [20.0, 0.0]], 'n1', 'n2'),
            make_link('L', [[0.0, 3.0], [10.0, 3.0]], 'l0', 'l1'),
            make_link('R', [[0.0, -3.0], [10.0, -3.0]], 'r0', 'r1'),
            make_link('U', [[0.0, 1.0], [20.0, 1.0]], 'u0', 'u1'),
            make_link('D', [[30.0, 0.0], [40.0, 0.0]], 'd0', 'd1'),
        ])

    def test_segment_projection_endpoints_and_degenerate_segment(self):
        for pose, expected in [
            ((5.0, 2.0), 2.0),
            ((-3.0, 4.0), 5.0),
            ((13.0, 4.0), 5.0),
        ]:
            with self.subTest(pose=pose):
                distance, heading = point_to_segment(*pose, 0.0, 0.0, 10.0, 0.0)
                self.assertAlmostEqual(distance, expected)
                self.assertEqual(heading, 0.0)
        self.assertEqual(point_to_segment(3, 4, 0, 0, 0, 0), (5.0, 0.0))

    def test_closest_polyline_segment_and_heading_wrap(self):
        link = make_link('turn', [[0, 0], [10, 0], [10, 10]])
        self.assertEqual(closest_point_on_link(12, 5, link), (2.0, 90.0))
        self.assertEqual(angle_difference_deg(179, -179), 2.0)
        west = self.matcher([make_link('west', [[10, 0], [0, 0]])])
        self.assertEqual(west.match(5, 0, -179)['heading_diff'], 1.0)

    def test_default_score_and_measured_diagnostics(self):
        result = self.matcher().match(5.0, 2.0, 20.0)
        self.assertEqual(result, {
            'link_id': 'A', 'distance': 2.0, 'link_heading': 0.0,
            'heading_diff': 20.0, 'score': 2.5,
        })

    def test_map_is_read_once_and_short_links_are_skipped(self):
        with patch('edgenode_planning.map_matcher.json.load', wraps=json.load) as load:
            matcher = self.matcher([
                make_link('empty', []),
                make_link('point', [[0, 0]]),
                make_link('A', [[0, 0], [10, 0]]),
            ])
            with patch.object(Path, 'open', side_effect=AssertionError('Map reread')):
                for _ in range(3):
                    self.assertEqual(matcher.match(5, 0, 0)['link_id'], 'A')
            load.assert_called_once()
        self.assertEqual(set(matcher.links_by_id), {'A'})
        self.assertEqual(matcher.outgoing, {'n0': {'A'}})

    def test_distance_and_heading_thresholds_are_inclusive(self):
        matcher = self.matcher()
        self.assertIsNotNone(matcher.match(5, 4, 80))
        self.assertIsNone(matcher.match(5, 4.0001, 0))
        self.assertIsNone(matcher.match(5, 0, 80.0001))
        self.assertIsNone(matcher.match(5, 0, 180))

    def test_bbox_search_radius_is_configurable(self):
        matcher = self.matcher(search_radius_m=1.0)
        self.assertIsNotNone(matcher.match(5, 1, 0))
        self.assertIsNone(matcher.match(5, 1.01, 0))

    def test_same_link_continuity_over_nearer_unrelated_link(self):
        matcher = self.connected_matcher()
        self.assertEqual(matcher.match(1, 0, 0)['link_id'], 'A')
        result = matcher.match(5, 0.6, 0)
        self.assertEqual(result['link_id'], 'A')
        self.assertAlmostEqual(result['score'], 0.2)

    def test_next_connected_link_can_win_at_junction(self):
        matcher = self.connected_matcher()
        matcher.match(1, 0, 0)
        self.assertEqual(matcher.get_connected_links(), {'A', 'B', 'L', 'R'})
        result = matcher.match(9.9, 0, 0)
        self.assertEqual(result['link_id'], 'B')
        self.assertAlmostEqual(result['score'], -0.6)

    def test_left_and_right_lane_change_links(self):
        for y, expected in [(2.0, 'L'), (-2.0, 'R')]:
            with self.subTest(link=expected):
                matcher = self.connected_matcher()
                matcher.match(1, 0, 0)
                result = matcher.match(5, y, 0)
                self.assertEqual(result['link_id'], expected)
                self.assertAlmostEqual(result['score'], 0.3)

    def test_unrelated_link_penalty(self):
        matcher = self.connected_matcher()
        matcher.match(1, 0, 0)
        result = matcher.match(35, 0, 0)
        self.assertEqual(result['link_id'], 'D')
        self.assertEqual(result['score'], 1.0)

    def test_match_loss_retains_previous_link_and_recovers(self):
        matcher = self.matcher()
        matcher.match(5, 0, 0)
        self.assertIsNone(matcher.match(100, 100, 0))
        self.assertEqual(matcher.previous_link_id, 'A')
        self.assertEqual(matcher.match(5, 0, 0)['score'], -0.4)

    def test_nonfinite_pose_is_rejected_without_losing_history(self):
        matcher = self.matcher()
        matcher.match(5, 0, 0)
        for axis in range(3):
            for value in [math.nan, math.inf, -math.inf]:
                pose = [5, 0, 0]
                pose[axis] = value
                with self.subTest(pose=pose):
                    self.assertIsNone(matcher.match(*pose))
                    self.assertEqual(matcher.previous_link_id, 'A')


if __name__ == '__main__':
    unittest.main()
