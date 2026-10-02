"""Offline planner tests using temporary synthetic MGeo JSON only."""

import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from edgenode_planning.dijkstra_planner import DijkstraPlanner


def make_link(link_id, from_node, to_node, length=1.0, points=None, **extra):
    if points is None:
        points = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
    return {
        'idx': link_id,
        'from_node_idx': from_node,
        'to_node_idx': to_node,
        'link_length': length,
        'points': points,
        **extra,
    }


class DijkstraPlannerTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.directory = Path(self.temp_dir.name)

    def planner(self, links=None, node_ids=('A', 'B', 'C', 'D', 'S')):
        if links is None:
            links = [
                make_link('AB', 'A', 'B', 5.0,
                          [[0.0, 0.0, 2.0], [3.0, 4.0, 3.0]]),
                make_link('BC', 'B', 'C', 7.0,
                          [[3.0, 4.0, 3.0], [10.0, 4.0, 5.0]]),
            ]
        nodes = [{'idx': node_id, 'point': [0, 0, 0]} for node_id in node_ids]
        self.write_map(nodes, links)
        return DijkstraPlanner(self.directory)

    def write_map(self, nodes, links):
        for filename, data in [('node_set.json', nodes), ('link_set.json', links)]:
            (self.directory / filename).write_text(json.dumps(data), encoding='utf-8')

    def test_chain_route_includes_current_link_and_full_node_order(self):
        result = self.planner().plan('AB', 'C')
        self.assertEqual(result, {
            'start_link_id': 'AB',
            'start_node_id': 'B',
            'goal_node_id': 'C',
            'node_ids': ['A', 'B', 'C'],
            'link_ids': ['AB', 'BC'],
            'points': [[0.0, 0.0, 2.0], [3.0, 4.0, 3.0], [10.0, 4.0, 5.0]],
            'total_cost_m': 12.0,
        })
        self.assertIsInstance(result['total_cost_m'], float)

    def test_shortest_route_uses_link_length_not_geometry_or_hop_count(self):
        planner = self.planner([
            make_link('SA', 'S', 'A', 11.0),
            make_link('AB', 'A', 'B', 1.0),
            make_link('BD', 'B', 'D', 20.0),
            make_link('AC', 'A', 'C', 4.0,
                      [[0, 0, 0], [1000, 0, 0]]),
            make_link('CD', 'C', 'D', 3.0),
            make_link('AD', 'A', 'D', 9.0),
        ])
        result = planner.plan('SA', 'D')
        self.assertEqual(result['link_ids'], ['SA', 'AC', 'CD'])
        self.assertEqual(result['node_ids'], ['S', 'A', 'C', 'D'])
        self.assertEqual(result['total_cost_m'], 18.0)

    def test_search_starts_at_current_link_to_node(self):
        planner = self.planner([
            make_link('AB', 'A', 'B', 3.0),
            make_link('AC', 'A', 'C', 1.0),
            make_link('BC', 'B', 'C', 8.0),
        ])
        self.assertEqual(planner.plan('AB', 'C')['link_ids'], ['AB', 'BC'])

    def test_reverse_route_requires_explicit_reverse_edges(self):
        self.assertIsNone(self.planner().plan('BC', 'A'))

    def test_missing_start_link_returns_none(self):
        self.assertIsNone(self.planner().plan('missing', 'C'))

    def test_missing_goal_node_returns_none(self):
        self.assertIsNone(self.planner().plan('AB', 'missing'))

    def test_unreachable_goal_returns_none(self):
        self.assertIsNone(self.planner().plan('AB', 'D'))

    def test_goal_at_search_start_returns_only_current_link(self):
        result = self.planner().plan('AB', 'B')
        self.assertEqual(result['node_ids'], ['A', 'B'])
        self.assertEqual(result['link_ids'], ['AB'])
        self.assertEqual(result['total_cost_m'], 5.0)

    def test_shared_boundary_points_are_deduplicated_without_losing_z(self):
        planner = self.planner([
            make_link('AB', 'A', 'B', points=[[0, 0, 2], [1, 0, 3]]),
            make_link('BC', 'B', 'C', points=[[1, 0, 3], [2, 0, 4]]),
            make_link('CD', 'C', 'D', points=[[2, 0, 5], [3, 0, 6]]),
        ])
        self.assertEqual(planner.plan('AB', 'D')['points'], [
            [0, 0, 2], [1, 0, 3], [2, 0, 4], [2, 0, 5], [3, 0, 6],
        ])

    def test_total_cost_includes_entire_start_link(self):
        planner = self.planner([
            make_link('AB', 'A', 'B', 100.5),
            make_link('BC', 'B', 'C', 2.5),
        ])
        self.assertEqual(planner.plan('AB', 'C')['total_cost_m'], 103.0)

    def test_lane_change_metadata_does_not_connect_disjoint_roads(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                planner = self.planner([
                    make_link('AB', 'A', 'B', **{
                        side + '_lane_change_dst_link_idx': 'CD',
                        'can_move_' + side + '_lane': True,
                    }),
                    make_link('CD', 'C', 'D'),
                ])
                self.assertIsNone(planner.plan('AB', 'D'))

    def test_invalid_length_falls_back_to_xy_polyline_for_all_route_links(self):
        for length in (None, math.nan, math.inf, -math.inf, 0, -1, 'bad', True):
            with self.subTest(length=length):
                planner = self.planner([
                    make_link('AB', 'A', 'B', length,
                              [[0, 0, 0], [3, 4, 100], [6, 8, 200]]),
                    make_link('BC', 'B', 'C', length,
                              [[6, 8, 200], [9, 12, 0]]),
                ])
                self.assertEqual(planner.plan('AB', 'C')['total_cost_m'], 15.0)

    def test_missing_length_uses_xy_fallback(self):
        link = make_link('AB', 'A', 'B', points=[[0, 0, 0], [3, 4, 10]])
        del link['link_length']
        self.assertEqual(self.planner([link]).plan('AB', 'B')['total_cost_m'], 5.0)

    def test_fallback_cost_participates_in_shortest_route_selection(self):
        planner = self.planner([
            make_link('SA', 'S', 'A'),
            make_link('AB', 'A', 'B', None, [[0, 0, 0], [30, 40, 0]]),
            make_link('BD', 'B', 'D'),
            make_link('AD', 'A', 'D', 10.0),
        ])
        self.assertEqual(planner.plan('SA', 'D')['link_ids'], ['SA', 'AD'])

    def test_invalid_geometry_excludes_start_and_required_continuation(self):
        invalid_points = (
            None, [], [[0, 0, 0]], 'bad', [None, None],
            [[0, 0], [1, 0]], [[0, 0, 0, 0], [1, 0, 0, 0]],
            [[0, 0, 0], [1, 0, math.nan]],
            [[0, 0, 0], [math.inf, 0, 0]],
            [[0, 0, 0], ['1', 0, 0]], [[0, 0, 0], [True, 0, 0]],
            [[0, 0, 0], [10 ** 400, 0, 0]],
        )
        for points in invalid_points:
            for bad_id in ('AB', 'BC'):
                with self.subTest(points=points, bad_id=bad_id):
                    links = [make_link('AB', 'A', 'B'), make_link('BC', 'B', 'C')]
                    next(link for link in links if link['idx'] == bad_id)['points'] = points
                    self.assertIsNone(self.planner(links).plan('AB', 'C'))

    def test_unusable_fallback_excludes_link(self):
        for points in (
            [[0, 0, 0], [0, 0, 1]],
            [[-1e308, 0, 0], [1e308, 0, 0]],
            [[-10 ** 308, 0, 0], [10 ** 308, 0, 0]],
        ):
            with self.subTest(points=points):
                planner = self.planner([
                    make_link('AB', 'A', 'B'),
                    make_link('BC', 'B', 'C', None, points),
                ])
                self.assertIsNone(planner.plan('BC', 'C'))
                self.assertIsNone(planner.plan('AB', 'C'))
                self.assertEqual(planner.graph['B'], [])

    def test_missing_or_invalid_endpoints_exclude_link(self):
        for field in ('from_node_idx', 'to_node_idx'):
            for value in ('missing', None, [], {}):
                for bad_id in ('AB', 'BC'):
                    with self.subTest(field=field, value=value, bad_id=bad_id):
                        links = [make_link('AB', 'A', 'B'), make_link('BC', 'B', 'C')]
                        next(link for link in links if link['idx'] == bad_id)[field] = value
                        self.assertIsNone(self.planner(links).plan('AB', 'C'))

    def test_missing_required_link_fields_return_none(self):
        for field in ('points', 'from_node_idx', 'to_node_idx'):
            with self.subTest(field=field):
                link = make_link('AB', 'A', 'B')
                del link[field]
                self.assertIsNone(self.planner([link]).plan('AB', 'B'))

    def test_bad_edge_does_not_prevent_valid_alternative(self):
        planner = self.planner([
            make_link('AB', 'A', 'B'),
            make_link('bad', 'B', 'C', points=[[0, 0, 0]]),
            make_link('BD', 'B', 'D', 4.0),
            make_link('DC', 'D', 'C', 3.0),
        ])
        self.assertEqual(planner.plan('AB', 'C')['link_ids'], ['AB', 'BD', 'DC'])

    def test_parallel_links_cycles_and_improved_predecessor(self):
        planner = self.planner([
            make_link('SA', 'S', 'A'),
            make_link('AB_expensive', 'A', 'B', 8.0),
            make_link('AB', 'A', 'B', 4.0),
            make_link('AC', 'A', 'C'),
            make_link('CB', 'C', 'B'),
            make_link('BA', 'B', 'A'),
            make_link('BB', 'B', 'B'),
            make_link('BD', 'B', 'D', 10.0),
        ])
        result = planner.plan('SA', 'D')
        self.assertEqual(result['link_ids'], ['SA', 'AC', 'CB', 'BD'])
        self.assertEqual(result['node_ids'], ['S', 'A', 'C', 'B', 'D'])
        self.assertEqual(result['total_cost_m'], 13.0)
        self.assertEqual(planner.plan('AB', 'A')['link_ids'], ['AB', 'BA'])

    def test_equal_cost_paths_produce_a_minimum_cost_route(self):
        planner = self.planner([
            make_link('SA', 'S', 'A'),
            make_link('AB', 'A', 'B'), make_link('BD', 'B', 'D'),
            make_link('AC', 'A', 'C'), make_link('CD', 'C', 'D'),
        ])
        result = planner.plan('SA', 'D')
        self.assertIn(result['link_ids'], [['SA', 'AB', 'BD'], ['SA', 'AC', 'CD']])
        self.assertEqual(result['total_cost_m'], 3.0)

    def test_cost_overflow_returns_none(self):
        planner = self.planner([
            make_link('SA', 'S', 'A', 1e308),
            make_link('AB', 'A', 'B', 1e308),
            make_link('BC', 'B', 'C', 1e308),
        ])
        self.assertIsNone(planner.plan('SA', 'B'))
        self.assertIsNone(planner.plan('SA', 'C'))

    def test_map_is_read_once_and_returned_points_do_not_mutate_map(self):
        with patch('edgenode_planning.dijkstra_planner.json.load', wraps=json.load) as load:
            planner = self.planner()
            self.assertEqual(load.call_count, 2)
            with patch.object(Path, 'open', side_effect=AssertionError('Map reread')):
                result = planner.plan('AB', 'C')
                result['points'][0][0] = 999
                result['link_ids'].clear()
                result['node_ids'].clear()
                again = planner.plan('AB', 'C')
                self.assertEqual(again['points'][0], [0.0, 0.0, 2.0])
                self.assertEqual(again['link_ids'], ['AB', 'BC'])
                self.assertEqual(again['node_ids'], ['A', 'B', 'C'])
                self.assertIsNone(planner.plan('BC', 'A'))
                self.assertIsNotNone(planner.plan('AB', 'B'))

    def test_malformed_records_and_ambiguous_ids_are_excluded(self):
        planner = self.planner([
            None, [], 'bad', {}, {'idx': []},
            make_link('AB', 'A', 'B'),
            make_link('BC', 'B', 'C'), make_link('BC', 'B', 'D'),
        ])
        self.assertIsNotNone(planner.plan('AB', 'B'))
        self.assertIsNone(planner.plan('AB', 'C'))
        self.write_map(
            [{'idx': 'A'}, {'idx': 'B'}, {'idx': 'B'}, None, {'idx': []}],
            [make_link('AB', 'A', 'B')],
        )
        self.assertIsNone(DijkstraPlanner(self.directory).plan('AB', 'B'))

    def test_missing_or_malformed_map_files_fail_closed(self):
        self.assertIsNone(DijkstraPlanner(self.directory).plan('AB', 'B'))
        for filename in ('node_set.json', 'link_set.json'):
            for contents in ('{broken', '{}', 'null'):
                with self.subTest(filename=filename, contents=contents):
                    self.planner()
                    (self.directory / filename).write_text(contents, encoding='utf-8')
                    self.assertIsNone(DijkstraPlanner(self.directory).plan('AB', 'B'))

    def test_non_string_query_ids_return_none(self):
        planner = self.planner()
        for value in (None, [], {}, 1, True):
            with self.subTest(value=value):
                self.assertIsNone(planner.plan(value, 'C'))
                self.assertIsNone(planner.plan('AB', value))


if __name__ == '__main__':
    unittest.main()
