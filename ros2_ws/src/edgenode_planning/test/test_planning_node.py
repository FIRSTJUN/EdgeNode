"""Check ROS message handling and zero commands without starting ROS nodes."""

import json
import math
from pathlib import Path as FilePath
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.time import Time

from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.map_matcher import MapMatcher
from edgenode_planning.planning_node import PlanningNode


class PlanningNodeTest(unittest.TestCase):
    def setUp(self):
        self.parameters = {
            'goal_node_id': 'goal_default',
            'global_path_frame_id': 'map',
            'cruise_speed_kmh': 50.0,
        }
        self.route = {'points': [[0, 1, 2], [3.5, 4.5, 5.5], [6, 7, 8]]}
        self.stamp = Time(seconds=123, nanoseconds=456).to_msg()
        self.node = SimpleNamespace(
            latest_pose=None,
            map_matcher=Mock(),
            dijkstra_planner=Mock(),
            _route_key=None,
            _route_result=None,
            get_parameter=lambda name: SimpleNamespace(value=self.parameters[name]),
            get_clock=Mock(),
        )
        self.node.get_clock.return_value.now.return_value.to_msg.return_value = self.stamp
        self.node.dijkstra_planner.plan.return_value = self.route
        self.node.map_matcher.match.return_value = {
            'link_id': 'link_42', 'distance': 1.25, 'heading_diff': 12.5,
        }
        for name in (
            'target_error', 'target_speed', 'state', 'current_link',
            'map_match_status', 'map_match_distance', 'map_match_heading_diff',
            'global_path',
        ):
            setattr(self.node, name + '_pub', Mock())

    def output(self, name):
        return getattr(self.node, name + '_pub').publish.call_args.args[0].data

    def assert_stopped(self):
        self.assertEqual(self.output('target_speed'), 0.0)
        self.assertEqual(self.output('target_error'), 0.0)

    def assert_path(self, expected_points, frame_id='map', stamp=None):
        path = self.node.global_path_pub.publish.call_args.args[0]
        self.assertIsInstance(path, Path)
        self.assertEqual(path.header.frame_id, frame_id)
        self.assertEqual(path.header.stamp, self.stamp if stamp is None else stamp)
        self.assertEqual(len(path.poses), len(expected_points))
        self.assertGreater(len(path.poses), 0)
        for pose, point in zip(path.poses, expected_points):
            self.assertIsInstance(pose, PoseStamped)
            self.assertEqual(pose.header, path.header)
            p = pose.pose.position
            self.assertEqual([p.x, p.y, p.z], point)
            q = pose.pose.orientation
            self.assertEqual([q.x, q.y, q.z, q.w], [0.0, 0.0, 0.0, 1.0])

    def test_wait_for_localization(self):
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.assert_not_called()
        self.node.dijkstra_planner.plan.assert_not_called()
        self.node.global_path_pub.publish.assert_not_called()
        self.assertEqual(self.output('state'), 'WAIT_FOR_LOCALIZATION')
        self.assertEqual(self.output('map_match_status'), 'WAIT_FOR_LOCALIZATION')
        self.assertEqual(self.output('current_link'), '')
        self.assertTrue(math.isnan(self.output('map_match_distance')))
        self.assertTrue(math.isnan(self.output('map_match_heading_diff')))
        self.assert_stopped()

    def test_callback_stores_only_latest_pose_with_enu_yaw(self):
        for degrees in [0.0, 90.0, -90.0, 179.0, -179.0]:
            with self.subTest(degrees=degrees):
                msg = Odometry()
                msg.pose.pose.position.x = 12.0
                msg.pose.pose.position.y = -3.0
                yaw = math.radians(degrees)
                msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
                msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
                PlanningNode.localization_callback(self.node, msg)
                self.assertEqual(self.node.latest_pose[:2], (12.0, -3.0))
                self.assertAlmostEqual(self.node.latest_pose[2], yaw)
        self.node.map_matcher.match.assert_not_called()
        self.node.dijkstra_planner.plan.assert_not_called()
        self.node.global_path_pub.publish.assert_not_called()
        self.node.target_speed_pub.publish.assert_not_called()

    def test_quaternion_with_roll_and_pitch(self):
        # Standard ROS quaternion (x, y, z, w) for roll=30, pitch=-20, yaw=70 deg.
        msg = Odometry()
        q = msg.pose.pose.orientation
        q.x = 0.30499790703769875
        q.y = 0.00879977431167487
        q.z = 0.5824308212572792
        q.w = 0.7534408929201135
        PlanningNode.localization_callback(self.node, msg)
        self.assertAlmostEqual(math.degrees(self.node.latest_pose[2]), 70.0)

    def test_match_success_publishes_actual_diagnostics_and_degrees(self):
        self.node.latest_pose = (12.0, -3.0, math.pi / 2.0)
        self.node.map_matcher.match.return_value = {
            'link_id': 'link_42', 'distance': 1.25, 'heading_diff': 12.5,
        }
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.assert_called_once_with(12.0, -3.0, 90.0)
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.assertEqual(self.output('map_match_status'), 'MATCHED')
        self.assertEqual(self.output('current_link'), 'link_42')
        self.assertEqual(self.output('map_match_distance'), 1.25)
        self.assertEqual(self.output('map_match_heading_diff'), 12.5)
        self.node.dijkstra_planner.plan.assert_called_once_with('link_42', 'goal_default')
        self.node.global_path_pub.publish.assert_called_once()
        self.assert_path(self.route['points'])
        self.assert_stopped()

    def test_match_loss_clears_link_and_diagnostics_after_success(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        self.node.map_matcher.match.return_value = {
            'link_id': 'link_42', 'distance': 1.25, 'heading_diff': 12.5,
        }
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.return_value = None
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('state'), 'MAP_MATCH_LOST')
        self.assertEqual(self.output('map_match_status'), 'NO_MATCH')
        self.assertEqual(self.output('current_link'), '')
        self.assertTrue(math.isnan(self.output('map_match_distance')))
        self.assertTrue(math.isnan(self.output('map_match_heading_diff')))
        self.node.dijkstra_planner.plan.assert_called_once()
        self.node.global_path_pub.publish.assert_called_once()
        self.assert_stopped()

    def test_initial_match_failure_does_not_plan_or_publish_path(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        self.node.map_matcher.match.return_value = None
        PlanningNode.plan(self.node)
        self.node.dijkstra_planner.plan.assert_not_called()
        self.node.global_path_pub.publish.assert_not_called()
        self.assertEqual(self.output('state'), 'MAP_MATCH_LOST')
        self.assertEqual(self.output('map_match_status'), 'NO_MATCH')
        self.assert_stopped()

    def test_dijkstra_failure_preserves_matching_diagnostics_and_stops(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        self.node.dijkstra_planner.plan.return_value = None
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_NOT_FOUND')
        self.assertEqual(self.output('map_match_status'), 'MATCHED')
        self.assertEqual(self.output('current_link'), 'link_42')
        self.assertEqual(self.output('map_match_distance'), 1.25)
        self.assertEqual(self.output('map_match_heading_diff'), 12.5)
        self.node.global_path_pub.publish.assert_not_called()
        self.assert_stopped()

    def test_same_link_and_goal_reuse_route_while_publishing_each_tick(self):
        for x in (12.0, 13.0, 14.0):
            self.node.latest_pose = (x, -3.0, 0.0)
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
            self.assert_path(self.route['points'])
            self.assert_stopped()
        self.node.dijkstra_planner.plan.assert_called_once_with('link_42', 'goal_default')
        self.assertEqual(self.node.map_matcher.match.call_count, 3)
        self.assertEqual(self.node.global_path_pub.publish.call_count, 3)

    def test_same_link_and_goal_cache_failed_search(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        self.node.dijkstra_planner.plan.return_value = None
        for _ in range(3):
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('state'), 'GLOBAL_PATH_NOT_FOUND')
            self.assert_stopped()
        self.node.dijkstra_planner.plan.assert_called_once()
        self.node.global_path_pub.publish.assert_not_called()

    def test_link_change_replans_and_publishes_new_points(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.return_value['link_id'] = 'link_43'
        next_points = [[3.5, 4.5, 5.5], [6, 7, 8]]
        self.node.dijkstra_planner.plan.return_value = {'points': next_points}
        PlanningNode.plan(self.node)
        self.assertEqual(self.node.dijkstra_planner.plan.call_args_list, [
            call('link_42', 'goal_default'), call('link_43', 'goal_default'),
        ])
        self.assert_path(next_points)
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.assert_stopped()

    def test_goal_parameter_change_replans_on_next_tick(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        PlanningNode.plan(self.node)
        self.parameters['goal_node_id'] = 'goal_updated'
        next_points = [[0, 1, 2], [9, 10, 11]]
        self.node.dijkstra_planner.plan.return_value = {'points': next_points}
        PlanningNode.plan(self.node)
        self.assertEqual(self.node.dijkstra_planner.plan.call_args_list, [
            call('link_42', 'goal_default'), call('link_42', 'goal_updated'),
        ])
        self.assert_path(next_points)
        self.assert_stopped()

    def test_failed_replan_stops_publishing_previous_successful_route(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        PlanningNode.plan(self.node)
        self.node.global_path_pub.publish.reset_mock()
        self.parameters['goal_node_id'] = 'unreachable'
        self.node.dijkstra_planner.plan.return_value = None
        for _ in range(3):
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('state'), 'GLOBAL_PATH_NOT_FOUND')
            self.assert_stopped()
        self.assertEqual(self.node.dijkstra_planner.plan.call_count, 2)
        self.node.global_path_pub.publish.assert_not_called()

        self.parameters['goal_node_id'] = 'goal_default'
        self.node.dijkstra_planner.plan.return_value = self.route
        PlanningNode.plan(self.node)
        self.assertEqual(self.node.dijkstra_planner.plan.call_count, 3)
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.assert_path(self.route['points'])
        self.assert_stopped()

    def test_match_recovery_reuses_cache_only_after_matching_succeeds(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        match = self.node.map_matcher.match.return_value
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.return_value = None
        for _ in range(3):
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('state'), 'MAP_MATCH_LOST')
            self.assert_stopped()
        self.node.global_path_pub.publish.assert_called_once()
        self.node.map_matcher.match.return_value = match
        PlanningNode.plan(self.node)
        self.node.dijkstra_planner.plan.assert_called_once()
        self.assertEqual(self.node.global_path_pub.publish.call_count, 2)
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.assert_stopped()

    def test_goal_change_during_match_loss_is_used_on_recovery(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        match = self.node.map_matcher.match.return_value
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.return_value = None
        self.parameters['goal_node_id'] = 'goal_updated'
        PlanningNode.plan(self.node)
        self.node.dijkstra_planner.plan.assert_called_once()
        self.node.global_path_pub.publish.assert_called_once()
        self.node.map_matcher.match.return_value = match
        PlanningNode.plan(self.node)
        self.assertEqual(self.node.dijkstra_planner.plan.call_args_list, [
            call('link_42', 'goal_default'), call('link_42', 'goal_updated'),
        ])
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.assert_stopped()

    def test_path_frame_and_timestamp_refresh_without_replanning(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        PlanningNode.plan(self.node)
        previous_path = self.node.global_path_pub.publish.call_args.args[0]
        self.parameters['global_path_frame_id'] = 'mgeo_map'
        next_stamp = Time(seconds=124).to_msg()
        self.node.get_clock.return_value.now.return_value.to_msg.return_value = next_stamp
        PlanningNode.plan(self.node)
        self.assert_path(self.route['points'], frame_id='mgeo_map', stamp=next_stamp)
        self.assertEqual(previous_path.header.frame_id, 'map')
        self.assertEqual(previous_path.header.stamp, self.stamp)
        self.node.dijkstra_planner.plan.assert_called_once()
        self.assert_stopped()

    def test_real_matcher_and_planner_publish_synthetic_route(self):
        with tempfile.TemporaryDirectory() as directory:
            nodes = [
                {'idx': 'A', 'point': [0, 0, 1]},
                {'idx': 'B', 'point': [10, 0, 2]},
                {'idx': 'C', 'point': [20, 0, 3]},
            ]
            links = [
                {'idx': 'AB', 'from_node_idx': 'A', 'to_node_idx': 'B',
                 'points': [[0, 0, 1], [10, 0, 2]], 'link_length': 10.0},
                {'idx': 'BC', 'from_node_idx': 'B', 'to_node_idx': 'C',
                 'points': [[10, 0, 2], [20, 0, 3]], 'link_length': 10.0},
            ]
            for filename, data in [('node_set.json', nodes), ('link_set.json', links)]:
                (FilePath(directory) / filename).write_text(json.dumps(data), encoding='utf-8')
            self.node.map_matcher = MapMatcher(directory)
            self.node.dijkstra_planner = Mock(wraps=DijkstraPlanner(directory))
            self.parameters['goal_node_id'] = 'C'
            self.node.latest_pose = (5.0, 0.0, 0.0)
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('current_link'), 'AB')
            self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
            self.assert_path([[0, 0, 1], [10, 0, 2], [20, 0, 3]])
            self.assert_stopped()

            self.node.latest_pose = (15.0, 0.0, 0.0)
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('current_link'), 'BC')
            self.assert_path([[10, 0, 2], [20, 0, 3]])
            self.assertEqual(self.node.dijkstra_planner.plan.call_args_list, [
                call('AB', 'C'), call('BC', 'C'),
            ])
            self.assert_stopped()

    def test_constructor_shares_map_and_creates_configured_path_publisher(self):
        # Mock ROS node infrastructure only; exercise the actual constructor.
        overrides = {
            'mgeo_dir': '/synthetic/map',
            'goal_node_id': 'custom_goal',
            'global_path_topic': '/custom/global_path',
            'global_path_frame_id': 'custom_map',
        }
        declared = {}

        def declare_parameter(name, value):
            declared[name] = value

        def declare_parameters(namespace, parameters):
            declared.update(parameters)

        publisher = Mock()
        timer = Mock()
        node_methods = {
            '__init__': Mock(return_value=None),
            'declare_parameter': Mock(side_effect=declare_parameter),
            'declare_parameters': Mock(side_effect=declare_parameters),
            'get_parameter': Mock(side_effect=lambda name: SimpleNamespace(
                value=overrides.get(name, declared[name]),
            )),
            'create_publisher': publisher,
            'create_subscription': Mock(),
            'create_timer': timer,
            'get_logger': Mock(),
        }
        with patch.multiple('edgenode_planning.planning_node.Node', **node_methods):
            with patch('edgenode_planning.planning_node.MapMatcher') as matcher:
                with patch('edgenode_planning.planning_node.DijkstraPlanner') as planner:
                    node = PlanningNode()
                    matcher.assert_called_once()
                    self.assertEqual(matcher.call_args.kwargs['mgeo_dir'], '/synthetic/map')
                    planner.assert_called_once_with(mgeo_dir='/synthetic/map')
                    publisher.assert_any_call(Path, '/custom/global_path', 10)
                    timer.assert_called_once_with(0.05, node.plan)
                    self.assertEqual(declared['goal_node_id'], 'A122CC001096')
                    self.assertEqual(declared['global_path_topic'], '/planning/global_path')
                    self.assertEqual(declared['global_path_frame_id'], 'map')
                    self.assertIsNone(node._route_key)
                    self.assertIsNone(node._route_result)


if __name__ == '__main__':
    unittest.main()
