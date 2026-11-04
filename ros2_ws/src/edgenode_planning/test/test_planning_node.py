"""Check ROS message handling and zero commands without starting ROS nodes."""

import json
import math
from pathlib import Path as FilePath
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.time import Time
from std_msgs.msg import Float32, Float64, String
from std_srvs.srv import Trigger

from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.local_path_planner import LocalPathPlanner
from edgenode_planning.lattice_planner import LatticePlanner
from edgenode_planning.map_matcher import MapMatcher
from edgenode_planning.planning_fsm import PlanningFSM
from edgenode_planning.planning_node import PlanningNode
from edgenode_planning.speed_planner import SpeedPlanner
from edgenode_planning.adjacent_lane_planner import AdjacentLanePlanner
from edgenode_planning.lane_geometry import LaneNetwork
from edgenode_planning.corridor_scene import json_value, replay_scene


def speed_spy(**kwargs):
    planner = SpeedPlanner(**kwargs)
    planner.plan = Mock(wraps=planner.plan)
    return planner


class PlanningNodeTest(unittest.TestCase):
    def setUp(self):
        self.parameters = {
            'goal_node_id': 'goal_default',
            'route_node_ids': [],
            'global_path_frame_id': 'map',
            'local_path_length_m': 20.0,
            'local_path_resample_spacing_m': 2.0,
            'local_path_search_back': 3,
            'local_path_search_forward': 10,
            'cruise_speed_kmh': 1.0,
            'curve_speed_kmh': 0.7,
            'sharp_curve_speed_kmh': 0.5,
            'obstacle_points_per_cluster': 1,
            'enable_drive': False,
            'max_drive_speed_kmh': 1.0,
            'drive_duration_sec': 15.0,
            'obstacle_avoid_speed_cap_kmh': 3.0,
        }
        self.route = {'points': [[0, 1, 2], [3.5, 4.5, 5.5], [6, 7, 8]]}
        self.stamp = Time(seconds=123, nanoseconds=456).to_msg()
        self.node = SimpleNamespace(
            latest_pose=None,
            map_matcher=Mock(),
            dijkstra_planner=Mock(),
            local_path_planner=Mock(wraps=LocalPathPlanner()),
            speed_planner=speed_spy(),
            _route_key=None,
            _route_result=None,
            _progress_index=None,
            _progress_arc_m=None,
            _previous_speed_mode=None,
            fsm=PlanningFSM(),
            lattice_planner=Mock(wraps=LatticePlanner()),
            latest_obstacles=[],
            _drive_was_enabled=False,
            _drive_session_start_ns=None,
            _drive_session_expired=False,
            _drive_session_duration_sec=None,
            _drive_last_time_ns=None,
            get_parameter=lambda name: SimpleNamespace(value=self.parameters[name]),
            get_clock=Mock(),
            get_logger=Mock(),
        )
        self.node.lattice_planner.usable_curvature_limit = LatticePlanner().usable_curvature_limit
        self.node.get_clock.return_value.now.return_value.to_msg.return_value = self.stamp
        self.node.get_clock.return_value.now.return_value.nanoseconds = 123_000_000_456
        self.node.dijkstra_planner.plan.return_value = self.route
        self.node.map_matcher.match.return_value = {
            'link_id': 'link_42', 'distance': 1.25, 'heading_diff': 12.5,
        }
        for name in (
            'target_error', 'target_speed', 'state', 'current_link',
            'map_match_status', 'map_match_distance', 'map_match_heading_diff',
            'global_path', 'local_path',
            'recommended_speed', 'speed_mode', 'speed_curvature', 'speed_max_curvature',
            'drive_status', 'fsm_state', 'lattice_status', 'lattice_offset',
            'lattice_clearance', 'local_path_index', 'local_path_progress',
            'obstacle_avoid_speed_cap_kmh', 'applied_speed_cap_kmh',
            'lattice_max_curvature', 'lattice_curvature_limit',
            'lattice_obstacle_front_s', 'lattice_obstacle_rear_s', 'lattice_blocked_reason',
            'lattice_entry_length', 'lattice_full_offset_start_s', 'lattice_recovery_length',
            'runtime_cruise_speed', 'runtime_curve_speed', 'runtime_sharp_curve_speed',
            'runtime_max_drive_speed', 'actual_lateral_offset', 'last_approved_offset',
            'evaluated_candidate_offset', 'evaluated_candidate_clearance',
            'evaluated_candidate_profile_curvature', 'evaluated_candidate_polyline_curvature',
            'evaluated_candidate_reject_reason',
        ):
            setattr(self.node, name + '_pub', Mock())

    def output(self, name):
        return getattr(self.node, name + '_pub').publish.call_args.args[0].data

    def assert_stopped(self):
        self.assertEqual(self.output('target_speed'), 0.0)
        self.assertEqual(self.output('target_error'), 0.0)

    def assert_invalid_speed_preview(self):
        self.assertEqual(self.output('recommended_speed'), 0.0)
        self.assertEqual(self.output('speed_mode'), 'INVALID')
        self.assertTrue(math.isnan(self.output('speed_curvature')))
        self.assertTrue(math.isnan(self.output('speed_max_curvature')))
        self.assert_stopped()

    def assert_speed_preview(self, speed, mode, curvature):
        self.assertEqual(self.output('recommended_speed'), speed)
        self.assertEqual(self.output('speed_mode'), mode)
        self.assertAlmostEqual(self.output('speed_curvature'), curvature)
        self.assertAlmostEqual(self.output('speed_max_curvature'), curvature)
        for name in ('recommended_speed', 'speed_curvature', 'speed_max_curvature'):
            self.assertIsInstance(getattr(self.node, name + '_pub').publish.call_args.args[0], Float32)
        self.assertIsInstance(self.node.speed_mode_pub.publish.call_args.args[0], String)
        self.assertFalse(self.parameters['enable_drive'])
        self.assertEqual(self.output('drive_status'), 'DISABLED')
        self.assert_stopped()

    def set_speed_route(self, points):
        self.node.dijkstra_planner.plan.return_value = {'points': points}
        self.node.latest_pose = (points[0][0], points[0][1], 0.0)

    def advance_time(self, seconds):
        now = self.node.get_clock.return_value.now.return_value
        now.nanoseconds += round(seconds * 1e9)
        now.to_msg.return_value = Time(nanoseconds=now.nanoseconds).to_msg()

    def enable_straight_drive(self):
        self.set_speed_route([[x, 0, 0] for x in range(31)])
        self.parameters['enable_drive'] = True
        PlanningNode.plan(self.node)

    def assert_drive(self, status, speed=0.0):
        self.assertEqual(self.output('drive_status'), status)
        self.assertAlmostEqual(self.output('target_speed'), speed)
        self.assertEqual(self.output('target_error'), 0.0)

    def assert_path(self, expected_points, frame_id='map', stamp=None, topic='global_path'):
        path = getattr(self.node, topic + '_pub').publish.call_args.args[0]
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

    def test_runtime_readback_20_replaces_cached_3_5_in_every_speed_mode(self):
        for curvature, expected_mode, parameter in (
                (0.0, 'CRUISE', 'cruise_speed_kmh'),
                (0.07, 'CURVE', 'curve_speed_kmh'),
                (0.15, 'SHARP_CURVE', 'sharp_curve_speed_kmh')):
            with self.subTest(mode=expected_mode):
                self.setUp()
                self.node.speed_planner = SpeedPlanner(
                    cruise_speed_kmh=3.5, curve_speed_kmh=3.5, sharp_curve_speed_kmh=3.5)
                self.parameters.update(cruise_speed_kmh=3.5, curve_speed_kmh=3.5,
                                       sharp_curve_speed_kmh=3.5, enable_drive=True,
                                       max_drive_speed_kmh=20.0, obstacle_avoid_speed_cap_kmh=20.0)
                self.set_local_curvature(curvature or 1e-9)
                PlanningNode.plan(self.node)
                self.assertEqual(self.output('recommended_speed'), 3.5)
                self.assertEqual(self.output('target_speed'), 3.5)
                points = self.node.local_path_planner.extract.return_value['points']
                self.parameters.update(cruise_speed_kmh=20.0, curve_speed_kmh=20.0,
                                       sharp_curve_speed_kmh=20.0)
                # Reproduce the old failure before the next Planning cycle:
                # ROS readback is 20, while the cached planner still emits 3.5.
                self.assertEqual(self.node.get_parameter(parameter).value, 20.0)
                self.assertEqual(self.node.speed_planner.plan(points)['target_speed_kmh'], 3.5)
                PlanningNode.plan(self.node)
                self.assertEqual(getattr(self.node.speed_planner, parameter), 20.0)
                self.assertEqual(self.output('speed_mode'), expected_mode)
                self.assertEqual(self.output('recommended_speed'), 20.0)
                self.assertEqual(self.output('target_speed'), 20.0)
                for topic in ('runtime_cruise_speed', 'runtime_curve_speed',
                              'runtime_sharp_curve_speed', 'runtime_max_drive_speed'):
                    self.assertEqual(self.output(topic), 20.0)

    def test_grouped_footprints_preserve_cluster_boundary_and_all_collision_points(self):
        self.node._obstacle_points_per_cluster = 8
        msg = PoseArray()
        for x, y in [(14., 0.)]*8 + [(30., 7.)]*8:
            pose = Pose()
            pose.position.x, pose.position.y = x, y
            msg.poses.append(pose)
        PlanningNode.obstacle_callback(self.node, msg)
        self.assertEqual(len(self.node.latest_obstacles), 16)
        self.assertEqual([len(g) for g in self.node.latest_obstacle_groups], [8, 8])
        msg.poses[0].position.x = math.nan
        PlanningNode.obstacle_callback(self.node, msg)
        self.assertEqual(len(self.node.latest_obstacles), 15)
        self.assertIsNone(self.node.latest_obstacle_groups)

    def test_blocked_diagnostics_separate_actual_approved_and_failed_candidate(self):
        self.configure_s4_drive()
        PlanningNode.plan(self.node)
        approved = self.output('last_approved_offset')
        self.node.latest_pose = (0., -0.4, 0.)
        self.node.latest_obstacles = [(4., y*.25) for y in range(-20, 21)]
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('lattice_status'), 'BLOCKED')
        self.assertEqual(self.output('actual_lateral_offset'), -0.4)
        self.assertEqual(self.output('last_approved_offset'), approved)
        self.assertNotEqual(self.output('evaluated_candidate_offset'), -0.4)
        self.assertTrue(self.output('evaluated_candidate_reject_reason'))
        self.assertEqual(self.output('fsm_state'), 'EMERGENCY_STOP')
        self.assertEqual(self.output('target_speed'), 0.)
        self.assertEqual(self.output('applied_speed_cap_kmh'), 0.)

    def test_recorded_first_final_blocked_fixtures_keep_fsm_emergency_and_zero_target(self):
        fixture = json.loads((FilePath(__file__).parent / 'fixtures/s4_20kph_blocked_scenes.json').read_text())
        for case in fixture['cases']:
            with self.subTest(scene=case['name']):
                self.setUp()
                self.node.lattice_planner = LatticePlanner()
                self.set_speed_route(case['reference'])
                self.node.latest_pose = (*case['position'], case['yaw'])
                self.node.local_path_planner.extract.return_value = {
                    'points': case['reference'], 'nearest_index': 0, 'progress_arc_m': 100.}
                PlanningNode.plan(self.node)  # establish route key before maneuver history
                self.node.lattice_planner._avoidance_active = True
                self.node.lattice_planner._selected_offset = -2.4
                self.node.latest_obstacles = case['obstacles']
                self.node.latest_obstacle_groups = case['groups']
                self.parameters.update(enable_drive=True, max_drive_speed_kmh=20.,
                                       obstacle_avoid_speed_cap_kmh=20., cruise_speed_kmh=20.,
                                       curve_speed_kmh=20., sharp_curve_speed_kmh=20.)
                PlanningNode.plan(self.node)
                self.assertEqual(self.output('lattice_status'), 'BLOCKED')
                self.assertEqual(self.output('last_approved_offset'), -2.4)
                self.assertEqual(self.output('fsm_state'), 'EMERGENCY_STOP')
                self.assertEqual(self.output('target_speed'), 0.)
                self.assertEqual(self.output('applied_speed_cap_kmh'), 0.)

    def test_safe_hold_without_current_near_cluster_keeps_target_20_then_stops_if_unsafe(self):
        self.configure_s4_drive()
        self.node.lattice_planner = LatticePlanner()
        self.parameters.update(cruise_speed_kmh=20., curve_speed_kmh=20.,
                               sharp_curve_speed_kmh=20., max_drive_speed_kmh=20.,
                               obstacle_avoid_speed_cap_kmh=20.)
        near = [(14., -.5), (18., -.5), (18., .5), (14., .5)]
        self.node.latest_pose = (0., -.1, 0.)
        self.node.latest_obstacles = near
        self.node.latest_obstacle_groups = [near]
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('last_approved_offset'), -1.8)
        far = [(28., 7.), (30., 7.), (30., 8.), (28., 8.)]
        self.node.latest_pose = (0., -1.75, 0.)
        self.node.latest_obstacles = far
        self.node.latest_obstacle_groups = [far]
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('lattice_status'), 'AVOIDING')
        self.assertEqual(self.output('fsm_state'), 'OBSTACLE_AVOID')
        self.assertEqual(self.output('target_speed'), 20.)
        self.assertNotEqual(self.output('evaluated_candidate_reject_reason'), 'NOT_EVALUATED')
        self.assertTrue(math.isfinite(self.output('lattice_max_curvature')))
        barrier = [(4., i*.25) for i in range(-20, 21)]
        self.node.latest_obstacles = barrier
        self.node.latest_obstacle_groups = [barrier]
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('lattice_status'), 'BLOCKED')
        self.assertEqual(self.output('fsm_state'), 'EMERGENCY_STOP')
        self.assertEqual(self.output('target_speed'), 0.)
        self.assertEqual(self.output('applied_speed_cap_kmh'), 0.)
        self.assertNotEqual(self.output('evaluated_candidate_reject_reason'), 'NOT_EVALUATED')

    def configure_adjacent_drive(self, has_neighbor=True):
        self.parameters.update(enable_drive=True, cruise_speed_kmh=20., curve_speed_kmh=20.,
                               sharp_curve_speed_kmh=20., max_drive_speed_kmh=20.,
                               obstacle_avoid_speed_cap_kmh=20.)
        links={}
        for name,y in [('O',0.),('L',3.5)]:
            links[name]=dict(idx=name,from_node_idx=name+'0',to_node_idx=name+'1',max_speed=30,
                points=[[i*.5,y,0.] for i in range(161)],
                left_lane_change_dst_link_idx='L' if name=='O' and has_neighbor else None,
                right_lane_change_dst_link_idx='O' if name=='L' else None)
        geometry=SimpleNamespace(links_by_id=links,graph={'x':[{'link_id':i} for i in links]})
        self.node.lattice_planner=AdjacentLanePlanner(LaneNetwork(geometry))
        self.node.dijkstra_planner.plan.return_value={'points':links['O']['points'],'link_ids':['O']}
        self.node.map_matcher.match.return_value.update(link_id='O')
        self.node.latest_pose=(0.,0.,0.)
        self.node.latest_obstacles=[(18.,-.5),(22.,-.5),(22.,.5),(18.,.5)]
        for name in ('lane_change_state','lane_change_side','selected_adjacent_link',
                     'lane_change_obstacle_safe','lane_change_steering_feasible','lane_change_corridor_safe',
                     'lane_change_corridor_violation_m','lane_change_corridor_failed_index',
                     'lane_change_corridor_failed_point_count','lane_change_corridor_debug','lane_change_maneuver_debug',
                     'adjacent_lane_separation','original_lane_center','adjacent_lane_center',
                     'lane_change_entry_path','adjacent_lane_hold_path','lane_change_recovery_path','lane_change_recovery_gate'):
            setattr(self.node,name+'_pub',Mock())

    def test_adjacent_lane_mode_keeps_dijkstra_global_exact_and_local_is_final_control_path(self):
        self.configure_adjacent_drive()
        PlanningNode.plan(self.node)
        global_before=self.node.global_path_pub.publish.call_args.args[0]
        self.assertTrue(all(p.pose.position.y==0. for p in global_before.poses))
        local=self.node.local_path_pub.publish.call_args.args[0]
        self.assertAlmostEqual(local.poses[-1].pose.position.y,3.5)
        self.assertEqual(self.output('lane_change_state'),'ENTRY')
        self.assertTrue(self.output('lane_change_obstacle_safe'))
        self.assertTrue(self.output('lane_change_steering_feasible'))
        self.assertTrue(self.output('lane_change_corridor_safe'))
        self.assertEqual(self.output('lane_change_corridor_failed_index'), -1)
        self.assertEqual(self.output('lane_change_corridor_failed_point_count'), 0)
        self.assertEqual(self.output('lane_change_corridor_violation_m'), 0.)
        self.assertEqual(self.output('target_speed'),20.)
        self.node.latest_pose=(17.,3.5,0.)
        self.node.map_matcher.match.return_value.update(link_id='L')
        PlanningNode.plan(self.node)
        global_after=self.node.global_path_pub.publish.call_args.args[0]
        self.assertEqual([p.pose.position for p in global_before.poses],
                         [p.pose.position for p in global_after.poses])
        self.node.dijkstra_planner.plan.assert_called_once()
        self.assertEqual(self.output('lane_change_state'),'ADJACENT_LANE_HOLD')
        self.assertTrue(all(p.pose.position.y==3.5 for p in self.node.local_path_pub.publish.call_args.args[0].poses))
        self.assertEqual(self.output('runtime_cruise_speed'),20.)

    def test_corridor_failure_topics_dump_and_invalidation_use_the_same_cycle(self):
        self.configure_adjacent_drive()
        self.parameters['enable_drive'] = False
        self.node.latest_pose = (0., 0., -.12)
        PlanningNode.plan(self.node)
        debug = json.loads(self.output('lane_change_corridor_debug'))
        self.assertFalse(debug['safe'])
        self.assertEqual(self.output('lane_change_corridor_failed_index'), debug['failed_index'])
        self.assertEqual(self.output('lane_change_corridor_failed_point_count'), debug['failed_point_count'])
        self.assertAlmostEqual(self.output('lane_change_corridor_violation_m'), debug['max_violation_m'])
        self.assertEqual(self.output('target_speed'), 0.)
        self.assertFalse(self.node.local_path_pub.publish.call_args.args[0].poses)
        with tempfile.TemporaryDirectory() as directory:
            self.parameters['lane_change_corridor_dump_dir'] = directory
            response = PlanningNode.dump_lane_change_corridor_scene(self.node, Trigger.Request(), Trigger.Response())
            self.assertTrue(response.success)
            scene = json.loads(FilePath(response.message).read_text())
            self.assertEqual(scene['result']['corridor_diagnostics'], debug)
            self.assertEqual(json_value(replay_scene(scene)), scene['result'])
            self.node.latest_pose = None
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('lane_change_corridor_failed_index'), -1)
            self.assertTrue(math.isnan(self.output('lane_change_corridor_violation_m')))
            self.assertFalse(json.loads(self.output('lane_change_corridor_debug'))['evaluated'])
            response = PlanningNode.dump_lane_change_corridor_scene(self.node, Trigger.Request(), Trigger.Response())
            self.assertFalse(response.success)
            self.assertEqual(len(list(FilePath(directory).glob('*.json'))), 1)

    def test_corridor_dump_io_failure_returns_an_error_without_changing_control(self):
        self.configure_adjacent_drive()
        self.parameters['enable_drive'] = False
        self.parameters['lane_change_corridor_dump_dir'] = '/synthetic/unwritable'
        PlanningNode.plan(self.node)
        with patch('edgenode_planning.corridor_scene.Path.mkdir', side_effect=PermissionError('denied')):
            response = PlanningNode.dump_lane_change_corridor_scene(self.node, Trigger.Request(), Trigger.Response())
        self.assertFalse(response.success)
        self.assertIn('denied', response.message)
        self.assertEqual(self.output('target_speed'), 0.)

    def test_adjacent_lane_missing_neighbor_blocks_and_publishes_no_unapproved_local_path(self):
        self.configure_adjacent_drive(has_neighbor=False)
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('lattice_status'),'BLOCKED')
        self.assertEqual(self.output('target_speed'),0.)
        self.assertEqual(self.output('fsm_state'),'EMERGENCY_STOP')
        self.assertEqual(self.node.local_path_pub.publish.call_args.args[0].poses,[])
        self.assertTrue(self.node.global_path_pub.publish.call_args.args[0].poses)

    def test_footprint_pose_array_reaches_lattice_without_changing_fsm_motion_gate(self):
        self.set_speed_route([[float(x), 0.0, 0.0] for x in range(31)])
        msg = PoseArray()
        msg.header.frame_id = 'map'
        for x, y in ((14.98, -0.597), (15.5555, -0.597), (16.131, -0.597),
                     (16.131, 0.2515), (16.131, 1.1), (15.5555, 1.1),
                     (14.98, 1.1), (14.98, 0.2515)):
            pose = Pose()
            pose.position.x, pose.position.y = x, y
            msg.poses.append(pose)
        PlanningNode.obstacle_callback(self.node, msg)
        self.assertEqual(len(self.node.latest_obstacles), 8)
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('lattice_status'), 'AVOIDING')
        self.assertLessEqual(self.output('lattice_offset'), -1.8)
        self.assertGreaterEqual(self.output('lattice_clearance'), 1.2)
        self.assertEqual(self.output('fsm_state'), 'READY')
        self.assert_stopped()
        self.parameters['enable_drive'] = True
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('fsm_state'), 'OBSTACLE_AVOID')
        self.assertGreater(self.output('target_speed'), 0.0)

    def configure_s4_drive(self):
        self.set_speed_route([[float(x), 0.0, 0.0] for x in range(51)])
        self.node.speed_planner = SpeedPlanner(
            cruise_speed_kmh=50.0, curve_speed_kmh=25.0, sharp_curve_speed_kmh=3.5)
        self.parameters.update(enable_drive=True, max_drive_speed_kmh=50.0,
                               cruise_speed_kmh=50.0, curve_speed_kmh=25.0,
                               sharp_curve_speed_kmh=3.5)
        self.node.latest_obstacles = [(14.0, -0.8), (18.0, -0.8),
                                      (18.0, 0.8), (14.0, 0.8)]

    def test_s4_final_speed_cap_overrides_high_recommendation(self):
        self.configure_s4_drive()
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('fsm_state'), 'OBSTACLE_AVOID')
        self.assertGreater(self.output('recommended_speed'), 3.0)
        self.assertEqual(self.output('target_speed'), 3.0)
        self.assertEqual(self.output('obstacle_avoid_speed_cap_kmh'), 3.0)
        self.assertEqual(self.output('applied_speed_cap_kmh'), 3.0)
        self.assertAlmostEqual(self.output('lattice_curvature_limit'),
                               math.tan(.70 * .610865) / 2.84)

    def test_normal_cruise_is_not_permanently_reduced(self):
        self.configure_s4_drive()
        self.node.latest_obstacles = []
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('fsm_state'), 'NORMAL_DRIVE')
        self.assertEqual(self.output('target_speed'), 50.0)
        self.assertEqual(self.output('applied_speed_cap_kmh'), 50.0)

    def test_s4_cap_runtime_change_and_independent_lower_drive_limit(self):
        self.configure_s4_drive()
        PlanningNode.plan(self.node)
        self.parameters['obstacle_avoid_speed_cap_kmh'] = 2.0
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('target_speed'), 2.0)
        self.assertEqual(self.output('obstacle_avoid_speed_cap_kmh'), 2.0)
        self.parameters['max_drive_speed_kmh'] = 1.5
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('target_speed'), 1.5)
        self.assertEqual(self.output('applied_speed_cap_kmh'), 1.5)

    def test_s4_invalid_cap_stops_instead_of_falling_back_to_cruise(self):
        self.configure_s4_drive()
        for cap in [0.0, -1.0, float('nan'), float('inf'), True]:
            self.parameters['obstacle_avoid_speed_cap_kmh'] = cap
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('fsm_state'), 'ERROR')
            self.assert_stopped()

    def test_s4_blocked_and_release_pending_keep_zero_target(self):
        self.configure_s4_drive()
        self.node.latest_obstacles = [(14.0, i * .25) for i in range(-20, 21)]
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('fsm_state'), 'EMERGENCY_STOP')
        self.assert_stopped()
        self.node.latest_obstacles = [(14.0, -0.8), (18.0, -0.8),
                                      (18.0, 0.8), (14.0, 0.8)]
        for _ in range(2):
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('lattice_blocked_reason'), 'RELEASE_PENDING')
            self.assert_stopped()
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('fsm_state'), 'OBSTACLE_AVOID')
        self.assertEqual(self.output('target_speed'), 3.0)

    def test_s4_match_loss_breaks_blocked_release_confirmation(self):
        self.configure_s4_drive()
        self.node.latest_obstacles = [(14.0, i * .25) for i in range(-20, 21)]
        PlanningNode.plan(self.node)
        self.node.latest_obstacles = [(14.0, -.8), (18.0, -.8), (18.0, .8), (14.0, .8)]
        PlanningNode.plan(self.node)
        match = self.node.map_matcher.match.return_value
        self.node.map_matcher.match.return_value = None
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.return_value = match
        for _ in range(2):
            PlanningNode.plan(self.node)
            self.assert_stopped()
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('target_speed'), 3.0)

    def test_s4_unavailable_lattice_result_cannot_bypass_cap_and_motion_gate(self):
        self.configure_s4_drive()
        self.node.lattice_planner.plan.return_value = None
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('lattice_blocked_reason'), 'INVALID_AVOIDANCE_REFERENCE')
        self.assertEqual(self.output('fsm_state'), 'EMERGENCY_STOP')
        self.assertEqual(self.output('applied_speed_cap_kmh'), 0.0)
        self.assert_stopped()

    def test_s4_adaptive_transition_runtime_diagnostics(self):
        self.configure_s4_drive()
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('lattice_status'), 'AVOIDING')
        self.assertGreater(self.output('lattice_entry_length'), 0.0)
        self.assertLess(self.output('lattice_full_offset_start_s'),
                        self.output('lattice_obstacle_front_s'))
        self.assertLessEqual(self.output('lattice_max_curvature'),
                             self.output('lattice_curvature_limit'))
        self.assertEqual(self.output('target_speed'), 3.0)

    def test_wait_for_localization(self):
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.assert_not_called()
        self.node.dijkstra_planner.plan.assert_not_called()
        self.node.global_path_pub.publish.assert_not_called()
        self.node.local_path_pub.publish.assert_not_called()
        self.node.local_path_planner.extract.assert_not_called()
        self.assertEqual(self.output('state'), 'WAIT_FOR_LOCALIZATION')
        self.assertEqual(self.output('map_match_status'), 'WAIT_FOR_LOCALIZATION')
        self.assertEqual(self.output('current_link'), '')
        self.assertTrue(math.isnan(self.output('map_match_distance')))
        self.assertTrue(math.isnan(self.output('map_match_heading_diff')))
        self.assert_stopped()

        self.assert_invalid_speed_preview()
        self.node.speed_planner.plan.assert_not_called()

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
        self.assert_invalid_speed_preview()

    def test_initial_match_failure_does_not_plan_or_publish_path(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        self.node.map_matcher.match.return_value = None
        PlanningNode.plan(self.node)
        self.node.dijkstra_planner.plan.assert_not_called()
        self.node.global_path_pub.publish.assert_not_called()
        self.node.local_path_pub.publish.assert_not_called()
        self.node.local_path_planner.extract.assert_not_called()
        self.assertEqual(self.output('state'), 'MAP_MATCH_LOST')
        self.assertEqual(self.output('map_match_status'), 'NO_MATCH')
        self.assert_stopped()
        self.assert_invalid_speed_preview()
        self.node.speed_planner.plan.assert_not_called()

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
        self.node.local_path_pub.publish.assert_not_called()
        self.node.local_path_planner.extract.assert_not_called()
        self.assert_stopped()
        self.assert_invalid_speed_preview()
        self.node.speed_planner.plan.assert_not_called()

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
        self.node.local_path_pub.publish.assert_not_called()

    def assert_local_subdivision(self, vertices, length, frame_id='map', stamp=None):
        path = self.node.local_path_pub.publish.call_args.args[0]
        self.assertEqual(path.header.frame_id, frame_id)
        self.assertEqual(path.header.stamp, stamp or self.stamp)
        points = [[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                  for p in path.poses]
        for actual, expected in zip(points[0], vertices[0]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(points[-1], vertices[-1]):
            self.assertAlmostEqual(actual, expected)
        for vertex in vertices[1:-1]:
            self.assertIn(vertex, points)
        distances = [math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:])]
        self.assertTrue(all(0 < distance <= 2.0 + 1e-9 for distance in distances))
        self.assertAlmostEqual(sum(distances), length)
        for pose in path.poses:
            self.assertEqual(pose.header, path.header)

    def test_pose_changes_refresh_local_path_without_replanning_global_route(self):
        points = [[x, 0, x / 10.0] for x in range(0, 61, 5)]
        self.node.dijkstra_planner.plan.return_value = {'points': points}
        for x, expected in ((10.0, points[2:7]), (20.0, points[4:9]), (50.0, points[10:])):
            self.node.latest_pose = (x, 0.0, 0.0)
            PlanningNode.plan(self.node)
            self.assert_path(points)
            self.assert_local_subdivision(expected, expected[-1][0] - expected[0][0])
            self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
            self.assert_stopped()
        self.node.dijkstra_planner.plan.assert_called_once_with('link_42', 'goal_default')
        self.assertEqual(self.node.local_path_planner.extract.call_args_list, [
            call(points, x, 0.0, horizon_m=20.0, previous_index=previous,
                 search_back=3, search_forward=10, resample_spacing_m=2.0,
                 previous_arc_m=arc)
            for x, previous, arc in ((10.0, None, None), (20.0, 2, 10.0), (50.0, 4, 20.0))
        ])
        self.assertEqual(self.node.local_path_pub.publish.call_count, 3)

    def test_local_horizon_change_applies_without_global_replanning(self):
        points = [[x, 0, 1] for x in range(0, 51, 5)]
        self.node.dijkstra_planner.plan.return_value = {'points': points}
        self.node.latest_pose = (10.0, 0.0, 0.0)
        PlanningNode.plan(self.node)
        self.assert_local_subdivision(points[2:7], 20.0)
        self.parameters['local_path_length_m'] = 8.0
        PlanningNode.plan(self.node)
        self.assert_local_subdivision([points[2], points[3], [18, 0, 1]], 8.0)
        self.node.dijkstra_planner.plan.assert_called_once()
        self.assert_stopped()

    def test_invalid_local_horizon_suppresses_local_publish_and_keeps_zero_commands(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        self.parameters['local_path_length_m'] = 0.0
        PlanningNode.plan(self.node)
        self.node.global_path_pub.publish.assert_called_once()
        self.node.local_path_pub.publish.assert_not_called()
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.assert_stopped()
        self.assert_invalid_speed_preview()
        self.node.speed_planner.plan.assert_not_called()

    def test_straight_local_path_previews_cruise_but_target_speed_stays_zero(self):
        points = [[x, 0, x / 10.0] for x in range(31)]
        self.set_speed_route(points)
        PlanningNode.plan(self.node)
        self.assert_speed_preview(1.0, 'CRUISE', 0.0)
        self.assert_path(points)
        self.assert_path(points[:21], topic='local_path')
        self.node.speed_planner.plan.assert_called_once_with(points[:21], previous_speed_mode=None)
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')

    def test_gentle_local_curve_previews_curve_speed_but_stays_stopped(self):
        points = [[20 * math.sin(i * 0.1), 20 * (1 - math.cos(i * 0.1)), i]
                  for i in range(6)]
        self.set_speed_route(points)
        PlanningNode.plan(self.node)
        self.assert_speed_preview(0.7, 'CURVE', 0.05)
        self.assert_path(points)
        self.assert_path(points, topic='local_path')

    def test_sharp_local_curve_previews_sharp_speed_but_stays_stopped(self):
        points = [[5 * math.sin(i * 0.1), 5 * (1 - math.cos(i * 0.1)), i]
                  for i in range(6)]
        self.set_speed_route(points)
        PlanningNode.plan(self.node)
        self.assert_speed_preview(0.5, 'SHARP_CURVE', 0.2)
        self.assert_path(points)
        self.assert_path(points, topic='local_path')

    def test_speed_planner_failure_clears_previous_recommendation_without_changing_paths(self):
        points = [[x, 0, 0] for x in range(21)]
        self.set_speed_route(points)
        PlanningNode.plan(self.node)
        self.assert_speed_preview(1.0, 'CRUISE', 0.0)
        self.node.speed_planner.plan.return_value = None
        PlanningNode.plan(self.node)
        self.assert_invalid_speed_preview()
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.assert_path(points)
        self.assert_path(points, topic='local_path')
        self.assertEqual(self.node.global_path_pub.publish.call_count, 2)
        self.assertEqual(self.node.local_path_pub.publish.call_count, 2)
        self.node.dijkstra_planner.plan.assert_called_once()

    def test_too_short_local_path_produces_invalid_speed_preview(self):
        self.set_speed_route([[0, 0, 1], [0.1, 0, 2]])
        PlanningNode.plan(self.node)
        self.assert_invalid_speed_preview()
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.node.global_path_pub.publish.assert_called_once()
        self.node.local_path_pub.publish.assert_called_once()
        self.node.speed_planner.plan.assert_called_once()

    def test_two_original_points_with_sufficient_length_have_valid_speed_preview(self):
        self.set_speed_route([[0, 0, 1], [5, 0, 2]])
        PlanningNode.plan(self.node)
        self.assert_speed_preview(1.0, 'CRUISE', 0.0)
        self.assert_local_subdivision([[0, 0, 1], [5, 0, 2]], 5.0)

    def test_sparse_segment_at_global_index_527_keeps_driving_ready(self):
        points = [[i * 0.5, 0, 0] for i in range(528)] + [[300, 0, 0], [350, 0, 0]]
        self.set_speed_route(points)
        self.node.latest_pose = (263.5, 0, 0)
        self.node.speed_planner = speed_spy(cruise_speed_kmh=50.0)
        self.parameters.update(enable_drive=True, max_drive_speed_kmh=50.0,
                               cruise_speed_kmh=50.0, curve_speed_kmh=25.0,
                               sharp_curve_speed_kmh=3.5)
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.assertEqual(self.output('fsm_state'), PlanningFSM.NORMAL_DRIVE)
        self.assertEqual(self.output('local_path_index'), 527)
        self.assertEqual(self.node._progress_index, 527)
        self.assert_drive('ACTIVE', 50.0)
        self.assertEqual(self.output('recommended_speed'), 50.0)
        self.assertEqual(self.output('speed_mode'), 'CRUISE')
        self.assertAlmostEqual(self.output('speed_curvature'), 0.0)
        self.assert_path(points)
        self.assert_local_subdivision([[263.5, 0, 0], [283.5, 0, 0]], 20.0)
        self.assertEqual(len(self.node.local_path_pub.publish.call_args.args[0].poses), 11)

    def test_invalid_resample_spacing_suppresses_local_path_and_stops(self):
        self.set_speed_route([[0, 0, 0], [30, 0, 0]])
        self.parameters['local_path_resample_spacing_m'] = 0.0
        PlanningNode.plan(self.node)
        self.node.global_path_pub.publish.assert_called_once()
        self.node.local_path_pub.publish.assert_not_called()
        self.assert_invalid_speed_preview()
        self.assert_stopped()

    def test_resample_spacing_change_applies_without_global_replanning(self):
        self.set_speed_route([[0, 0, 0], [30, 0, 0]])
        PlanningNode.plan(self.node)
        self.assertEqual(len(self.node.local_path_pub.publish.call_args.args[0].poses), 11)
        self.parameters['local_path_resample_spacing_m'] = 0.5
        PlanningNode.plan(self.node)
        self.assertEqual(len(self.node.local_path_pub.publish.call_args.args[0].poses), 41)
        self.assertEqual(self.output('local_path_index'), 0)
        self.node.dijkstra_planner.plan.assert_called_once()

    def test_arc_progress_moves_inside_sparse_segment_without_index_change(self):
        points = [[0, 0, 0], [85, 0, 0], [120, 0, 0]]
        self.set_speed_route(points)
        for x in (5.0, 25.0, 30.0):
            self.node.latest_pose = (x, 0.0, 0.0)
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('local_path_index'), 0)
            self.assertAlmostEqual(self.node._progress_arc_m, x)
            self.assertAlmostEqual(self.output('local_path_progress'), x)
            self.assertIsInstance(self.node.local_path_progress_pub.publish.call_args.args[0], Float64)
            self.assert_local_subdivision([[x, 0, 0], [x + 20, 0, 0]], 20.0)
            self.assertEqual(self.output('speed_mode'), 'CRUISE')
        self.node.dijkstra_planner.plan.assert_called_once()

    def test_arc_progress_does_not_retreat_inside_sparse_segment_on_next_cycle(self):
        self.set_speed_route([[0, 0, 0], [85, 0, 0]])
        for x in (25.0, 24.8):
            self.node.latest_pose = (x, 0.0, 0.0)
            PlanningNode.plan(self.node)
            self.assertAlmostEqual(self.node._progress_arc_m, 25.0)
            self.assert_local_subdivision([[25, 0, 0], [45, 0, 0]], 20.0)
        self.assertEqual(self.node.local_path_planner.extract.call_args.kwargs['previous_arc_m'], 25.0)

    def test_preview_refreshes_each_pose_cycle_using_cached_global_route(self):
        points = [[x, 0, 0] for x in range(31)]
        self.set_speed_route(points)
        for x in (0.0, 1.0, 2.0):
            self.node.latest_pose = (x, 0.0, 0.0)
            PlanningNode.plan(self.node)
            self.assert_speed_preview(1.0, 'CRUISE', 0.0)
        self.node.dijkstra_planner.plan.assert_called_once()
        self.assertEqual(self.node.speed_planner.plan.call_args_list, [
            call(points[:21], previous_speed_mode=None),
            call(points[1:22], previous_speed_mode='CRUISE'),
            call(points[2:23], previous_speed_mode='CRUISE'),
        ])
        for name in ('global_path', 'local_path', 'recommended_speed', 'speed_mode',
                     'speed_curvature', 'speed_max_curvature'):
            self.assertEqual(getattr(self.node, name + '_pub').publish.call_count, 3)

    def test_match_loss_and_failed_route_clear_valid_speed_preview(self):
        points = [[x, 0, 0] for x in range(21)]
        self.set_speed_route(points)
        PlanningNode.plan(self.node)
        self.assert_speed_preview(1.0, 'CRUISE', 0.0)
        match = self.node.map_matcher.match.return_value
        self.node.map_matcher.match.return_value = None
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('state'), 'MAP_MATCH_LOST')
        self.assert_invalid_speed_preview()
        self.node.global_path_pub.publish.assert_called_once()
        self.node.local_path_pub.publish.assert_called_once()
        self.node.speed_planner.plan.assert_called_once()
        self.node.map_matcher.match.return_value = match
        self.parameters['goal_node_id'] = 'unreachable'
        self.node.dijkstra_planner.plan.return_value = None
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_NOT_FOUND')
        self.assert_invalid_speed_preview()
        self.node.global_path_pub.publish.assert_called_once()
        self.node.local_path_pub.publish.assert_called_once()
        self.node.speed_planner.plan.assert_called_once()

    def test_invalid_speed_parameters_fail_closed_with_paths_still_available(self):
        self.set_speed_route([[x, 0, 0] for x in range(21)])
        self.parameters['cruise_speed_kmh'] = -1.0
        self.node.speed_planner = SpeedPlanner(cruise_speed_kmh=-1.0)
        PlanningNode.plan(self.node)
        self.assert_invalid_speed_preview()
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.node.global_path_pub.publish.assert_called_once()
        self.node.local_path_pub.publish.assert_called_once()

    def test_speed_preview_uses_identical_numeric_list_from_local_extraction(self):
        points = [[x, 0, x] for x in range(31)]
        local_points = points[5:10]
        self.set_speed_route(points)
        self.node.local_path_planner.extract.return_value = {
            'points': local_points, 'nearest_index': 5, 'progress_arc_m': 5.0,
        }
        self.node.lattice_planner.plan.return_value = None
        PlanningNode.plan(self.node)
        self.assertIs(self.node.speed_planner.plan.call_args.args[0], local_points)
        self.assert_speed_preview(1.0, 'CRUISE', 0.0)
        self.assert_path(local_points, topic='local_path')

    def test_representative_and_maximum_curvature_publish_distinct_diagnostics(self):
        points = [[i * 0.25, 0, 0] for i in range(31)]
        points[15][1] = 1.0
        self.set_speed_route(points)
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('speed_mode'), 'CRUISE')
        self.assertEqual(self.output('recommended_speed'), 1.0)
        self.assertEqual(self.output('speed_curvature'), 0.0)
        self.assertGreater(self.output('speed_max_curvature'), 0.10)
        self.assert_stopped()

    def test_drive_activation_straight_is_active_with_one_kmh_target(self):
        self.enable_straight_drive()
        self.assert_drive('ACTIVE', 1.0)
        self.assertEqual(self.output('recommended_speed'), 1.0)
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.assertEqual(self.node._drive_session_start_ns, 123_000_000_456)

    def test_drive_activation_gentle_curve_targets_point_seven_kmh(self):
        points = [[20 * math.sin(i * 0.1), 20 * (1 - math.cos(i * 0.1)), i]
                  for i in range(6)]
        self.set_speed_route(points)
        self.parameters['enable_drive'] = True
        PlanningNode.plan(self.node)
        self.assert_drive('ACTIVE', 0.7)
        self.assertEqual(self.output('speed_mode'), 'CURVE')

    def test_drive_activation_sharp_curve_targets_point_five_kmh(self):
        points = [[5 * math.sin(i * 0.1), 5 * (1 - math.cos(i * 0.1)), i]
                  for i in range(6)]
        self.set_speed_route(points)
        self.parameters['enable_drive'] = True
        PlanningNode.plan(self.node)
        self.assert_drive('ACTIVE', 0.5)
        self.assertEqual(self.output('speed_mode'), 'SHARP_CURVE')

    def test_drive_hard_cap_is_independent_of_recommended_speed(self):
        self.parameters['cruise_speed_kmh'] = 3.0
        self.node.speed_planner = SpeedPlanner(cruise_speed_kmh=3.0)
        self.enable_straight_drive()
        for limit in (1.0, 0.4):
            with self.subTest(limit=limit):
                self.parameters['max_drive_speed_kmh'] = limit
                PlanningNode.plan(self.node)
                self.assert_drive('ACTIVE', limit)
                self.assertEqual(self.output('recommended_speed'), 3.0)
                self.assertLessEqual(self.output('target_speed'), limit)

    def test_drive_map_match_loss_is_not_ready_and_recovers_with_same_session(self):
        self.enable_straight_drive()
        start = self.node._drive_session_start_ns
        match = self.node.map_matcher.match.return_value
        self.node.map_matcher.match.return_value = None
        self.advance_time(5.0)
        PlanningNode.plan(self.node)
        self.assert_drive('NOT_READY')
        self.assertEqual(self.output('state'), 'MAP_MATCH_LOST')
        self.assert_invalid_speed_preview()
        self.node.map_matcher.match.return_value = match
        self.advance_time(1.0)
        PlanningNode.plan(self.node)
        self.assert_drive('ACTIVE', 1.0)
        self.assertEqual(self.node._drive_session_start_ns, start)
        self.node.dijkstra_planner.plan.assert_called_once()

    def test_drive_without_localization_is_not_ready(self):
        self.parameters['enable_drive'] = True
        PlanningNode.plan(self.node)
        self.assert_drive('NOT_READY')
        self.assertEqual(self.output('state'), 'WAIT_FOR_LOCALIZATION')
        self.node.speed_planner.plan.assert_not_called()

    def test_drive_without_global_route_is_not_ready(self):
        self.enable_straight_drive()
        self.parameters['goal_node_id'] = 'unreachable'
        self.node.dijkstra_planner.plan.return_value = None
        PlanningNode.plan(self.node)
        self.assert_drive('NOT_READY')
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_NOT_FOUND')

    def test_drive_local_path_failure_is_not_ready(self):
        self.enable_straight_drive()
        self.parameters['local_path_length_m'] = 0.0
        PlanningNode.plan(self.node)
        self.assert_drive('NOT_READY')
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')
        self.node.speed_planner.plan.assert_called_once()

    def test_drive_speed_planner_none_is_not_ready(self):
        self.enable_straight_drive()
        self.node.speed_planner.plan.return_value = None
        PlanningNode.plan(self.node)
        self.assert_drive('NOT_READY')
        self.assert_invalid_speed_preview()
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')

    def test_drive_invalid_recommended_speed_is_not_ready_and_never_published_as_command(self):
        self.enable_straight_drive()
        valid = SpeedPlanner().plan([[x, 0, 0] for x in range(21)])
        for speed in (math.nan, math.inf, -math.inf, -1.0, None, True, 10 ** 400):
            with self.subTest(speed=speed):
                self.node.speed_planner.plan.return_value = {**valid, 'target_speed_kmh': speed}
                PlanningNode.plan(self.node)
                self.assert_drive('NOT_READY')
                self.assert_invalid_speed_preview()

    def test_drive_is_active_before_lease_and_expired_at_fifteen_seconds(self):
        self.enable_straight_drive()
        self.advance_time(14.999)
        PlanningNode.plan(self.node)
        self.assert_drive('ACTIVE', 1.0)
        self.advance_time(0.001)
        PlanningNode.plan(self.node)
        self.assert_drive('EXPIRED')
        self.assertEqual(self.output('recommended_speed'), 1.0)
        self.assertEqual(self.output('speed_mode'), 'CRUISE')

    def test_expired_drive_stays_expired_while_true_and_after_duration_increase(self):
        self.enable_straight_drive()
        start = self.node._drive_session_start_ns
        self.advance_time(15.0)
        PlanningNode.plan(self.node)
        self.assert_drive('EXPIRED')
        self.parameters['drive_duration_sec'] = 30.0
        for seconds in (1.0, 1.0, 20.0):
            self.advance_time(seconds)
            PlanningNode.plan(self.node)
            self.assert_drive('EXPIRED')
            self.assertEqual(self.node._drive_session_start_ns, start)

    def test_false_then_true_resets_and_starts_new_drive_session(self):
        self.enable_straight_drive()
        start = self.node._drive_session_start_ns
        self.advance_time(16.0)
        PlanningNode.plan(self.node)
        self.assert_drive('EXPIRED')
        self.parameters['enable_drive'] = False
        PlanningNode.plan(self.node)
        self.assert_drive('DISABLED')
        self.assertIsNone(self.node._drive_session_start_ns)
        self.assertFalse(self.node._drive_session_expired)
        self.assertIsNone(self.node._drive_session_duration_sec)
        self.assertEqual(self.output('recommended_speed'), 1.0)
        self.parameters['enable_drive'] = True
        PlanningNode.plan(self.node)
        self.assert_drive('ACTIVE', 1.0)
        self.assertEqual(self.node._drive_session_start_ns, start + 16_000_000_000)

    def test_disabling_active_drive_stops_immediately_and_keeps_preview(self):
        self.enable_straight_drive()
        self.parameters['enable_drive'] = False
        PlanningNode.plan(self.node)
        self.assert_drive('DISABLED')
        self.assertEqual(self.output('recommended_speed'), 1.0)
        self.assertEqual(self.output('speed_mode'), 'CRUISE')

    def test_live_duration_increase_does_not_extend_session_and_decrease_expires_it(self):
        self.enable_straight_drive()
        self.advance_time(5.0)
        self.parameters['drive_duration_sec'] = 30.0
        PlanningNode.plan(self.node)
        self.assert_drive('ACTIVE', 1.0)
        self.advance_time(10.0)
        PlanningNode.plan(self.node)
        self.assert_drive('EXPIRED')
        self.parameters['enable_drive'] = False
        PlanningNode.plan(self.node)
        self.parameters['enable_drive'] = True
        self.parameters['drive_duration_sec'] = 15.0
        PlanningNode.plan(self.node)
        self.advance_time(5.0)
        self.parameters['drive_duration_sec'] = 4.0
        PlanningNode.plan(self.node)
        self.assert_drive('EXPIRED')

    def test_invalid_drive_duration_and_max_speed_fail_closed(self):
        self.enable_straight_drive()
        for name in ('max_drive_speed_kmh', 'drive_duration_sec'):
            for value in (0.0, -1.0, math.nan, math.inf, -math.inf,
                          None, 'bad', True, [], 10 ** 400):
                with self.subTest(name=name, value=value):
                    self.parameters['max_drive_speed_kmh'] = 1.0
                    self.parameters['drive_duration_sec'] = 15.0
                    self.parameters[name] = value
                    PlanningNode.plan(self.node)
                    self.assert_drive('INVALID_CONFIG')
                    self.assertEqual(self.output('recommended_speed'), 1.0)
                    self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')

    def test_drive_session_starts_at_enablement_even_while_not_ready(self):
        self.parameters['enable_drive'] = True
        PlanningNode.plan(self.node)
        self.assert_drive('NOT_READY')
        start = self.node._drive_session_start_ns
        self.advance_time(16.0)
        self.set_speed_route([[x, 0, 0] for x in range(21)])
        PlanningNode.plan(self.node)
        self.assert_drive('EXPIRED')
        self.assertEqual(self.node._drive_session_start_ns, start)

    def test_invalid_config_recovery_never_restarts_lease(self):
        self.parameters['enable_drive'] = True
        self.parameters['drive_duration_sec'] = -1.0
        PlanningNode.plan(self.node)
        self.assert_drive('INVALID_CONFIG')
        self.advance_time(16.0)
        self.parameters['drive_duration_sec'] = 15.0
        self.set_speed_route([[x, 0, 0] for x in range(21)])
        PlanningNode.plan(self.node)
        self.assert_drive('EXPIRED')

    def test_clock_rewind_expires_drive_until_false_then_true(self):
        self.enable_straight_drive()
        self.advance_time(5.0)
        PlanningNode.plan(self.node)
        self.assert_drive('ACTIVE', 1.0)
        # Rewind remains after the original start, but must still stop the lease.
        self.advance_time(-1.0)
        PlanningNode.plan(self.node)
        self.assert_drive('EXPIRED')
        self.advance_time(2.0)
        PlanningNode.plan(self.node)
        self.assert_drive('EXPIRED')

    def test_drive_session_start_at_ros_time_zero_is_valid(self):
        now = self.node.get_clock.return_value.now.return_value
        now.nanoseconds = 0
        now.to_msg.return_value = Time(nanoseconds=0).to_msg()
        self.enable_straight_drive()
        self.assert_drive('ACTIVE', 1.0)
        self.assertEqual(self.node._drive_session_start_ns, 0)
        self.advance_time(15.0)
        PlanningNode.plan(self.node)
        self.assert_drive('EXPIRED')

    def test_valid_zero_recommendation_does_not_invent_positive_speed(self):
        self.parameters['cruise_speed_kmh'] = 0.0
        self.node.speed_planner = SpeedPlanner(cruise_speed_kmh=0.0)
        self.enable_straight_drive()
        self.assert_drive('ACTIVE', 0.0)
        self.assertEqual(self.output('recommended_speed'), 0.0)

    def test_link_change_retains_fixed_route_and_progress(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.return_value['link_id'] = 'link_43'
        next_points = [[3.5, 4.5, 5.5], [6, 7, 8]]
        self.node.dijkstra_planner.plan.return_value = {'points': next_points}
        PlanningNode.plan(self.node)
        self.assertEqual(self.node.dijkstra_planner.plan.call_args_list, [
            call('link_42', 'goal_default'),
        ])
        self.assert_path(self.route['points'])
        self.assertEqual(self.node._progress_index, 1)
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
        self.node.local_path_pub.publish.reset_mock()
        self.parameters['goal_node_id'] = 'unreachable'
        self.node.dijkstra_planner.plan.return_value = None
        for _ in range(3):
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('state'), 'GLOBAL_PATH_NOT_FOUND')
            self.assert_stopped()
        self.assertEqual(self.node.dijkstra_planner.plan.call_count, 2)
        self.node.global_path_pub.publish.assert_not_called()
        self.node.local_path_pub.publish.assert_not_called()

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
        self.node.local_path_pub.publish.assert_called_once()
        self.node.local_path_planner.extract.assert_called_once()
        self.node.map_matcher.match.return_value = match
        PlanningNode.plan(self.node)
        self.node.dijkstra_planner.plan.assert_called_once()
        self.assertEqual(self.node.global_path_pub.publish.call_count, 2)
        self.assertEqual(self.node.local_path_pub.publish.call_count, 2)
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
        self.assert_local_subdivision([[4, 5, 6], self.route['points'][2]],
                                      math.hypot(2, 2),
                                      frame_id='mgeo_map', stamp=next_stamp)
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
            self.assert_path([[0, 0, 1], [10, 0, 2], [20, 0, 3]])
            self.assert_local_subdivision([[15, 0, 2.5], [20, 0, 3]], 5.0)
            self.assertEqual(self.node.dijkstra_planner.plan.call_args_list, [
                call('AB', 'C'),
            ])
            self.assert_stopped()

    def test_constructor_shares_map_and_creates_configured_path_publisher(self):
        # Mock ROS node infrastructure only; exercise the actual constructor.
        overrides = {
            'obstacle_avoidance_mode': 'legacy_offset',
            'mgeo_dir': '/synthetic/map',
            'goal_node_id': 'custom_goal',
            'global_path_topic': '/custom/global_path',
            'local_path_topic': '/custom/local_path',
            'local_path_length_m': 12.0,
            'global_path_frame_id': 'custom_map',
            'enable_drive': False,
            'cruise_speed_kmh': 0.9,
            'curve_speed_kmh': 0.6,
            'sharp_curve_speed_kmh': 0.3,
            'speed_preview_distance_m': 5.0,
            'curve_curvature_threshold': 0.03,
            'sharp_curvature_threshold': 0.08,
            'speed_curvature_smoothing_window_m': 1.5,
            'speed_curvature_persistence_m': 2.5,
            'speed_curvature_hysteresis': 0.003,
            'recommended_speed_topic': '/custom/recommended_speed',
            'speed_mode_topic': '/custom/speed_mode',
            'speed_curvature_topic': '/custom/speed_curvature',
            'speed_max_curvature_topic': '/custom/speed_max_curvature',
            'drive_status_topic': '/custom/drive_status',
            'ego_half_width_m': 1.0,
            'obstacle_safety_margin_m': 0.3,
            'lattice_clearance_m': 1.1,
            'lattice_candidate_offsets_m': [0.0, 1.5, -1.5, 2.4, -2.4],
            'lattice_transition_after_m': 8.0,
        }
        declared = {}

        def declare_parameter(name, value):
            declared[name] = value

        def declare_parameters(namespace, parameters):
            declared.update((item[0], item[1]) for item in parameters)

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
            'create_service': Mock(),
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
                    publisher.assert_any_call(Path, '/custom/local_path', 10)
                    publisher.assert_any_call(Float32, '/custom/recommended_speed', 10)
                    publisher.assert_any_call(String, '/custom/speed_mode', 10)
                    publisher.assert_any_call(Float32, '/custom/speed_curvature', 10)
                    publisher.assert_any_call(Float32, '/custom/speed_max_curvature', 10)
                    publisher.assert_any_call(String, '/custom/drive_status', 10)
                    publisher.assert_any_call(Float64, '/planning/local_path_progress_m', 10)
                    timer.assert_called_once_with(0.05, node.plan)
                    self.assertEqual(declared['goal_node_id'], 'A119BS010245')
                    self.assertEqual(declared['route_node_ids'], [])
                    self.assertEqual(declared['local_path_search_back'], 3)
                    self.assertEqual(declared['local_path_search_forward'], 10)
                    self.assertIsNone(node._progress_index)
                    self.assertIsNone(node._progress_arc_m)
                    self.assertEqual(declared['global_path_topic'], '/planning/global_path')
                    self.assertEqual(declared['global_path_frame_id'], 'map')
                    self.assertEqual(declared['local_path_topic'], '/planning/local_path')
                    self.assertEqual(declared['local_path_length_m'], 20.0)
                    self.assertEqual(declared['local_path_resample_spacing_m'], 2.0)
                    self.assertEqual(declared['ego_half_width_m'], 0.95)
                    self.assertEqual(declared['obstacle_safety_margin_m'], 0.25)
                    self.assertEqual(declared['lattice_transition_after_m'], 10.0)
                    self.assertEqual(node.lattice_planner.required_clearance_m, 1.3)
                    self.assertEqual(node.lattice_planner.transition_after_m, 8.0)
                    self.assertEqual(node.lattice_planner.candidate_offsets_m,
                                     (0.0, 1.5, -1.5, 2.4, -2.4))
                    self.assertIsInstance(node.local_path_planner, LocalPathPlanner)
                    self.assertIsInstance(node.speed_planner, SpeedPlanner)
                    self.assertFalse(declared['enable_drive'])
                    self.assertEqual(declared['max_drive_speed_kmh'], 1.0)
                    self.assertEqual(declared['drive_duration_sec'], 15.0)
                    self.assertEqual(declared['drive_status_topic'], '/planning/drive_status')
                    self.assertFalse(node._drive_was_enabled)
                    self.assertIsNone(node._drive_session_start_ns)
                    self.assertFalse(node._drive_session_expired)
                    self.assertIsNone(node._drive_session_duration_sec)
                    self.assertIsNone(node._drive_last_time_ns)
                    self.assertEqual(declared['cruise_speed_kmh'], 1.0)
                    self.assertEqual(declared['curve_speed_kmh'], 0.7)
                    self.assertEqual(declared['sharp_curve_speed_kmh'], 0.5)
                    self.assertEqual(declared['speed_preview_distance_m'], 10.0)
                    self.assertEqual(declared['curve_curvature_threshold'], 0.04)
                    self.assertEqual(declared['sharp_curvature_threshold'], 0.10)
                    self.assertEqual(declared['speed_curvature_smoothing_window_m'], 2.0)
                    self.assertEqual(declared['speed_curvature_persistence_m'], 2.0)
                    self.assertEqual(declared['speed_curvature_hysteresis'], 0.005)
                    self.assertEqual(node.speed_planner.curvature_smoothing_window_m, 1.5)
                    self.assertEqual(node.speed_planner.curvature_persistence_m, 2.5)
                    self.assertEqual(node.speed_planner.curvature_hysteresis, 0.003)
                    self.assertIsNone(node._previous_speed_mode)
                    for name in ('recommended_speed', 'speed_mode', 'speed_curvature', 'speed_max_curvature'):
                        self.assertEqual(declared[name + '_topic'], '/planning/' + name)
                    for name in ('cruise_speed_kmh', 'curve_speed_kmh', 'sharp_curve_speed_kmh',
                                 'curve_curvature_threshold', 'sharp_curvature_threshold'):
                        self.assertEqual(getattr(node.speed_planner, name), overrides[name])
                    self.assertEqual(node.speed_planner.preview_distance_m, 5.0)
                    self.assertIsNone(node._route_key)
                    self.assertIsNone(node._route_result)

    def use_waypoint_map(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        nodes = [{'idx': name} for name in ('A', 'B', 'C', 'D', 'E')]
        links = []
        for link_id, start, end, first, last in (
                ('AB', 'A', 'B', 0, 10), ('BC', 'B', 'C', 10, 20),
                ('CD', 'C', 'D', 20, 30)):
            links.append({
                'idx': link_id, 'from_node_idx': start, 'to_node_idx': end,
                'points': [[x, 0, 0] for x in range(first, last + 1)],
                'link_length': 10.0,
            })
        for filename, data in [('node_set.json', nodes), ('link_set.json', links)]:
            (FilePath(directory.name) / filename).write_text(json.dumps(data), encoding='utf-8')
        self.node.dijkstra_planner = Mock(wraps=DijkstraPlanner(directory.name))
        self.node.map_matcher.match.return_value['link_id'] = 'AB'
        self.parameters['route_node_ids'] = ['C', 'D']
        self.node.latest_pose = (2.0, 0.0, 0.0)

    def test_waypoint_route_ignores_link_fluctuations_and_legacy_goal_changes(self):
        self.use_waypoint_map()
        PlanningNode.plan(self.node)
        route = self.node._route_result
        self.assertEqual(route['link_ids'], ['AB', 'BC', 'CD'])
        for link in ('BC', 'CD', 'AB'):
            self.node.map_matcher.match.return_value['link_id'] = link
            self.parameters['goal_node_id'] = 'ignored_' + link
            PlanningNode.plan(self.node)
            self.assertIs(self.node._route_result, route)
            self.assert_path(route['points'])
        self.node.dijkstra_planner.plan_route.assert_called_once_with('AB', ['C', 'D'])
        self.node.dijkstra_planner.plan.assert_not_called()

    def test_waypoint_change_rebuilds_once_and_resets_progress(self):
        self.use_waypoint_map()
        self.node.latest_pose = (25.0, 0.0, 0.0)
        PlanningNode.plan(self.node)
        self.assertEqual(self.node._progress_index, 25)
        # In-place list mutation must be detected too. The new route is shorter.
        self.parameters['route_node_ids'][:] = ['C']
        self.node.latest_pose = (5.0, 0.0, 0.0)
        PlanningNode.plan(self.node)
        self.assertEqual(self.node._progress_index, 5)
        self.assertEqual(self.output('local_path_index'), 5)
        self.assertEqual(self.node._route_result['goal_node_id'], 'C')
        self.assertIsNone(self.node.local_path_planner.extract.call_args.kwargs['previous_index'])
        self.assertIsNone(self.node.local_path_planner.extract.call_args.kwargs['previous_arc_m'])
        self.assertAlmostEqual(self.node._progress_arc_m, 5.0)
        PlanningNode.plan(self.node)
        self.assertEqual(self.node.dijkstra_planner.plan_route.call_count, 2)

    def test_goal_change_resets_progress_and_can_initialize_in_middle(self):
        self.set_speed_route([[x, 0, 0] for x in range(100)])
        self.node.latest_pose = (70.0, 0.0, 0.0)
        PlanningNode.plan(self.node)
        self.assertEqual(self.node._progress_index, 70)
        self.parameters['goal_node_id'] = 'new_goal'
        self.set_speed_route([[x, 0, 0] for x in range(31)])
        self.node.latest_pose = (12.0, 0.0, 0.0)
        PlanningNode.plan(self.node)
        self.assertEqual(self.node._progress_index, 12)
        self.assertAlmostEqual(self.node._progress_arc_m, 12.0)
        self.assertIsNone(self.node.local_path_planner.extract.call_args.kwargs['previous_arc_m'])

    def test_progress_survives_match_loss_and_noise_without_full_reacquisition(self):
        self.set_speed_route([[x, 0, 0] for x in range(100)])
        self.node.latest_pose = (20.0, 0.0, 0.0)
        PlanningNode.plan(self.node)
        match = self.node.map_matcher.match.return_value
        self.node.map_matcher.match.return_value = None
        PlanningNode.plan(self.node)
        self.assertEqual(self.node._progress_index, 20)
        self.assertEqual(self.output('local_path_index'), -1)
        self.assertAlmostEqual(self.node._progress_arc_m, 20.0)
        self.assertTrue(math.isnan(self.output('local_path_progress')))
        self.node.map_matcher.match.return_value = match
        self.node.latest_pose = (19.2, 0.0, 0.0)
        PlanningNode.plan(self.node)
        self.assertEqual(self.node._progress_index, 20)
        self.node.latest_pose = (90.0, 0.0, 0.0)
        PlanningNode.plan(self.node)
        self.assertEqual(self.node._progress_index, 30)
        self.node.dijkstra_planner.plan.assert_called_once()

    def test_route_change_during_match_loss_discards_old_route_and_progress(self):
        self.use_waypoint_map()
        PlanningNode.plan(self.node)
        match = self.node.map_matcher.match.return_value
        self.node.map_matcher.match.return_value = None
        self.parameters['route_node_ids'] = ['C']
        PlanningNode.plan(self.node)
        self.assertIsNone(self.node._route_result)
        self.assertIsNone(self.node._progress_index)
        self.assertIsNone(self.node._progress_arc_m)
        self.node.map_matcher.match.return_value = match
        PlanningNode.plan(self.node)
        self.assertEqual(self.node.dijkstra_planner.plan_route.call_count, 2)
        self.assertEqual(self.node._route_result['goal_node_id'], 'C')

    def test_empty_waypoints_switch_back_to_legacy_goal_and_reset_progress(self):
        self.use_waypoint_map()
        PlanningNode.plan(self.node)
        self.parameters['route_node_ids'] = []
        self.parameters['goal_node_id'] = 'C'
        PlanningNode.plan(self.node)
        self.node.dijkstra_planner.plan.assert_called_once_with('AB', 'C')
        self.assertEqual(self.node._route_result['goal_node_id'], 'C')
        self.assertIsNone(self.node.local_path_planner.extract.call_args.kwargs['previous_index'])

    def test_yaml_empty_array_not_set_uses_legacy_goal(self):
        # Humble's YAML parser represents route_node_ids: [] as NOT_SET/None.
        self.parameters['route_node_ids'] = None
        self.node.latest_pose = (12.0, -3.0, 0.0)
        PlanningNode.plan(self.node)
        self.node.dijkstra_planner.plan.assert_called_once_with('link_42', 'goal_default')
        self.node.dijkstra_planner.plan_route.assert_not_called()
        self.assertEqual(self.output('state'), 'GLOBAL_PATH_READY')

    def test_invalid_or_unreachable_waypoint_stops_and_caches_failure(self):
        self.use_waypoint_map()
        self.parameters['enable_drive'] = True
        PlanningNode.plan(self.node)
        self.assert_drive('ACTIVE', 1.0)
        for waypoints in (['C', 'missing'], ['C', 'E'], ['C', 'B'], 'C', [1]):
            with self.subTest(waypoints=waypoints):
                self.parameters['route_node_ids'] = waypoints
                self.node.global_path_pub.publish.reset_mock()
                self.node.local_path_pub.publish.reset_mock()
                count = self.node.dijkstra_planner.plan_route.call_count
                for link in ('AB', 'BC', 'CD'):
                    self.node.map_matcher.match.return_value['link_id'] = link
                    PlanningNode.plan(self.node)
                    self.assertEqual(self.output('state'), 'GLOBAL_PATH_NOT_FOUND')
                    self.assert_drive('NOT_READY')
                    self.assertEqual(self.output('local_path_index'), -1)
                self.assertEqual(self.node.dijkstra_planner.plan_route.call_count, count + 1)
                self.node.global_path_pub.publish.assert_not_called()
                self.node.local_path_pub.publish.assert_not_called()

    def test_path_end_holds_last_index_and_stops_when_geometry_is_too_short(self):
        self.use_waypoint_map()
        self.parameters['enable_drive'] = True
        self.node.latest_pose = (30.0, 0.0, 0.0)
        for x in (30.0, 29.0, 31.0):
            self.node.latest_pose = (x, 0.0, 0.0)
            PlanningNode.plan(self.node)
            self.assertEqual(self.node._progress_index, 30)
            self.assert_path([[30, 0, 0]], topic='local_path')
            self.assert_drive('NOT_READY')

    def set_local_curvature(self, curvature):
        radius = 1.0 / curvature
        points = [[radius * math.sin(curvature * i * 0.5),
                   radius * (1.0 - math.cos(curvature * i * 0.5)), 0]
                  for i in range(26)]
        self.node.latest_pose = (0.0, 0.0, 0.0)
        self.node.local_path_planner.extract.return_value = {
            'points': points, 'nearest_index': 0, 'progress_arc_m': 0.0,
        }
        self.node.lattice_planner.plan.return_value = None

    def test_speed_mode_hysteresis_is_carried_between_planning_cycles(self):
        self.node.speed_planner = speed_spy(
            cruise_speed_kmh=10.0, curve_speed_kmh=5.0, sharp_curve_speed_kmh=3.5)
        self.parameters['enable_drive'] = True
        self.parameters.update(max_drive_speed_kmh=10.0, cruise_speed_kmh=10.0,
                               curve_speed_kmh=5.0, sharp_curve_speed_kmh=3.5)
        for curvature, mode, speed in (
                (0.0395, 'CRUISE', 10.0), (0.0405, 'CURVE', 5.0),
                (0.0395, 'CURVE', 5.0), (0.0405, 'CURVE', 5.0),
                (0.0395, 'CURVE', 5.0), (0.034, 'CRUISE', 10.0)):
            self.set_local_curvature(curvature)
            PlanningNode.plan(self.node)
            self.assertEqual(self.output('speed_mode'), mode)
            self.assert_drive('ACTIVE', speed)
        self.assertEqual([c.kwargs['previous_speed_mode'] for c in self.node.speed_planner.plan.call_args_list],
                         [None, 'CRUISE', 'CURVE', 'CURVE', 'CURVE', 'CURVE'])
        self.node.dijkstra_planner.plan.assert_called_once()

    def test_match_loss_resets_only_speed_hysteresis_and_recovers_without_replan(self):
        self.set_local_curvature(0.041)
        PlanningNode.plan(self.node)
        self.assertEqual(self.node._previous_speed_mode, 'CURVE')
        match = self.node.map_matcher.match.return_value
        self.node.map_matcher.match.return_value = None
        PlanningNode.plan(self.node)
        self.assertIsNone(self.node._previous_speed_mode)
        self.assert_invalid_speed_preview()
        self.node.map_matcher.match.return_value = match
        self.set_local_curvature(0.039)
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('speed_mode'), 'CRUISE')
        self.node.dijkstra_planner.plan.assert_called_once()

    def test_route_change_does_not_carry_old_curve_hysteresis(self):
        self.set_local_curvature(0.041)
        PlanningNode.plan(self.node)
        self.assertEqual(self.node._previous_speed_mode, 'CURVE')
        self.parameters['goal_node_id'] = 'new_speed_test_goal'
        self.set_local_curvature(0.039)
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('speed_mode'), 'CRUISE')
        self.assertEqual(self.node.dijkstra_planner.plan.call_count, 2)

    def test_invalid_speed_geometry_resets_hysteresis_before_recovery(self):
        self.set_local_curvature(0.041)
        PlanningNode.plan(self.node)
        self.node.local_path_planner.extract.return_value = {
            'points': [[0, 0, 0]], 'nearest_index': 0, 'progress_arc_m': 0.0,
        }
        PlanningNode.plan(self.node)
        self.assertIsNone(self.node._previous_speed_mode)
        self.assert_invalid_speed_preview()
        self.set_local_curvature(0.039)
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('speed_mode'), 'CRUISE')


if __name__ == '__main__':
    unittest.main()
