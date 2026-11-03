"""Real ROS parameter storage with manual Planning cycles; all topics isolated."""

import math
import uuid
from unittest.mock import Mock, patch

import rclpy
from rclpy.parameter import Parameter

from edgenode_planning.planning_node import PlanningNode


def test_accepted_ros_parameter_change_reaches_actual_sharp_speed_next_cycle(route_editor_map):
    prefix = '/runtime_speed_test_' + uuid.uuid4().hex[:8]
    args = ['--ros-args', '-r', '__ns:=' + prefix,
            '-p', f'mgeo_dir:={route_editor_map}', '-p', 'goal_node_id:=D',
            '-p', 'cruise_speed_kmh:=3.5', '-p', 'curve_speed_kmh:=3.5',
            '-p', 'sharp_curve_speed_kmh:=3.5']
    # Absolute project topics must be remapped explicitly; no live input/output.
    topics = ('target_error', 'target_speed', 'state', 'current_link', 'map_match_status',
              'map_match_distance', 'map_match_heading_diff', 'global_path', 'local_path',
              'local_path_index', 'local_path_progress_m', 'recommended_speed', 'speed_mode',
              'speed_curvature', 'speed_max_curvature', 'drive_status', 'fsm_state',
              'lattice_status', 'lattice_offset', 'lattice_clearance', 'lattice_blocked_reason',
              'lattice_max_curvature', 'lattice_curvature_limit', 'lattice_obstacle_front_s',
              'lattice_obstacle_rear_s', 'lattice_entry_length', 'lattice_full_offset_start_s',
              'lattice_recovery_length', 'obstacle_avoid_speed_cap_kmh', 'applied_speed_cap_kmh',
              'runtime_cruise_speed', 'runtime_curve_speed', 'runtime_sharp_curve_speed',
              'runtime_max_drive_speed', 'actual_lateral_offset', 'last_approved_offset',
              'evaluated_candidate_offset', 'evaluated_candidate_clearance',
              'evaluated_candidate_profile_curvature', 'evaluated_candidate_polyline_curvature',
              'evaluated_candidate_reject_reason', 'obstacles', 'lane_change_state', 'lane_change_side',
              'selected_adjacent_link', 'adjacent_lane_separation', 'original_lane_center',
              'adjacent_lane_center', 'lane_change_entry_path', 'adjacent_lane_hold_path',
              'lane_change_recovery_path', 'lane_change_recovery_gate', 'lane_change_obstacle_safe',
              'lane_change_steering_feasible', 'lane_change_corridor_safe')
    for topic in topics:
        args.extend(['-r', f'/planning/{topic}:={prefix}/{topic}'])
    args.extend(['-r', f'/localization/odometry:={prefix}/odometry'])
    rclpy.init(args=args)
    node = None
    try:
        with patch.object(PlanningNode, 'create_timer'):
            node = PlanningNode()
        radius = 1/.15
        points = [[radius*math.sin(.15*i*.5), radius*(1-math.cos(.15*i*.5)), 0.]
                  for i in range(26)]
        node.latest_pose = (0., 0., 0.)
        node.map_matcher.match = Mock(return_value={'link_id': 'AB', 'distance': 0., 'heading_diff': 0.})
        node.local_path_planner.extract = Mock(return_value={
            'points': points, 'nearest_index': 0, 'progress_arc_m': 0.})
        node.plan()
        assert node.speed_planner.plan(points)['target_speed_kmh'] == 3.5
        results = node.set_parameters([Parameter(name, value=20.) for name in (
            'cruise_speed_kmh', 'curve_speed_kmh', 'sharp_curve_speed_kmh')])
        assert all(result.successful for result in results)
        assert node.get_parameter('sharp_curve_speed_kmh').value == 20.
        assert node.speed_planner.sharp_curve_speed_kmh == 3.5  # old cached state before cycle
        node.plan()
        assert node.speed_planner.sharp_curve_speed_kmh == 20.
        assert node.speed_planner.plan(points)['target_speed_kmh'] == 20.
        assert node.get_parameter('enable_drive').value is False
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
