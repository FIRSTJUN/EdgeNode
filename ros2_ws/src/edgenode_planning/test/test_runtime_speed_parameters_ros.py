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
              'runtime_cruise_speed', 'runtime_curve_speed', 'runtime_sharp_curve_speed',
              'runtime_max_drive_speed','obstacles','avoidance_state','avoidance_debug',
              'lattice_candidate_markers','original_lane_center','left_lane_center','right_lane_center',
              'avoidance_selected_path','avoidance_return_path','reset_avoidance')
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
