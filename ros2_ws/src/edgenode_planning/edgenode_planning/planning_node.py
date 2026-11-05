import json
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, Float32, Float64, Int32, String
from std_srvs.srv import Trigger

from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.local_path_planner import LocalPathPlanner
from edgenode_planning.map_matcher import MapMatcher
from edgenode_planning.speed_planner import SpeedPlanner


def _finite_number(value):
    """Validate drive values without accepting booleans or overflowing ints."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


from .planning_fsm import PlanningFSM
from .adjacent_lane_planner import AdjacentLanePlanner
from .static_obstacle_lane_change import StaticObstacleLaneChangeBehavior
from .lane_geometry import LaneNetwork
from .maneuver_diagnostics import inspect_maneuver
from .corridor_scene import capture_scene, json_value, planner_snapshot, write_scene
from .lattice_planner import DEFAULT_CANDIDATE_OFFSETS_M, LatticePlanner


class PlanningNode(Node):
    """
    Match localization and publish global and local paths at 20 Hz.

    Current stage:
      - Perception supplies world-frame obstacle footprint samples.
      - Localization odometry is the only pose input.
      - Global routes are fixed until the active waypoint list or goal changes.
      - Local paths start at a bounded, forward-only global-segment projection.
      - Local geometry supplies speed recommendations for preview diagnostics.
      - Driving requires explicit enablement, valid geometry and a finite lease.
      - Default target speed remains 0 km/h with enable_drive=false.

    Future flow:
      GPS + IMU
          ↓
         EKF
          ↓
      Current Pose
          ↓
         MGeo
          ↓
      Global / Local Path
          ↓
       Planning
          ↓
      target_error
      target_speed
          ↓
       Control
    """

    def __init__(self):
        super().__init__('planning_node')

        # Output topics
        self.declare_parameter(
            'target_error_topic',
            '/planning/target_error',
        )

        self.declare_parameter(
            'target_speed_topic',
            '/planning/target_speed',
        )

        self.declare_parameter(
            'state_topic',
            '/planning/state',
        )

        # Recommendation is independent of the drive enablement and hard limit.
        self.declare_parameter(
            'cruise_speed_kmh',
            1.0,
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('localization_topic', '/localization/odometry'),
                ('current_link_topic', '/planning/current_link'),
                ('map_match_status_topic', '/planning/map_match_status'),
                ('map_match_distance_topic', '/planning/map_match_distance'),
                ('map_match_heading_diff_topic', '/planning/map_match_heading_diff'),
                ('global_path_topic', '/planning/global_path'),
                ('local_path_topic', '/planning/local_path'),
                ('local_path_length_m', 20.0),
                ('local_path_resample_spacing_m', 2.0),
                ('local_path_search_back', 3),
                ('local_path_search_forward', 10),
                ('local_path_index_topic', '/planning/local_path_index'),
                ('local_path_progress_topic', '/planning/local_path_progress_m'),
                ('enable_drive', False),
                ('max_drive_speed_kmh', 1.0),
                ('drive_duration_sec', 15.0),
                ('drive_status_topic', '/planning/drive_status'),
                ('obstacle_topic', '/planning/obstacles'),
                # Perception publishes ordered eight-point footprints; legacy
                # center publishers can retain the default ungrouped contract.
                ('obstacle_points_per_cluster', 1, ParameterDescriptor(read_only=True)),
                ('obstacle_avoidance_mode', 'adjacent_lane', ParameterDescriptor(read_only=True)),
                ('lane_change_primitive_length_m', 20.0, ParameterDescriptor(read_only=True)),
                ('lane_change_min_duration_sec', 3.0, ParameterDescriptor(read_only=True)),
                ('lane_change_corridor_dump_dir', '/workspace/logs/kcity_s4_corridor_diagnostics',
                 ParameterDescriptor(read_only=True)),
                ('lattice_status_topic', '/planning/lattice_status'),
                ('lattice_offset_topic', '/planning/lattice_offset'),
                ('lattice_clearance_topic', '/planning/lattice_clearance'),
                # Legacy clearance is a floor, never smaller than width+margin.
                ('lattice_clearance_m', 0.55),
                ('ego_half_width_m', 0.95),
                ('obstacle_safety_margin_m', 0.25),
                ('lattice_candidate_offsets_m', list(DEFAULT_CANDIDATE_OFFSETS_M)),
                ('lattice_transition_after_m', 10.0),
                ('lattice_transition_before_m', 5.0, ParameterDescriptor(read_only=True)),
                ('obstacle_avoid_speed_cap_kmh', 3.0),
                # Static planner geometry: the bringup launch takes these from
                # the same control.yaml used to start Control.
                ('lattice_wheelbase_m', 2.84, ParameterDescriptor(read_only=True)),
                ('lattice_max_wheel_angle_rad', 0.610865, ParameterDescriptor(read_only=True)),
                ('lattice_max_front_steer_normalized', 0.70, ParameterDescriptor(read_only=True)),
                ('lattice_curvature_safety_factor', 1.0, ParameterDescriptor(read_only=True)),
                ('lattice_obstacle_rear_margin_m', 0.5, ParameterDescriptor(read_only=True)),
                ('lattice_blocked_release_cycles', 3, ParameterDescriptor(read_only=True)),
                ('fsm_state_topic', '/planning/fsm_state'),
                ('curve_speed_kmh', 0.7),
                ('sharp_curve_speed_kmh', 0.5),
                ('speed_preview_distance_m', 10.0),
                ('curve_curvature_threshold', 0.04),
                ('sharp_curvature_threshold', 0.10),
                ('speed_curvature_smoothing_window_m', 2.0),
                ('speed_curvature_persistence_m', 2.0),
                ('speed_curvature_hysteresis', 0.005),
                ('recommended_speed_topic', '/planning/recommended_speed'),
                ('speed_mode_topic', '/planning/speed_mode'),
                ('speed_curvature_topic', '/planning/speed_curvature'),
                ('speed_max_curvature_topic', '/planning/speed_max_curvature'),
                ('goal_node_id', 'A119BS010245'),
                # Humble infers Python [] as BYTE_ARRAY and YAML [] as NOT_SET.
                # Allow both and STRING_ARRAY; plan_route validates nonempty values.
                ('route_node_ids', [], ParameterDescriptor(dynamic_typing=True)),
                ('global_path_frame_id', 'map'),
                ('mgeo_dir', '/workspace/local_data/kcity_mgeo'),
                ('map_match_search_radius_m', 6.0),
                ('map_match_max_distance_m', 4.0),
                ('map_match_max_heading_diff_deg', 80.0),
                ('map_match_heading_weight', 0.025),
                ('map_match_same_link_bonus', 0.4),
                ('map_match_connected_link_bonus', 0.7),
                ('map_match_unrelated_link_penalty', 1.0),
            ],
        )

        mgeo_dir = self.get_parameter('mgeo_dir').value
        self.map_matcher = MapMatcher(
            mgeo_dir=mgeo_dir,
            search_radius_m=self.get_parameter('map_match_search_radius_m').value,
            max_match_distance_m=self.get_parameter('map_match_max_distance_m').value,
            max_heading_diff_deg=self.get_parameter('map_match_max_heading_diff_deg').value,
            heading_weight=self.get_parameter('map_match_heading_weight').value,
            same_link_bonus=self.get_parameter('map_match_same_link_bonus').value,
            connected_link_bonus=self.get_parameter('map_match_connected_link_bonus').value,
            unrelated_link_penalty=self.get_parameter('map_match_unrelated_link_penalty').value,
        )
        self.dijkstra_planner = DijkstraPlanner(mgeo_dir=mgeo_dir)
        self.local_path_planner = LocalPathPlanner()
        self.speed_planner = SpeedPlanner(
            cruise_speed_kmh=self.get_parameter('cruise_speed_kmh').value,
            curve_speed_kmh=self.get_parameter('curve_speed_kmh').value,
            sharp_curve_speed_kmh=self.get_parameter('sharp_curve_speed_kmh').value,
            preview_distance_m=self.get_parameter('speed_preview_distance_m').value,
            curve_curvature_threshold=self.get_parameter('curve_curvature_threshold').value,
            sharp_curvature_threshold=self.get_parameter('sharp_curvature_threshold').value,
            curvature_smoothing_window_m=self.get_parameter('speed_curvature_smoothing_window_m').value,
            curvature_persistence_m=self.get_parameter('speed_curvature_persistence_m').value,
            curvature_hysteresis=self.get_parameter('speed_curvature_hysteresis').value,
        )
        self._previous_speed_mode = None
        self._route_key = None
        self._route_result = None
        self._progress_index = None
        self._progress_arc_m = None
        self.latest_pose = None
        self._drive_was_enabled = False
        self._drive_session_start_ns = None
        self._drive_session_expired = False
        self._drive_session_duration_sec = None
        self._drive_last_time_ns = None
        self.fsm = PlanningFSM()
        self.lattice_planner = LatticePlanner(
            obstacle_clearance_m=self.get_parameter('lattice_clearance_m').value,
            ego_half_width_m=self.get_parameter('ego_half_width_m').value,
            obstacle_safety_margin_m=self.get_parameter('obstacle_safety_margin_m').value,
            candidate_offsets_m=self.get_parameter('lattice_candidate_offsets_m').value,
            transition_after_m=self.get_parameter('lattice_transition_after_m').value,
            transition_before_m=self.get_parameter('lattice_transition_before_m').value,
            wheelbase_m=self.get_parameter('lattice_wheelbase_m').value,
            max_wheel_angle_rad=self.get_parameter('lattice_max_wheel_angle_rad').value,
            max_front_steer_normalized=self.get_parameter(
                'lattice_max_front_steer_normalized').value,
            curvature_safety_factor=self.get_parameter('lattice_curvature_safety_factor').value,
            obstacle_rear_margin_m=self.get_parameter('lattice_obstacle_rear_margin_m').value,
            blocked_release_cycles=self.get_parameter('lattice_blocked_release_cycles').value,
        )
        mode = self.get_parameter('obstacle_avoidance_mode').value
        if mode not in ('legacy_offset', 'adjacent_lane', 'lane_change_primitive'):
            raise ValueError('unknown obstacle_avoidance_mode')
        if mode in ('adjacent_lane', 'lane_change_primitive'):
            legacy = self.lattice_planner
            planner_type = (StaticObstacleLaneChangeBehavior if mode == 'lane_change_primitive'
                            else AdjacentLanePlanner)
            extra = (dict(transition_length_m=self.get_parameter('lane_change_primitive_length_m').value)
                     if mode == 'lane_change_primitive' else {})
            self.lattice_planner = planner_type(
                LaneNetwork(self.dijkstra_planner),
                min_transition_time_sec=self.get_parameter('lane_change_min_duration_sec').value,
                obstacle_clearance_m=legacy.required_clearance_m,
                ego_half_width_m=legacy.ego_half_width_m,
                obstacle_safety_margin_m=legacy.obstacle_safety_margin_m,
                transition_before_m=legacy.transition_before_m,
                transition_after_m=legacy.transition_after_m,
                wheelbase_m=legacy.wheelbase_m,max_wheel_angle_rad=legacy.max_wheel_angle_rad,
                max_front_steer_normalized=legacy.max_front_steer_normalized,
                curvature_safety_factor=legacy.curvature_safety_factor,
                obstacle_rear_margin_m=legacy.obstacle_rear_margin_m,
                blocked_release_cycles=legacy.blocked_release_cycles, **extra)
        self.latest_speed_kmh = 0.0
        self.latest_obstacles = []

        self.target_error_pub = self.create_publisher(
            Float32,
            self.get_parameter('target_error_topic').value,
            10,
        )

        self.target_speed_pub = self.create_publisher(
            Float32,
            self.get_parameter('target_speed_topic').value,
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            self.get_parameter('state_topic').value,
            10,
        )

        self.current_link_pub = self.create_publisher(
            String,
            self.get_parameter('current_link_topic').value,
            10,
        )
        self.map_match_status_pub = self.create_publisher(
            String,
            self.get_parameter('map_match_status_topic').value,
            10,
        )
        self.map_match_distance_pub = self.create_publisher(
            Float32,
            self.get_parameter('map_match_distance_topic').value,
            10,
        )
        self.map_match_heading_diff_pub = self.create_publisher(
            Float32,
            self.get_parameter('map_match_heading_diff_topic').value,
            10,
        )
        self.global_path_pub = self.create_publisher(
            Path,
            self.get_parameter('global_path_topic').value,
            10,
        )
        self.local_path_pub = self.create_publisher(
            Path,
            self.get_parameter('local_path_topic').value,
            10,
        )
        self.local_path_index_pub = self.create_publisher(
            Int32,
            self.get_parameter('local_path_index_topic').value,
            10,
        )
        self.local_path_progress_pub = self.create_publisher(
            Float64,
            self.get_parameter('local_path_progress_topic').value,
            10,
        )
        self.recommended_speed_pub = self.create_publisher(
            Float32,
            self.get_parameter('recommended_speed_topic').value,
            10,
        )
        self.speed_mode_pub = self.create_publisher(
            String,
            self.get_parameter('speed_mode_topic').value,
            10,
        )
        self.speed_curvature_pub = self.create_publisher(
            Float32,
            self.get_parameter('speed_curvature_topic').value,
            10,
        )
        self.speed_max_curvature_pub = self.create_publisher(
            Float32,
            self.get_parameter('speed_max_curvature_topic').value,
            10,
        )
        self.drive_status_pub = self.create_publisher(
            String,
            self.get_parameter('drive_status_topic').value,
            10,
        )
        self.fsm_state_pub = self.create_publisher(
            String,
            self.get_parameter('fsm_state_topic').value,
            10,
        )
        self.lattice_status_pub = self.create_publisher(
            String,
            self.get_parameter('lattice_status_topic').value,
            10,
        )
        self.lattice_offset_pub = self.create_publisher(
            Float32,
            self.get_parameter('lattice_offset_topic').value,
            10,
        )
        self.lattice_clearance_pub = self.create_publisher(
            Float32,
            self.get_parameter('lattice_clearance_topic').value,
            10,
        )
        for diagnostic in ('obstacle_avoid_speed_cap_kmh', 'applied_speed_cap_kmh',
                           'lattice_max_curvature', 'lattice_curvature_limit',
                           'lattice_obstacle_front_s', 'lattice_obstacle_rear_s',
                           'lattice_entry_length', 'lattice_full_offset_start_s',
                           'lattice_recovery_length', 'runtime_cruise_speed',
                           'runtime_curve_speed', 'runtime_sharp_curve_speed',
                           'runtime_max_drive_speed', 'actual_lateral_offset',
                           'last_approved_offset', 'evaluated_candidate_offset',
                           'evaluated_candidate_clearance',
                           'evaluated_candidate_profile_curvature',
                           'evaluated_candidate_polyline_curvature'):
            setattr(self, diagnostic + '_pub', self.create_publisher(
                Float32, '/planning/' + diagnostic, 10))
        self.lattice_blocked_reason_pub = self.create_publisher(
            String, '/planning/lattice_blocked_reason', 10)
        self.evaluated_candidate_reject_reason_pub = self.create_publisher(
            String, '/planning/evaluated_candidate_reject_reason', 10)
        for diagnostic in ('lane_change_obstacle_safe', 'lane_change_steering_feasible', 'lane_change_corridor_safe'):
            setattr(self, diagnostic+'_pub', self.create_publisher(Bool, '/planning/'+diagnostic, 10))
        self.lane_change_state_pub = self.create_publisher(String, '/planning/lane_change_state', 10)
        self.lane_change_side_pub = self.create_publisher(String, '/planning/lane_change_side', 10)
        self.selected_adjacent_link_pub = self.create_publisher(String, '/planning/selected_adjacent_link', 10)
        self.adjacent_lane_separation_pub = self.create_publisher(Float32, '/planning/adjacent_lane_separation', 10)
        self.lane_change_corridor_violation_m_pub = self.create_publisher(
            Float32, '/planning/lane_change_corridor_violation_m', 10)
        for diagnostic in ('lane_change_corridor_failed_index', 'lane_change_corridor_failed_point_count'):
            setattr(self, diagnostic+'_pub', self.create_publisher(Int32, '/planning/'+diagnostic, 10))
        self.lane_change_corridor_debug_pub = self.create_publisher(
            String, '/planning/lane_change_corridor_debug', 10)
        self.lane_change_maneuver_debug_pub = self.create_publisher(
            String, '/planning/lane_change_maneuver_debug', 10)
        self._corridor_scene = None
        self.corridor_dump_service = self.create_service(
            Trigger, '/planning/dump_lane_change_corridor_scene', self.dump_lane_change_corridor_scene)
        for diagnostic in ('original_lane_center', 'adjacent_lane_center', 'lane_change_entry_path',
                           'adjacent_lane_hold_path', 'lane_change_recovery_path', 'lane_change_recovery_gate'):
            setattr(self, diagnostic + '_pub', self.create_publisher(Path, '/planning/' + diagnostic, 10))
        self.latest_obstacle_groups = None
        self._obstacle_points_per_cluster = self.get_parameter('obstacle_points_per_cluster').value
        self.obstacle_sub = self.create_subscription(
            PoseArray,
            self.get_parameter('obstacle_topic').value,
            self.obstacle_callback,
            10,
        )
        self.localization_sub = self.create_subscription(
            Odometry,
            self.get_parameter('localization_topic').value,
            self.localization_callback,
            10,
        )

        # Planning loop: 20 Hz
        self.create_timer(
            0.05,
            self.plan,
        )

        self.get_logger().info(
            f'Planning node ready - loaded {len(self.map_matcher.links)} MGeo links; '
            'waiting for localization (drive disabled by default)'
        )

    def dump_lane_change_corridor_scene(self, request, response):
        """Write one current input snapshot on explicit request; no control action."""
        try:
            response.message = write_scene(self._corridor_scene,
                self.get_parameter('lane_change_corridor_dump_dir').value)
            response.success = True
        except (OSError, ValueError, TypeError) as error:
            response.success = False
            response.message = str(error)
        return response

    def obstacle_callback(self, msg):
        """Store world-frame obstacle XY positions for Lattice planning."""
        obstacles = []

        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y

            if _finite_number(x) and _finite_number(y):
                obstacles.append((float(x), float(y)))

        self.latest_obstacles = obstacles
        size = getattr(self, '_obstacle_points_per_cluster', 1)
        # Never discard malformed or legacy samples. Grouping is metadata only;
        # all finite input points remain in the collision set.
        self.latest_obstacle_groups = (
            [obstacles[i:i + size] for i in range(0, len(obstacles), size)]
            if size == 8 and len(obstacles) == len(msg.poses) and len(obstacles) % size == 0
            else None)

    def localization_callback(self, msg):
        """Store the latest pose; map matching runs only in the planning timer."""
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.latest_pose = (position.x, position.y, yaw)
        self.latest_pose_stamp_ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.latest_pose_receipt_ns = self.get_clock().now().nanoseconds
        speed = msg.twist.twist.linear.x
        self.latest_speed_kmh = abs(float(speed))*3.6 if _finite_number(speed) else math.nan

    def plan(self):
        """Publish routes, recommendations and guarded, time-limited commands.

        Cache failures too, so an unreachable route does not trigger a search
        every tick. Match loss retains progress/cache but publishes no Path.
        An active route configuration change invalidates both immediately.
        """
        # A missing/invalid current cycle must not dump an older scene.
        self._corridor_scene = None
        # Read on every cycle, before the recommendation. ROS parameter
        # readback and the actual planner share the same runtime contract.
        for name in ('cruise_speed_kmh', 'curve_speed_kmh', 'sharp_curve_speed_kmh'):
            setattr(self.speed_planner, name, self.get_parameter(name).value)
        lattice_result = None
        corridor_before = None
        input_odometry_stamp_ns = getattr(self, 'latest_pose_stamp_ns', None)
        input_receipt_stamp_ns = getattr(self, 'latest_pose_receipt_ns', None)
        planning_start_stamp_ns = self.get_clock().now().nanoseconds
        state_value = 'WAIT_FOR_LOCALIZATION'
        status_value = 'WAIT_FOR_LOCALIZATION'
        current_link = ''
        distance = float('nan')
        heading_diff = float('nan')
        recommended_speed = 0.0
        speed_mode = 'INVALID'
        speed_curvature = float('nan')
        speed_max_curvature = float('nan')
        local_result = None
        speed_result = None
        speed_valid = False
        lattice_status = 'IDLE'
        lattice_offset = 0.0
        lattice_clearance = float('nan')
        lattice_max_curvature = float('nan')
        lattice_obstacle_front_s = float('nan')
        lattice_obstacle_rear_s = float('nan')
        lattice_blocked_reason = ''
        lattice_entry_length = math.nan
        lattice_full_offset_start_s = math.nan
        lattice_recovery_length = math.nan
        lattice_blocked = False
        obstacle_avoid_requested = False

        route_node_ids = self.get_parameter('route_node_ids').value
        use_waypoints = (route_node_ids is not None
                         and not (isinstance(route_node_ids, (list, tuple))
                                  and len(route_node_ids) == 0))
        if use_waypoints:
            route_key = ('waypoints', tuple(route_node_ids)
                         if isinstance(route_node_ids, (list, tuple))
                         else repr(route_node_ids))
        else:
            goal_node_id = self.get_parameter('goal_node_id').value
            route_key = ('goal', goal_node_id)
        if route_key != self._route_key:
            self._route_key = None
            self._route_result = None
            self._progress_index = None
            self._progress_arc_m = None
            self._previous_speed_mode = None
            self.lattice_planner.reset()

        now_ns = self.get_clock().now().nanoseconds
        enable_drive = self.get_parameter('enable_drive').value is True
        if not enable_drive:
            self._drive_was_enabled = False
            self._drive_session_start_ns = None
            self._drive_session_expired = False
            self._drive_session_duration_sec = None
            self._drive_last_time_ns = None
        else:
            if not self._drive_was_enabled:
                self._drive_session_start_ns = now_ns
                self._drive_session_expired = False
                self._drive_session_duration_sec = None
            elif now_ns < self._drive_last_time_ns:
                # A clock rewind must not extend or resurrect the drive lease.
                self._drive_session_expired = True
            self._drive_was_enabled = True
            self._drive_last_time_ns = now_ns

        if self.latest_pose is not None:
            x, y, yaw = self.latest_pose
            result = self.map_matcher.match(x, y, math.degrees(yaw))
            if result is None:
                state_value = 'MAP_MATCH_LOST'
                status_value = 'NO_MATCH'
            else:
                status_value = 'MATCHED'
                current_link = result['link_id']
                distance = result['distance']
                heading_diff = result['heading_diff']

                # Match fluctuations never change a fixed route. Only the
                # active waypoint list (or legacy goal) can invalidate it.
                if route_key != self._route_key:
                    if use_waypoints:
                        self._route_result = self.dijkstra_planner.plan_route(
                            current_link, route_node_ids,
                        )
                    else:
                        self._route_result = self.dijkstra_planner.plan(
                            current_link, goal_node_id,
                        )
                    self._route_key = route_key
                    self._progress_index = None
                    self._progress_arc_m = None

                if self._route_result is None:
                    state_value = 'GLOBAL_PATH_NOT_FOUND'
                else:
                    state_value = 'GLOBAL_PATH_READY'
                    path = Path()
                    path.header.frame_id = self.get_parameter('global_path_frame_id').value
                    path.header.stamp = self.get_clock().now().to_msg()
                    # Keep the entire start link; trimming belongs to local planning.
                    for point in self._route_result['points']:
                        pose = PoseStamped()
                        pose.header = path.header
                        pose.pose.position.x = float(point[0])
                        pose.pose.position.y = float(point[1])
                        pose.pose.position.z = float(point[2])
                        pose.pose.orientation.w = 1.0
                        path.poses.append(pose)
                    self.global_path_pub.publish(path)

                    local_result = self.local_path_planner.extract(
                        self._route_result['points'], x, y,
                        horizon_m=self.get_parameter('local_path_length_m').value,
                        previous_index=self._progress_index,
                        search_back=self.get_parameter('local_path_search_back').value,
                        search_forward=self.get_parameter('local_path_search_forward').value,
                        resample_spacing_m=self.get_parameter('local_path_resample_spacing_m').value,
                        previous_arc_m=self._progress_arc_m,
                    )
                    if local_result is not None:
                        self._progress_index = local_result['nearest_index']
                        self._progress_arc_m = local_result['progress_arc_m']
                        active_local_points = local_result['points']

                        lane_options = {}
                        corridor_before = None
                        if isinstance(self.lattice_planner, AdjacentLanePlanner):
                            corridor_before = planner_snapshot(self.lattice_planner)
                        if isinstance(self.lattice_planner, (AdjacentLanePlanner, StaticObstacleLaneChangeBehavior)):
                            recommendation_bound = max(self.get_parameter(name).value for name in (
                                'cruise_speed_kmh', 'curve_speed_kmh', 'sharp_curve_speed_kmh'))
                            configured_speed = min(recommendation_bound,
                                self.get_parameter('max_drive_speed_kmh').value,
                                self.get_parameter('obstacle_avoid_speed_cap_kmh').value)
                            planned_speed = max(getattr(self, 'latest_speed_kmh', 0.0), configured_speed)
                            lane_options = dict(current_link_id=current_link,
                                route_link_ids=self._route_result['link_ids'], speed_kmh=planned_speed)
                        lattice_result = self.lattice_planner.plan(
                            active_local_points,
                            self.latest_obstacles,
                            current_position=(x, y),
                            current_heading=yaw,
                            obstacle_groups=getattr(self, 'latest_obstacle_groups', None),
                            road_progress_m=self._progress_arc_m, **lane_options,
                        )
                        if corridor_before is not None:
                            self._corridor_scene = capture_scene(self.lattice_planner, corridor_before,
                                dict(reference_points=local_result['points'], obstacles=self.latest_obstacles,
                                     current_position=[x, y], current_heading=yaw,
                                     obstacle_groups=getattr(self, 'latest_obstacle_groups', None),
                                     road_progress_m=self._progress_arc_m, **lane_options),
                                lattice_result, now_ns, self.get_parameter('global_path_frame_id').value)

                        if lattice_result is not None:
                            lattice_status = lattice_result['status']
                            lattice_offset = lattice_result['selected_offset_m']
                            lattice_clearance = lattice_result['minimum_clearance_m']
                            lattice_max_curvature = lattice_result['maximum_curvature']
                            lattice_obstacle_front_s = lattice_result['obstacle_front_s']
                            lattice_obstacle_rear_s = lattice_result['obstacle_rear_s']
                            lattice_blocked_reason = lattice_result['blocked_reason']
                            lattice_entry_length = lattice_result['entry_length']
                            lattice_full_offset_start_s = lattice_result['full_offset_start_s']
                            lattice_recovery_length = lattice_result['recovery_length']
                            lattice_blocked = lattice_result['blocked']
                            obstacle_avoid_requested = lattice_result[
                                'obstacle_avoid_requested'
                            ]

                            if not lattice_blocked:
                                active_local_points = lattice_result['points']
                            elif isinstance(self.lattice_planner, (AdjacentLanePlanner, StaticObstacleLaneChangeBehavior)):
                                # No approved local path exists. Publish an empty
                                # path rather than an unapproved road-center fallback.
                                active_local_points = []
                        elif self.latest_obstacles or isinstance(self.lattice_planner, (AdjacentLanePlanner, StaticObstacleLaneChangeBehavior)):
                            lattice_status = 'BLOCKED'
                            lattice_blocked = True
                            lattice_blocked_reason = 'INVALID_AVOIDANCE_REFERENCE'
                            if isinstance(self.lattice_planner, (AdjacentLanePlanner, StaticObstacleLaneChangeBehavior)):
                                active_local_points = []

                        local_path = Path()
                        local_path.header = path.header

                        for point in active_local_points:
                            pose = PoseStamped()
                            pose.header = local_path.header
                            pose.pose.position.x = float(point[0])
                            pose.pose.position.y = float(point[1])
                            pose.pose.position.z = float(point[2])
                            pose.pose.orientation.w = 1.0
                            local_path.poses.append(pose)

                        self.local_path_pub.publish(local_path)

                        speed_result = self.speed_planner.plan(
                            active_local_points, previous_speed_mode=self._previous_speed_mode,
                        )
                        if (speed_result is not None
                                and _finite_number(speed_result['target_speed_kmh'])
                                and speed_result['target_speed_kmh'] >= 0):
                            speed_valid = True
                            recommended_speed = speed_result['target_speed_kmh']
                            speed_mode = speed_result['speed_mode']
                            speed_curvature = speed_result['representative_curvature']
                            speed_max_curvature = speed_result['max_curvature']

        if local_result is None:
            self.lattice_planner.break_release_confirmation()

        # Speed classification keeps its own hysteresis. Invalid planning
        # input or a route change must not carry a stale mode into recovery.
        self._previous_speed_mode = speed_mode if speed_valid else None

        target_error = Float32()
        target_error.data = 0.0

        target_speed = Float32()
        target_speed.data = 0.0
        requested_target_speed = 0.0
        drive_status = 'DISABLED'
        if enable_drive:
            max_speed = self.get_parameter('max_drive_speed_kmh').value
            duration = self.get_parameter('drive_duration_sec').value
            if (not _finite_number(max_speed) or max_speed <= 0
                    or not _finite_number(duration) or duration <= 0):
                drive_status = 'INVALID_CONFIG'
            else:
                # Live duration changes may shorten, but never extend a session.
                if self._drive_session_duration_sec is None:
                    self._drive_session_duration_sec = duration
                else:
                    self._drive_session_duration_sec = min(self._drive_session_duration_sec, duration)
                elapsed_sec = (now_ns - self._drive_session_start_ns) / 1e9
                if self._drive_session_expired or elapsed_sec >= self._drive_session_duration_sec:
                    # Expiry is latched until an observed false -> true edge.
                    self._drive_session_expired = True
                    drive_status = 'EXPIRED'
                elif (state_value == 'GLOBAL_PATH_READY' and status_value == 'MATCHED'
                      and local_result is not None and speed_result is not None
                      and speed_valid):
                    requested_target_speed = float(min(recommended_speed, max_speed))
                    drive_status = 'ACTIVE'
                else:
                    drive_status = 'NOT_READY'

        avoid_cap = self.get_parameter('obstacle_avoid_speed_cap_kmh').value
        avoid_cap_valid = _finite_number(avoid_cap) and avoid_cap > 0
        if obstacle_avoid_requested and not avoid_cap_valid:
            drive_status = 'INVALID_CONFIG'

        previous_fsm_state = self.fsm.state

        fsm_state_value = self.fsm.update(
            localization_ready=self.latest_pose is not None,
            map_match_ready=status_value == 'MATCHED',
            global_path_ready=state_value == 'GLOBAL_PATH_READY',
            local_path_ready=local_result is not None,
            speed_ready=speed_valid,
            drive_status=drive_status,
            # Future behavior inputs.
            # These remain False until Lattice / Perception / goal detection
            # are connected to the planning layer.
            obstacle_avoid_requested=obstacle_avoid_requested,
            traffic_stop_requested=False,
            emergency_stop_requested=lattice_blocked,
            goal_reached=False,
        )

        if fsm_state_value != previous_fsm_state:
            self.get_logger().info(
                f'FSM transition: {previous_fsm_state} -> {fsm_state_value}'
            )

        # The S4 cap is applied at the final command, after speed classification
        # and FSM arbitration. NORMAL_DRIVE keeps its existing cruise/cap.
        max_speed = self.get_parameter('max_drive_speed_kmh').value
        applied_cap = float(max_speed) if _finite_number(max_speed) and max_speed > 0 else 0.0
        if fsm_state_value == PlanningFSM.OBSTACLE_AVOID:
            applied_cap = min(applied_cap, float(avoid_cap)) if avoid_cap_valid else 0.0
        if self.fsm.motion_allowed:
            target_speed.data = min(requested_target_speed, applied_cap)
        else:
            target_speed.data = 0.0
            applied_cap = 0.0

        state = String()
        state.data = state_value

        self.target_error_pub.publish(target_error)
        self.target_speed_pub.publish(target_speed)
        self.state_pub.publish(state)
        self.local_path_index_pub.publish(Int32(
            data=self._progress_index if local_result is not None else -1,
        ))
        self.local_path_progress_pub.publish(Float64(
            data=self._progress_arc_m if local_result is not None else float('nan'),
        ))
        self.fsm_state_pub.publish(String(data=fsm_state_value))
        self.current_link_pub.publish(String(data=current_link))
        self.map_match_status_pub.publish(String(data=status_value))
        self.map_match_distance_pub.publish(Float32(data=distance))
        self.map_match_heading_diff_pub.publish(Float32(data=heading_diff))
        self.recommended_speed_pub.publish(Float32(data=recommended_speed))
        for diagnostic, parameter in (
                ('runtime_cruise_speed', 'cruise_speed_kmh'),
                ('runtime_curve_speed', 'curve_speed_kmh'),
                ('runtime_sharp_curve_speed', 'sharp_curve_speed_kmh')):
            getattr(self, diagnostic + '_pub').publish(Float32(
                data=float(getattr(self.speed_planner, parameter))))
        self.runtime_max_drive_speed_pub.publish(Float32(
            data=float(max_speed) if _finite_number(max_speed) else math.nan))
        for diagnostic in ('actual_lateral_offset', 'last_approved_offset',
                           'evaluated_candidate_offset', 'evaluated_candidate_clearance',
                           'evaluated_candidate_profile_curvature',
                           'evaluated_candidate_polyline_curvature'):
            getattr(self, diagnostic + '_pub').publish(Float32(
                data=lattice_result[diagnostic] if lattice_result else math.nan))
        self.evaluated_candidate_reject_reason_pub.publish(String(
            data=lattice_result['evaluated_candidate_reject_reason']
            if lattice_result else 'NOT_EVALUATED'))
        self.speed_mode_pub.publish(String(data=speed_mode))
        self.speed_curvature_pub.publish(Float32(data=speed_curvature))
        self.speed_max_curvature_pub.publish(Float32(data=speed_max_curvature))
        self.drive_status_pub.publish(String(data=drive_status))
        self.lattice_status_pub.publish(String(data=lattice_status))
        self.lattice_offset_pub.publish(Float32(data=lattice_offset))
        self.lattice_clearance_pub.publish(Float32(data=lattice_clearance))
        self.obstacle_avoid_speed_cap_kmh_pub.publish(Float32(
            data=float(avoid_cap) if avoid_cap_valid else 0.0))
        self.applied_speed_cap_kmh_pub.publish(Float32(data=applied_cap))
        self.lattice_max_curvature_pub.publish(Float32(data=lattice_max_curvature))
        self.lattice_curvature_limit_pub.publish(Float32(
            data=self.lattice_planner.usable_curvature_limit))
        self.lattice_obstacle_front_s_pub.publish(Float32(data=lattice_obstacle_front_s))
        self.lattice_obstacle_rear_s_pub.publish(Float32(data=lattice_obstacle_rear_s))
        self.lattice_blocked_reason_pub.publish(String(data=lattice_blocked_reason))
        self.lattice_entry_length_pub.publish(Float32(data=lattice_entry_length))
        self.lattice_full_offset_start_s_pub.publish(Float32(data=lattice_full_offset_start_s))
        self.lattice_recovery_length_pub.publish(Float32(data=lattice_recovery_length))
        if hasattr(self, 'lane_change_state_pub'):
            diagnostic = lattice_result or {}
            phase = ('BLOCKED' if lattice_blocked else diagnostic.get('lane_change_phase',
                     'LEGACY_OFFSET' if obstacle_avoid_requested else 'CLEAR'))
            self.lane_change_state_pub.publish(String(data=phase))
            self.lane_change_obstacle_safe_pub.publish(Bool(data=bool(
                diagnostic.get('evaluated_candidate_clearance', math.nan)>=self.lattice_planner.required_clearance_m)))
            self.lane_change_steering_feasible_pub.publish(Bool(data=bool(
                diagnostic.get('maximum_curvature', math.inf)<=self.lattice_planner.usable_curvature_limit)))
            self.lane_change_corridor_safe_pub.publish(Bool(data=bool(diagnostic.get('drivable_safe', False))))
            corridor = diagnostic.get('corridor_diagnostics') or dict(
                evaluated=False, reason='NOT_EVALUATED', max_violation_m=None,
                failed_index=-1, failed_point_count=0)
            self.lane_change_corridor_violation_m_pub.publish(Float32(
                data=corridor['max_violation_m'] if corridor['max_violation_m'] is not None else math.nan))
            self.lane_change_corridor_failed_index_pub.publish(Int32(data=corridor['failed_index']))
            self.lane_change_corridor_failed_point_count_pub.publish(Int32(data=corridor['failed_point_count']))
            self.lane_change_corridor_debug_pub.publish(String(
                data=json.dumps(json_value(corridor), allow_nan=False, ensure_ascii=False)))
            if isinstance(self.lattice_planner, AdjacentLanePlanner) and hasattr(self, 'lane_change_maneuver_debug_pub'):
                maneuver=inspect_maneuver(self.lattice_planner,corridor_before,lattice_result,
                    input_odometry_stamp_ns=input_odometry_stamp_ns,input_receipt_stamp_ns=input_receipt_stamp_ns,
                    planning_start_stamp_ns=planning_start_stamp_ns,output_stamp_ns=self.get_clock().now().nanoseconds,
                    current_position=self.latest_pose[:2] if self.latest_pose is not None else None,
                    current_heading=self.latest_pose[2] if self.latest_pose is not None else None)
                self.lane_change_maneuver_debug_pub.publish(String(
                    data=json.dumps(maneuver,allow_nan=False,ensure_ascii=False)))
                if self._corridor_scene is not None:
                    self._corridor_scene['maneuver_diagnostics']=maneuver
            self.lane_change_side_pub.publish(String(data=diagnostic.get('selected_side', '')))
            self.selected_adjacent_link_pub.publish(String(data=diagnostic.get('adjacent_link_id', '')))
            self.adjacent_lane_separation_pub.publish(Float32(data=diagnostic.get('adjacent_separation', math.nan)))
            for topic, field in (
                    ('original_lane_center', 'original_lane_points'),
                    ('adjacent_lane_center', 'adjacent_lane_points'),
                    ('lane_change_entry_path', 'entry_points'),
                    ('adjacent_lane_hold_path', 'hold_points'),
                    ('lane_change_recovery_path', 'recovery_points'),
                    ('lane_change_recovery_gate', 'recovery_gate_points')):
                msg = Path()
                msg.header.frame_id = self.get_parameter('global_path_frame_id').value
                msg.header.stamp = self.get_clock().now().to_msg()
                points = diagnostic.get(field, [])
                if field == 'original_lane_points' and not points and local_result is not None:
                    points = local_result['points']
                for point in points:
                    pose = PoseStamped()
                    pose.header = msg.header
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
                    pose.pose.orientation.w = 1.0
                    msg.poses.append(pose)
                getattr(self, topic + '_pub').publish(msg)



def main(args=None):
    rclpy.init(args=args)

    node = PlanningNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
