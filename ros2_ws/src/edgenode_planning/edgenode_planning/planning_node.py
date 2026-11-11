import json
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from morai_ros2_msgs.msg import EgoVehicleStatus
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from std_msgs.msg import Float32, Float64, Int32, String
from std_srvs.srv import Trigger

from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.local_path_planner import LocalPathPlanner
from edgenode_planning.stopped_progress import reanchor_progress
from edgenode_planning.map_matcher import MapMatcher
from edgenode_planning.speed_planner import SpeedPlanner
from edgenode_planning.longitudinal_speed_planner import (
    LongitudinalSpeedPlanner, RouteProfileBuilder, SpeedPlanSettings,
)


def _finite_number(value):
    """Validate drive values without accepting booleans or overflowing ints."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


from .planning_fsm import PlanningFSM


_SPEED_PLANNER_PARAMETERS = (
    ('cruise_speed_kmh', 'cruise_speed_kmh'),
    ('curve_speed_kmh', 'curve_speed_kmh'),
    ('sharp_curve_speed_kmh', 'sharp_curve_speed_kmh'),
    ('speed_preview_distance_m', 'preview_distance_m'),
    ('curve_curvature_threshold', 'curve_curvature_threshold'),
    ('sharp_curvature_threshold', 'sharp_curvature_threshold'),
    ('speed_curvature_smoothing_window_m', 'curvature_smoothing_window_m'),
    ('speed_curvature_persistence_m', 'curvature_persistence_m'),
    ('speed_curvature_hysteresis', 'curvature_hysteresis'),
)
_SPEED_PARAMETER_NAMES = tuple(name for name, _ in _SPEED_PLANNER_PARAMETERS)
_LONG_SPEED_DEFAULTS = (
    ('speed_long_preview_enabled', False),
    ('speed_long_preview_distance_m', 80.0),
    ('speed_plan_deceleration_mps2', 1.5),
    ('speed_plan_reaction_time_sec', 0.5),
    ('speed_plan_distance_margin_m', 5.0),
    ('speed_plan_lateral_accel_mps2', 1.5),
    ('speed_plan_speed_timeout_sec', 0.2),
    ('speed_plan_acceleration_mps2', 1.0),
    ('speed_plan_recovery_acceleration_mps2', 2.0),
    ('speed_plan_recovery_clear_sec', 1.0),
)
_LONG_SPEED_NAMES = tuple(name for name, _ in _LONG_SPEED_DEFAULTS)


def _long_configuration_error(snapshot):
    if not isinstance(snapshot[0], bool):
        return 'speed_long_preview_enabled must be boolean'
    if not SpeedPlanSettings(*snapshot[1:]).valid():
        return 'Long speed settings must be finite and within the documented distance, acceleration, delay, margin and clear-time bounds'
    return ''


def _read_speed_planner_snapshot(node):
    return tuple(parameter.value for parameter in node.get_parameters(_SPEED_PARAMETER_NAMES))


def _speed_planner_from_snapshot(snapshot):
    return SpeedPlanner(**{
        argument: value for (_, argument), value in zip(_SPEED_PLANNER_PARAMETERS, snapshot)
    })


def _speed_configuration_error(snapshot):
    """Match SpeedPlanner's configuration contract without evaluating geometry."""
    if not all(_finite_number(value) for value in snapshot):
        return 'SpeedPlanner parameters must be finite numbers (not booleans)'
    (cruise, curve, sharp, preview, curve_threshold, sharp_threshold,
     smoothing, persistence, hysteresis) = snapshot
    if min(cruise, curve, sharp) < 0:
        return 'SpeedPlanner speeds must be nonnegative'
    if min(preview, smoothing, persistence) <= 0:
        return 'SpeedPlanner preview, smoothing and persistence distances must be positive'
    if not 0 <= curve_threshold < sharp_threshold:
        return 'SpeedPlanner requires 0 <= curve_curvature_threshold < sharp_curvature_threshold'
    if hysteresis < 0:
        return 'SpeedPlanner curvature hysteresis must be nonnegative'
    return ''


class PlanningNode(Node):
    """
    Match localization and publish global and local paths at 20 Hz.

    Current stage:
      - Perception is intentionally disconnected.
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
                ('ego_status_topic', '/ego_vehicle_status'),
                ('long_speed_plan_topic', '/planning/long_speed_plan'),
                *_LONG_SPEED_DEFAULTS,
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
        self._speed_planner_snapshot = _read_speed_planner_snapshot(self)
        self.speed_planner = _speed_planner_from_snapshot(self._speed_planner_snapshot)
        self._previous_speed_mode = None
        self._previous_curve_state = None
        self._long_speed_snapshot = tuple(p.value for p in self.get_parameters(_LONG_SPEED_NAMES))
        self._long_settings = SpeedPlanSettings(*self._long_speed_snapshot[1:])
        self._long_builder = None
        self._long_planner = None
        self._long_build_failed = False
        self._ego_speed = None
        self._long_last_clock_ns = None
        self.add_on_set_parameters_callback(self._validate_speed_parameters)
        self._route_key = None
        self._route_result = None
        self._progress_index = None
        self._progress_arc_m = None
        self.latest_pose = None
        self._pose_received_ns = None
        self._drive_was_enabled = False
        self._drive_session_start_ns = None
        self._drive_session_expired = False
        self._drive_session_duration_sec = None
        self._drive_last_time_ns = None
        self.fsm = PlanningFSM()
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
        self.localization_sub = self.create_subscription(
            Odometry,
            self.get_parameter('localization_topic').value,
            self.localization_callback,
            # These callbacks store a current value, not a history. A busy
            # planning cycle must not replay queued poses behind the vehicle.
            1,
        )
        self.ego_status_sub = self.create_subscription(
            EgoVehicleStatus, self.get_parameter('ego_status_topic').value,
            # MORAI publishes faster than a full route can be serialized.
            # Keep the newest sample so the header-age guard checks live data.
            self.ego_status_callback, 1,
        )
        self.long_speed_plan_pub = self.create_publisher(
            String, self.get_parameter('long_speed_plan_topic').value, 10,
        )
        self.create_service(Trigger, '~/reset_local_progress', self.reset_local_progress)

        # Planning loop: 20 Hz
        self.create_timer(
            0.05,
            self.plan,
        )

        self.get_logger().info(
            f'Planning node ready - loaded {len(self.map_matcher.links)} MGeo links; '
            'waiting for localization (drive disabled by default)'
        )

    def _validate_speed_parameters(self, parameters):
        """Validate the proposed transaction; never mutate the active planner here.

        Humble invokes this before committing values, and another callback can
        still reject the transaction after this one accepts it.
        """
        names = _SPEED_PARAMETER_NAMES + _LONG_SPEED_NAMES
        changes = {parameter.name: parameter.value for parameter in parameters
                   if parameter.name in names}
        if not changes:
            return SetParametersResult(successful=True)
        values = dict(zip(names, (p.value for p in self.get_parameters(names))))
        values.update(changes)
        error = _speed_configuration_error(tuple(values[name] for name in _SPEED_PARAMETER_NAMES))
        error = error or _long_configuration_error(tuple(values[name] for name in _LONG_SPEED_NAMES))
        return SetParametersResult(successful=not error, reason=error)

    def _refresh_speed_planner(self):
        """Apply committed settings once per change, before evaluating a path.

        The timer and parameter services share the default mutually-exclusive
        callback group, so one cycle reads a complete committed snapshot.
        """
        committed = tuple(p.value for p in self.get_parameters(_SPEED_PARAMETER_NAMES + _LONG_SPEED_NAMES))
        snapshot, long_snapshot = committed[:9], committed[9:]
        changed = snapshot != self._speed_planner_snapshot
        long_changed = long_snapshot != self._long_speed_snapshot
        geometry_changed = any(snapshot[i] != self._speed_planner_snapshot[i] for i in (4,5,6,7))
        enabled_changed = long_snapshot[0] != self._long_speed_snapshot[0]
        if changed:
            planner = _speed_planner_from_snapshot(snapshot)
            self.speed_planner = planner
            self._speed_planner_snapshot = snapshot
            self._previous_speed_mode = None
            self._previous_curve_state = None
        if changed or long_changed:
            self._long_speed_snapshot = long_snapshot
            self._long_settings = SpeedPlanSettings(*long_snapshot[1:])
            if geometry_changed or enabled_changed:
                self._reset_long_plan(rebuild=True)
            elif self._long_planner is not None:
                previous = self._long_planner
                self._long_planner = LongitudinalSpeedPlanner(
                    previous.profile, self.speed_planner, self._long_settings)
                self._long_planner.inherit_release_history(previous)
            if self._long_builder is not None:
                self._long_builder.local_planner = self.speed_planner

    def _reset_long_plan(self, rebuild=False):
        if self._long_planner is not None:
            self._long_planner.reset()
        if rebuild:
            self._long_builder = None
            self._long_planner = None
            self._long_build_failed = False

    def ego_status_callback(self, msg):
        values = (msg.velocity.x,msg.velocity.y,msg.velocity.z)
        now = self.get_clock().now().nanoseconds
        if not all(_finite_number(v) for v in values):
            self._ego_speed = None
            return
        speed = math.hypot(*values)*3.6
        stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        self._ego_speed = (speed,stamp,now) if _finite_number(speed) else None

    def _long_speed_recommendation(self, progress, local_speed, now_ns, local_mode=None):
        diag = dict(enabled=True,status='BUILDING',ready=False)
        if (_long_configuration_error(self._long_speed_snapshot)
                or _speed_configuration_error(self._speed_planner_snapshot)):
            return None,dict(diag,status='INVALID_CONFIG')
        if self._long_build_failed:
            return None,dict(diag,status='INVALID_ROUTE')
        try:
            if self._long_planner is None:
                if self._long_builder is None:
                    self._long_builder = RouteProfileBuilder(self._route_result['points'],self.speed_planner)
                profile = self._long_builder.advance()
                diag['built_windows'] = self._long_builder.next_window
                diag['total_windows'] = len(self._long_builder.centers)
                if profile is None:
                    return None,diag
                self._long_planner = LongitudinalSpeedPlanner(profile,self.speed_planner,self._long_settings)
                self._long_builder = None
            decision_ns = self.get_clock().now().nanoseconds
            if decision_ns < now_ns:
                self._ego_speed = None
                self._reset_long_plan()
                return None,dict(diag,status='CLOCK_REWIND')
            now_ns = decision_ns
            value = self._ego_speed
            if value is None:
                self._reset_long_plan()
                return None,dict(diag,status='MISSING_SPEED')
            speed,stamp,received = value
            age,receive_age = (now_ns-stamp)/1e9,(now_ns-received)/1e9
            diag.update(actual_speed_kmh=speed,speed_age_sec=age,speed_receive_age_sec=receive_age)
            if not (0 <= age <= self._long_settings.speed_timeout_sec
                    and 0 <= receive_age <= self._long_settings.speed_timeout_sec):
                self._reset_long_plan()
                return None,dict(diag,status='STALE_SPEED')
            result = self._long_planner.plan(progress,speed,local_speed,
                time_sec=now_ns/1e9,local_mode=local_mode)
            if result is None:
                return None,dict(diag,status='INVALID_INPUT')
            return result,dict(diag,**result,status='READY',ready=True)
        except (ArithmeticError,ValueError,KeyError,TypeError) as error:
            self._reset_long_plan(rebuild=True)
            self._long_build_failed = True
            return None,dict(diag,status='INVALID_ROUTE_OR_CONFIG',error=str(error))

    def localization_callback(self, msg):
        """Store the latest pose; map matching runs only in the planning timer."""
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.latest_pose = (position.x, position.y, yaw)
        self._pose_received_ns = self.get_clock().now().nanoseconds

    def reset_local_progress(self, request, response):
        """Re-anchor only while disabled and stopped, retaining the exact route."""
        response.success = False
        response.message = 'requires disabled, stationary, fresh pose/speed and a nearby route projection'
        now = self.get_clock().now().nanoseconds
        speed = self._ego_speed
        pose_stamp = self._pose_received_ns
        if (self.get_parameter('enable_drive').value is not False
                or speed is None or self.latest_pose is None or pose_stamp is None
                or not 0 <= (now-pose_stamp)/1e9 <= .3
                or not 0 <= speed[0] < .3
                or not all(0 <= (now-t)/1e9 <= self._long_settings.speed_timeout_sec
                           for t in speed[1:])
                or self._route_result is None or self._progress_arc_m is None):
            return response
        projection = reanchor_progress(self._route_result['points'], *self.latest_pose,
                                       self._progress_arc_m)
        if projection is None:
            return response
        self._progress_index = projection['nearest_index']
        self._progress_arc_m = projection['progress_arc_m']
        self._previous_speed_mode = self._previous_curve_state = None
        self._reset_long_plan()
        response.success = True
        response.message = json.dumps(projection)
        return response

    def plan(self):
        """Publish routes, recommendations and guarded, time-limited commands.

        Cache failures too, so an unreachable route does not trigger a search
        every tick. Match loss retains progress/cache but publishes no Path.
        An active route configuration change invalidates both immediately.
        """
        self._refresh_speed_planner()
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
            self._global_path_cache = None
            self._progress_index = None
            self._progress_arc_m = None
            self._previous_speed_mode = None
            self._previous_curve_state = None
            self._reset_long_plan(rebuild=True)

        now_ns = self.get_clock().now().nanoseconds
        if self._long_last_clock_ns is not None and now_ns < self._long_last_clock_ns:
            self._reset_long_plan()
            self._ego_speed = None
            self._previous_speed_mode = None
            self._previous_curve_state = None
        self._long_last_clock_ns = now_ns
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
                    frame = self.get_parameter('global_path_frame_id').value
                    cached = getattr(self, '_global_path_cache', None)
                    if (cached is None or cached[0] is not self._route_result
                            or cached[1] != frame):
                        path = Path()
                        path.header.frame_id = frame
                        # Fixed routes are immutable until the route key changes.
                        # Rebuilding thousands of PoseStamped objects every tick
                        # delays current-input callbacks beyond their age guard.
                        for point in self._route_result['points']:
                            pose = PoseStamped()
                            pose.header = path.header
                            pose.pose.position.x = float(point[0])
                            pose.pose.position.y = float(point[1])
                            pose.pose.position.z = float(point[2])
                            pose.pose.orientation.w = 1.0
                            path.poses.append(pose)
                        self._global_path_cache = (self._route_result, frame, path)
                    else:
                        path = cached[2]
                    # Every pose shares this header. Publishing serializes it
                    # synchronously within the mutually exclusive planning cycle.
                    path.header.stamp = self.get_clock().now().to_msg()
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
                            previous_curve_state=self._previous_curve_state,
                        )
                        if (speed_result is not None
                                and _finite_number(speed_result['target_speed_kmh'])
                                and speed_result['target_speed_kmh'] >= 0):
                            speed_valid = True
                            recommended_speed = speed_result['target_speed_kmh']
                            speed_mode = speed_result['speed_mode']
                            speed_curvature = speed_result['representative_curvature']
                            speed_max_curvature = speed_result['max_curvature']

        # Hysteresis belongs to speed classification only. Invalid planning
        # input or a route change must not carry a stale mode into recovery.
        self._previous_speed_mode = speed_mode if speed_valid else None
        self._previous_curve_state = (speed_result.get('curve_state') or None) if speed_valid else None

        long_diag = dict(enabled=self._long_speed_snapshot[0],status='DISABLED',ready=False)
        if self._long_speed_snapshot[0]:
            if speed_valid:
                long_result,long_diag = self._long_speed_recommendation(
                    self._progress_arc_m,recommended_speed,self.get_clock().now().nanoseconds,
                    local_mode=speed_mode)
                if long_result is None:
                    recommended_speed = 0.0
                    speed_valid = False
                    self._previous_speed_mode = None
                    self._previous_curve_state = None
                else:
                    recommended_speed = min(recommended_speed,long_result['target_speed_kmh'])
            else:
                self._reset_long_plan()
                long_diag['status'] = 'WAITING_FOR_PATH'

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

        previous_fsm_state = self.fsm.state

        fsm_state_value = self.fsm.update(
            localization_ready=self.latest_pose is not None,
            map_match_ready=status_value == 'MATCHED',
            global_path_ready=state_value == 'GLOBAL_PATH_READY',
            local_path_ready=local_result is not None,
            speed_ready=speed_valid,
            drive_status=drive_status,
            # Obstacle avoidance is intentionally disconnected.
            obstacle_avoid_requested=False,
            traffic_stop_requested=False,
            emergency_stop_requested=False,
            goal_reached=False,
        )

        if fsm_state_value != previous_fsm_state:
            self.get_logger().info(
                f'FSM transition: {previous_fsm_state} -> {fsm_state_value}'
            )

        # High-level FSM is the final motion gate.
        # Speed Planning may recommend a speed, but motion is permitted
        # only while the FSM is explicitly in DRIVING.
        if self.fsm.motion_allowed:
            target_speed.data = requested_target_speed
        else:
            target_speed.data = 0.0

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
        self.speed_mode_pub.publish(String(data=speed_mode))
        self.speed_curvature_pub.publish(Float32(data=speed_curvature))
        self.speed_max_curvature_pub.publish(Float32(data=speed_max_curvature))
        self.drive_status_pub.publish(String(data=drive_status))
        self.long_speed_plan_pub.publish(String(data=json.dumps(long_diag,allow_nan=False)))


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
