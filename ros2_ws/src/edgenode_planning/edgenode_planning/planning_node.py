import json
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float32, Float64, Int32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

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
from .lane_geometry import LaneNetwork
from .static_obstacle_planner import AvoidanceConfig, StaticObstaclePlanner
from .lattice_path import CostWeights, control_local_path
from .avoidance_geometry import load_drivable_area
from .avoidance_visualization import candidate_markers, diagnostic


class PlanningNode(Node):
    """
    Match localization and publish global and local paths at 20 Hz.

    Current stage:
      - Perception supplies map-frame grouped obstacle footprints; unknown road boundaries stop avoidance.
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

        self.declare_parameter('obstacle_topic', '/planning/obstacles')
        self.declare_parameter('obstacle_points_per_cluster', 8, ParameterDescriptor(read_only=True))
        self.declare_parameter('avoidance_boundary_file', '', ParameterDescriptor(read_only=True))
        self.declare_parameter('avoidance_observation_timeout_sec', .6, ParameterDescriptor(read_only=True))
        defaults = AvoidanceConfig()
        for name, value in defaults.__dict__.items():
            self.declare_parameter('avoidance_'+name, list(value) if isinstance(value, tuple) else value,
                                   ParameterDescriptor(read_only=True))
        values = {name:self.get_parameter('avoidance_'+name).value for name in defaults.__dict__}
        values['transition_lengths_m'] = tuple(values['transition_lengths_m'])
        weights=CostWeights()
        for name,value in weights.__dict__.items():
            self.declare_parameter('avoidance_cost_'+name,value,ParameterDescriptor(read_only=True))
        self.avoidance_planner = StaticObstaclePlanner(LaneNetwork(self.dijkstra_planner),
            config=AvoidanceConfig(**values),
            weights=CostWeights(**{name:self.get_parameter('avoidance_cost_'+name).value
                                   for name in weights.__dict__}),
            drivable_area=load_drivable_area(self.get_parameter('avoidance_boundary_file').value))
        self.latest_obstacle_groups = []
        self.latest_obstacle_valid = True
        self.obstacle_received_ns = None
        self.obstacle_stamp_ns = None
        self.latest_speed_kmh = 0.
        self.latest_turn_curvature = 0.
        self.latest_pose_stamp_ns = None
        self.latest_pose_received_ns = None
        self._validation_pending_stamp_ns = None
        self._validation_fresh_stamps = set()
        self._candidate_marker_ids = set()
        self._candidate_cache=None
        self._diagnostic_path_cache={}
        self.avoidance_state_pub = self.create_publisher(String, '/planning/avoidance_state', 10)
        self.avoidance_debug_pub = self.create_publisher(String, '/planning/avoidance_debug', 10)
        self.candidate_pub = self.create_publisher(MarkerArray, '/planning/lattice_candidate_markers', 10)
        for name in ('runtime_cruise_speed','runtime_curve_speed','runtime_sharp_curve_speed','runtime_max_drive_speed'):
            setattr(self,name+'_pub',self.create_publisher(Float32,'/planning/'+name,10))
        self._avoidance_path_publishers = {name:self.create_publisher(Path, '/planning/'+name, 10)
            for name in ('original_lane_center','left_lane_center','right_lane_center',
                         'avoidance_selected_path','avoidance_return_path')}
        self.obstacle_sub = self.create_subscription(PoseArray,
            self.get_parameter('obstacle_topic').value, self.obstacle_callback, 10)
        self.reset_avoidance_service = self.create_service(Trigger, '/planning/reset_avoidance',
                                                          self.reset_avoidance)

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

    def reset_avoidance(self, request, response):
        response.success = self.get_parameter('enable_drive').value is not True
        if response.success:
            self.avoidance_planner.reset()
            self._validation_pending_stamp_ns=None
            self._validation_fresh_stamps.clear()
        response.message = 'reset complete' if response.success else 'disable drive before resetting avoidance'
        return response

    def obstacle_callback(self, msg):
        now = self.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        points = [(p.position.x,p.position.y) for p in msg.poses]
        valid = (msg.header.frame_id == 'map' and len(points)%8 == 0
            and self.get_parameter('obstacle_points_per_cluster').value == 8
            and all(_finite_number(v) for p in points for v in p)
            and (self.obstacle_stamp_ns is None or stamp >= self.obstacle_stamp_ns))
        # Do not turn malformed/reordered observations into an apparent empty
        # free road. The planner retains previous obstacle solids on a miss.
        self.latest_obstacle_valid = valid
        if valid:
            self.latest_obstacle_groups = [points[i:i+8] for i in range(0,len(points),8)]
            self.obstacle_received_ns = now
            self.obstacle_stamp_ns = stamp

    def publish_avoidance(self, result):
        stamp = self.get_clock().now().to_msg()
        self.avoidance_state_pub.publish(String(data=result['state']))
        self.avoidance_debug_pub.publish(String(data=json.dumps(diagnostic(result),allow_nan=False)))
        key=(id(result.get('candidates')),result['state'],result.get('reason'),result.get('selected_lane'))
        cache=getattr(self,'_candidate_cache',None)
        if cache is None or cache[0]!=key:
            markers,self._candidate_marker_ids = candidate_markers(result,stamp,self._candidate_marker_ids)
            self._candidate_cache=(key,markers,result.get('candidates'))
        else:
            markers=cache[1]
            for marker in markers.markers:marker.header.stamp=stamp
        self.candidate_pub.publish(markers)
        paths = {'original_lane_center':result.get('original_lane',()),
                 'left_lane_center':result.get('left_lane',()), 'right_lane_center':result.get('right_lane',()),
                 'avoidance_selected_path':result.get('points',()) if result.get('active') else (),
                 'avoidance_return_path':result.get('return_path',()) if not result.get('blocked') else ()}
        for name, points in paths.items():
            if not hasattr(self,'_diagnostic_path_cache'):self._diagnostic_path_cache={}
            cached=self._diagnostic_path_cache.get(name)
            if cached is None or cached[0] is not points:
                msg=Path();msg.header.frame_id='map'
                for point in points:
                    p=PoseStamped();p.header=msg.header
                    p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,point)
                    p.pose.orientation.w=1.;msg.poses.append(p)
                self._diagnostic_path_cache[name]=(points,msg)
            else:msg=cached[1]
            msg.header.stamp=stamp
            for p in msg.poses:p.header.stamp=stamp
            self._avoidance_path_publishers[name].publish(msg)

    def localization_callback(self, msg):
        """Store the latest pose; map matching runs only in the planning timer."""
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.latest_pose = (position.x, position.y, yaw)
        self.latest_pose_stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.latest_pose_received_ns=self.get_clock().now().nanoseconds
        speed = msg.twist.twist.linear.x
        turn = msg.twist.twist.angular.z
        self.latest_speed_kmh = float(speed)*3.6 if _finite_number(speed) else math.nan
        self.latest_turn_curvature = (float(turn)/speed if _finite_number(speed)
            and _finite_number(turn) and abs(speed)>.05 else 0. if _finite_number(turn) and abs(turn)<.005 else math.nan)

    def plan(self):
        """Publish routes, recommendations and guarded, time-limited commands.

        Cache failures too, so an unreachable route does not trigger a search
        every tick. Match loss retains progress/cache but publishes no Path.
        An active route configuration change invalidates both immediately.
        """
        for name in ('cruise_speed_kmh', 'curve_speed_kmh', 'sharp_curve_speed_kmh'):
            setattr(self.speed_planner, name, self.get_parameter(name).value)
        avoidance_result = None
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
            if self._route_key is not None and self.avoidance_planner.state!='NORMAL':
                # A route edit must not erase a committed maneuver, remembered
                # hazard or latched STOP. Explicit disarmed reset is required.
                self.avoidance_planner.stop('ROUTE_CHANGED_DURING_AVOIDANCE')
            self._route_key = None
            self._route_result = None
            self._progress_index = None
            self._progress_arc_m = None
            self._previous_speed_mode = None
            self._validation_pending_stamp_ns=None
            self._validation_fresh_stamps.clear()

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

                        receipt = self.obstacle_received_ns
                        observation_valid = self.latest_obstacle_valid and (receipt is None or
                            0 <= (now_ns-receipt)/1e9 <= self.get_parameter('avoidance_observation_timeout_sec').value)
                        pose_receipt=getattr(self,'latest_pose_received_ns',None)
                        if pose_receipt is not None:
                            observation_valid=observation_valid and 0 <= (now_ns-pose_receipt)/1e9 <= self.get_parameter('avoidance_observation_timeout_sec').value
                        stamp=getattr(self,'latest_pose_stamp_ns',None)
                        validation_start=time.perf_counter()
                        validation_stop=False
                        control_points=None
                        try:
                            try:
                                validation_stop=enable_drive and self.avoidance_planner.needs_validation_stop(
                                    active_local_points,self.latest_obstacle_groups)
                            except (ValueError,TypeError,OverflowError):
                                # Unknown footprint extent must still reach the
                                # planner's latched rejection, not become a
                                # transient preprocessing exception/free road.
                                validation_stop=enable_drive
                            if validation_stop:
                                # Send STOP BEFORE lengthy validation, never
                                # leave the previous motion command live while
                                # searching a new maneuver/changed obstacle.
                                stop=Path();stop.header=path.header
                                self.local_path_pub.publish(stop)
                                self.target_speed_pub.publish(Float32(data=0.))
                            avoidance_result = self.avoidance_planner.plan(active_local_points,
                                self.latest_obstacle_groups,(x,y,yaw),speed_kmh=self.latest_speed_kmh,
                                actual_curvature=self.latest_turn_curvature,current_link_id=current_link,
                                route_link_ids=self._route_result.get('link_ids',()),observation_valid=observation_valid,
                                control_horizon_m=self.get_parameter('local_path_length_m').value)
                            if (not isinstance(avoidance_result,dict) or
                                    not isinstance(avoidance_result.get('blocked'),bool) or
                                    'points' not in avoidance_result or 'state' not in avoidance_result):
                                raise ValueError('missing avoidance result')
                            if not avoidance_result['blocked'] and avoidance_result.get('active'):
                                points=avoidance_result['points']
                                if (len(points)<2 or any(len(p)!=3 or not all(_finite_number(v) for v in p)
                                                       for p in points)):
                                    raise ValueError('invalid avoidance control geometry')
                                # Window extraction is part of validation:
                                # failure must latch STOP before any publish.
                                control_points=control_local_path(points,(x,y,yaw),
                                    self.get_parameter('local_path_length_m').value)
                        except Exception as error:
                            self.get_logger().error(f'Avoidance validation failed: {error}')
                            avoidance_result=self.avoidance_planner.stop('INVALID_AVOIDANCE_RESULT')
                        elapsed=time.perf_counter()-validation_start
                        avoidance_result['validation_elapsed_ms']=elapsed*1000.
                        if not avoidance_result['blocked'] and avoidance_result.get('active') and enable_drive:
                            if validation_stop or elapsed>.05:
                                self._validation_pending_stamp_ns=stamp if stamp is not None else -1
                                self._validation_fresh_stamps.clear()
                            pending=self._validation_pending_stamp_ns
                            if pending is not None:
                                if stamp is not None and stamp>pending:
                                    self._validation_fresh_stamps.add(stamp)
                                if len(self._validation_fresh_stamps)<3:
                                    # A successful check on an old pose is not
                                    # permission to drive. Require three distinct
                                    # fresh odometry frames revalidated against
                                    # the fixed path/controller before release.
                                    avoidance_result=dict(avoidance_result,state='VALIDATING',points=(),
                                        blocked=True,reason='WAIT_FRESH_VALIDATION')
                                else:
                                    self._validation_pending_stamp_ns=None
                                    self._validation_fresh_stamps.clear()
                        active_local_points = avoidance_result['points']
                        if avoidance_result['blocked']:
                            active_local_points=()
                        elif control_points is not None:
                            active_local_points=control_points

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
            self.avoidance_planner.invalidate_observation()
        if avoidance_result is None:
            avoidance_result = dict(state='BLOCKED',points=(),blocked=True,reason='PLANNING_NOT_READY',active=False)
        PlanningNode.publish_avoidance(self, avoidance_result)

        # Hysteresis belongs to speed classification only. Invalid planning
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

        previous_fsm_state = self.fsm.state

        fsm_state_value = self.fsm.update(
            localization_ready=self.latest_pose is not None,
            map_match_ready=status_value == 'MATCHED',
            global_path_ready=state_value == 'GLOBAL_PATH_READY',
            local_path_ready=local_result is not None,
            speed_ready=speed_valid,
            drive_status=drive_status,
            obstacle_avoid_requested=avoidance_result.get('active',False),
            traffic_stop_requested=False,
            emergency_stop_requested=avoidance_result['blocked'] and local_result is not None,
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
            cap = avoidance_result.get('speed_cap_kmh')
            target_speed.data = min(requested_target_speed,cap) if cap is not None else requested_target_speed
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
        for topic,parameter in (('runtime_cruise_speed','cruise_speed_kmh'),
                ('runtime_curve_speed','curve_speed_kmh'),('runtime_sharp_curve_speed','sharp_curve_speed_kmh'),
                ('runtime_max_drive_speed','max_drive_speed_kmh')):
            value=self.get_parameter(parameter).value
            getattr(self,topic+'_pub').publish(Float32(data=float(value) if _finite_number(value) else math.nan))


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
