import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType

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
from .lattice_path import CostWeights, control_local_path, curvatures
from .avoidance_geometry import load_drivable_area
from .avoidance_visualization import candidate_markers, diagnostic
from .sensor_readiness import SensorReadiness
from .s4_distance_safety import DistancePolicy, DistanceSafety, load_evidence
from .lane_geometry import arc


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
        self.declare_parameter('avoidance_boundary_approved_sha256', '', ParameterDescriptor(read_only=True))
        self.declare_parameter('avoidance_observation_timeout_sec', .6, ParameterDescriptor(read_only=True))
        for name, value in (
                ('s4_sensor_validation_enabled', False),
                ('s4_sensor_status_topic', '/perception/lidar_status'),
                ('s4_sensor_timeout_sec', .6),
                ('s4_sensor_max_source_age_sec', .6),
                ('s4_sensor_future_tolerance_sec', .05)):
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.sensor_guard = None
        if self.get_parameter('s4_sensor_validation_enabled').value:
            self.sensor_guard = SensorReadiness(
                self.get_parameter('s4_sensor_timeout_sec').value,
                self.get_parameter('s4_sensor_max_source_age_sec').value,
                self.get_parameter('s4_sensor_future_tolerance_sec').value)
        defaults = AvoidanceConfig()
        for name, value in defaults.__dict__.items():
            self.declare_parameter('avoidance_'+name, list(value) if isinstance(value, tuple) else value,
                                   ParameterDescriptor(read_only=True))
        values = {name:self.get_parameter('avoidance_'+name).value for name in defaults.__dict__}
        values['transition_lengths_m'] = tuple(values['transition_lengths_m'])
        weights=CostWeights()
        for name,value in weights.__dict__.items():
            self.declare_parameter('avoidance_cost_'+name,value,ParameterDescriptor(read_only=True))
        self.avoidance_planner = StaticObstaclePlanner(LaneNetwork(self.dijkstra_planner,
            require_explicit_permission=self.get_parameter('s4_sensor_validation_enabled').value),
            config=AvoidanceConfig(**values),
            weights=CostWeights(**{name:self.get_parameter('avoidance_cost_'+name).value
                                   for name in weights.__dict__}),
            drivable_area=load_drivable_area(self.get_parameter('avoidance_boundary_file').value,
                approved_sha256=(self.get_parameter('avoidance_boundary_approved_sha256').value
                    if self.get_parameter('s4_sensor_validation_enabled').value else None),
                mgeo_dir=mgeo_dir))
        self.distance_safety = None
        for name, value in DistancePolicy().__dict__.items():
            self.declare_parameter('s4_distance_' + name, value, ParameterDescriptor(read_only=True))
        for name in ('s4_observation_evidence_file', 's4_observation_evidence_approved_sha256',
                     's4_lidar_config_file', 's4_control_config_file'):
            self.declare_parameter(name, '', ParameterDescriptor(read_only=True))
        if self.sensor_guard is not None:
            policy = DistancePolicy(**{name: self.get_parameter('s4_distance_' + name).value
                                       for name in DistancePolicy().__dict__})
            evidence, reason = load_evidence(
                self.get_parameter('s4_observation_evidence_file').value,
                self.get_parameter('s4_observation_evidence_approved_sha256').value,
                self.get_parameter('s4_lidar_config_file').value,
                self.get_parameter('s4_control_config_file').value,
                mgeo_dir, policy, self.avoidance_planner.config)
            self.distance_safety = DistanceSafety(policy, self.avoidance_planner.config,
                self.avoidance_planner.body, evidence, reason)
            self.s4_distance_state_pub = self.create_publisher(String, '/planning/s4_distance_state', 10)
        self._s4_distance_state = {}
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
        if self.sensor_guard is not None:
            self.sensor_status_sub = self.create_subscription(String,
                self.get_parameter('s4_sensor_status_topic').value, self.sensor_status_callback, 10)
            self.s4_sensor_state_pub = self.create_publisher(String, '/planning/s4_sensor_state', 10)
            self.sensor_watchdog_timer = self.create_timer(.05, self.sensor_watchdog,
                clock=Clock(clock_type=ClockType.STEADY_TIME))
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
        self._last_planning_input = None
        self._planning_execution_id = 0
        self._planning_skipped = False
        self._last_wakeup_frame = None
        self._planning_wakeup = (self.create_guard_condition(self.planning_tick)
                                 if self.sensor_guard is not None else None)
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
            self.planning_tick if self.sensor_guard is not None else self.plan,
        )

        self.get_logger().info(
            f'Planning node ready - loaded {len(self.map_matcher.links)} MGeo links; '
            'waiting for localization (drive disabled by default)'
        )

    def reset_avoidance(self, request, response):
        response.success = self.get_parameter('enable_drive').value is not True
        guard = getattr(self, 'sensor_guard', None)
        if response.success and guard is not None and self.sensor_reason(self.get_clock().now().nanoseconds):
            response.success = False
            response.message = 'fresh paired S4 sensor data required before reset'
            return response
        if response.success and guard is not None and self.distance_reason(self.get_clock().now().nanoseconds):
            response.success = False
            response.message = 'validated S4 observation and stopping distance required before reset'
            return response
        if response.success:
            self.avoidance_planner.reset()
            self._last_planning_input = None
            self._validation_pending_stamp_ns=None
            self._validation_fresh_stamps.clear()
        response.message = 'reset complete' if response.success else 'disable drive before resetting avoidance'
        return response

    def obstacle_callback(self, msg):
        now = self.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        points = [(p.position.x,p.position.y) for p in msg.poses]
        guard = getattr(self, 'sensor_guard', None)
        if guard is not None:
            reason = (guard.observation(stamp, points, now) if msg.header.frame_id == 'map'
                      else 'INVALID_OBSTACLE_FRAME')
            if reason:
                guard.failure = reason
                self.stop_for_sensor(reason)
            else:
                self.wake_planning_for_frame()
            return
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

    def stop_for_sensor(self, reason):
        return self.stop_for_safety('S4_SENSOR_' + reason)

    def stop_for_safety(self, reason):
        # A recovered producer must never clear an existing avoidance latch.
        if self.avoidance_planner.state != 'BLOCKED':
            result = self.avoidance_planner.stop(reason)
        else:
            result = self.avoidance_planner.stop(self.avoidance_planner.reason)
        stop = Path()
        stop.header.frame_id = 'map'
        stop.header.stamp = self.get_clock().now().to_msg()
        self.local_path_pub.publish(stop)
        self.target_speed_pub.publish(Float32(data=0.))
        self._validation_pending_stamp_ns = None
        self._validation_fresh_stamps.clear()
        return result

    def sensor_status_callback(self, msg):
        guard = getattr(self, 'sensor_guard', None)
        if guard is not None:
            now = self.get_clock().now().nanoseconds
            reason = guard.receive(msg.data, now)
            if reason:
                self.stop_for_sensor(reason)
            self.refresh_sensor(now)
            self.wake_planning_for_frame()

    def sensor_watchdog(self):
        self.refresh_sensor(self.get_clock().now().nanoseconds)

    def wake_planning_for_frame(self):
        """Wake the same serialized callback group on a complete fresh frame.

        The 20 Hz timer remains the safety/lease fallback. Do not spend another
        timer period ageing a newly paired observation before a long search.
        Heartbeats and unmatched half-frames never create a wakeup.
        """
        wakeup = getattr(self, '_planning_wakeup', None)
        guard = getattr(self, 'sensor_guard', None)
        if wakeup is None or guard is None or guard.matched is None:
            return
        now = self.get_clock().now().nanoseconds
        status = guard.matched[0]
        pose_stamp = getattr(self, 'latest_pose_stamp_ns', None)
        if (pose_stamp is None or pose_stamp < status['source_stamp_ns']-guard.future_ns
                or self.sensor_reason(now)):
            return
        key = (guard.session_id, status['frame_seq'], status['source_stamp_ns'], status['obstacle_digest'])
        if key != self._last_wakeup_frame:
            self._last_wakeup_frame = key
            wakeup.trigger()

    def planning_tick(self):
        """Coalesce identical S4 inputs without refreshing a command or frame.

        All callbacks still use the single mutually exclusive callback group.
        No worker/thread owns a second mutable planner. A due timer on unchanged
        input does only the existing safety checks, allowing ready sensor work
        to run instead of repeating an expensive validation on the same frame.
        """
        guard = getattr(self, 'sensor_guard', None)
        self._planning_skipped = False
        if guard is None:
            self.plan()
            return
        now = self.get_clock().now().nanoseconds
        self.refresh_sensor(now)
        route = (getattr(self, '_route_result', None) or {}).get('link_ids', ())
        self.refresh_distance(time.perf_counter(), route)
        matched = guard.matched[0] if guard.matched else {}
        parameters = tuple(repr(self.get_parameter(name).value) for name in (
            'route_node_ids', 'goal_node_id', 'enable_drive', 'max_drive_speed_kmh',
            'drive_duration_sec', 'cruise_speed_kmh', 'curve_speed_kmh', 'sharp_curve_speed_kmh',
            'global_path_frame_id', 'local_path_length_m', 'local_path_resample_spacing_m',
            'local_path_search_back', 'local_path_search_forward'))
        start = self._drive_session_start_ns
        duration = self._drive_session_duration_sec
        lease_expired = (start is not None and duration is not None
                         and now-start >= duration*1e9)
        key = (guard.session_id, guard.fault_seq,
            tuple(matched.get(name) for name in ('frame_seq', 'source_stamp_ns', 'obstacle_digest')),
            self.latest_pose_stamp_ns, self.latest_pose, self.latest_speed_kmh, self.latest_turn_curvature,
            id(self.avoidance_planner), id(self.avoidance_planner.network),
            id(self.avoidance_planner.drivable_area), self.avoidance_planner.body,
            self.avoidance_planner.config,
            self.distance_safety.evidence if self.distance_safety is not None else None,
            self.avoidance_planner.state == 'BLOCKED', parameters, lease_expired)
        if key == self._last_planning_input:
            self._planning_skipped = True
            return
        self._last_planning_input = key
        self._planning_execution_id += 1
        try:
            self.plan()
        except Exception:
            self._last_planning_input = None
            raise

    def sensor_reason(self, now_ns):
        guard = getattr(self, 'sensor_guard', None)
        if guard is None:
            return ''
        reason = guard.reason(now_ns)
        if reason:
            return reason
        stamp = getattr(self, 'latest_pose_stamp_ns', None)
        if stamp is None or self.latest_pose is None:
            return 'PLANNING_LOCALIZATION_UNAVAILABLE'
        if not getattr(self, '_s4_pose_valid', False):
            return 'PLANNING_LOCALIZATION_INVALID'
        if not -guard.future_ns <= now_ns - stamp <= guard.max_age_ns:
            return 'PLANNING_LOCALIZATION_STALE'
        return ''

    def refresh_sensor(self, now_ns):
        guard = getattr(self, 'sensor_guard', None)
        if guard is None:
            return
        reason = self.sensor_reason(now_ns)
        if reason:
            self.stop_for_sensor(reason)
        else:
            self.latest_obstacle_groups = guard.groups()
            self.latest_obstacle_valid = True
            self.obstacle_received_ns = guard.matched[2]
            self.obstacle_stamp_ns = guard.matched[0]['source_stamp_ns']
        self.s4_sensor_state_pub.publish(String(data=json.dumps(dict(
            enabled=True, ready=not reason, reason=reason,
            session_id=guard.session_id,
            source_stamp_ns=guard.matched[0]['source_stamp_ns'] if guard.matched else None))))

    def distance_reason(self, now_ns, *, elapsed_s=0., route_link_ids=()):
        safety = getattr(self, 'distance_safety', None)
        if safety is None:
            return ''
        guard = self.sensor_guard
        if self.sensor_reason(now_ns):
            return ''  # Sensor gate supplies the more specific first failure.
        age = max(0., (now_ns - guard.matched[0]['source_stamp_ns']) / 1e9,
                  (now_ns - self.latest_pose_stamp_ns) / 1e9)
        result = safety.evaluate(self.latest_speed_kmh, age, self.latest_turn_curvature, route_link_ids)
        result['planning_elapsed_s'] = elapsed_s
        if elapsed_s > safety.policy.planning_budget_s:
            result.update(ready=False, reason='PLANNING_BUDGET_EXCEEDED')
        self._s4_distance_state = result
        self.s4_distance_state_pub.publish(String(data=json.dumps(result, allow_nan=False)))
        return result['reason']

    def refresh_distance(self, started, route_link_ids=()):
        if getattr(self, 'distance_safety', None) is None:
            return None
        reason = self.distance_reason(self.get_clock().now().nanoseconds,
            elapsed_s=time.perf_counter() - started, route_link_ids=route_link_ids)
        if reason:
            return self.stop_for_safety('S4_DISTANCE_' + reason)
        return None

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
        guard = getattr(self, 'sensor_guard', None)
        if guard is not None:
            source_stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            previous_stamp = getattr(self, 'latest_pose_stamp_ns', None)
            norm = math.hypot(q.x, q.y, q.z, q.w)
            now_ns = self.get_clock().now().nanoseconds
            self._s4_pose_valid = (msg.header.frame_id.lstrip('/') == 'map' and
                all(_finite_number(v) for v in (position.x, position.y, q.x, q.y, q.z, q.w)) and
                math.isclose(norm, 1., abs_tol=1e-3) and source_stamp > 0 and
                (previous_stamp is None or source_stamp > previous_stamp) and
                -guard.future_ns <= now_ns - source_stamp <= guard.max_age_ns)
            if not self._s4_pose_valid:
                self.stop_for_sensor('PLANNING_LOCALIZATION_INVALID')
                return
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
        self.wake_planning_for_frame()

    def plan(self):
        """Publish routes, recommendations and guarded, time-limited commands.

        Cache failures too, so an unreachable route does not trigger a search
        every tick. Match loss retains progress/cache but publishes no Path.
        An active route configuration change invalidates both immediately.
        """
        # Whole timer duration includes route extraction, global path publication
        # and candidate search. Steady time also covers a paused ROS clock.
        s4_started = time.perf_counter() if getattr(self, 'distance_safety', None) else None
        PlanningNode.refresh_sensor(self, self.get_clock().now().nanoseconds)
        if s4_started is not None:
            self.refresh_distance(s4_started)
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
                        hazard_points = active_local_points
                        s4_kwargs = {}
                        control_horizon = self.get_parameter('local_path_length_m').value
                        if s4_started is not None:
                            policy = self.distance_safety.policy
                            preview = self.local_path_planner.extract(
                                self._route_result['points'], x, y, horizon_m=policy.preview_m,
                                previous_index=self._progress_index,
                                search_back=self.get_parameter('local_path_search_back').value,
                                search_forward=self.get_parameter('local_path_search_forward').value,
                                resample_spacing_m=self.get_parameter('local_path_resample_spacing_m').value,
                                previous_arc_m=self._progress_arc_m)
                            hazard_points = preview['points'] if preview else ()
                            self.refresh_distance(s4_started, self._route_result.get('link_ids', ()))
                            metrics = self._s4_distance_state
                            if (not hazard_points or arc(hazard_points)[-1] < metrics.get('stopping_required_m', math.inf)):
                                self.stop_for_safety('S4_DISTANCE_MAP_STOPPING_SUPPORT')
                            evidence = self.distance_safety.evidence
                            if evidence and hazard_points and metrics.get('stopping_required_m'):
                                corridor = control_local_path(hazard_points, (x, y, yaw), metrics['stopping_required_m'])
                                if max(map(abs, curvatures(corridor)), default=math.inf) > evidence.max_curvature:
                                    self.stop_for_safety('S4_DISTANCE_OBSERVATION_CURVATURE_SCOPE')
                            control_horizon = policy.control_horizon_m
                            s4_kwargs = dict(hazard_reference=hazard_points,
                                entry_reserve_m=metrics.get('entry_reserve_m', 0.),
                                stopping_required_m=metrics.get('stopping_required_m'),
                                entry_observation_m=self.distance_safety.evidence.forward_range_m
                                    if self.distance_safety.evidence else 0.,
                                observation_curvature_limit=self.distance_safety.evidence.max_curvature
                                    if self.distance_safety.evidence else 0.)

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
                                    hazard_points,self.latest_obstacle_groups)
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
                                control_horizon_m=control_horizon, **s4_kwargs)
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
                                    control_horizon)
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
                        guard = getattr(self, 'sensor_guard', None)
                        sensor_reason = self.sensor_reason(self.get_clock().now().nanoseconds) if guard else ''
                        if sensor_reason:
                            avoidance_result = self.stop_for_sensor(sensor_reason)
                        if s4_started is not None:
                            distance_stop = self.refresh_distance(s4_started, self._route_result.get('link_ids', ()))
                            if distance_stop:
                                avoidance_result = distance_stop
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

        guard = getattr(self, 'sensor_guard', None)
        sensor_reason = self.sensor_reason(self.get_clock().now().nanoseconds) if guard else ''
        if sensor_reason:
            avoidance_result = self.stop_for_sensor(sensor_reason)
        if s4_started is not None:
            distance_stop = self.refresh_distance(s4_started,
                self._route_result.get('link_ids', ()) if self._route_result else ())
            if distance_stop:
                avoidance_result = distance_stop
                self.publish_avoidance(avoidance_result)
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

        if s4_started is not None:
            target_speed.data = min(target_speed.data, self.distance_safety.policy.speed_cap_kmh)

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
