import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from morai_ros2_msgs.msg import EgoVehicleStatus
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
from .avoidance_visualization import candidate_markers, diagnostic
from .sensor_readiness import SensorReadiness
from .return_search import ReturnSearch
from .s4_distance_safety import DistancePolicy, DistanceSafety, load_evidence
from .lane_geometry import arc
from .route_resume import mission_suffix, link_index_at_progress
from .morai_speed_safety import stopping_speed_cap, required_deceleration_distance


# Only freshness/initial-pair gaps may suspend a maneuver temporarily. Invalid
# content, clock/replay/session faults and collision stops remain latched.
TRANSIENT_SENSOR_REASONS=frozenset((
    'SENSOR_STATUS_NEVER_RECEIVED','WAITING_FOR_INPUT','SENSOR_FRAME_UNPAIRED',
    'SENSOR_STATUS_TIMEOUT','SENSOR_STATUS_STALE','OBSTACLE_FRAME_TIMEOUT',
    'SENSOR_INPUT_TIMEOUT','SENSOR_SOURCE_STALE','INPUT_TIMEOUT','STALE_INPUT',
    'LOCALIZATION_UNAVAILABLE','PLANNING_LOCALIZATION_UNAVAILABLE',
    'PLANNING_LOCALIZATION_STALE','EGO_STATUS_STALE','IMU_UNAVAILABLE','SENSOR_IMU_UNAVAILABLE'))


class PlanningNode(Node):
    """
    Match localization and publish global and local paths at 20 Hz.

    Current stage:
      - Perception supplies map-frame grouped obstacle footprints for staged avoidance.
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
                ('route_start_link_id', ''),
                ('route_link_occurrence', -1),
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
        for name,value in (('speed_anticipatory_enabled',False),('speed_deceleration_mps2',2.),
                ('speed_reaction_time_s',.6),('speed_max_lateral_acceleration_mps2',1.5),
                ('morai_speed_admission_enabled',False),('morai_validated_speed_cap_kmh',5.),
                ('morai_verified_detection_distance_m',20.),('morai_braking_delay_s',1.6)):
            self.declare_parameter(name,value,ParameterDescriptor(read_only=True))
        self.speed_planner.anticipatory=self.get_parameter('speed_anticipatory_enabled').value
        self.speed_planner.braking_deceleration_mps2=self.get_parameter('speed_deceleration_mps2').value
        self.speed_planner.reaction_time_s=self.get_parameter('speed_reaction_time_s').value
        self.speed_planner.max_lateral_acceleration_mps2=self.get_parameter('speed_max_lateral_acceleration_mps2').value
        self._morai_speed_policy=None
        if self.get_parameter('morai_speed_admission_enabled').value:
            self._morai_speed_policy={name:self.get_parameter('morai_'+name).value for name in
                ('validated_speed_cap_kmh','verified_detection_distance_m','braking_delay_s')}
            if not all(_finite_number(v) and v>0 for v in self._morai_speed_policy.values()):
                raise ValueError('positive measured MORAI speed/range/delay limits required')

        self.declare_parameter('obstacle_topic', '/planning/obstacles')
        self.declare_parameter('obstacle_points_per_cluster', 8, ParameterDescriptor(read_only=True))
        self.declare_parameter('avoidance_observation_timeout_sec', .6, ParameterDescriptor(read_only=True))
        for name, value in (
                ('s4_sensor_validation_enabled', False),
                ('s4_distance_validation_enabled', True),
                ('s4_sensor_status_topic', '/perception/lidar_status'),
                ('s4_sensor_timeout_sec', .6),
                ('s4_sensor_max_source_age_sec', .6),
                ('s4_sensor_future_tolerance_sec', .05)):
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.sensor_guard = None
        self._sensor_pause=None
        self._sensor_pause_offset_ns=0
        self._sensor_resume_validation=False
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
        self.declare_parameter('avoidance_require_explicit_permission',
            self.get_parameter('s4_sensor_validation_enabled').value,ParameterDescriptor(read_only=True))
        self.avoidance_planner = StaticObstaclePlanner(LaneNetwork(self.dijkstra_planner,
            require_explicit_permission=self.get_parameter('avoidance_require_explicit_permission').value),
            config=AvoidanceConfig(**values),
            weights=CostWeights(**{name:self.get_parameter('avoidance_cost_'+name).value
                                   for name in weights.__dict__}))
        self.distance_safety = None
        for name, value in DistancePolicy().__dict__.items():
            self.declare_parameter('s4_distance_' + name, value, ParameterDescriptor(read_only=True))
        for name in ('s4_observation_evidence_file', 's4_observation_evidence_approved_sha256',
                     's4_lidar_config_file', 's4_control_config_file'):
            self.declare_parameter(name, '', ParameterDescriptor(read_only=True))
        if self.sensor_guard is not None and self.get_parameter('s4_distance_validation_enabled').value:
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
        self.declare_parameter('avoidance_use_ego_status',False,ParameterDescriptor(read_only=True))
        self._use_ego_status=self.get_parameter('avoidance_use_ego_status').value
        self._ego_status_received_ns=None
        if self._use_ego_status:
            self.ego_status_sub=self.create_subscription(EgoVehicleStatus,'/ego_vehicle_status',
                                                         self.ego_status_callback,1)
        self.latest_pose_stamp_ns = None
        self.latest_pose_received_ns = None
        self._validation_pending_stamp_ns = None
        self._validation_fresh_stamps = set()
        self._return_search = ReturnSearch()
        self._candidate_marker_ids = set()
        self._candidate_cache=None
        self._diagnostic_path_cache={}
        self.avoidance_state_pub = self.create_publisher(String, '/planning/avoidance_state', 10)
        self.avoidance_stage_pub = self.create_publisher(String, '/planning/avoidance_stage', 10)
        self.avoidance_debug_pub = self.create_publisher(String, '/planning/avoidance_debug', 10)
        self.candidate_pub = self.create_publisher(MarkerArray, '/planning/lattice_candidate_markers', 10)
        for name in ('runtime_cruise_speed','runtime_curve_speed','runtime_sharp_curve_speed','runtime_max_drive_speed'):
            setattr(self,name+'_pub',self.create_publisher(Float32,'/planning/'+name,10))
        self._avoidance_path_publishers = {name:self.create_publisher(Path, '/planning/'+name, 10)
            for name in ('original_lane_center','left_lane_center','right_lane_center',
                         'avoidance_selected_path','avoidance_return_path')}
        self.obstacle_sub = self.create_subscription(PoseArray,
            self.get_parameter('obstacle_topic').value, self.obstacle_callback, 5 if self.sensor_guard else 10)
        if self.sensor_guard is not None:
            self.sensor_status_sub = self.create_subscription(String,
                self.get_parameter('s4_sensor_status_topic').value, self.sensor_status_callback, 5)
            self.s4_sensor_state_pub = self.create_publisher(String, '/planning/s4_sensor_state', 10)
            self.sensor_watchdog_timer = self.create_timer(.05, self.sensor_watchdog,
                clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.reset_avoidance_service = self.create_service(Trigger, '/planning/reset_avoidance',
                                                          self.reset_avoidance)

        self._previous_speed_mode = None
        self._mission_routing_enabled=True
        self._relocation=None;self._relocation_stamps=set();self._last_reset_event=None
        self._reset_history=[];self._route_error=''
        self._safety_history=[]
        self.declare_parameter('s4_control_horizon_m',20.,ParameterDescriptor(read_only=True))
        self._s4_control_horizon=self.get_parameter('s4_control_horizon_m').value
        self.declare_parameter('max_local_path_start_distance_m',3.5,ParameterDescriptor(read_only=True))
        self._max_local_path_start_distance=self.get_parameter('max_local_path_start_distance_m').value
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
        self.create_subscription(String,'/localization/reset_event',self.localization_reset_callback,
            QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.diagnostics_pub=self.create_publisher(String,'/planning/diagnostics',10)

        self.declare_parameter('planning_period_sec',.05,ParameterDescriptor(read_only=True))
        period=self.get_parameter('planning_period_sec').value
        if not _finite_number(period) or not 0<period<=.2:
            raise ValueError('planning_period_sec must be in (0, .2]')
        self.create_timer(
            period,
            self.plan,
        )

        self.get_logger().info(
            f'Planning node ready - loaded {len(self.map_matcher.links)} MGeo links; '
            'waiting for localization (drive disabled by default)'
        )

    def destroy_node(self):
        self._return_search.close()
        return super().destroy_node()

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
            self._return_search.cancel()
            self._sensor_pause=None
            self._sensor_pause_offset_ns=0
            self._sensor_resume_validation=False
            self.avoidance_planner.reset()
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

    def _sensor_hold_result(self):
        result=dict(self.avoidance_planner._result(),state='SENSOR_HOLD',blocked=True,
                    points=(),speed_cap_kmh=0.,reason='S4_SENSOR_'+self._sensor_pause['reason'])
        stop=Path();stop.header.frame_id='map';stop.header.stamp=self.get_clock().now().to_msg()
        self.local_path_pub.publish(stop);self.target_speed_pub.publish(Float32(data=0.))
        return result

    def _finish_sensor_pause(self):
        pause=self._sensor_pause
        if pause is not None:
            # ROS elapsed time freezes the maneuver timer; steady elapsed time
            # below bounds recovery even with a paused simulator clock.
            self._sensor_pause_offset_ns+=max(0,self.get_clock().now().nanoseconds-pause['ros_started_ns'])
        self._sensor_pause=None

    def stop_for_sensor(self, reason):
        if reason not in TRANSIENT_SENSOR_REASONS or self.avoidance_planner.state=='BLOCKED':
            return self.stop_for_safety('S4_SENSOR_'+reason)
        now=self.get_clock().now().nanoseconds;steady=time.monotonic_ns()
        if self._sensor_pause is None:
            guard=self.sensor_guard
            frame=(guard.session_id,guard.matched[0]['frame_seq']) if guard.matched else None
            self._sensor_pause=dict(reason=reason,ros_started_ns=now,steady_started_ns=steady,
                                    frame=frame,confirmed=0,good_since_ns=None)
            self._validation_pending_stamp_ns=None;self._validation_fresh_stamps.clear()
        pause=self._sensor_pause
        pause.update(reason=reason,confirmed=0,good_since_ns=None)
        if steady-pause['steady_started_ns']>2_000_000_000:
            return self.stop_for_safety('S4_SENSOR_RECOVERY_TIMEOUT')
        return self._sensor_hold_result()

    def _advance_sensor_recovery(self,now_ns):
        pause=self._sensor_pause
        if pause is None:return
        steady=time.monotonic_ns()
        if steady-pause['steady_started_ns']>2_000_000_000:
            self.stop_for_safety('S4_SENSOR_RECOVERY_TIMEOUT');return
        guard=self.sensor_guard;frame=(guard.session_id,guard.matched[0]['frame_seq'])
        if frame!=pause['frame']:
            pause['frame']=frame;pause['confirmed']+=1
            if pause['good_since_ns'] is None:pause['good_since_ns']=steady
        stationary=_finite_number(self.latest_speed_kmh) and abs(self.latest_speed_kmh)<=.2
        if (pause['confirmed']>=3 and steady-pause['good_since_ns']>=250_000_000
                and stationary and self.avoidance_planner.state!='BLOCKED'):
            self._finish_sensor_pause()
            # Clearing the sensor hold alone never publishes motion. The normal
            # plan must revalidate geometry/controller on three fresh poses.
            self._sensor_resume_validation=True
            self._validation_pending_stamp_ns=self.latest_pose_stamp_ns
            self._validation_fresh_stamps.clear()

    def _stationary_route_edit(self):
        now=self.get_clock().now().nanoseconds
        guard=self.sensor_guard
        timeout=guard.timeout_ns if guard else int(self.get_parameter('avoidance_observation_timeout_sec').value*1e9)
        receipt=getattr(self,'latest_pose_received_ns',None)
        fresh=receipt is not None and 0<=now-receipt<=timeout
        if guard:
            stamp=self.latest_pose_stamp_ns
            fresh=fresh and getattr(self,'_s4_pose_valid',False) and stamp is not None and -guard.future_ns<=now-stamp<=guard.max_age_ns
        if getattr(self,'_use_ego_status',False):
            receipt=self._ego_status_received_ns
            fresh=fresh and receipt is not None and 0<=now-receipt<=timeout
        return (self.get_parameter('enable_drive').value is not True and fresh
                and _finite_number(self.latest_speed_kmh) and abs(self.latest_speed_kmh)<=.2)

    def stop_for_safety(self, reason):
        self._return_search.cancel()
        self._finish_sensor_pause()
        self._sensor_resume_validation=False
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

    def sensor_watchdog(self):
        self.refresh_sensor(self.get_clock().now().nanoseconds)

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
        if getattr(self,'_use_ego_status',False):
            receipt=self._ego_status_received_ns
            if receipt is None or not 0<=now_ns-receipt<=guard.timeout_ns:
                return 'EGO_STATUS_STALE'
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
            self._advance_sensor_recovery(now_ns)
        self.s4_sensor_state_pub.publish(String(data=json.dumps(dict(
            enabled=True, ready=not reason, reason=reason,
            recovery_pending=self._sensor_pause is not None,
            recovery_frames=self._sensor_pause['confirmed'] if self._sensor_pause else 0,
            latched_reason=self.avoidance_planner.reason if self.avoidance_planner.state=='BLOCKED' else '',
            matched_source_age_sec=(now_ns-guard.matched[0]['source_stamp_ns'])/1e9 if guard.matched else None,
            session_id=guard.session_id,
            source_stamp_ns=guard.matched[0]['source_stamp_ns'] if guard.matched else None,
            latest_status_stamp_ns=guard.status['source_stamp_ns'] if guard.status else None))))

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
        stage=result.get('stage',result['state'])
        state_key=(stage,result['state'],result.get('reason',''))
        if getattr(self,'_last_avoidance_state_key',None)!=state_key:
            overlay=f' [{state_key[1]}]' if state_key[1]!=stage else ''
            self.get_logger().info(f'S4 {stage}{overlay} {state_key[2]}')
            if state_key[2]=='NO_VALID_ADJACENT_LANE':
                self.get_logger().info('S4 entry context: '+json.dumps(result.get('entry_context',{})))
            history=getattr(self,'_safety_history',None)
            if history is not None and result['state']=='BLOCKED' and result.get('reason')!='PLANNING_NOT_READY':
                history.append(dict(stamp_ns=self.get_clock().now().nanoseconds,
                    reason=result.get('reason'),pose=getattr(self,'latest_pose',None),
                    entry_context=result.get('entry_context',{})))
                del history[:-20]
            self._last_avoidance_state_key=state_key
        self.avoidance_state_pub.publish(String(data=result['state']))
        self.avoidance_stage_pub.publish(String(data=stage))
        debug=diagnostic(result);debug.setdefault('stage',stage)
        self.avoidance_debug_pub.publish(String(data=json.dumps(debug,allow_nan=False)))
        key=(id(result.get('candidates')),stage,result['state'],result.get('reason'),result.get('selected_lane'))
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

    def ego_status_callback(self,msg):
        # MORAI native ROS2 front_steer_angle is right-positive degrees.
        # Stationary SIM measurement: command +.02 -> measured +.7 degrees.
        angle=math.radians(msg.front_steer_angle)/self.avoidance_planner.config.steering_sign
        self._ego_status_received_ns=self.get_clock().now().nanoseconds
        if not math.isfinite(angle) or abs(angle)>self.avoidance_planner.config.max_wheel_angle_rad+1e-6:
            self.latest_turn_curvature=math.nan
            self.stop_for_safety('INVALID_EGO_STEERING')
            return
        self.latest_turn_curvature=math.tan(angle)/self.avoidance_planner.config.wheelbase_m
        velocity=msg.velocity
        if not all(_finite_number(v) for v in (velocity.x,velocity.y,velocity.z)):
            self.latest_speed_kmh=math.nan;self.stop_for_safety('INVALID_EGO_SPEED');return
        speed=math.hypot(velocity.x,velocity.y,velocity.z)
        # Native MORAI velocity is vehicle-local; retain meaningful reverse.
        self.latest_speed_kmh=3.6*(-speed if velocity.x<-.02 else 0. if velocity.x<0. else speed)

    def localization_reset_callback(self,msg):
        try:
            event=json.loads(msg.data)
            if event.get('schema_version')!=1 or event.get('state') not in ('PENDING','CONFIRMED','CANCELLED'):
                return
            key=(event['session_id'],event['seq'],event['state'])
            if key==self._last_reset_event:return
            age=(self.get_clock().now().nanoseconds-event['stamp_ns'])/1e9
            if not 0<=age<=2.:return
            if event['state']=='CONFIRMED' and (len(event.get('position_xy',()))!=2 or
                    not all(_finite_number(v) for v in event['position_xy'])):return
        except (ValueError,KeyError,TypeError):return
        self._last_reset_event=key;self._reset_history.append(event)
        self._reset_history=self._reset_history[-20:]
        self.set_parameters([Parameter('enable_drive',value=False)])
        self._return_search.cancel()
        # A pose reset invalidates a maneuver, never an existing hard latch.
        if event['state']=='CONFIRMED' and self.avoidance_planner.state!='BLOCKED':self.avoidance_planner.reset()
        self._relocation=event;self._relocation_stamps.clear()
        self._validation_pending_stamp_ns=None;self._validation_fresh_stamps.clear()
        self._publish_relocalization_stop()
        if event['state']=='CONFIRMED':
            self._progress_index=None;self._progress_arc_m=None
            self._route_key=None;self._route_result=None;self._previous_speed_mode=None
            self.map_matcher.previous_link_id=None
        self.get_logger().info('Planning relocation: '+json.dumps(event))

    def _publish_relocalization_stop(self):
        path=Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        self.target_speed_pub.publish(Float32(data=0.));self.local_path_pub.publish(path)
        self.drive_status_pub.publish(String(data='DISABLED'))
        self.fsm_state_pub.publish(String(data='WAIT_FOR_LOCALIZATION'))
        self.state_pub.publish(String(data='RELOCALIZING'))
        self.publish_avoidance(dict(state='RELOCALIZING',stage='RELOCALIZING',blocked=True,
            points=(),active=False,reason='LOCALIZATION_RESET',reset_event=self._relocation))

    def _relocation_ready(self):
        event=self._relocation
        if event['state']=='PENDING' or self.latest_pose is None:return False
        now=self.get_clock().now().nanoseconds
        stamp=getattr(self,'latest_pose_stamp_ns',0)
        if stamp<=event['stamp_ns'] or not 0<=now-stamp<=600_000_000:return False
        if event['state']=='CONFIRMED' and math.dist(self.latest_pose[:2],event['position_xy'])>.75:return False
        if self.sensor_guard is not None and self.sensor_reason(now):return False
        if self.sensor_guard is not None and (self.sensor_guard.matched is None or
                self.sensor_guard.matched[0]['source_stamp_ns']<=event['stamp_ns']):return False
        self._relocation_stamps.add(stamp)
        return len(self._relocation_stamps)>=3

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
        # EKF roundoff near rest is not reverse motion; meaningful reverse
        # velocity still reaches the planner's rejection unchanged.
        if not getattr(self,'_use_ego_status',False):
            self.latest_speed_kmh = (0. if -.001 <= speed < 0. else float(speed)*3.6) if _finite_number(speed) else math.nan
            self.latest_turn_curvature = (float(turn)/speed if _finite_number(speed)
                and _finite_number(turn) and abs(speed)>.05 else 0. if _finite_number(turn) and abs(turn)<.005 else math.nan)

    def plan(self):
        """Publish routes, recommendations and guarded, time-limited commands.

        Cache failures too, so an unreachable route does not trigger a search
        every tick. Match loss retains progress/cache but publishes no Path.
        An active route configuration change invalidates both immediately.
        """
        plan_started=time.monotonic()
        if getattr(self,'_relocation',None) is not None:
            if not self._relocation_ready():
                self._publish_relocalization_stop();return
            self._relocation=None
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
        mission_start='';mission_occurrence=-1
        if getattr(self,'_mission_routing_enabled',False):
            mission_start=self.get_parameter('route_start_link_id').value
            mission_occurrence=self.get_parameter('route_link_occurrence').value
            if mission_start and use_waypoints:route_key=(*route_key,mission_start,mission_occurrence)
        if route_key != self._route_key:
            if self._route_key is not None and self.avoidance_planner.state!='NORMAL':
                # An operator may replace a mission while disarmed and parked.
                # Preserve genuine hard latches; never relabel them as a route fault.
                if self.avoidance_planner.state!='BLOCKED':
                    if self._stationary_route_edit():
                        self.avoidance_planner.reset()
                        self._sensor_resume_validation=False
                    else:
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
                        self._route_error=''
                        if mission_start:
                            try:
                                self._route_result=mission_suffix(self.dijkstra_planner,mission_start,
                                    route_node_ids,current_link,mission_occurrence)
                            except (ValueError,TypeError,KeyError,IndexError) as error:
                                self._route_result=None;self._route_error=str(error)
                                self.get_logger().warning('Mission resume rejected: '+str(error))
                        else:
                            self._route_result = self.dijkstra_planner.plan_route(current_link, route_node_ids)
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
                    # The route is fixed between editor/goal changes. Reuse its
                    # ROS message instead of allocating thousands of poses at
                    # every tick and starving the sensor callbacks.
                    key=(self._route_key,id(self._route_result),path.header.frame_id)
                    cached=getattr(self,'_global_path_cache',None)
                    if cached is None or cached[0]!=key:
                        global_path=Path();global_path.header=path.header
                        for point in self._route_result['points']:
                            pose=PoseStamped();pose.header=global_path.header
                            pose.pose.position.x=float(point[0])
                            pose.pose.position.y=float(point[1])
                            pose.pose.position.z=float(point[2])
                            pose.pose.orientation.w=1.
                            global_path.poses.append(pose)
                        self._global_path_cache=(key,global_path)
                    else:
                        global_path=cached[1]
                        # Pose headers share this header object from creation.
                        global_path.header.stamp=path.header.stamp
                    # A fixed full mission is display/reference data. Its ROS
                    # serialization can dominate the 10 Hz safety callback on
                    # K-City's long route even when Python poses are cached.
                    # S4 refreshes it at 1 Hz (immediately on route changes);
                    # local control paths and safety inputs keep their rate.
                    published=getattr(self,'_global_path_published_ns',None)
                    if (getattr(self,'sensor_guard',None) is None or cached is None or cached[0]!=key
                            or published is None or now_ns<published or now_ns-published>=1_000_000_000):
                        self.global_path_pub.publish(global_path)
                        self._global_path_published_ns=now_ns

                    prepared=getattr(self,'_prepared_route',None)
                    if prepared is None or prepared[0] is not self._route_result:
                        self._prepared_route=(self._route_result,
                            self.local_path_planner.prepare(self._route_result['points']))
                    local_result = self.local_path_planner.extract(
                        self._prepared_route[1], x, y,
                        horizon_m=self.get_parameter('local_path_length_m').value,
                        previous_index=self._progress_index,
                        search_back=self.get_parameter('local_path_search_back').value,
                        search_forward=self.get_parameter('local_path_search_forward').value,
                        resample_spacing_m=self.get_parameter('local_path_resample_spacing_m').value,
                        previous_arc_m=self._progress_arc_m,
                    )
                    if (local_result is not None and self.avoidance_planner.state in ('NORMAL','DETECT')
                            and local_result['nearest_distance_m']>getattr(self,'_max_local_path_start_distance',math.inf)):
                        self.get_logger().warning('Local path is too far from Ego; relocation/replanning required',throttle_duration_sec=5.)
                        local_result=None;state_value='LOCAL_PATH_OUT_OF_RANGE'
                    if local_result is not None:
                        self._progress_index = local_result['nearest_index']
                        self._progress_arc_m = local_result['progress_arc_m']
                        active_local_points = local_result['points']
                        hazard_points = active_local_points
                        s4_kwargs = {}
                        control_horizon = getattr(self,'_s4_control_horizon',self.get_parameter('local_path_length_m').value)
                        if s4_started is not None:
                            policy = self.distance_safety.policy
                            preview = self.local_path_planner.extract(
                                self._prepared_route[1], x, y, horizon_m=policy.preview_m,
                                previous_index=self._progress_index,
                                search_back=self.get_parameter('local_path_search_back').value,
                                search_forward=self.get_parameter('local_path_search_forward').value,
                                resample_spacing_m=self.get_parameter('local_path_resample_spacing_m').value,
                                previous_arc_m=self._progress_arc_m)
                            hazard_points = preview['points'] if preview else ()
                            self.refresh_distance(s4_started, self._route_result.get('link_ids', ()))
                            metrics = self._s4_distance_state
                            # An unpaired sensor frame already holds STOP with
                            # its bounded recovery timer; absent distance metrics
                            # must not relabel that first cause as a map fault.
                            if self._sensor_pause is None and (not hazard_points or arc(hazard_points)[-1] < metrics.get('stopping_required_m', math.inf)):
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
                        if self._use_ego_status:
                            receipt=self._ego_status_received_ns
                            observation_valid=observation_valid and receipt is not None and 0 <= (now_ns-receipt)/1e9 <= self.get_parameter('avoidance_observation_timeout_sec').value
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
                            if self._sensor_pause is not None:
                                avoidance_result=self._sensor_hold_result()
                            else:
                                avoidance_result = self.avoidance_planner.plan(active_local_points,
                                    self.latest_obstacle_groups,(x,y,yaw),speed_kmh=self.latest_speed_kmh,
                                    actual_curvature=self.latest_turn_curvature,current_link_id=current_link,
                                    route_link_ids=self._route_result.get('link_ids',()),observation_valid=observation_valid,
                                    route_link_index=link_index_at_progress(self._route_result,self.dijkstra_planner,
                                        current_link,self._progress_arc_m) if mission_start else None,
                                    observation_stamp_ns=getattr(self,'obstacle_stamp_ns',None),
                                    control_horizon_m=control_horizon,now_sec=(now_ns-self._sensor_pause_offset_ns)/1e9,
                                    global_reference=self._route_result.get('points',active_local_points),
                                    return_search=self._return_search, **s4_kwargs)
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
                        if not avoidance_result['blocked'] and enable_drive and (avoidance_result.get('active') or self._sensor_resume_validation):
                            # A verified fixed path still receives body, motion
                            # and PP checks every cycle. CPU time over 50 ms is
                            # not a new path/hazard and must not restart the
                            # three-pose interlock. Actual stale inputs are
                            # rejected by the unchanged sensor gates below.
                            if validation_stop:
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
                                    self._sensor_resume_validation=False
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

                        speed_kwargs=dict(previous_speed_mode=self._previous_speed_mode)
                        if getattr(self.speed_planner,'anticipatory',False):
                            speed_kwargs['measured_speed_kmh']=self.latest_speed_kmh
                            speed_kwargs['measured_curvature']=self.latest_turn_curvature
                        speed_result = self.speed_planner.plan(active_local_points,**speed_kwargs)
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
        admission=getattr(self,'_morai_speed_policy',None)
        admitted_cap=None
        if admission is not None:
            available=max(0.,local_result['path_length_m']-
                self.avoidance_planner.body.wheelbase_m-self.avoidance_planner.body.front_overhang_m) if local_result else 0.
            admitted_cap=min(admission['validated_speed_cap_kmh'],stopping_speed_cap(
                min(available,admission['verified_detection_distance_m']),
                self.avoidance_planner.config.deceleration_mps2,admission['braking_delay_s']))
            target_speed.data=min(target_speed.data,admitted_cap)
        if hasattr(self,'diagnostics_pub'):
            self.diagnostics_pub.publish(String(data=json.dumps(dict(
                planning_elapsed_ms=(time.monotonic()-plan_started)*1000.,
                local_path_length_m=local_result['path_length_m'] if local_result else 0.,
                nearest_path_distance_m=local_result['nearest_distance_m'] if local_result else None,
                s4_control_horizon_m=getattr(self,'_s4_control_horizon',None),
                route_error=getattr(self,'_route_error',''),
                remaining_waypoints=self._route_result.get('remaining_waypoints') if self._route_result else None,
                configured_cruise_kmh=self.get_parameter('cruise_speed_kmh').value,
                recommended_speed_kmh=recommended_speed,admitted_speed_cap_kmh=admitted_cap,
                speed_admission=admission,reset_history=getattr(self,'_reset_history',[]),
                safety_history=getattr(self,'_safety_history',[]),
                deceleration_to_avoidance_m=required_deceleration_distance(
                    max(0.,self.latest_speed_kmh),self.avoidance_planner.config.avoid_speed_kmh,
                    deceleration_mps2=self.avoidance_planner.config.deceleration_mps2)
                    if _finite_number(self.latest_speed_kmh) else None),allow_nan=False)))

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
