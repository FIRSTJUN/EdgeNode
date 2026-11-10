import math
import json
import time
import uuid

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from edgenode_planning.stop_diagnostics import snapshot as stop_snapshot
from edgenode_planning.auto_progress_recovery import AutoProgressRecovery
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, Float32, Float64, Int32, String
from std_srvs.srv import Trigger

from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.local_path_planner import LocalPathPlanner
from edgenode_planning.map_matcher import MapMatcher
from edgenode_planning.crosswalk_stop_correction import CrosswalkCorrectionConfig
from edgenode_planning.speed_planner import SpeedPlanner
from edgenode_planning.corner_speed_planner import CornerSpeedPlanner, SpeedSlewLimiter
from edgenode_planning.progress_recovery import ProgressRecoveryError, recover_progress


def _finite_number(value):
    """Validate drive values without accepting booleans or overflowing ints."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


from .apf_path_selection import APFPathSelection,path_hash
from .planning_fsm import PlanningFSM
from .traffic_light_gate import TrafficLightGate


class PlanningNode(Node):
    """
    Match localization and publish global and local paths at 20 Hz.

    Current stage:
      - Traffic evidence is opt-in and requires verified route/stop-line calibration.
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
                ('traffic_light_enabled', False),
                ('crosswalk_enabled', True),
                ('crosswalk_activation_distance_m',40.),
                ('crosswalk_consistency_tolerance_m',2.),
                ('crosswalk_min_score',.80),
                ('crosswalk_max_age_sec',.5),
                ('traffic_light_associations_file', ''),
                ('auto_progress_recovery_enabled', True),
                ('auto_route_replanning_enabled', True),
                ('recovery_gps_topic', '/localization/gps_odometry'),
                ('recovery_speed_topic', '/localization/gps_speed'),
                ('recovery_status_topic', '/localization/status'),
                ('recovery_imu_topic', '/Imu'),
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
                ('corner_lateral_accel_mps2', 1.5),
                ('corner_deceleration_mps2', 1.5),
                ('speed_acceleration_limit_mps2', 0.8),
                ('speed_deceleration_limit_mps2', 1.5),
                ('corner_preview_distance_m', 100.0),
                ('corner_response_time_sec', 1.0),
                ('corner_braking_buffer_m', 5.0),
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

        self.declare_parameters(namespace='',parameters=[(name,value,ParameterDescriptor(read_only=True)) for name,value in [('apf_integration_enabled',False),('apf_shadow_only',True),('apf_provider_name','/apf_planning_provider'),('apf_policy_mode','apf')]])
        self.apf_selector=None;self._apf_ego=None;self._apf_ego_received=None
        if self.get_parameter('apf_integration_enabled').value:
            from morai_ros2_msgs.msg import EgoVehicleStatus
            self.apf_selector=APFPathSelection(uuid.uuid4().hex,self.get_parameter('apf_policy_mode').value)
            self.apf_reference_path_pub=self.create_publisher(Path,'/planning/reference_local_path',5)
            self.apf_reference_pub=self.create_publisher(String,'/planning/apf_reference',5)
            self.apf_status_pub=self.create_publisher(String,'/planning/apf_status',5)
            self.apf_selected_pub=self.create_publisher(Path,'/planning/apf_selected_path',5)
            self.create_subscription(String,'/planning/apf_proposal',self.apf_proposal_callback,5)
            self.create_subscription(Bool,'/planning/apf_emergency_stop',self.apf_emergency_callback,10)
            self.create_subscription(EgoVehicleStatus,'/ego_vehicle_status',self.apf_ego_callback,qos_profile_sensor_data)

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
        self._speed_slew_override = False
        self.target_speed_limiter = SpeedSlewLimiter(
            self.get_parameter('speed_acceleration_limit_mps2').value,
            self.get_parameter('speed_deceleration_limit_mps2').value)
        self._previous_speed_mode = None
        self._route_key = None
        self._route_result = None
        self._progress_index = None
        self._progress_arc_m = None
        self.latest_pose = None
        # A separate validated snapshot gates the explicit recovery service only.
        self._recovery_pose = None
        self._recovery_pose_receive_ns = None
        self._recovery_pose_stamp_ns = None
        self._recovery_pose_frame = None
        self._auto_recovery = AutoProgressRecovery()
        self._auto_check_ns = 0
        self._auto_block_reason = None
        self._drive_was_enabled = False
        self._drive_session_start_ns = None
        self._drive_session_expired = False
        self._drive_expiry_reason = None
        self._diagnostic_sensor_times = {}
        self._diagnostic_odom_time = None
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
        self.last_safety_stop_pub = self.create_publisher(
            String, '/planning/last_safety_stop',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.corner_profile_pub = self.create_publisher(String, '/planning/corner_speed_profile', 10)
        self._traffic_assoc_file=self.get_parameter('traffic_light_associations_file').value
        self.traffic_gate=TrafficLightGate.from_file(self._traffic_assoc_file)
        self.traffic_status_pub=self.create_publisher(String,'/planning/traffic_light_gate',10)
        self.traffic_detector_sub=self.create_subscription(String,'/perception/traffic_light/detections',
            lambda msg:self.traffic_gate.observe_detector(msg.data,self.get_clock().now().nanoseconds),10)
        self.traffic_state_sub=self.create_subscription(String,'/perception/traffic_light/state',
            lambda msg:self.traffic_gate.observe_state(msg.data,self.get_clock().now().nanoseconds),10)
        self.crosswalk_subscriptions=[
            self.create_subscription(Bool,'/perception/crosswalk/detected',
                lambda msg:self.traffic_gate.crosswalk.observe_detected(msg.data,self.get_clock().now().nanoseconds),10),
            self.create_subscription(Float32,'/perception/crosswalk/distance',
                lambda msg:self.traffic_gate.crosswalk.observe_distance(msg.data,self.get_clock().now().nanoseconds),10),
            self.create_subscription(String,'/perception/crosswalk/status',
                lambda msg:self.traffic_gate.crosswalk.observe(msg.data,self.get_clock().now().nanoseconds),10),
        ]
        self._traffic_speed=None
        self.traffic_speed_sub=self.create_subscription(Float32,'/localization/gps_speed',
            self.traffic_speed_callback,qos_profile_sensor_data)
        self.localization_sub = self.create_subscription(
            Odometry,
            self.get_parameter('localization_topic').value,
            self.localization_callback,
            10,
        )

        self._recovery_subscriptions = [
            self.create_subscription(kind, self.get_parameter(param).value,
                lambda msg, channel=channel: self.recovery_sensor_callback(channel, msg),
                qos_profile_sensor_data)
            for kind, param, channel in (
                (Odometry, 'recovery_gps_topic', 'gps'),
                (Float32, 'recovery_speed_topic', 'speed'),
                (String, 'recovery_status_topic', 'status'),
                (Imu, 'recovery_imu_topic', 'imu'))]

        # Default mutually-exclusive callback group serializes service, timer,
        # localization and parameter callbacks. Recovery performs no waits/awaits.
        self.progress_recovery_service = self.create_service(
            Trigger, '/planning/reinitialize_progress', self.reinitialize_progress_callback)

        # Planning loop: 20 Hz
        self.create_timer(
            0.05,
            self.plan,
        )

        self.get_logger().info(
            f'Planning node ready - loaded {len(self.map_matcher.links)} MGeo links; '
            'waiting for localization (drive disabled by default)'
        )

    def apf_proposal_callback(self,msg):
        try:
            if len(msg.data)>2*1024*1024:raise ValueError('APF_MESSAGE_BUDGET')
            packet=json.loads(msg.data,parse_constant=lambda x:(_ for _ in ()).throw(ValueError('NONFINITE_APF_PACKET')))
        except (ValueError,TypeError):packet=None
        self.apf_selector.observe(packet,time.monotonic())

    def apf_emergency_callback(self,msg):
        if msg.data:self.apf_selector.stop(['EMERGENCY_STOP'])

    def apf_ego_callback(self,msg):self._apf_ego=msg;self._apf_ego_received=time.monotonic()

    def apf_world_ready(self,now):
        pubs=self.get_publishers_info_by_topic('/ctrl_cmd');consumers=self.get_subscriptions_info_by_topic('/ctrl_cmd')
        if len(pubs)!=1 or pubs[0].node_name!='control_node' or pubs[0].node_namespace!='/':return False
        if len(consumers)!=1 or consumers[0].node_name!='MoraiSimNode':return False
        msg=self._apf_ego
        if msg is None or self._apf_ego_received is None or not 0<=time.monotonic()-self._apf_ego_received<=.5:return False
        if msg.header.frame_id!='MoraiInfo' or not 0<=now-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)<=.5:return False
        sources=self.get_publishers_info_by_topic('/ego_vehicle_status')
        if len(sources)!=1 or sources[0].node_name!='MoraiSimNode':return False
        v=msg.velocity;speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)*3.6
        return math.isfinite(speed) and speed<=3 and self.latest_pose is not None and math.isfinite(msg.position.x) and math.isfinite(msg.position.y) and math.hypot(msg.position.x-self.latest_pose[0],msg.position.y-self.latest_pose[1])<=.5

    def apf_select(self,raw_path,now,enable_drive,recommended_speed):
        selector=self.apf_selector;route=self._route_result;route_digest=path_hash(route['points']) if route else path_hash([])
        points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in raw_path.poses]
        stamp=raw_path.header.stamp.sec+raw_path.header.stamp.nanosec*1e-9
        if points:selector.remember(points,stamp,route_digest)
        hint=min(float(recommended_speed),float(self.get_parameter('max_drive_speed_kmh').value),3.)
        if not math.isfinite(hint) or hint<=0:hint=1.
        meta=dict(planning_epoch=selector.epoch,route_hash=route_digest,reference_hash=path_hash(points),stamp_s=now,drive_requested=enable_drive,preview_speed_kmh=hint)
        meta['policy_mode']=selector.policy_mode
        self.apf_reference_pub.publish(String(data=json.dumps(meta,allow_nan=False)))
        endpoint=self.get_publishers_info_by_topic(self.resolve_topic_name('/planning/apf_proposal'))
        expected=self.get_parameter('apf_provider_name').value
        source_ok=len(endpoint)==1 and (endpoint[0].node_namespace.rstrip('/')+'/'+endpoint[0].node_name)==expected
        source_ok=source_ok and all(self.count_publishers(self.resolve_topic_name(t))==1 for t in ['/planning/reference_local_path',self.get_parameter('global_path_topic').value,self.get_parameter('localization_topic').value,self.get_parameter('local_path_topic').value,self.get_parameter('target_speed_topic').value])
        choice=selector.select(now,time.monotonic(),route_digest,self.latest_pose,enable_drive,source_ok,self.apf_world_ready(now) if enable_drive else True)
        return choice

    def apf_path_message(self,points,stamp):
        msg=Path();msg.header.frame_id='map';msg.header.stamp=stamp
        for i,point in enumerate(points):
            p=PoseStamped();p.header=msg.header;p.pose.position.x=float(point[0]);p.pose.position.y=float(point[1]);p.pose.position.z=float(point[2]) if len(point)>2 else 0.
            a,b=points[max(0,i-1)],points[min(len(points)-1,i+1)];yaw=math.atan2(b[1]-a[1],b[0]-a[0]);p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);msg.poses.append(p)
        return msg

    def traffic_speed_callback(self,msg):
        if _finite_number(msg.data) and msg.data>=0:
            self._traffic_speed=(float(msg.data),self.get_clock().now().nanoseconds)

    def localization_callback(self, msg):
        """Store the latest pose; map matching runs only in the planning timer."""
        self._diagnostic_odom_time = (time.monotonic_ns(),
            msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec)
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.latest_pose = (position.x, position.y, yaw)
        self._recovery_pose = None
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if (not all(_finite_number(v) for v in (position.x, position.y, norm))
                or norm < 1e-12 or msg.child_frame_id != 'base_link'):
            self._auto_recovery.reset_stability()
            return
        velocity = msg.twist.twist.linear
        if (not all(_finite_number(v) for v in (velocity.x, velocity.y, velocity.z))
                or math.hypot(velocity.x, velocity.y, velocity.z)*3.6 > 0.1):
            self._auto_recovery.reset_stability()
            return
        qx, qy, qz, qw = (v/norm for v in (q.x, q.y, q.z, q.w))
        normalized_yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
        self._recovery_pose = (position.x, position.y, normalized_yaw)
        self._auto_recovery.observe_localization(self._recovery_pose)
        self._recovery_pose_receive_ns = time.monotonic_ns()
        stamp = msg.header.stamp
        self._recovery_pose_stamp_ns = stamp.sec*1_000_000_000+stamp.nanosec
        self._recovery_pose_frame = msg.header.frame_id

    def _recovery_route_key(self):
        route_nodes = self.get_parameter('route_node_ids').value
        if route_nodes is None or isinstance(route_nodes, (list, tuple)) and not route_nodes:
            return ('goal', self.get_parameter('goal_node_id').value)
        return ('waypoints', tuple(route_nodes) if isinstance(route_nodes, (list, tuple)) else repr(route_nodes))

    def _prepare_recovery_context(self):
        """Common disabled/stationary/fresh pose and directed map-match validation."""
        if self.get_parameter('enable_drive').value is not False:
            raise ProgressRecoveryError('DRIVE_MUST_BE_DISABLED')
        route = self._route_result
        if route is None:
            raise ProgressRecoveryError('NO_GLOBAL_ROUTE')
        key = PlanningNode._recovery_route_key(self)
        if key != self._route_key:
            raise ProgressRecoveryError('ROUTE_CONFIGURATION_CHANGED')
        if self._recovery_pose is None:
            raise ProgressRecoveryError('VALID_STATIONARY_LOCALIZATION_REQUIRED')
        receive_age = (time.monotonic_ns()-self._recovery_pose_receive_ns)/1e9
        source_age = (self.get_clock().now().nanoseconds-self._recovery_pose_stamp_ns)/1e9
        if (not 0 <= receive_age < 0.6 or self._recovery_pose_stamp_ns <= 0
                or not 0 <= source_age < 0.6):
            raise ProgressRecoveryError('LOCALIZATION_STALE_OR_CLOCK_MISMATCH')
        frame = self.get_parameter('global_path_frame_id').value
        if not frame or frame != self._recovery_pose_frame:
            raise ProgressRecoveryError('FRAME_MISMATCH')
        pose = self._recovery_pose
        # Matching remains unchanged, and a rejected recovery must not alter
        # its continuity preference for the next ordinary timer callback.
        previous_link = self.map_matcher.previous_link_id
        try:
            match = self.map_matcher.match(pose[0], pose[1], math.degrees(pose[2]))
        finally:
            self.map_matcher.previous_link_id = previous_link
        if match is None:
            raise ProgressRecoveryError('MAP_MATCH_FAILED')
        if not _finite_number(self.map_matcher.max_heading_diff_deg):
            raise ProgressRecoveryError('INVALID_HEADING_LIMIT')
        heading_limit = min(80.0, self.map_matcher.max_heading_diff_deg)
        if (not _finite_number(match['distance']) or match['distance'] > 2.0
                or not _finite_number(match['heading_diff'])
                or match['heading_diff'] > heading_limit):
            raise ProgressRecoveryError('MATCH_DISTANCE_OR_HEADING_REJECTED')
        return dict(route=route, key=key, pose=pose, match=match, heading_limit=heading_limit)

    def _validate_recovery_context(self, context):
        receive_age = (time.monotonic_ns()-self._recovery_pose_receive_ns)/1e9
        source_age = (self.get_clock().now().nanoseconds-self._recovery_pose_stamp_ns)/1e9
        if not 0 <= receive_age < .6 or not 0 <= source_age < .6:
            raise ProgressRecoveryError('LOCALIZATION_EXPIRED_DURING_RECOVERY')
        if (self.get_parameter('enable_drive').value is not False
                or self._route_result is not context['route']
                or self._recovery_pose != context['pose']
                or self._route_key != context['key']
                or PlanningNode._recovery_route_key(self) != context['key']):
            raise ProgressRecoveryError('STATE_CHANGED_DURING_RECOVERY')

    def _project_recovery(self, route, context):
        result = recover_progress(route, self.map_matcher.links_by_id, context['match']['link_id'], context['pose'],
            max_heading_deg=context['heading_limit'],
            horizon_m=self.get_parameter('local_path_length_m').value,
            resample_spacing_m=self.get_parameter('local_path_resample_spacing_m').value,
            search_back=self.get_parameter('local_path_search_back').value,
            search_forward=self.get_parameter('local_path_search_forward').value)
        return result

    def _prepare_progress_recovery(self, context=None):
        """Shared all-or-nothing manual/automatic progress validation."""
        if context is None:
            context = PlanningNode._prepare_recovery_context(self)
        result = PlanningNode._project_recovery(self, context['route'], context)
        PlanningNode._validate_recovery_context(self, context)
        return result

    def _auto_replan_route(self, context, now):
        """Build and validate privately; keep the old route on every failure."""
        if self.get_parameter('auto_route_replanning_enabled').value is not True:
            raise ProgressRecoveryError('AUTO_ROUTE_REPLANNING_DISABLED')
        waypoints = list(context['key'][1]) if context['key'][0] == 'waypoints' else None
        if waypoints is not None and (not waypoints or any(
                not isinstance(node, str) or not node
                for node in waypoints)):
            raise ProgressRecoveryError('INVALID_WAYPOINT_ROUTE')
        guard = self._auto_recovery
        link = context['match']['link_id']
        reason = guard.check(context['pose'], self._recovery_pose_frame, now,
            self.get_clock().now().nanoseconds, context['route'], candidate_key=link)
        if reason != 'READY':
            raise ProgressRecoveryError(reason)
        if not guard.begin_replan(context['route'], context['key'], now):
            raise ProgressRecoveryError('ROUTE_REPLAN_RETRY_COOLDOWN')
        old_start = context['route'].get('start_link_id')
        goal = waypoints[-1] if waypoints is not None else context['key'][1]
        try:
            candidate = (self.dijkstra_planner.plan_route(link, waypoints) if waypoints is not None
                         else self.dijkstra_planner.plan(link, goal))
            if candidate is None:
                raise ProgressRecoveryError('NO_ROUTE_TO_EXISTING_GOAL')
            if (candidate.get('start_link_id') != link
                    or candidate.get('goal_node_id') != goal):
                raise ProgressRecoveryError('REPLANNED_ROUTE_ENDPOINT_MISMATCH')
            result = PlanningNode._project_recovery(self, candidate, context)
            PlanningNode._validate_recovery_context(self, context)
            # Slow searches cannot commit using sensors that expired while this
            # mutually-exclusive callback was busy. No live state changed yet.
            reason = guard.check(context['pose'], self._recovery_pose_frame,
                time.monotonic_ns(), self.get_clock().now().nanoseconds,
                context['route'], candidate_key=link)
            if reason != 'READY':
                raise ProgressRecoveryError(reason)
        except Exception as exc:
            detail = dict(previous_start_link=old_start, new_start_link=link,
                          goal_node_id=goal, route_node_ids=waypoints, reason=str(exc))
            self.get_logger().warning('AUTO_ROUTE_REPLAN_FAILED ' + json.dumps(detail, sort_keys=True))
            raise ProgressRecoveryError('AUTO_ROUTE_REPLAN_FAILED: ' + str(exc)) from exc
        # One serialized transaction. The same planning tick publishes global
        # and local Paths from this cache and progress pair; drive stays false.
        self._route_result, self._progress_index, self._progress_arc_m = (
            candidate, result['index'], result['arc_m'])
        self._previous_speed_mode = None
        self.target_speed_limiter.reset()
        self._speed_slew_override = False
        guard.committed(context['pose'], time.monotonic_ns())
        detail = dict(previous_start_link=old_start, new_start_link=link,
                      goal_node_id=goal, route_node_ids=waypoints, index=result['index'], arc_m=result['arc_m'],
                      distance_m=result['distance_m'], heading_error_deg=result['heading_error_deg'])
        self.get_logger().info('AUTO_ROUTE_REPLAN_SUCCEEDED ' + json.dumps(detail, sort_keys=True))

    def _commit_progress_recovery(self, result, source):
        detail = {k: v for k, v in result.items() if k != 'local_points'}
        detail.update(previous_index=self._progress_index, previous_arc_m=self._progress_arc_m,
                      source=source, delta_arc_m=(result['arc_m']-self._progress_arc_m
                                                if self._progress_arc_m is not None else None))
        message = 'PROGRESS_REINITIALIZED ' + json.dumps(detail, sort_keys=True)
        self._progress_index, self._progress_arc_m = result['index'], result['arc_m']
        self._previous_speed_mode = None
        self.get_logger().info(message)
        return message

    def reinitialize_progress_callback(self, request, response):
        """Explicit all-or-nothing reanchor; never enable or replan."""
        try:
            result = PlanningNode._prepare_progress_recovery(self)
            response.message = PlanningNode._commit_progress_recovery(self, result, 'manual')
            response.success = True
        except (ProgressRecoveryError, ValueError, TypeError, KeyError, OverflowError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def recovery_sensor_callback(self, channel, msg):
        receive_ns = time.monotonic_ns()
        header = getattr(msg, 'header', None)
        source_ns = header.stamp.sec*1_000_000_000+header.stamp.nanosec if header is not None else None
        self._diagnostic_sensor_times[channel] = (receive_ns, source_ns)
        self._auto_recovery.receive(channel, msg, receive_ns, self.get_clock().now().nanoseconds)

    def maybe_auto_recover_progress(self):
        guard = self._auto_recovery
        now = time.monotonic_ns()
        guard.observe_motion(self.latest_pose)
        if now - self._auto_check_ns < 200_000_000:
            return
        self._auto_check_ns = now
        try:
            if self.get_parameter('auto_progress_recovery_enabled').value is not True:
                raise ProgressRecoveryError('AUTO_DISABLED')
            context = PlanningNode._prepare_recovery_context(self)
            links = context['route'].get('link_ids')
            if not isinstance(links, (list, tuple)):
                raise ProgressRecoveryError('INVALID_ROUTE_LINKS')
            if context['match']['link_id'] not in links:
                PlanningNode._auto_replan_route(self, context, now)
                self._auto_block_reason = None
                return
            result = PlanningNode._prepare_progress_recovery(self, context)
            if self._progress_arc_m is None or abs(result['arc_m']-self._progress_arc_m) < 5.0:
                raise ProgressRecoveryError('PROGRESS_CONSISTENT')
            # Do not reset if the ordinary bounded extractor already serves a close path.
            local = self.local_path_planner.extract(self._route_result['points'],
                *self._recovery_pose[:2], previous_index=self._progress_index,
                previous_arc_m=self._progress_arc_m,
                horizon_m=self.get_parameter('local_path_length_m').value,
                resample_spacing_m=self.get_parameter('local_path_resample_spacing_m').value,
                search_back=self.get_parameter('local_path_search_back').value,
                search_forward=self.get_parameter('local_path_search_forward').value)
            if local is not None and math.dist(local['points'][0][:2], self._recovery_pose[:2]) <= 2.0:
                raise ProgressRecoveryError('LOCAL_PATH_ALREADY_CLOSE')
            reason = guard.check(self._recovery_pose, self._recovery_pose_frame,
                now, self.get_clock().now().nanoseconds, self._route_result)
            if reason != 'READY':
                raise ProgressRecoveryError(reason)
            # Revalidate immediately before atomic commit; callbacks are serialized.
            result = PlanningNode._prepare_progress_recovery(self)
            PlanningNode._commit_progress_recovery(self, result, 'automatic')
            guard.committed(self._recovery_pose, now)
            self._auto_block_reason = None
        except (ProgressRecoveryError, ValueError, TypeError, KeyError, OverflowError) as exc:
            reason = str(exc)
            if reason != 'STABILIZING':
                guard.reset_stability()
            last_log = getattr(self, '_auto_wait_log_ns', None)
            if reason != self._auto_block_reason or last_log is None or now-last_log >= 10_000_000_000:
                routine = {'DRIVE_MUST_BE_DISABLED', 'PROGRESS_CONSISTENT', 'LOCAL_PATH_ALREADY_CLOSE',
                           'AUTO_DISABLED', 'NO_GLOBAL_ROUTE', 'ROUTE_CONFIGURATION_CHANGED'}
                if reason in routine:
                    self.get_logger().debug('AUTO_PROGRESS_RECOVERY_WAIT: ' + reason)
                else:
                    pose = self._recovery_pose
                    gps = guard.samples.get('gps')
                    distance = (math.dist(gps[0], pose[:2]) if gps is not None and pose is not None else None)
                    detail = dict(reason=reason, index=self._progress_index, arc_m=self._progress_arc_m,
                                  gps_ekf_distance_m=distance,
                                  ignored_nonadvancing=guard.ignored_nonadvancing,
                                  sensor_receive_ages_s={k:(now-v[1])/1e9 for k,v in guard.samples.items()})
                    self.get_logger().warning('AUTO_PROGRESS_RECOVERY_WAIT ' + json.dumps(detail, sort_keys=True))
                self._auto_wait_log_ns = now
                self._auto_block_reason = reason

    def apply_corner_speed(self, legacy_result, local_points):
        """Preserve classifier diagnostics, replacing fixed speeds with dynamics."""
        if legacy_result is None:
            return None
        legacy_speed = legacy_result.get('target_speed_kmh')
        if not _finite_number(legacy_speed) or legacy_speed < 0:
            return None
        if legacy_speed == 0:
            return dict(legacy_result)  # preserve an explicit existing stop recommendation
        planner = CornerSpeedPlanner(
            lateral_accel=self.get_parameter('corner_lateral_accel_mps2').value,
            decel=self.get_parameter('corner_deceleration_mps2').value,
            preview=self.get_parameter('corner_preview_distance_m').value,
            response_time=self.get_parameter('corner_response_time_sec').value,
            buffer=self.get_parameter('corner_braking_buffer_m').value)
        cap = self.get_parameter('max_drive_speed_kmh').value
        cruise = self.get_parameter('cruise_speed_kmh').value
        if not all(_finite_number(v) and v >= 0 for v in (cap, cruise)):
            return None
        result = planner.plan(self._route_result['points'], self._progress_arc_m,
                              local_points, min(cap, cruise))
        if result is None:
            return None
        updated = dict(legacy_result)
        updated['target_speed_kmh'] = result['target_speed_kmh']
        updated['corner_profile'] = result
        return updated

    def plan(self):
        """Publish routes, recommendations and guarded, time-limited commands.

        Cache failures too, so an unreachable route does not trigger a search
        every tick. Match loss retains progress/cache but publishes no Path.
        An active route configuration change invalidates both immediately.
        """
        if getattr(self,'apf_selector',None) is None:self.maybe_auto_recover_progress()
        state_value = 'WAIT_FOR_LOCALIZATION'
        status_value = 'WAIT_FOR_LOCALIZATION'
        current_link = ''
        distance = float('nan')
        heading_diff = float('nan')
        recommended_speed = 0.0
        speed_mode = 'INVALID'
        speed_curvature = float('nan')
        speed_max_curvature = float('nan')
        raw_local_path=Path();apf_choice=None
        active_local_points = []
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
            self._progress_index = None
            self._progress_arc_m = None
            self._previous_speed_mode = None

        now_ns = self.get_clock().now().nanoseconds
        enable_drive = self.get_parameter('enable_drive').value is True
        if not enable_drive:
            self._drive_was_enabled = False
            self._drive_session_start_ns = None
            self._drive_session_expired = False
            self._drive_expiry_reason = None
            self._drive_session_duration_sec = None
            self._drive_last_time_ns = None
        else:
            if not self._drive_was_enabled:
                self._drive_session_start_ns = now_ns
                self._drive_session_expired = False
                self._drive_expiry_reason = None
                self._drive_session_duration_sec = None
            elif now_ns < self._drive_last_time_ns:
                # A clock rewind must not extend or resurrect the drive lease.
                self._drive_session_expired = True
                self._drive_expiry_reason = 'CLOCK_REWIND'
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

                # Ordinary matching never replaces a cached route. Disabled,
                # stable off-route recovery above is the only automatic exception.
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

                        raw_local_path=local_path
                        if getattr(self,'apf_selector',None) is not None:self.apf_reference_path_pub.publish(local_path)
                        else:self.local_path_pub.publish(local_path)

                        speed_result = self.speed_planner.plan(
                            active_local_points, previous_speed_mode=self._previous_speed_mode,
                        )
                        speed_result = self.apply_corner_speed(speed_result, active_local_points)
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
                    if not self._drive_session_expired:
                        self._drive_expiry_reason = 'DRIVE_DURATION_EXPIRED'
                    self._drive_session_expired = True
                    drive_status = 'EXPIRED'
                elif (state_value == 'GLOBAL_PATH_READY' and status_value == 'MATCHED'
                      and local_result is not None and speed_result is not None
                      and speed_valid):
                    requested_target_speed = float(min(recommended_speed, max_speed))
                    drive_status = 'ACTIVE'
                else:
                    drive_status = 'NOT_READY'

        self.traffic_gate.crosswalk.config=CrosswalkCorrectionConfig(
            enabled=self.get_parameter('crosswalk_enabled').value,
            activation_distance_m=self.get_parameter('crosswalk_activation_distance_m').value,
            consistency_tolerance_m=self.get_parameter('crosswalk_consistency_tolerance_m').value,
            min_score=self.get_parameter('crosswalk_min_score').value,
            max_age_sec=self.get_parameter('crosswalk_max_age_sec').value)
        target_before_traffic=requested_target_speed
        traffic=dict(enabled=False,hold=False,speed_cap_kmh=None,reason='DISABLED',state='UNKNOWN')
        if self.get_parameter('traffic_light_enabled').value is True:
            sample=self._traffic_speed
            speed=sample[0] if sample is not None and 0<=(now_ns-sample[1])/1e9<=.6 else None
            traffic=self.traffic_gate.decision(True,self._route_result,self._progress_arc_m,speed,now_ns,heading_diff,
                route_node_ids=self.get_parameter('route_node_ids').value,
                ego_yaw=self.latest_pose[2] if self.latest_pose is not None else None)
            if traffic['speed_cap_kmh'] is not None:
                requested_target_speed=min(requested_target_speed,traffic['speed_cap_kmh'])
        else:
            traffic=self.traffic_gate.decision(False,None,None,None,now_ns)

        if getattr(self,'apf_selector',None) is not None:
            if enable_drive and (drive_status!='ACTIVE' or status_value!='MATCHED' or state_value!='GLOBAL_PATH_READY' or local_result is None or not speed_valid):self.apf_selector.stop(['BASE_PLANNING_NOT_READY_OR_LEASE_EXPIRED'])
            if getattr(self.apf_selector,'was_requested',False) and not enable_drive:self.apf_selector.stop(['MANUAL_DRIVE_DISABLED'])
            self.apf_selector.was_requested=enable_drive
            apf_choice=self.apf_select(raw_local_path,now_ns/1e9,enable_drive,recommended_speed)
            if not apf_choice['motion_allowed']:requested_target_speed=0.
            else:requested_target_speed=min(requested_target_speed,apf_choice['speed_cap_kmh'])

        previous_fsm_state = self.fsm.state

        fsm_state_value = self.fsm.update(
            localization_ready=self.latest_pose is not None,
            map_match_ready=status_value == 'MATCHED',
            global_path_ready=state_value == 'GLOBAL_PATH_READY',
            local_path_ready=local_result is not None,
            speed_ready=speed_valid,
            drive_status=drive_status,
            obstacle_avoid_requested=apf_choice is not None and apf_choice['state'] in ('LANE_CHANGE_OUT','ADJACENT_LANE_FOLLOW','LANE_RETURN'),
            path_safety_stop_requested=apf_choice is not None and apf_choice['state']=='STOP',
            obstacle_check_requested=apf_choice is not None and apf_choice['state']=='AVOIDANCE_CHECK',
            traffic_stop_requested=traffic['hold'],
            emergency_stop_requested=False,
            goal_reached=False,
        )

        if fsm_state_value != previous_fsm_state:
            self.get_logger().info(
                f'FSM transition: {previous_fsm_state} -> {fsm_state_value}'
            )
            if fsm_state_value == PlanningFSM.SAFETY_STOP:
                detail = stop_snapshot(self, now_ns=now_ns, monotonic_ns=time.monotonic_ns(),
                    previous=previous_fsm_state, drive_status=drive_status, planning_state=state_value,
                    match_status=status_value, match_distance=distance, match_heading=heading_diff,
                    local_result=local_result, local_points=active_local_points,
                    speed_valid=speed_valid, recommended_speed=recommended_speed)
                payload = json.dumps(detail, sort_keys=True, allow_nan=False)
                self.get_logger().warning('PLANNING_SAFETY_STOP ' + payload)
                self.last_safety_stop_pub.publish(String(data=payload))

        # High-level FSM is the final motion gate.
        # Speed Planning may recommend a speed, but motion is permitted
        # only while the FSM is explicitly in DRIVING.
        if self.fsm.motion_allowed:
            self.target_speed_limiter.acceleration = self.get_parameter('speed_acceleration_limit_mps2').value
            self.target_speed_limiter.deceleration = self.get_parameter('speed_deceleration_limit_mps2').value
            target_speed.data = self.target_speed_limiter.limit(requested_target_speed, now_ns)
            override = self.target_speed_limiter.safety_override is True
            if override and not self._speed_slew_override:
                self.get_logger().warning(
                    ('TRAFFIC_SPEED_HARD_LIMIT' if traffic['enabled'] and traffic['speed_cap_kmh'] is not None and traffic['speed_cap_kmh']<=recommended_speed else 'CORNER_SPEED_HARD_LIMIT') + ': immediate lower target takes precedence over deceleration slew')
            self._speed_slew_override = override
        else:
            self.target_speed_limiter.reset()
            self._speed_slew_override = False
            target_speed.data = 0.0

        if apf_choice is not None:
            shadow=self.get_parameter('apf_shadow_only').value
            if shadow or not apf_choice['motion_allowed']:target_speed.data=0.
            else:target_speed.data=min(target_speed.data,apf_choice['speed_cap_kmh'],3.)
            final_points=apf_choice['points'] if not shadow and apf_choice['motion_allowed'] else []
            stamp=self.get_clock().now().to_msg()
            self.local_path_pub.publish(self.apf_path_message(final_points,stamp))
            self.apf_selected_pub.publish(self.apf_path_message(apf_choice['points'],stamp))
            report={k:v for k,v in apf_choice.items() if k not in ('points','reference')}
            report.update(stamp_s=now_ns/1e9,shadow_only=shadow,stop_latched=self.apf_selector.stopped,final_target_speed_kmh=float(target_speed.data),planning_epoch=self.apf_selector.epoch)
            report['policy_mode']=self.apf_selector.policy_mode
            report['behavior_state']={'AVOIDANCE_CHECK':'OBSTACLE_DETECTED','LANE_CHANGE_OUT':'AVOIDANCE','ADJACENT_LANE_FOLLOW':'AVOIDANCE','LANE_RETURN':'RETURN'}.get(apf_choice['state'],apf_choice['state'])
            self.apf_status_pub.publish(String(data=json.dumps(report,allow_nan=False)))

        sample=self._traffic_speed
        image=self.traffic_gate.payload if isinstance(self.traffic_gate.payload,dict) else {}
        traffic.update(stamp_ns=now_ns,configured_enabled=self.get_parameter('traffic_light_enabled').value,
            drive_enabled=enable_drive,drive_status=drive_status,fsm_state=fsm_state_value,
            target_before_traffic_kmh=target_before_traffic,
            target_after_traffic_kmh=requested_target_speed,
            stop_requested=bool(traffic.get('speed_cap_kmh') is not None),
            evidence_count=traffic.get('red_frames',0),
            target_before=target_before_traffic,target_after=requested_target_speed,
            association_file=getattr(self,'_traffic_assoc_file',''),
            corner_target_speed_kmh=recommended_speed,combined_requested_speed_kmh=requested_target_speed,
            final_target_speed_kmh=float(target_speed.data),
            signal_speed_cap_kmh=traffic.get('speed_cap_kmh'),
            traffic_hard_limit_override=bool(self._speed_slew_override and traffic.get('speed_cap_kmh') is not None and traffic['speed_cap_kmh']<=recommended_speed),
            speed_mps=sample[0] if sample else None,speed_age_sec=(now_ns-sample[1])/1e9 if sample else None,
            image_evidence_state=image.get('state','UNKNOWN'))
        raw_state=self.traffic_gate.observed_state
        state_stamp=self.traffic_gate.state_received
        raw_fresh=state_stamp is not None and 0<=now_ns-state_stamp<=1_000_000_000
        observed=raw_state if raw_fresh else 'UNKNOWN'
        traffic.update(perception_state=observed,perception_state_stale=not raw_fresh,
            active_stopline=traffic.get('stop_line_id'),gate_state=traffic['state'])
        self.traffic_status_pub.publish(String(data=json.dumps(traffic,allow_nan=False)))
        log_key=(observed,traffic['state'],traffic.get('stop_line_id'),traffic['reason'],traffic.get('selected_track_id'),traffic.get('stop_distance_source'),traffic.get('crosswalk_correction_reason'))
        if (log_key!=getattr(self,'_traffic_log_key',None)
                or now_ns-getattr(self,'_traffic_log_ns',0)>=1_000_000_000):
            candidate_summary=[dict(center=c.get('center'),state=c.get('class_name'),
                confidence=c.get('confidence'),score=c.get('score'),rejection=c.get('rejection'))
                for c in traffic.get('candidates',[])]
            self.get_logger().info(
                f"traffic={observed} stopline={traffic.get('stop_line_id') or 'none'} "
                f"distance={traffic.get('distance_to_stopline_m')} association={traffic.get('association_id') or 'none'} "
                f"mgeo_distance={traffic.get('mgeo_stopline_distance')} crosswalk={traffic.get('crosswalk_detected',False)} "
                f"crosswalk_stable={traffic.get('crosswalk_stable',False)} crosswalk_distance={traffic.get('crosswalk_distance')} "
                f"stop_distance_source={traffic.get('stop_distance_source','MGEO_FALLBACK')} final_stop_distance={traffic.get('final_stop_distance')} "
                f"crosswalk_reason={traffic.get('crosswalk_correction_reason')} "
                f"detections={traffic.get('detection_count',0)} selected={(traffic.get('selected_candidate') or {}).get('center')} "
                f"score={traffic.get('selected_score')} second={traffic.get('second_best_score')} "
                f"margin={traffic.get('ambiguity_margin')} ambiguous={str(traffic.get('ambiguous',False)).lower()} "
                f"candidates={json.dumps(candidate_summary,separators=(',',':'))} "
                f"stop_requested={str(traffic['stop_requested']).lower()} "
                f"target_speed_before={target_before_traffic:.2f} target_speed_after={requested_target_speed:.2f} "
                f"reason={traffic['reason']} gate_state={traffic['state']} evidence={traffic.get('red_frames',0)} evidence_reason={traffic.get('evidence_reason')} "
                f"final_target={float(target_speed.data):.2f}")
            self._traffic_log_key=log_key;self._traffic_log_ns=now_ns

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
        profile = dict(speed_result.get('corner_profile', {})) if speed_result is not None else {}
        profile.update(frame_id=self.get_parameter('global_path_frame_id').value,
                       stamp_ns=now_ns, status='VALID' if speed_valid else 'INVALID',
                       recommended_speed_kmh=recommended_speed if speed_valid else None,
                       target_speed_kmh=float(target_speed.data), drive_status=drive_status,
                       progress_arc_m=self._progress_arc_m,
                       cruise_speed_kmh=self.get_parameter('cruise_speed_kmh').value,
                       max_drive_speed_kmh=self.get_parameter('max_drive_speed_kmh').value,
                       slew_hard_override=self._speed_slew_override)
        # Diagnostic publisher only: values cannot change speed/FSM decisions.
        profile = {k: v if v is None or isinstance(v, (str, bool)) or _finite_number(v) else None
                   for k, v in profile.items()}
        self.corner_profile_pub.publish(String(data=json.dumps(profile, allow_nan=False)))


def main(args=None):
    import signal
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    running=[True]
    def stop_signal(signum,frame):running[0]=False
    signal.signal(signal.SIGINT,stop_signal);signal.signal(signal.SIGTERM,stop_signal)
    node=PlanningNode()
    try:
        while running[0] and rclpy.ok():rclpy.spin_once(node,timeout_sec=.05)
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        if rclpy.ok() and node.apf_selector is not None:
            node.target_speed_pub.publish(Float32(data=0.));node.local_path_pub.publish(Path())
        node.destroy_node();rclpy.try_shutdown()


if __name__ == '__main__':
    main()
