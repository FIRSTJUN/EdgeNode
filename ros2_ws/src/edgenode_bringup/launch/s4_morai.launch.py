"""MORAI operator targets; starts disarmed without admission speed ceilings."""
import os
import math
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_nodes(context):
    def config(package,name):
        return os.path.join(get_package_share_directory(package),'config',name+'.yaml')
    def value(name,kind):
        return ParameterValue(LaunchConfiguration(name),value_type=kind)
    control=config('edgenode_control','control')
    with open(control,encoding='utf-8') as stream:
        settings=yaml.safe_load(stream)['control_node']['ros__parameters']
    geometry={'avoidance_'+name:settings[name] for name in (
        'wheelbase_m','max_wheel_angle_rad','max_front_steer_normalized','steering_sign',
        'control_rate_hz','lookahead_min_m')}
    geometry.update(avoidance_lookahead_max_m=value('lookahead_max_m',float),
        avoidance_lookahead_preview_time_sec=value('lookahead_preview_time_sec',float),
        avoidance_lookahead_heading_budget_rad=value('lookahead_heading_budget_rad',float))
    avoid_speed=float(LaunchConfiguration('avoid_speed_kmh').perform(context))/3.6
    lookahead=max(settings['lookahead_min_m'],min(
        float(LaunchConfiguration('lookahead_max_m').perform(context)),
        avoid_speed*float(LaunchConfiguration('lookahead_preview_time_sec').perform(context))))
    # Same two-delay-horizon rollout as StaticObstaclePlanner, plus window
    # support. Extend output and validation together; no safety gate changes.
    s4_horizon=max(20.,5.*math.ceil((4.*avoid_speed+lookahead+.6+2.)/5.))
    route_file=LaunchConfiguration('route_file').perform(context)
    planning_profile=LaunchConfiguration('planning_profile_file').perform(context)
    if planning_profile and not os.path.isfile(planning_profile):
        raise ValueError('planning_profile_file must be an existing preserved ROS parameter YAML')
    mission={}
    if route_file:
        with open(route_file,encoding='utf-8') as stream:route=yaml.safe_load(stream)
        ids=route['waypoint_node_ids']
        if not isinstance(ids,list) or not ids or any(not isinstance(x,str) for x in ids):
            raise ValueError('route_file must contain ordered waypoint_node_ids')
        mission={'route_node_ids':ids,'route_start_link_id':
            route.get('start_link_id') or LaunchConfiguration('route_start_link_id').perform(context)}
        if not mission['route_start_link_id']:raise ValueError('saved route origin is required')
    env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}
    lane_candidates=LaunchConfiguration('static_avoidance_method').perform(context)=='lane_candidates'
    return [
        Node(package='edgenode_localization',executable='localization_node',output='screen',
             parameters=[config('edgenode_localization','localization'),{
                 'morai_pose_reset_enabled':True,'imu_yaw_frame':'enu'}],
             additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}),
        Node(package='edgenode_perception',executable='lidar_obstacle_node',output='screen',
             parameters=[config('edgenode_perception','lidar_obstacle'),{
                 'roi_min_x':-8.,'roi_max_x':value('lidar_roi_max_x',float),'roi_min_y':-40.,'roi_max_y':40.,
                 'oriented_footprints':True,'ground_plane_filter_enabled':True,
                 'clipped_footprints':value('lidar_clipped_footprints',bool),
                 'tiled_minimum_area_bounds':value('lidar_tiled_minimum_area_bounds',bool),
                 'native_geometry':value('lidar_native_geometry',bool),
                 'publish_debug_clouds':value('lidar_debug_clouds',bool),
                 'footprint_tile_size_m':value('lidar_footprint_tile_m',float),
                 # Measured Ego returns end below sensor Z=0. A 0.3m
                 # roof margin excludes the signal housings seen at +0.5m.
                 'obstacle_height_filter_enabled':True,
                 'obstacle_max_z_m':value('lidar_obstacle_max_z_m',float),
                 'gravity_ground_compensation':True,
                 'source_pair_wait_sec':value('lidar_source_pair_wait_sec',float),
                 'source_pose_max_gap_sec':value('lidar_source_pose_max_gap_sec',float),
                 'time_aligned_obstacles':True}],additional_env=env),
        Node(package='edgenode_planning',executable='planning_node',output='screen',
             parameters=[config('edgenode_planning','planning'),geometry,mission,{
                 'enable_drive':value('enable_drive',bool),
                 'cruise_speed_kmh':value('cruise_speed_kmh',float),
                 'sharp_curve_speed_kmh':value('sharp_curve_speed_kmh',float),
                 'drive_duration_sec':value('drive_duration_sec',float),
                 'avoidance_avoid_speed_kmh':value('avoid_speed_kmh',float),
                 # Simulator-only measured-braking experiment. The production
                 # AvoidanceConfig and NORMAL speed/steering profile stay intact.
                 'avoidance_deceleration_mps2':value('s4_deceleration_mps2',float),
                 'avoidance_memory_merge_extent_m':value('s4_memory_merge_extent_m',float),
                 'morai_unmarked_junction_experiment':value('morai_unmarked_junction_experiment',bool),
                 'morai_surface_offset_experiment':value('morai_surface_offset_experiment',bool),
                 'morai_transient_sensor_recovery':value('morai_transient_sensor_recovery',bool),
                 's4_atomic_sensor_frames':value('s4_atomic_sensor_frames',bool),
                 'goal_approach_enabled':value('goal_approach_enabled',bool),
                 'vehicle_geometry_profile':LaunchConfiguration('vehicle_geometry_profile'),
                 'normal_merge_correction_enabled':value('normal_merge_correction_enabled',bool),
                 'normal_surveyed_center_correction_enabled':value('normal_surveyed_center_correction_enabled',bool),
                 'normal_occlusion_speed_enabled':value('normal_occlusion_speed_enabled',bool),
                 'surveyed_road_file':LaunchConfiguration('surveyed_road_file'),
                 'surveyed_road_sha256':LaunchConfiguration('surveyed_road_sha256'),
                 'avoidance_search_budget_sec':value('avoidance_search_budget_sec',float),
                 'avoidance_normal_prediction_margin_m':value('normal_prediction_margin_m',float),
                 'static_avoidance_method':LaunchConfiguration('static_avoidance_method'),
                 'avoidance_continuous_lane_follow':True if lane_candidates else value('morai_continuous_avoidance',bool),
                 'avoidance_integrated_strategy':True if lane_candidates else value('morai_integrated_lattice',bool),
                 'avoidance_optimize_entry_onset':value('morai_natural_avoidance_onset',bool),
                 'avoidance_hold_duration_s':value('hold_duration_s',float),
                 'local_path_length_m':80.,'s4_control_horizon_m':s4_horizon,
                 'speed_preview_distance_m':80.,'speed_anticipatory_enabled':True,
                 # 60 km/h at k=.03 requires 8.33 m/s². Use the configured
                 # 30 target before those turns; tighter turns also respect
                 # the measured dry-MORAI friction budget (mu=1, g=9.81).
                 'curve_curvature_threshold':.02,
                 'sharp_curvature_threshold':value('normal_sharp_curvature_threshold',float),
                 'normal_physical_lateral_acceleration_mps2':value('normal_lateral_limit_mps2',float),
                 'normal_tracking_lateral_acceleration_mps2':value('normal_tracking_lateral_limit_mps2',float),
                 'avoidance_normal_obstacle_confirmations':2,
                 'avoidance_normal_detection_latency_s':1.6,
                 'avoidance_entry_geometry_wait_distance_m':6.,
                 's4_sensor_validation_enabled':True,
                 'avoidance_use_ego_status':True,
                 'avoidance_steering_slew_limit_radps':.6,
                 'avoidance_observation_padding_m':.1,
                 'planning_period_sec':.1,
                 # Keep live sensor validation; historical surveyed visibility
                 # evidence is separate from this simulator-only experiment.
                 's4_distance_validation_enabled':False,
                 # Keep directed topology and explicit denials; add original HD-map boundaries.
                 'avoidance_require_explicit_permission':False,
                 'avoidance_boundary_dir':'/workspace/local_data/kcity_hdmap'}]+
                 ([planning_profile] if planning_profile else []),additional_env=env),
        Node(package='edgenode_control',executable='control_node',output='screen',parameters=[control,{
            'steering_slew_limit_radps':.6,'lookahead_max_m':value('lookahead_max_m',float),
            'normal_lookahead_min_m':value('normal_lookahead_min_m',float),
            # NORMAL limits corner cutting independently of the validated S4 rollout.
            'normal_lookahead_heading_budget_rad':value('normal_lookahead_heading_budget_rad',float),
            'normal_prediction_time_sec':value('normal_prediction_time_sec',float),
            'normal_max_front_steer_normalized':value('normal_max_front_steer_normalized',float),
            'lookahead_preview_time_sec':value('lookahead_preview_time_sec',float),
            'lookahead_heading_budget_rad':value('lookahead_heading_budget_rad',float)}],additional_env=env),
    ]


def generate_launch_description():
    arguments=[
        ('enable_drive','false'),('speed_kmh','60.0'),
        ('planning_profile_file',''),
        ('cruise_speed_kmh',LaunchConfiguration('speed_kmh')),
        ('sharp_curve_speed_kmh','30.0'),('avoidance_speed_kmh','10.0'),
        ('avoid_speed_kmh',LaunchConfiguration('avoidance_speed_kmh')),
        ('hold_duration_s','3.0'),('drive_duration_sec','3600.0'),
        ('s4_deceleration_mps2','2.0'),
        ('s4_memory_merge_extent_m','3.5'),
        ('morai_unmarked_junction_experiment','false'),
        ('morai_continuous_avoidance','false'),
        ('morai_integrated_lattice','false'),
        ('morai_natural_avoidance_onset','false'),
        ('static_avoidance_method','lattice'),
        ('morai_surface_offset_experiment','false'),
        ('morai_transient_sensor_recovery','false'),
        ('s4_atomic_sensor_frames','true'),
        ('goal_approach_enabled','true'),
        ('vehicle_geometry_profile','sonata_nominal'),
        ('normal_merge_correction_enabled','false'),
        ('normal_surveyed_center_correction_enabled','false'),
        ('normal_occlusion_speed_enabled','false'),
        ('surveyed_road_file',''),('surveyed_road_sha256',''),
        ('avoidance_search_budget_sec','6.0'),
        ('normal_prediction_margin_m','0.0'),
        ('lookahead_max_m','12.0'),('lookahead_preview_time_sec','0.8'),
        ('normal_lookahead_min_m','4.0'),('lookahead_heading_budget_rad','1.6'),
        ('normal_lookahead_heading_budget_rad','0.25'),
        ('normal_prediction_time_sec','0.1'),('normal_max_front_steer_normalized','1.0'),
        ('normal_sharp_curvature_threshold','0.03'),('normal_lateral_limit_mps2','9.0'),
        ('normal_tracking_lateral_limit_mps2','3.0'),
        ('lidar_roi_max_x','100.0'),
        ('lidar_clipped_footprints','true'),
        ('lidar_tiled_minimum_area_bounds','false'),
        ('lidar_native_geometry','false'),
        ('lidar_debug_clouds','true'),
        ('lidar_footprint_tile_m','3.0'),
        ('lidar_obstacle_max_z_m','0.3'),
        ('lidar_source_pair_wait_sec','0.25'),('lidar_source_pose_max_gap_sec','0.25'),
        ('route_file','/workspace/local_data/kcity_routes/kcity_demo_route.yaml'),
        ('route_start_link_id','A219BS010380'),
    ]
    return LaunchDescription([DeclareLaunchArgument(name,default_value=value) for name,value in arguments]
        +[OpaqueFunction(function=launch_nodes)])
