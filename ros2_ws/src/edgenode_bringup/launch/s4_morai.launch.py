"""MORAI speed profiles, disarmed and limited to independently verified speeds."""
import os
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
    route_file=LaunchConfiguration('route_file').perform(context)
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
    return [
        Node(package='edgenode_localization',executable='localization_node',output='screen',
             parameters=[config('edgenode_localization','localization'),{'morai_pose_reset_enabled':True}],
             additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}),
        Node(package='edgenode_perception',executable='lidar_obstacle_node',output='screen',
             parameters=[config('edgenode_perception','lidar_obstacle'),{
                 'roi_min_x':-8.,'roi_max_x':value('lidar_roi_max_x',float),'roi_min_y':-40.,'roi_max_y':40.,
                 'oriented_footprints':True,'ground_plane_filter_enabled':True,
                 # Measured Ego returns end below sensor Z=0. A 0.3m
                 # roof margin excludes the signal housings seen at +0.5m.
                 'obstacle_height_filter_enabled':True,
                 'obstacle_max_z_m':value('lidar_obstacle_max_z_m',float),
                 'gravity_ground_compensation':True}],additional_env=env),
        Node(package='edgenode_planning',executable='planning_node',output='screen',
             parameters=[config('edgenode_planning','planning'),geometry,mission,{
                 'enable_drive':value('enable_drive',bool),
                 'cruise_speed_kmh':value('cruise_speed_kmh',float),
                 'curve_speed_kmh':value('curve_speed_kmh',float),
                 'sharp_curve_speed_kmh':value('sharp_curve_speed_kmh',float),
                 'max_drive_speed_kmh':value('max_drive_speed_kmh',float),
                 'drive_duration_sec':value('drive_duration_sec',float),
                 'avoidance_avoid_speed_kmh':value('avoid_speed_kmh',float),
                 'avoidance_hold_duration_s':value('hold_duration_s',float),
                 'local_path_length_m':80.,'s4_control_horizon_m':20.,
                 'speed_preview_distance_m':80.,'speed_anticipatory_enabled':True,
                 'avoidance_normal_obstacle_confirmations':2,
                 'avoidance_normal_detection_latency_s':1.6,
                 'avoidance_entry_geometry_wait_distance_m':6.,
                 'morai_speed_admission_enabled':True,
                 'morai_validated_speed_cap_kmh':value('validated_speed_cap_kmh',float),
                 'morai_verified_detection_distance_m':value('verified_detection_distance_m',float),
                 's4_sensor_validation_enabled':True,
                 'avoidance_use_ego_status':True,
                 'avoidance_steering_slew_limit_radps':.6,
                 'avoidance_observation_padding_m':.1,
                 'planning_period_sec':.1,
                 # Keep live sensor validation; historical surveyed visibility
                 # evidence is separate from this simulator-only experiment.
                 's4_distance_validation_enabled':False,
                 # Use directed MGeo neighbor fields; explicit denials still win.
                 'avoidance_require_explicit_permission':False}],additional_env=env),
        Node(package='edgenode_control',executable='control_node',output='screen',parameters=[control,{
            'steering_slew_limit_radps':.6,'lookahead_max_m':value('lookahead_max_m',float),
            'normal_lookahead_min_m':value('normal_lookahead_min_m',float),
            'lookahead_preview_time_sec':value('lookahead_preview_time_sec',float),
            'lookahead_heading_budget_rad':value('lookahead_heading_budget_rad',float)}],additional_env=env),
    ]


def generate_launch_description():
    arguments=[
        ('enable_drive','false'),('speed_kmh','50.0'),
        ('cruise_speed_kmh',LaunchConfiguration('speed_kmh')),('curve_speed_kmh','25.0'),
        ('sharp_curve_speed_kmh','15.0'),('avoidance_speed_kmh','10.0'),
        ('avoid_speed_kmh',LaunchConfiguration('avoidance_speed_kmh')),
        ('max_drive_speed_kmh',LaunchConfiguration('speed_kmh')),
        ('validated_speed_cap_kmh','15.0'),('verified_detection_distance_m','20.0'),
        ('hold_duration_s','3.0'),('drive_duration_sec','3600.0'),
        ('lookahead_max_m','12.0'),('lookahead_preview_time_sec','0.8'),
        ('normal_lookahead_min_m','3.5'),('lookahead_heading_budget_rad','1.6'),
        ('lidar_roi_max_x','100.0'),
        ('lidar_obstacle_max_z_m','0.3'),
        ('route_file','/workspace/local_data/kcity_routes/kcity_demo_route.yaml'),
        ('route_start_link_id','A219BS010380'),
    ]
    return LaunchDescription([DeclareLaunchArgument(name,default_value=value) for name,value in arguments]
        +[OpaqueFunction(function=launch_nodes)])
