"""Low-speed MORAI trial. Disarmed by default; one node per responsibility."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def config(package,name):
        return os.path.join(get_package_share_directory(package),'config',name+'.yaml')
    def value(name,kind):
        return ParameterValue(LaunchConfiguration(name),value_type=kind)
    control=config('edgenode_control','control')
    with open(control,encoding='utf-8') as stream:
        settings=yaml.safe_load(stream)['control_node']['ros__parameters']
    geometry={'avoidance_'+name:settings[name] for name in (
        'wheelbase_m','max_wheel_angle_rad','max_front_steer_normalized','steering_sign',
        'control_rate_hz','lookahead_min_m','lookahead_max_m','lookahead_preview_time_sec')}
    return LaunchDescription([
        DeclareLaunchArgument('enable_drive',default_value='false'),
        DeclareLaunchArgument('speed_kmh',default_value='3.0'),
        DeclareLaunchArgument('hold_duration_s',default_value='3.0'),
        DeclareLaunchArgument('drive_duration_sec',default_value='180.0'),
        Node(package='edgenode_localization',executable='localization_node',output='screen',
             parameters=[config('edgenode_localization','localization')],
             additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}),
        Node(package='edgenode_perception',executable='lidar_obstacle_node',output='screen',
             parameters=[config('edgenode_perception','lidar_obstacle'),{
                 'roi_min_x':-8.,'roi_max_x':40.,'roi_min_y':-6.,'roi_max_y':6.,
                 'oriented_footprints':True}]),
        Node(package='edgenode_planning',executable='planning_node',output='screen',
             parameters=[config('edgenode_planning','planning'),geometry,{
                 'enable_drive':value('enable_drive',bool),
                 'cruise_speed_kmh':value('speed_kmh',float),
                 'curve_speed_kmh':value('speed_kmh',float),
                 'sharp_curve_speed_kmh':value('speed_kmh',float),
                 'max_drive_speed_kmh':value('speed_kmh',float),
                 'drive_duration_sec':value('drive_duration_sec',float),
                 'avoidance_avoid_speed_kmh':value('speed_kmh',float),
                 'avoidance_hold_duration_s':value('hold_duration_s',float),
                 's4_sensor_validation_enabled':True,
                 'avoidance_use_ego_status':True,
                 'avoidance_steering_slew_limit_radps':.6,
                 'avoidance_observation_padding_m':.1,
                 'planning_period_sec':.1,
                 # Keep live sensor validation; historical surveyed visibility
                 # evidence is separate from this simulator-only experiment.
                 's4_distance_validation_enabled':False,
                 # Use directed MGeo neighbor fields; explicit denials still win.
                 'avoidance_require_explicit_permission':False}]),
        Node(package='edgenode_control',executable='control_node',output='screen',parameters=[control,{'steering_slew_limit_radps':.6}]),
    ])
