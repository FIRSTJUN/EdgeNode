"""C tracking only, with preserved A NORMAL-only control and B source-time pose."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
def generate_launch_description():
    env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}
    return LaunchDescription([
        DeclareLaunchArgument('enable_drive',default_value='false'),
        DeclareLaunchArgument('planning_profile_file',default_value='/workspace/config/s4_c/planning_normal.yaml'),
        Node(package='edgenode_static20_experiment',executable='static20_localization_node',parameters=['/workspace/config/s4_c/localization_source_time.yaml'],additional_env=env,output='screen'),
        Node(package='edgenode_perception',executable='lidar_obstacle_node',parameters=['/workspace/config/s4_c/perception.yaml'],additional_env=env,output='screen'),
        Node(package='edgenode_dynamic_tracking',executable='tracking_normal_planning_node',parameters=[LaunchConfiguration('planning_profile_file'),{'enable_drive':ParameterValue(LaunchConfiguration('enable_drive'),value_type=bool)}],additional_env=env,output='screen'),
        Node(package='edgenode_control',executable='control_node',parameters=['/workspace/config/s4_c/control_node.yaml'],additional_env=env,output='screen'),
        Node(package='edgenode_dynamic_tracking',executable='dynamic_tracking_node',parameters=['/workspace/config/s4_c/tracking.yaml'],additional_env=env,output='screen'),
    ])
