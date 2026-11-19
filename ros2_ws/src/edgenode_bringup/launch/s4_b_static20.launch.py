"""Opt-in Phase B static avoidance. Preserved v61 implementation, B profiles."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition,UnlessCondition
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}
    source_time=PythonExpression(["'",LaunchConfiguration('source_time_localization'),"'.lower() == 'true' and '",LaunchConfiguration('use_v61_baseline'),"'.lower() != 'true'"])
    return LaunchDescription([
        DeclareLaunchArgument('enable_drive',default_value='false'),
        DeclareLaunchArgument('use_v61_baseline',default_value='false'),
        DeclareLaunchArgument('source_time_localization',default_value='true'),
        DeclareLaunchArgument('localization_profile_file',default_value='/workspace/config/s4_b/localization_source_time.yaml'),
        DeclareLaunchArgument('planning_profile_file',default_value='/workspace/config/s4_b/planning_20.yaml'),
        Node(package='edgenode_localization',executable='localization_node',parameters=['/workspace/config/s4_b/localization_node.yaml'],condition=UnlessCondition(source_time),additional_env=env,output='screen'),
        Node(package='edgenode_static20_experiment',executable='static20_localization_node',parameters=[LaunchConfiguration('localization_profile_file')],condition=IfCondition(source_time),additional_env=env,output='screen'),
        Node(package='edgenode_perception',executable='lidar_obstacle_node',parameters=['/workspace/config/s4_b/perception.yaml'],additional_env=env,output='screen'),
        Node(package='edgenode_planning',executable='planning_node',parameters=[LaunchConfiguration('planning_profile_file'),{'enable_drive':ParameterValue(LaunchConfiguration('enable_drive'),value_type=bool)}],condition=IfCondition(LaunchConfiguration('use_v61_baseline')),additional_env=env,output='screen'),
        Node(package='edgenode_static20_experiment',executable='static20_planning_node',parameters=[LaunchConfiguration('planning_profile_file'),{'enable_drive':ParameterValue(LaunchConfiguration('enable_drive'),value_type=bool)}],condition=UnlessCondition(LaunchConfiguration('use_v61_baseline')),additional_env=env,output='screen'),
        Node(package='edgenode_control',executable='control_node',parameters=['/workspace/config/s4_b/control_node.yaml'],additional_env=env,output='screen'),
    ])
