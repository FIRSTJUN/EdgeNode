"""D dynamic overtaking with preserved A/B safety and source-time C tracks."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
def generate_launch_description():
    env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}
    return LaunchDescription([
        DeclareLaunchArgument('enable_drive',default_value='false'),
        DeclareLaunchArgument('planning_profile_file',default_value='/workspace/config/s4_d/planning_20.yaml'),
        Node(package='edgenode_static20_experiment',executable='static20_localization_node',parameters=['/workspace/config/s4_d/localization_source_time.yaml'],additional_env=env,output='screen'),
        Node(package='edgenode_perception',executable='lidar_obstacle_node',parameters=['/workspace/config/s4_d/perception.yaml'],additional_env=env,output='screen'),
        Node(package='edgenode_perception',executable='lidar_obstacle_node',name='rear_lidar_obstacle_node',parameters=['/workspace/config/s4_d/perception_rear.yaml'],remappings=[('/perception/lidar_frame','/perception/rear_lidar_frame'),('/perception/lidar_diagnostics','/perception/rear_lidar_diagnostics')],additional_env=env,output='screen'),
        Node(package='edgenode_dynamic_overtake',executable='rear_sensing_node',additional_env=env,output='screen'),
        Node(package='edgenode_dynamic_overtake',executable='dynamic_planning_node',parameters=[LaunchConfiguration('planning_profile_file'),{'enable_drive':ParameterValue(LaunchConfiguration('enable_drive'),value_type=bool)}],additional_env=env,output='screen'),
        Node(package='edgenode_control',executable='control_node',parameters=['/workspace/config/s4_d/control_node.yaml'],additional_env=env,output='screen'),
        Node(package='edgenode_dynamic_overtake',executable='road_tracking_node',parameters=['/workspace/config/s4_d/tracking.yaml'],additional_env=env,output='screen'),
    ])
