"""Disarmed S4 sensor validation only; no Control or vehicle commands.

Supply map odometry separately. Cloud stamps must use the selected ROS clock.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    planning = get_package_share_directory('edgenode_planning')
    perception = get_package_share_directory('edgenode_perception')
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='edgenode_perception', executable='lidar_obstacle_node',
             name='lidar_obstacle_node', output='screen', parameters=[
                 os.path.join(perception, 'config', 'lidar_obstacle.yaml'), sim_time]),
        Node(package='edgenode_planning', executable='planning_node',
             name='planning_node', output='screen', parameters=[
                 os.path.join(planning, 'config', 'planning.yaml'),
                 os.path.join(planning, 'config', 's4_sensor_validation.yaml'), sim_time]),
    ])
