"""Start only LiDAR perception; leaves autonomy and enable_drive untouched."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('edgenode_perception'),
                          'config', 'lidar_obstacle.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        Node(package='edgenode_perception', executable='lidar_obstacle_node',
             name='lidar_obstacle_node', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
    ])
