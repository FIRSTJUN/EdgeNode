from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    perception_cfg = os.path.join(
        get_package_share_directory('edgenode_perception'),
        'config', 'perception.yaml')
    planning_cfg = os.path.join(
        get_package_share_directory('edgenode_planning'),
        'config', 'planning.yaml')
    control_cfg = os.path.join(
        get_package_share_directory('edgenode_control'),
        'config', 'control.yaml')

    return LaunchDescription([
        Node(
            package='edgenode_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[perception_cfg],
        ),
        Node(
            package='edgenode_planning',
            executable='planning_node',
            name='planning_node',
            output='screen',
            parameters=[planning_cfg],
        ),
        Node(
            package='edgenode_control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[control_cfg],
        ),
    ])
