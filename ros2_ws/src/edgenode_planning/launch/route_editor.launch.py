"""Start the draft route editor and, when installed, RViz2 separately from autonomy."""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start_rviz(context):
    if not IfCondition(LaunchConfiguration('use_rviz')).evaluate(context):
        return []
    try:
        get_package_share_directory('rviz2')
    except PackageNotFoundError:
        return [LogInfo(msg=(
            'RViz2 is not installed in this environment. The Route Editor is running. '
            'Use RViz2 on a ROS-connected GUI host with the supplied kcity_route_editor.rviz '
            'or install ros-humble-rviz2 in this container.'
        ))]
    return [Node(package='rviz2', executable='rviz2', name='route_editor_rviz',
                 arguments=['-d', LaunchConfiguration('rviz_config')], output='screen')]


def generate_launch_description():
    share = get_package_share_directory('edgenode_planning')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('mgeo_dir', default_value='/workspace/local_data/kcity_mgeo'),
        DeclareLaunchArgument('start_link_id', default_value=''),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            share, 'rviz', 'kcity_route_editor.rviz')),
        Node(package='edgenode_planning', executable='route_editor_node',
             name='route_editor_node', output='screen', parameters=[
                 os.path.join(share, 'config', 'route_editor.yaml'),
                 {'mgeo_dir': ParameterValue(LaunchConfiguration('mgeo_dir'), value_type=str),
                  'start_link_id': ParameterValue(LaunchConfiguration('start_link_id'), value_type=str)},
             ]),
        OpaqueFunction(function=start_rviz),
    ])
