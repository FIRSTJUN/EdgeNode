"""Complete MORAI C-track pipeline; drive authorization is an explicit argument."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nodes = []
    for package, config in [('edgenode_perception', 'perception.yaml'),
                            ('edgenode_planning', 'planning.yaml'),
                            ('edgenode_control', 'control.yaml')]:
        executable = package.replace('edgenode_', '') + '_node'
        params = [os.path.join(get_package_share_directory(package), 'config', config)]
        if package == 'edgenode_planning':
            params.append({'enable_drive': ParameterValue(LaunchConfiguration('enable_drive'), value_type=bool),
                           'drive_duration_sec': ParameterValue(LaunchConfiguration('drive_duration_sec'), value_type=float),
                           'turn_preference': LaunchConfiguration('turn_preference'),
                           'map_file': LaunchConfiguration('map_file')})
        nodes.append(Node(package=package, executable=executable, name=executable,
                          output='screen', parameters=params))
    return LaunchDescription([
        DeclareLaunchArgument('enable_drive', default_value='false'),
        DeclareLaunchArgument('turn_preference', default_value='straight'),
        DeclareLaunchArgument('drive_duration_sec', default_value='60.0'),
        DeclareLaunchArgument('map_file', default_value='/workspace/local_data/c_track_mgeo/link_set.json'),
        *nodes,
    ])
