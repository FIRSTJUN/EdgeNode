from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import yaml


def generate_launch_description():
    localization_cfg = os.path.join(
        get_package_share_directory('edgenode_localization'),
        'config',
        'localization.yaml',
    )

    planning_cfg = os.path.join(
        get_package_share_directory('edgenode_planning'),
        'config',
        'planning.yaml',
    )

    control_cfg = os.path.join(
        get_package_share_directory('edgenode_control'),
        'config',
        'control.yaml',
    )

    # One startup source for planner feasibility and the unchanged Control.
    with open(control_cfg, encoding='utf-8') as stream:
        control_geometry = yaml.safe_load(stream)['control_node']['ros__parameters']
    lattice_geometry = {
        'lattice_' + name: control_geometry[name]
        for name in ('wheelbase_m', 'max_wheel_angle_rad', 'max_front_steer_normalized')
    }

    return LaunchDescription([
        Node(
            package='edgenode_localization',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[localization_cfg],
            additional_env={
                'OPENBLAS_NUM_THREADS': '1',
                'OMP_NUM_THREADS': '1',
                'MKL_NUM_THREADS': '1',
                'NUMEXPR_NUM_THREADS': '1',
            },
        ),

        Node(
            package='edgenode_planning',
            executable='planning_node',
            name='planning_node',
            output='screen',
            parameters=[planning_cfg, lattice_geometry],
        ),

        Node(
            package='edgenode_control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[control_cfg],
        ),
    ])
