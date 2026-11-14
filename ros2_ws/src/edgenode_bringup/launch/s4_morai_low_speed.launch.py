"""5 km/h normal / 3 km/h S4 regression using the production nodes."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path=os.path.join(get_package_share_directory('edgenode_bringup'),'launch','s4_morai.launch.py')
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(path),launch_arguments={
        'speed_kmh':'5.0','cruise_speed_kmh':'5.0','curve_speed_kmh':'5.0','sharp_curve_speed_kmh':'5.0',
        'avoidance_speed_kmh':'3.0','avoid_speed_kmh':'3.0','max_drive_speed_kmh':'5.0',
        'validated_speed_cap_kmh':'5.0',
        'normal_lookahead_min_m':'2.5','lookahead_preview_time_sec':'0.75',
        'lookahead_heading_budget_rad':'0.4',
    }.items())])
