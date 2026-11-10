from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'edgenode_perception'


setup(
    name=package_name,
    version='0.2.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/lane_maneuver_*.json') + glob('config/apf_simple_*.json'),
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='EdgeNode Team',
    maintainer_email='poj285895@gmail.com',

    description=(
        'Minimal ROS2 camera perception development package for EdgeNode.'
    ),

    license='MIT',

    entry_points={
        'console_scripts': [
            'apf_build_evidence = edgenode_perception.apf_measurement_evidence:main',
            'lane_occupancy_node = edgenode_perception.lane_occupancy_node:main',
            'apf_planning_provider = edgenode_perception.apf_planning_provider_node:main',
            'apf_drive_controller = edgenode_perception.apf_drive_runtime:controller_main',
            'apf_drive_test_node = edgenode_perception.apf_drive_test_node:main',
            'apf_drive_command_gate = edgenode_perception.apf_drive_command_gate_node:main',
            'lane_maneuver_shadow = edgenode_perception.lane_maneuver_shadow_node:main',
            'lane_detection_node = edgenode_perception.lane_detection_node:main',
            'pedestrian_morai_trial = edgenode_perception.morai_trial_node:main',
            'person_detector_node = edgenode_perception.person_detector_node:main',
            'pedestrian_fusion_shadow_node = edgenode_perception.pedestrian_fusion_node:main',
            'pedestrian_shadow_node = edgenode_perception.pedestrian_node:main',
            'crosswalk_detector = edgenode_perception.crosswalk_detector:main',
            'traffic_light_node = edgenode_perception.traffic_light_node:main',
            'perception_node = edgenode_perception.perception_node:main',
            'traffic_light_detector = edgenode_perception.traffic_light_detector:main',
            'traffic_light_debug_viewer = edgenode_perception.traffic_light_debug_viewer:main',
            'traffic_light_image_collector = edgenode_perception.traffic_light_image_collector:main',
        ],
    },
)
