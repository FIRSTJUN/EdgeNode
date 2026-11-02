from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'edgenode_perception'


setup(
    name=package_name,
    version='0.2.0',

    packages=find_packages(exclude=['test']),

    data_files=[
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
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='EdgeNode Team',
    maintainer_email='poj285895@gmail.com',

    description=(
        'ROS2 camera development and local LiDAR obstacle detection for EdgeNode.'
    ),

    license='MIT',

    entry_points={
        'console_scripts': [
            'perception_node = '
            'edgenode_perception.perception_node:main',
            'lidar_obstacle_node = '
            'edgenode_perception.lidar_obstacle_node:main',
        ],
    },
)
