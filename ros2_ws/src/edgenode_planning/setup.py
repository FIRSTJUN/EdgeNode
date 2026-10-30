from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'edgenode_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='EdgeNode Team',
    maintainer_email='poj285895@gmail.com',
    description='FSM and lane-offset candidate planner.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planning_node = edgenode_planning.planning_node:main',
            'route_editor_node = edgenode_planning.route_editor_node:main',
        ],
    },
)
