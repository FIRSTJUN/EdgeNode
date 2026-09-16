from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'edgenode_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EdgeNode Team',
    maintainer_email='poj285895@gmail.com',
    description='Camera lane detection and LiDAR DBSCAN obstacle perception.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perception_node = edgenode_perception.perception_node:main',
        ],
    },
)
