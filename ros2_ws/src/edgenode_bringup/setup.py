from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'edgenode_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EdgeNode Team',
    maintainer_email='poj285895@gmail.com',
    description='Launch files for EdgeNode autonomy baseline.',
    license='MIT',
)
