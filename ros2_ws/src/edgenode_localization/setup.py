from setuptools import find_packages, setup


package_name = 'edgenode_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/localization.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EdgeNode Team',
    maintainer_email='poj285895@gmail.com',
    description='GPS/IMU reception and localization scaffolding for MORAI.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'localization_node = edgenode_localization.localization_node:main',
        ],
    },
)
