from setuptools import setup,find_packages
setup(name='edgenode_dynamic_tracking',version='0.1.0',packages=find_packages(exclude=['test']),
 data_files=[('share/ament_index/resource_index/packages',['resource/edgenode_dynamic_tracking']),('share/edgenode_dynamic_tracking',['package.xml'])],
 install_requires=['setuptools'],zip_safe=False,maintainer='EdgeNode Team',maintainer_email='poj285895@gmail.com',license='MIT',
 entry_points={'console_scripts':['dynamic_tracking_node = edgenode_dynamic_tracking.node:main',
 'tracking_normal_planning_node = edgenode_dynamic_tracking.normal_node:main']})
