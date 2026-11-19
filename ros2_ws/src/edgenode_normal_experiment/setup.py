from setuptools import setup,find_packages
setup(name='edgenode_normal_experiment',version='0.1.0',packages=find_packages(exclude=['test']),
 data_files=[('share/ament_index/resource_index/packages',['resource/edgenode_normal_experiment']),
 ('share/edgenode_normal_experiment',['package.xml'])],install_requires=['setuptools'],zip_safe=False,
 maintainer='EdgeNode Team',maintainer_email='poj285895@gmail.com',license='MIT',
 entry_points={'console_scripts':['normal_planning_node = edgenode_normal_experiment.normal_node:main']})
