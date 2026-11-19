from setuptools import setup,find_packages
setup(name='edgenode_static20_experiment',version='0.1.0',packages=find_packages(exclude=['test']),
 data_files=[('share/ament_index/resource_index/packages',['resource/edgenode_static20_experiment']),('share/edgenode_static20_experiment',['package.xml'])],
 install_requires=['setuptools'],zip_safe=False,maintainer='EdgeNode Team',maintainer_email='poj285895@gmail.com',license='MIT',
 entry_points={'console_scripts':['static20_planning_node = edgenode_static20_experiment.static20_node:main',
 'static20_localization_node = edgenode_static20_experiment.source_time_localization:main']})
