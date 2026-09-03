from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'map_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
       
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
        # Launch files in this repository use the conventional *.launch.py
        # suffix only in their filename stem, so install every Python launch file.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
     
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amirhossein',
    maintainer_email='ahmadinejad.sut@gmail.com',
    description='ROS 2 package for map publishing, localization and path planning',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'map_publisher_node = map_publisher.map_publisher_node:main',
            'astar_planner_node = map_publisher.astar_planner_node:main',
            'frame_id_converter = map_publisher.frame_id_converter:main',
            'motor_command_node = map_publisher.motor_command:main',
            'ekf_node = map_publisher.ekf_node:main',
            'particle_filter_node = map_publisher.particle_filter_node:main',
        ],
    },
)
