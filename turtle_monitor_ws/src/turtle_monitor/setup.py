from setuptools import setup
import os
from glob import glob

package_name = 'turtle_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Required ROS 2 package files
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='...',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'patrol_server = turtle_monitor.patrol_server:main',
            'patrol_client = turtle_monitor.patrol_client:main',
            'monitor_node = turtle_monitor.monitor_node:main',
            'max_speed_service = turtle_monitor.max_speed_service:main',
        ],
    },
)
