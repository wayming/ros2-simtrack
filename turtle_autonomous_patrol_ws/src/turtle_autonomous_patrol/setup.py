from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtle_autonomous_patrol'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'turtle_interfaces'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='TurtleBot3 patrol bot with Lidar obstacle avoidance and camera alert',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'patrol_server = turtle_autonomous_patrol.patrol_server:main',
            'patrol_client = turtle_autonomous_patrol.patrol_client:main',
            'laser_avoider = turtle_autonomous_patrol.laser_avoider:main',
            'camera_alert = turtle_autonomous_patrol.camera_alert:main',
        ],
    },
)
