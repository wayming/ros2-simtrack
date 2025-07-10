from setuptools import setup

package_name = 'turtle_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='TurtleBot3 monitoring and control node with topics, services, actions.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_node = turtle_monitor.monitor_node:main',
            'set_max_speed_service = turtle_monitor.set_max_speed_service:main',
            'patrol_server = turtle_monitor.patrol_server:main',
            'patrol_client = turtle_monitor.patrol_client:main',
        ],
    },
)
