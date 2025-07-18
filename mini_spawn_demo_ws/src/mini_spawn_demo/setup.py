from setuptools import setup

package_name = 'mini_spawn_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mini_spawn.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/turtlebot3_burger_base.xacro']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Pure Python ROS 2 app to spawn TurtleBot3 Burger with xacro wrapper',
    license='Apache-2.0',
)
