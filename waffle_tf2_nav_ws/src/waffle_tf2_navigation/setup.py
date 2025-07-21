from setuptools import setup

package_name = 'waffle_tf2_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_tf2.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/tf2_perception.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='TF2 framework for TurtleBot3 Waffle in Gazebo with perception sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_broadcaster = waffle_tf2_navigation.static_broadcaster:main',
            'listener = waffle_tf2_navigation.listener:main',
            'tf_debugger = waffle_tf2_navigation.tf_debugger:main',
        ],
    },
)
