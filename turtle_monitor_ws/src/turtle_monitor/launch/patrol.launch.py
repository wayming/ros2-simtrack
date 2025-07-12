# turtle_monitor/launch/patrol_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_monitor',
            executable='patrol_server',
            name='patrol_server'
        ),
        Node(
            package='turtle_monitor',
            executable='monitor_node',
            name='monitor_node'
        ),
        Node(
            package='turtle_monitor',
            executable='max_speed_service',
            name='max_speed_service'
        ),
        Node(
            package='turtle_monitor',
            executable='patrol_client',
            name='patrol_client',
            output='screen'
        ),
    ])
