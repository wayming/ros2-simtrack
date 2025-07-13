from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ])
    )

    patrol_server_node = Node(
        package='turtle_monitor',
        executable='patrol_server',
        name='patrol_server',
        output='screen'
    )

    patrol_client_node = Node(
        package='turtle_monitor',
        executable='patrol_client',
        name='patrol_client',
        output='screen'
    )

    monitor_node = Node(
            package='turtle_monitor',
            executable='monitor_node',
            name='monitor_node',
            output='screen'
    )
    
    max_speed_service_node = Node(
        package='turtle_monitor',
        executable='max_speed_service',
        name='max_speed_service',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,

        # 使用 TimerAction 延迟启动 ROS 2 节点，确保 Gazebo 启动完成
        TimerAction(
            period=5.0,  # 延迟5秒
            actions=[
                patrol_server_node,
                monitor_node,
                max_speed_service_node
            ]
        ),
        
        # 等 patrol_server 启动后再启动 patrol_client
        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=patrol_server_node,
        #         on_start=[patrol_client_node]
        #     )
        # )
    ])
