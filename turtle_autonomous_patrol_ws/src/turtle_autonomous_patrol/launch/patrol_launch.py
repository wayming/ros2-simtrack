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

    patrol_laster_avoider = Node(
        package='turtle_autonomous_patrol',
        executable='laser_avoider',
        output='screen'
    )

    patrol_camera_alert = Node(
        package='turtle_autonomous_patrol',
        executable='camera_alert',
        output='screen'
    )

    patrol_server_node = Node(
        package='turtle_autonomous_patrol',
        executable='patrol_server',
        name='patrol_server',
        output='screen'
    )

    patrol_client_node = Node(
        package='turtle_autonomous_patrol',
        executable='patrol_client',
        name='patrol_client',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,

        # 使用 TimerAction 延迟启动 ROS 2 节点，确保 Gazebo 启动完成
        TimerAction(
            period=5.0,  # 延迟5秒
            actions=[
                patrol_laster_avoider,
                patrol_camera_alert,
                patrol_server_node,
            ]
        ),
        
    ])
