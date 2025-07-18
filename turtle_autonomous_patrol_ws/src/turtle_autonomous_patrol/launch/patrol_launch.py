import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

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
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('turtle_autonomous_patrol'), 'config', 'params.yaml')],
    )

    patrol_camera_alert = Node(
        package='turtle_autonomous_patrol',
        executable='camera_alert',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,

        # 使用 TimerAction 延迟启动 ROS 2 节点，确保 Gazebo 启动完成
        TimerAction(
            period=5.0,  # 延迟5秒
            actions=[
<<<<<<< HEAD
                robot_state_publisher_node,
                spawn_entity,
                ros_gz_bridge_camera,
                # patrol_laster_avoider,
                # patrol_camera_alert,
=======
                patrol_laster_avoider,
                patrol_camera_alert,
>>>>>>> parent of f6724d4 (custom model)
            ]
        ),
        
    ])
