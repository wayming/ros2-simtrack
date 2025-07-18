import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    xacro_path = PathJoinSubstitution([
        FindPackageShare('mini_spawn_demo'), 'urdf', 'turtlebot3_burger_base.xacro'
    ])
    xacro_abs = os.path.join(
        get_package_share_directory('mini_spawn_demo'), 'urdf', 'turtlebot3_burger_base.xacro'
    )

    return LaunchDescription([
        # 启动 Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', '-r', 'empty.sdf'],
            output='screen'
        ),

        # 启动 robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_path])
            }]
        ),

        # 转换并注入模型
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'"xacro {xacro_abs} > /tmp/tb3.urdf && ' +
                'gz sdf -p /tmp/tb3.urdf > /tmp/tb3.sdf && ' +
                'ros2 run ros_gz_sim create -file /tmp/tb3.sdf -name turtlebot3_burger"'
            ],
            shell=True,
            output='screen'
        ),
    ])
