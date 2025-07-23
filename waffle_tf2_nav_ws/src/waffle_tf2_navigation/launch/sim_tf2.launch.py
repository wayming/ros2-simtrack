from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace prefix for frames'
    )

    urdf_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_description'),
        'urdf',
        'turtlebot3_waffle.urdf'
    ])

    robot_description_content = Command([
        'xacro ',
        urdf_file,
        ' namespace:=', LaunchConfiguration('namespace')
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('waffle_tf2_navigation'),
        'rviz',
        'tf2_perception.rviz'
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ])
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    static_broadcaster_node = Node(
        package='waffle_tf2_navigation',
        executable='static_broadcaster',
        output='screen'
    )

    listener_node = Node(
        package='waffle_tf2_navigation',
        executable='listener',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        gazebo_launch,
        TimerAction(period=5.0, actions=[robot_state_publisher_node]),
        TimerAction(period=8.0, actions=[static_broadcaster_node]),
        TimerAction(period=11.0, actions=[listener_node]),
        TimerAction(period=13.0, actions=[rviz_node]),
    ])
