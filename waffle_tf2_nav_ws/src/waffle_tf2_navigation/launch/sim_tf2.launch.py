from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
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
        'turtlebot3_waffle.urdf',
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

    return LaunchDescription([
        namespace_arg,
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': True
            }]
        ),

        Node(
            package='waffle_tf2_navigation',
            executable='static_broadcaster',
            output='screen'
        ),

        Node(
            package='waffle_tf2_navigation',
            executable='listener',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        )
    ])
