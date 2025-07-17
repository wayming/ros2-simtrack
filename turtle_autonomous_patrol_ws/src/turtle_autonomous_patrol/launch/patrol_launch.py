import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 获取turtlebot3_description包的路径
    tb3_description_path = get_package_share_directory('turtlebot3_description')
    # 正确设置Gazebo资源路径（使用冒号分隔）
    current_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_path = f"{current_path}:{tb3_description_path}" if current_path else tb3_description_path
    os.environ["GZ_SIM_RESOURCE_PATH"] = new_path
    print("Current GZ_SIM_RESOURCE_PATH:", os.environ["GZ_SIM_RESOURCE_PATH"])
    
    # 路径
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtle_autonomous_patrol = get_package_share_directory('turtle_autonomous_patrol')

    # 启动Gazebo空世界
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'empty_world.launch.py')
        )
    )

    # 加载自定义xacro生成urdf，并发布robot_state_publisher节点
    robot_description_config = os.path.join(
        pkg_turtle_autonomous_patrol,
        'urdf',
        'turtlebot3_burger_auto_patrol.xacro'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', robot_description_config])}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'turtlebot3_burger_auto_petrol',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # 启动激光避障节点
    patrol_laser_avoider = Node(
        package='turtle_autonomous_patrol',
        executable='laser_avoider',
        output='screen',
        parameters=[os.path.join(pkg_turtle_autonomous_patrol, 'config', 'params.yaml')],
    )

    # 启动摄像头警报节点
    patrol_camera_alert = Node(
        package='turtle_autonomous_patrol',
        executable='camera_alert',
        output='screen'
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
                robot_state_publisher_node,
                spawn_entity,
                # patrol_laster_avoider,
                # patrol_camera_alert,
            ]
        ),
        
    ])
