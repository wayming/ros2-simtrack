rm -rf build install log
colcon build
source /opt/ros/iron/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger

# ros2 launch gazebo_ros gazebo.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


ros2 run turtle_monitor monitor_node

ros2 run turtle_monitor set_max_speed_service
ros2 run turtle_monitor patrol_server



拓展：

    自定义动作文件（如 Patrol.action）

    使用 .srv 文件而不是 SetBool

    将 max_speed 写入 ROS 2 参数服务中使用动态重配置（如 ros2 param set）