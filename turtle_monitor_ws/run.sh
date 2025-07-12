git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git src/DynamixelSDK
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git src/turtlebot3_msgs
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git src/turtlebot3
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git src/turtlebot3_simulations

rm -rf build install log
ps -ef|grep ros2 |awk '{print $2}' | xargs kill -SIGINT
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
ros2 run turtle_monitor monitor_node &
ros2 run turtle_monitor max_speed_service &
ros2 run turtle_monitor patrol_server &
ros2 topic echo /alert &

ros2 run turtle_monitor patrol_client
ros2 service call /set_max_speed std_srvs/srv/SetBool "{data: true}"
ros2 param set /max_speed_service max_speed 0.7
ros2 launch turtle_monitor patrol.launch.py

ros2 action send_goal /patrol turtlebot3_msgs/action/Patrol "{goal: {x: 1.0, y: 1.0, z: 0.0}}"
拓展：

    自定义动作文件（如 Patrol.action）

    使用 .srv 文件而不是 SetBool

    将 max_speed 写入 ROS 2 参数服务中使用动态重配置（如 ros2 param set）