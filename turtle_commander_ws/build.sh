
# mkdir -p src
# colcon build
# source install/setup.bash
# cd src
# ros2 pkg create --build-type ament_python turtle_commander

rm -rf build install log
colcon build --packages-select turtle_commander
source install/setup.bash

ps -ef|grep ros2 | awk '{print $2}' | xargs kill -2

ros2 run turtlesim turtlesim_node &
ros2 run turtle_commander commander_node &
ros2 run turtle_commander reset_service &
