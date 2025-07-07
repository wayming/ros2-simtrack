
# mkdir -p src
# colcon build
# source install/setup.bash
# cd src
# ros2 pkg create --build-type ament_python turtle_commander
# ros2 pkg create --build-type ament_cmake turtle_interfaces
rm -rf build install log ament* rosidl* CTest* AMENT* CMakeCache.txt CMakeFiles
source /opt/ros/iron/setup.bash 
colcon build --packages-select turtle_interfaces turtle_commander
source install/setup.bash

ps -ef|grep ros2 | awk '{print $2}' | xargs kill -2

ros2 run turtlesim turtlesim_node &
ros2 run turtle_commander commander_node &
ros2 run turtle_commander reset_service &
ros2 service call /reset_turtle turtle_interfaces/srv/ResetTurtle "{}"

colcon build --packages-select turtle_commander
find src
cat src/turtle_commander/package.xml
cat src/turtle_commander/CMakeLists.txt
cat src/turtle_commander/setup.py