
# mkdir -p src
# colcon build
# source install/setup.bash
# cd src
# ros2 pkg create --build-type ament_python turtle_commander

colcon build
source install/setup.bash
ros2 run turtlesim turtlesim_node &
ros2 run turtle_commander commander_node &
ros2 run turtle_commander reset_service &