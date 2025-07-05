cmake_minimum_required(VERSION 3.8)
project(turtle_commander)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ResetTurtle.srv"
  DEPENDENCIES std_msgs
)

ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
