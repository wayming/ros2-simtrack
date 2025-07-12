cmake_minimum_required(VERSION 3.10)
project(turtle_monitor)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

find_package(turtle_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_package()
