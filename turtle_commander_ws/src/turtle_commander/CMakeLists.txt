cmake_minimum_required(VERSION 3.5)
project(turtle_commander)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)  # ADD THIS LINE
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

# Add these debug messages above rosidl_generate_interfaces
message(STATUS "======================== DEBUG START ========================")
message(STATUS "Project name: ${PROJECT_NAME}")
message(STATUS "Current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "Package.xml path: ${CMAKE_CURRENT_SOURCE_DIR}/package.xml")

# Verify package.xml exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
  message(STATUS "package.xml FOUND at ${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" PACKAGE_XML_CONTENT)
  string(FIND "${PACKAGE_XML_CONTENT}" "rosidl_interface_packages" POSITION)
  if(NOT POSITION EQUAL -1)
    message(STATUS "<member_of_group> found in package.xml at position ${POSITION}")
  else()
    message(WARNING "<member_of_group> NOT FOUND in package.xml")
  endif()
else()
  message(FATAL_ERROR "package.xml NOT FOUND at ${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
endif()

message(STATUS "======================== DEBUG END ========================")

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ResetTurtle.srv"
  DEPENDENCIES std_msgs
)

ament_python_install_package(${PROJECT_NAME})

ament_package()