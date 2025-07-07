cmake_minimum_required(VERSION 3.8)
project(turtle_commander)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)

# Python 安装
ament_python_install_package(${PROJECT_NAME})

# 安装资源目录
install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_package()
