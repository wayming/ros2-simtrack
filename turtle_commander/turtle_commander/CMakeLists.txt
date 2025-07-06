cmake_minimum_required(VERSION 3.8)
project(turtle_commander)

# 查找必要包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 确保包含 Python 生成器
find_package(rosidl_generator_py REQUIRED)

# 生成接口 - 正确写法
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ResetTurtle.srv"
)

# 安装Python包
ament_python_install_package(${PROJECT_NAME})

# 安装接口
install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()