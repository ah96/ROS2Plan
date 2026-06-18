cmake_minimum_required(VERSION 3.8)
project(rosplan_action_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/action_interface_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
