cmake_minimum_required(VERSION 3.8)
project(ros2plan_sensing_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts_ros2/sensing_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  example.yaml
  icaps2019_tutorial.yaml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
