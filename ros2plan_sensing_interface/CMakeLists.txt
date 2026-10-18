cmake_minimum_required(VERSION 3.10)
project(ros2plan_sensing_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts_ros2/sensing_interface
  DESTINATION lib/${PROJECT_NAME}
)

# The .py files are the sensing scripts each .yaml references through its
# `functions:` entry, so they have to land in the share directory alongside it.
install(FILES
  example.yaml
  example.py
  icaps2019_tutorial.yaml
  icaps2019_tutorial.py
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sensing_interface_test test/test_sensing_interface.py
    TIMEOUT 120)
endif()

ament_package()
