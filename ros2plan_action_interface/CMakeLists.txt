cmake_minimum_required(VERSION 3.10)
project(ros2plan_action_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/action_interface_manager
  DESTINATION lib/${PROJECT_NAME}
)

# The manager takes its action definitions from a YAML config; the example one has to
# be installed for `actions_config` to be able to point at it in an installed workspace.
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(action_interface_test test/test_action_interface.py
    TIMEOUT 120)
endif()

ament_package()
