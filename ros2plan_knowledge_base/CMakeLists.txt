cmake_minimum_required(VERSION 3.8)
project(ros2plan_knowledge_base)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2plan_knowledge_msgs REQUIRED)
find_package(ros2plan_dependencies REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(FLEX REQUIRED)

# The knowledge base embeds the VAL1_2-namespace copy of VAL directly,
# rather than linking against val_kcl (which uses the 'VAL' namespace).
set(VAL_SOURCES
    src/VALfiles/DebugWriteController.cpp
    src/VALfiles/pddl+.cpp
    src/VALfiles/ptree.cpp
)

set(KNOWLEDGE_SOURCES
    ${VAL_SOURCES}
    src/KnowledgeBaseFactory.cpp
    src/KnowledgeBase.cpp
    src/KnowledgeComparitor.cpp
    src/PDDLDomainParser.cpp
    src/PDDLProblemParser.cpp
    src/PDDLKnowledgeBase.cpp
    src/HDDLKnowledgeBase.cpp
    src/VALVisitorOperator.cpp
    src/VALVisitorPredicate.cpp
    src/VALVisitorProblem.cpp
)

option(ROS2PLAN_BUILD_RDDL  "Build RDDL knowledge base support"  OFF)
option(ROS2PLAN_BUILD_PPDDL "Build PPDDL knowledge base support" OFF)

if(ROS2PLAN_BUILD_RDDL)
  list(APPEND KNOWLEDGE_SOURCES
    src/RDDLTaskParser.cpp
    src/RDDLKnowledgeBase.cpp
    src/RDDLExprUtils.cpp
    src/RDDLUtils.cpp)
endif()

if(ROS2PLAN_BUILD_PPDDL)
  list(APPEND KNOWLEDGE_SOURCES
    src/PPDDLKnowledgeBase.cpp
    src/PPDDLParser.cpp
    src/PPDDLUtils.cpp)
endif()

add_executable(knowledgeBase ${KNOWLEDGE_SOURCES})

target_include_directories(knowledgeBase PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/VALfiles>
    ${ros2plan_dependencies_INCLUDE_DIRS}
)

target_compile_options(knowledgeBase PRIVATE
    -Wno-deprecated-declarations
    -Wno-register
)

ament_target_dependencies(knowledgeBase
    rclcpp
    std_msgs
    std_srvs
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_ros
    ros2plan_knowledge_msgs
)

# Link against hddl_parser and ippc_server (not val_kcl — we compile VAL inline)
find_library(HDDL_PARSER_LIB hddl_parser
    PATHS "${ros2plan_dependencies_DIR}/../../../lib"
    NO_DEFAULT_PATH)
find_library(IPPC_SERVER_LIB ippc_server
    PATHS "${ros2plan_dependencies_DIR}/../../../lib"
    NO_DEFAULT_PATH)

target_link_libraries(knowledgeBase
    ${HDDL_PARSER_LIB}
    ${IPPC_SERVER_LIB}
    Boost::filesystem
)

if(ROS2PLAN_BUILD_RDDL)
  find_library(RDDL_PARSER_LIB rddl_parser
      PATHS "${ros2plan_dependencies_DIR}/../../../lib"
      NO_DEFAULT_PATH)
  target_link_libraries(knowledgeBase ${RDDL_PARSER_LIB})
  target_compile_definitions(knowledgeBase PRIVATE ROS2PLAN_BUILD_RDDL)
endif()

if(ROS2PLAN_BUILD_PPDDL)
  find_library(PPDDL_PARSER_LIB ppddl_parser
      PATHS "${ros2plan_dependencies_DIR}/../../../lib"
      NO_DEFAULT_PATH)
  target_link_libraries(knowledgeBase ${PPDDL_PARSER_LIB})
  target_compile_definitions(knowledgeBase PRIVATE ROS2PLAN_BUILD_PPDDL)
endif()

install(TARGETS knowledgeBase
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
