cmake_minimum_required(VERSION 3.5)
project(rosplan_planning_system)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosplan_dispatch_msgs REQUIRED)
find_package(rosplan_knowledge_msgs REQUIRED)
find_package(rosplan_dependencies REQUIRED)

set(DEPS
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  diagnostic_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rosplan_dependencies
)

include_directories(include)

###########
## Build ##
###########

## rosplan_action_interface shared library
add_library(rosplan_action_interface
  src/ActionInterface/RPActionInterface.cpp
)
ament_target_dependencies(rosplan_action_interface ${DEPS})
target_include_directories(rosplan_action_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

## problemInterface executable
add_executable(problemInterface
  src/ProblemGeneration/ProblemInterface.cpp
  src/ProblemGeneration/PDDLProblemGenerator.cpp
  src/ProblemGeneration/ProblemGenerator.cpp
  src/ProblemGeneration/RDDLProblemGenerator.cpp
  src/ProblemGeneration/CHIMPFluent.cpp
  src/ProblemGeneration/CHIMPProblem.cpp
  src/ProblemGeneration/CHIMPProblemGenerator.cpp
  src/ProblemGeneration/ProblemGeneratorFactory.cpp
)
ament_target_dependencies(problemInterface ${DEPS})

## Planner interface executables
add_executable(popf_planner_interface
  src/PlannerInterface/POPFPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(popf_planner_interface ${DEPS})

add_executable(panda_planner_interface
  src/PlannerInterface/PANDAPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(panda_planner_interface ${DEPS})

add_executable(ff_planner_interface
  src/PlannerInterface/FFPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(ff_planner_interface ${DEPS})

add_executable(metricff_planner_interface
  src/PlannerInterface/FFPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(metricff_planner_interface ${DEPS})

add_executable(rddlsim_planner_interface
  src/PlannerInterface/RDDLSIMPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(rddlsim_planner_interface ${DEPS})

add_executable(smt_planner_interface
  src/PlannerInterface/SMTPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(smt_planner_interface ${DEPS})

add_executable(fd_planner_interface
  src/PlannerInterface/FDPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(fd_planner_interface ${DEPS})

add_executable(tfd_planner_interface
  src/PlannerInterface/TFDPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(tfd_planner_interface ${DEPS})

add_executable(lpg_planner_interface
  src/PlannerInterface/LPGPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(lpg_planner_interface ${DEPS})

add_executable(upm_planner_interface
  src/PlannerInterface/UPMPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(upm_planner_interface ${DEPS})

add_executable(pprp_planner_interface
  src/PlannerInterface/PPRPPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(pprp_planner_interface ${DEPS})

add_executable(chimp_planner_interface
  src/PlannerInterface/CHIMPPlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(chimp_planner_interface ${DEPS})

add_executable(online_planner_interface
  src/PlannerInterface/OnlinePlannerInterface.cpp
  src/PlannerInterface/PlannerInterface.cpp
)
ament_target_dependencies(online_planner_interface ${DEPS})

## Plan parsers
add_executable(pddl_simple_plan_parser
  src/PlanParsing/PDDLSimplePlanParser.cpp
  src/PlanParsing/PlanParser.cpp
)
ament_target_dependencies(pddl_simple_plan_parser ${DEPS})

add_executable(pddl_esterel_plan_parser
  src/PlanParsing/PDDLEsterelPlanParser.cpp
  src/PlanParsing/PlanParser.cpp
)
ament_target_dependencies(pddl_esterel_plan_parser ${DEPS})

## Plan dispatchers
add_executable(pddl_simple_plan_dispatcher
  src/PlanDispatch/SimplePlanDispatcher.cpp
  src/PlanDispatch/PlanDispatcher.cpp
)
ament_target_dependencies(pddl_simple_plan_dispatcher ${DEPS})

add_executable(pddl_esterel_plan_dispatcher
  src/PlanDispatch/EsterelPlanDispatcher.cpp
  src/PlanDispatch/PlanDispatcher.cpp
)
ament_target_dependencies(pddl_esterel_plan_dispatcher ${DEPS})

add_executable(online_plan_dispatcher
  src/PlanDispatch/OnlinePlanDispatcher.cpp
  src/PlanDispatch/PlanDispatcher.cpp
)
ament_target_dependencies(online_plan_dispatcher ${DEPS})

## Simulated action
add_executable(simulatedAction
  src/ActionInterface/RPSimulatedActionInterface.cpp
  src/ActionInterface/RPActionInterface.cpp
)
ament_target_dependencies(simulatedAction ${DEPS})

#############
## Install ##
#############

install(TARGETS
  rosplan_action_interface
  EXPORT rosplan_action_interfaceTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  problemInterface
  popf_planner_interface
  panda_planner_interface
  ff_planner_interface
  metricff_planner_interface
  rddlsim_planner_interface
  smt_planner_interface
  fd_planner_interface
  tfd_planner_interface
  lpg_planner_interface
  upm_planner_interface
  pprp_planner_interface
  chimp_planner_interface
  online_planner_interface
  pddl_simple_plan_parser
  pddl_esterel_plan_parser
  pddl_simple_plan_dispatcher
  pddl_esterel_plan_dispatcher
  online_plan_dispatcher
  simulatedAction
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY common/bin/
  DESTINATION share/${PROJECT_NAME}/common/bin
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY test/
  DESTINATION share/${PROJECT_NAME}/test
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(rosplan_action_interfaceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS})

ament_package()
