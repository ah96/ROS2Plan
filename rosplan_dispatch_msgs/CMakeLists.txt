cmake_minimum_required(VERSION 3.8)
project(rosplan_dispatch_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionDispatch.msg"
  "msg/ActionFeedback.msg"
  "msg/CompletePlan.msg"
  "msg/EsterelPlan.msg"
  "msg/EsterelPlanEdge.msg"
  "msg/EsterelPlanNode.msg"
  "srv/DispatchService.srv"
  "srv/GetPlanningParams.srv"
  "srv/ParsingService.srv"
  "srv/PlanningService.srv"
  "srv/ProblemService.srv"
  "action/Plan.action"
  "action/NonBlockingDispatch.action"
  DEPENDENCIES builtin_interfaces std_msgs diagnostic_msgs action_msgs
)

ament_package()
