cmake_minimum_required(VERSION 3.8)
project(rosplan_knowledge_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DomainAssignment.msg"
  "msg/DomainFormula.msg"
  "msg/DomainInequality.msg"
  "msg/DomainOperator.msg"
  "msg/ExprBase.msg"
  "msg/ExprComposite.msg"
  "msg/KnowledgeItem.msg"
  "msg/ProbabilisticEffect.msg"
  "msg/StatusUpdate.msg"
  "srv/GenerateProblemService.srv"
  "srv/GetAttributeService.srv"
  "srv/GetDomainAttributeService.srv"
  "srv/GetDomainNameService.srv"
  "srv/GetDomainOperatorDetailsService.srv"
  "srv/GetDomainOperatorService.srv"
  "srv/GetDomainPredicateDetailsService.srv"
  "srv/GetDomainTypeService.srv"
  "srv/GetEnumerableTypeService.srv"
  "srv/GetInstanceService.srv"
  "srv/GetMetricService.srv"
  "srv/GetRDDLFluentType.srv"
  "srv/GetRDDLImmediateReward.srv"
  "srv/GetRDDLParams.srv"
  "srv/KnowledgeQueryService.srv"
  "srv/KnowledgeUpdateService.srv"
  "srv/KnowledgeUpdateServiceArray.srv"
  "srv/ReloadRDDLDomainProblem.srv"
  "srv/SetFloat.srv"
  "srv/SetInt.srv"
  "srv/SetNamedBool.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs diagnostic_msgs
)

ament_package()
