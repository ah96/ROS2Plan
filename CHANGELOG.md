# Changelog

All notable changes to ROS2Plan are documented here.

## [1.0.0] — 2026-06-18

Initial release of ROS2Plan: full port of ROSPlan to ROS2 Humble Hawksbill on Ubuntu 22.04.

### Added

- All 9 ROSPlan packages ported and building under `ament_cmake` / `ament_cmake_python`
- Python launch files replacing all ROS1 XML launch files
- Migration guide: `ros2_migration.md` and `ros2_migration.pdf`
- PDF generation script: `md_to_pdf.py`

### Changed

#### Build system
- `package.xml` converted from format 1 (catkin) to format 3 (ament)
- All `CMakeLists.txt` migrated from catkin to `ament_cmake`; C++ standard set to C++17
- `rosidl_generate_interfaces` replaces `generate_messages`
- `actionlib_msgs` dependency replaced by `action_msgs`
- `hddl_parser` built as a shared library (avoids flex symbol collision)

#### C++ nodes
- All nodes inherit `rclcpp::Node` and run under `MultiThreadedExecutor`
- Two callback groups per node: `MutuallyExclusive` for service servers, `Reentrant` for subscriptions
- Latched publishers converted to `rclcpp::QoS(1).transient_local()`
- Matching `transient_local()` QoS added to all subscribers of latched topics
  (`planner_output_sub_` in `PDDLSimplePlanParser`, `plan_sub_` in `SimplePlanDispatcher`)
- Service callbacks: `bool (Req&, Res&)` → `void (shared_ptr<Req>, shared_ptr<Res>)`
- All KB service servers use `~/service_name` prefix for correct namespace resolution
- `RPActionInterface::setup()` deadlock fixed with satellite node + `rclcpp::spin_until_future_complete`
- Persistent `kb_svc_node_` member in `RPActionInterface` for dispatch-time KB update calls
- `ros::Time` / `ros::Duration` → `rclcpp::Time` / `rclcpp::Duration`
- `ROS_INFO` / `ROS_WARN` → `RCLCPP_INFO` / `RCLCPP_WARN`
- Action servers: `actionlib::SimpleActionServer` → `rclcpp_action::create_server`
- `callServiceSync` template helper in `PlanDispatcher` / `PlanParser` base classes

#### Python nodes
- `rospy` → `rclpy`; all nodes inherit `rclpy.Node`
- `rospy.Rate` + spin loops → `create_timer`
- Parameters loaded with `declare_parameter` + `get_parameter`
- Complex config (nested dicts) passed as YAML file path via `config_path` parameter
- `rospkg.RosPack().get_path()` → `ament_index_python.packages.get_package_share_directory()`
- Blocking service calls use satellite node + `rclpy.spin_until_future_complete`
- Action clients use async future chain with `add_done_callback`
- Python 2 → Python 3: `dict.iteritems()`, `reduce`, `imp.load_source`, `print` statement
- Dynamic action/message import via `importlib.import_module`

#### VAL / FlexLexer
- `rosplan_knowledge_base/src/VALfiles/FlexLexer.h` kept as pointer-API version to match pre-generated scanner
- GCC 11 `-O1` internal compiler error avoided by removing explicit optimization flag

### Fixed

- KB service namespace: all 19 KB services moved to `~/` prefix
- POPF binary install path: trailing slash added to `install(DIRECTORY common/bin/ ...)`
- `ros2 service call --spin-time`: replaced with `timeout N ros2 service call`

## [Unreleased]

<!-- Add new changes here -->
