# Changelog

All notable changes to ROS2Plan are documented here.

## [2.0.0] — 2026-08-04

Feature parity with ROSPlan verified end to end, and the shipped launch files made to
actually run. All nine packages now carry the same version.

### Fixed

- **Launch topic wiring.** Every include launch file passed ROS1-style node-namespaced topic
  names (`/ros2plan_problem_interface/problem_instance`) while the nodes register relative
  names that resolve to the root namespace (`/problem_instance`). No publisher/subscriber
  pair in `interfaced_planning_system.launch.py` was connected. Both ends now use the
  relative name.
- **Plan dispatcher crash on startup.** `action_timeout_fraction` was declared as `'0'`
  (integer) in `dispatch_interface.launch.py` but as a double by the node, so the dispatcher
  aborted with `InvalidParameterTypeException` before it could run.
- **Sensing interface could not start.** Three separate causes: `main_rate` declared `'10'`
  against a double parameter; `example.py` / `icaps2019_tutorial.py` referenced by the YAML
  `functions:` entry but never installed; and both scripts still being unported ROS 1
  (Python 2 `print` statement, `rospy.get_param`).
- **Node name collisions.** `IncludeLaunchDescription` does not push a scope, so the first
  `node_name` declared leaked into every later include and all four pipeline nodes came up
  as `ros2plan_problem_interface`. Each include is now wrapped in a scoped `GroupAction`.
- **Satellite service nodes inherited the main node's name**, because `launch_ros` passes it
  as a process-wide `-r __node:=` remap. They now set `use_global_arguments(false)` /
  `use_global_arguments=False` (`ProblemInterface`, `RPActionInterface` ×2,
  `ServiceActionInterface`, `RPKnowledgeBaseLink`, sensing interface).
- **`ros2plan_action_interface/config/example_config.yaml` was never installed**, so
  `action_interface_manager` had no config to point `actions_config` at in an installed
  workspace.
- **RDDL was disabled by default.** Both `CMakeLists.txt` claimed the RDDL parser hits an
  internal compiler error on GCC 11 and gated it behind GCC >= 12 / Clang — which disabled
  RDDL on Humble's own platform. GCC 11.4.0 compiles all four RDDL sources cleanly; the
  gate has been removed and `ROS2PLAN_BUILD_RDDL` now defaults to `ON`.

### Changed

- `RDDLExprUtils::getExpression(const Connective*)` folds n-ary connectives
  left-associatively instead of asserting exactly two operands. RDDL connectives are n-ary
  (`a ^ b ^ c`), and the assertion aborted the knowledge base on 6 of 8 IPC benchmark
  domains. Two-operand expressions emit exactly the same tokens as before.
- The sensing interface skips a single predicate whose message or service type is
  unavailable, instead of aborting the whole node. The shipped `example.yaml` uses
  `kobuki_msgs`, which has no ROS 2 release.
- Removed two `rclpy.shutdown()` calls from `load_params()`. They were carried over from
  ROS 1's `signal_shutdown`, but in ROS 2 they invalidate the context immediately, so every
  subsequent service call in the predicate loop raised `RCLError`.
- All nine packages set to version `2.0.0`, maintained by Amar Halilovic. Upstream ROSPlan
  maintainers are retained as `<author>` entries, and package URLs point at this fork.

### Removed

- Nine ROS 1 XML `.launch` files superseded by their `.launch.py` equivalents, and the
  unported ROS 1 `ros2plan_sensing_interface/scripts/sensing_interface.py` (the installed
  entry point is `scripts_ros2/sensing_interface`).

## [1.0.0] — 2026-06-18

Initial release of ROS2Plan: full port of ROSPlan to ROS2 Humble Hawksbill on Ubuntu 22.04.

### Added

- All 9 ROSPlan packages ported and building under `ament_cmake` / `ament_cmake_python`
- Python launch files replacing all ROS1 XML launch files
- Runnable turtlebot example under `docs/examples/turtlebot/`

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
- `ros2plan_knowledge_base/src/VALfiles/FlexLexer.h` kept as pointer-API version to match pre-generated scanner
- GCC 11 `-O1` internal compiler error avoided by removing explicit optimization flag

### Fixed

- KB service namespace: all 19 KB services moved to `~/` prefix
- POPF binary install path: trailing slash added to `install(DIRECTORY common/bin/ ...)`
- `ros2 service call --spin-time`: replaced with `timeout N ros2 service call`

## [Unreleased]

<!-- Add new changes here -->
