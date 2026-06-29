# ROSPlan ROS1 → ROS2 Migration: Change Log

This document records every category of change applied during the port of ROSPlan from ROS1
(catkin, C++14, Ubuntu 20.04) to ROS2 Humble (ament_cmake, C++17, Ubuntu 22.04). It is
intended as reference material for a publication describing the migration effort.

---

## 1. Build System (all packages)

| ROS1 (catkin) | ROS2 (ament_cmake) |
|---|---|
| `package.xml` format 1 | `package.xml` format 3 |
| `<buildtool_depend>catkin</buildtool_depend>` | `<buildtool_depend>ament_cmake</buildtool_depend>` |
| `<build_depend>` + `<run_depend>` | `<depend>` (or `<build_depend>` + `<exec_depend>`) |
| `cmake_minimum_required(VERSION 2.8.3)` | `cmake_minimum_required(VERSION 3.8)` |
| `find_package(catkin REQUIRED COMPONENTS ...)` | `find_package(ament_cmake REQUIRED)` + individual `find_package()` per dep |
| `catkin_package(...)` | `ament_package()` |
| `include_directories(${catkin_INCLUDE_DIRS})` | `ament_target_dependencies(target dep1 dep2 ...)` |
| `target_link_libraries(target ${catkin_LIBRARIES})` | `ament_target_dependencies()` handles this |
| `catkin_install_python(...)` | `install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})` |
| No explicit install needed for headers/targets in catkin devel space | `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})` required |

### Message/service/action generation

| ROS1 | ROS2 |
|---|---|
| `generate_messages(DEPENDENCIES ...)` + `add_dependencies(target _generate_messages_cpp)` | `rosidl_generate_interfaces(${PROJECT_NAME} "msg/Foo.msg" ... DEPENDENCIES ...)` |
| `actionlib_msgs` as action dependency | `action_msgs` (built-in ROS2) |
| Include: `#include "pkg/Foo.h"` | Include: `#include "pkg/msg/foo.hpp"` (snake_case) |
| Namespace: `pkg::Foo` | Namespace: `pkg::msg::Foo` (msg) or `pkg::srv::Foo` (srv) or `pkg::action::Foo` (action) |

### C++ standard
All packages upgraded from C++14 to C++17 (`set(CMAKE_CXX_STANDARD 17)`).

---

## 2. Node Initialization and Lifecycle

| ROS1 | ROS2 |
|---|---|
| `ros::init(argc, argv, "name")` | `rclcpp::init(argc, argv)` |
| `ros::NodeHandle nh("~")` | Inherit from `rclcpp::Node`: `class KB : public rclcpp::Node` |
| `ros::spin()` | `rclcpp::executors::MultiThreadedExecutor exec; exec.add_node(node); exec.spin()` |
| `ros::ok()` | `rclcpp::ok()` |
| `ros::shutdown()` | `rclcpp::shutdown()` |
| Node name: `ros::this_node::getName()` | Node method: `get_name()` |

**Pattern change — node as class:**
In ROS1, `KnowledgeBase` took a `ros::NodeHandle&` in its constructor. In ROS2 it inherits from
`rclcpp::Node`, with the node name passed to the `Node` base constructor. The factory creates a
`shared_ptr<KnowledgeBase>` directly; there is no separate handle object.

**MultiThreadedExecutor:**
`MultiThreadedExecutor` is used in all nodes with service servers to allow concurrent service
handling. A single `MutuallyExclusiveCallbackGroup` is shared by all service servers to prevent
re-entrant calls within the same server, while still allowing concurrent calls across different
servers.

---

## 3. Parameters

| ROS1 | ROS2 |
|---|---|
| `nh.param<double>("name", var, default)` | `declare_parameter("name", default); var = get_parameter("name").as_double()` |
| `ros::param::get("name", var)` | `get_parameter("name", var)` |
| Parameters read at any time | Parameters must be declared before first access |
| `private_nh` for private parameters (`~name`) | Node declares parameters in its own namespace by default |

---

## 4. Service Servers

| ROS1 | ROS2 |
|---|---|
| `nh.advertiseService("name", &Class::method, this)` | `create_service<SrvType>("name", std::bind(&Class::method, this, _1, _2), rmw_qos_profile_services_default, callback_group)` |
| Callback: `bool method(Req&, Res&)` (returns bool) | Callback: `void method(const shared_ptr<Req>, shared_ptr<Res>)` (void) |
| `ros::ServiceEvent<Req,Res>` for caller metadata | No equivalent in ROS2; caller tracking dropped |
| Service advertised in catkin devel namespace | Service name relative to node namespace |

**Callback group:**
A `MutuallyExclusiveCallbackGroup` is created in the node constructor and passed to every
`create_service()` call. Without this, concurrent service calls with the `MultiThreadedExecutor`
could cause data races on shared state.

---

## 5. Publishers and Subscribers

| ROS1 | ROS2 |
|---|---|
| `nh.advertise<T>("topic", queue, latch)` where `latch=true` | `create_publisher<T>("topic", rclcpp::QoS(N).transient_local())` |
| `nh.subscribe("topic", queue, callback)` | `create_subscription<T>("topic", rclcpp::QoS(N), callback)` |
| Latched publisher: third arg `true` | Latched publisher: `QoS.transient_local()` — subscriber must also use `transient_local()` |

---

## 6. Time and Duration

| ROS1 | ROS2 |
|---|---|
| `ros::Time::now()` (static) | `now()` (node method) or `rclcpp::Clock().now()` (static context) |
| `ros::Time t` | `rclcpp::Time t` |
| `ros::Duration(x)` | `rclcpp::Duration::from_seconds(x)` |
| `builtin_interfaces/Time` msg field used as `ros::Time` | Must wrap: `rclcpp::Time(msg.some_time_field)` for arithmetic/comparison |
| `ros::Rate rate(hz)` + `rate.sleep()` in a loop | `create_wall_timer(period, callback)` — timer-based instead of spin loop |

---

## 7. Logging

| ROS1 | ROS2 |
|---|---|
| `ROS_INFO(fmt, ...)` | `RCLCPP_INFO(get_logger(), fmt, ...)` (inside node) |
| `ROS_INFO(fmt, ...)` | `RCLCPP_INFO(rclcpp::get_logger("name"), fmt, ...)` (outside node / static context) |
| `ROS_WARN`, `ROS_ERROR`, `ROS_DEBUG` | `RCLCPP_WARN`, `RCLCPP_ERROR`, `RCLCPP_DEBUG` |
| `ros::this_node::getName()` used as logger name | `get_name()` or `get_logger()` inside the node class |

---

## 8. tf

| ROS1 | ROS2 |
|---|---|
| `#include <tf/...>` | `#include <tf2/...>` or `#include <tf2_ros/...>` |
| `tf::TransformListener` | `tf2_ros::TransformListener` |
| `tf::StampedTransform` | `geometry_msgs::msg::TransformStamped` |

---

## 9. Message Type Includes and Namespaces

**Headers:**
```
ROS1: #include "rosplan_knowledge_msgs/KnowledgeItem.h"
ROS2: #include "rosplan_knowledge_msgs/msg/knowledge_item.hpp"
```
Header names are converted to `snake_case` and suffixed `.hpp` under a `msg/` or `srv/` subdirectory.

**Type namespaces:**
```
ROS1: rosplan_knowledge_msgs::KnowledgeItem
ROS2: rosplan_knowledge_msgs::msg::KnowledgeItem

ROS1: rosplan_knowledge_msgs::GetDomainNameService
ROS2: rosplan_knowledge_msgs::srv::GetDomainNameService

ROS1: rosplan_dispatch_msgs::CompletePlan
ROS2: rosplan_dispatch_msgs::msg::CompletePlan
```

**diagnostic_msgs:**
```
ROS1: diagnostic_msgs::KeyValue
ROS2: diagnostic_msgs::msg::KeyValue
```

---

## 10. VAL Parser Integration (rosplan_knowledge_base-specific)

ROSPlan embeds two copies of the VAL PDDL validation library:
- `rosplan_dependencies/VAL/` — namespace `VAL`, compiled into `libval_kcl.so`
- `rosplan_knowledge_base/src/VALfiles/` — namespace `VAL1_2`, compiled inline

These two copies have incompatible namespaces and cannot be linked together. The knowledge base
compiles its VALfiles directly into the `knowledgeBase` executable rather than linking `val_kcl`.

### FlexLexer.h API version
The pre-generated PDDL flex scanner (`lex.yy.cc`, embedded in `pddl+.cpp`) was built against an
older FlexLexer API that uses `std::istream*` (pointer). The system header on Ubuntu 22.04
(`/usr/include/FlexLexer.h`) has been updated to use `std::istream&` (reference). The solution
is to keep a copy of the old pointer-API `FlexLexer.h` alongside the scanner source
(`src/VALfiles/FlexLexer.h`, copied from `rosplan_dependencies/VAL/include/FlexLexer.h`).

### hddl_parser shared vs. static
The HDDL parser (`rosplan_dependencies`) uses its own flex scanner (`hddl_scanner.lex`). Both
the PDDL scanner and the HDDL scanner define global flex memory functions (`yyalloc`,
`yyrealloc`, `yyfree`). When `libhddl_parser` was a static library (`.a`), these symbols
conflicted at link time. Building it as a **shared library** (`.so`) isolates the HDDL
scanner's symbols and resolves the conflict.

---

## 11. GCC 11 Compiler Issues (Ubuntu 22.04)

### Internal compiler error with -O1
GCC 11 (the default compiler on Ubuntu 22.04) produces an internal compiler error (segmentation
fault) when compiling files that include complex ROS2 template headers (`rcpputils/join.hpp`,
`rclcpp/parameter_client.hpp`) at optimization level `-O1`. Removing the explicit `-O1` flag
(leaving the CMake default, which is no optimization in Debug or `-O2` in Release) avoids the
crash.

### Bison-generated header installation
Bison generates several auxiliary headers alongside the main parser tab file:
`hddl_parser.tab.hpp`, `location.hh`, `position.hh`, `stack.hh`. These must be explicitly
installed alongside `hddl_parser.h` because `hddl_parser.h` `#include`s them. The CMake
install command must list them from `${CMAKE_CURRENT_BINARY_DIR}/hddl_parser/`.

---

## 12. Python (rospy → rclpy)

Applies to `rosplan_action_interface`, `rosplan_sensing_interface`, `rosplan_rqt`.

| ROS1 (rospy) | ROS2 (rclpy) |
|---|---|
| `import rospy` | `import rclpy` |
| `rospy.init_node("name")` | `rclpy.init(); node = Node("name")` |
| `rospy.get_param("name", default)` | `node.declare_parameter("name", default); node.get_parameter("name").value` |
| `rospy.loginfo(msg)` | `node.get_logger().info(msg)` |
| `rospy.ServiceProxy("name", Type)` | `node.create_client(Type, "name")` |
| `rospy.wait_for_service("name")` | `client.wait_for_service()` |
| `rospy.spin()` | `rclpy.spin(node)` |
| `rospkg.RosPack().get_path("pkg")` | `ament_index_python.packages.get_package_share_directory("pkg")` |
| `actionlib.SimpleActionClient(...)` | `rclpy.action.ActionClient(node, ActionType, "name")` |
| `actionlib.SimpleActionServer(...)` | `rclpy.action.ActionServer(node, ActionType, "name", callback)` |

### Node reference in Python

In ROS1, `rospy` functions are global (no explicit node object needed). In ROS2, all ROS
operations require an explicit `Node` instance. The pattern used in `rosplan_action_interface`:
- The top-level manager class inherits `rclpy.Node`
- The node reference (`self`) is passed down to every helper class constructor
- Helper classes store it as `self._node` and use `self._node.get_logger()`, etc.

### Blocking service calls from non-executor threads (Python)

When making service calls from threads that are not part of the rclpy executor (e.g., from a
`threading.Thread` launched by a callback), use a **satellite node** not added to any executor:

```python
self._svc_node = Node('helper_svc_node')  # not added to executor
client = self._svc_node.create_client(SrvType, 'topic')
future = client.call_async(request)
rclpy.spin_until_future_complete(self._svc_node, future, timeout_sec=5.0)
result = future.result()
```

This avoids executor re-entry and deadlocks. Applied in `RPKnowledgeBaseLink` and
`ServiceActionInterface`.

### Action client pattern (Python)

`actionlib.SimpleActionClient.send_goal(goal, done_cb=cb)` → async future chain:

```python
send_future = client.send_goal_async(goal)
send_future.add_done_callback(lambda f: self._goal_response_cb(f, ...))

def _goal_response_cb(self, future, ...):
    goal_handle = future.result()
    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(lambda f: self._result_cb(f.result().status, f.result().result))
```

Goal status: `actionlib.GoalStatus.SUCCEEDED` → `action_msgs.msg.GoalStatus.STATUS_SUCCEEDED`.

### Rate loop → timer (Python)

`rospy.Rate(10)` + `rate.sleep()` in a `while not rospy.is_shutdown()` loop → `create_timer(0.1, callback)` in the node constructor. The timer fires in the executor thread; no explicit loop needed.

### Complex parameter config (Python)

ROS2 parameter values cannot hold nested dicts or lists-of-dicts. Action interface
configurations (which are deeply nested YAML) are loaded from a YAML **file path** passed
as a plain string parameter:

```python
self.declare_parameter('actions_config', '')
config_path = self.get_parameter('actions_config').value
with open(config_path) as f:
    config = yaml.safe_load(f)
self.cfg_actions = config.get('actions', [])
```

Usage: `ros2 run ... --ros-args -p actions_config:=/path/to/config.yaml`

### Dynamic service import (Python)

ROS1: `pkg = importlib.import_module(pkg+".srv"); req = getattr(pkg, name+"Request")()`
ROS2: `pkg = importlib.import_module(pkg+".srv"); req = getattr(pkg, name).Request()`

ROS1: `pkg.msg.FooAction`, `pkg.msg.FooGoal`
ROS2: `pkg.action.Foo` (the action type), `pkg.action.Foo.Goal`

### Python package structure for ament_cmake

For ament_cmake + ament_cmake_python, the Python source must be in a directory matching the
package name (as a proper Python package with `__init__.py`). Entry point scripts go in
`scripts/` and are installed via `install(PROGRAMS ...)`:

```cmake
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/my_node DESTINATION lib/${PROJECT_NAME})
```

---

## 13. Sensing Interface (rosplan_sensing_interface)

The sensing interface subscribes to ROS topics and periodically polls ROS services, then
pushes predicate/function updates into the knowledge base.

### Python 2 → Python 3 changes

| ROS1 (Python 2) | ROS2 (Python 3) |
|---|---|
| `dict.iteritems()` | `dict.items()` |
| `reduce(f, lst)` (builtin) | `from functools import reduce` |
| `e.message` (exception attribute) | `str(e)` |
| `imp.load_source('name', path)` | `importlib.util.spec_from_file_location` + `module_from_spec` + `spec.loader.exec_module` |
| `exec "code"` (statement) | `exec("code")` (function) |

### Package finding

| ROS1 | ROS2 |
|---|---|
| `rospkg.RosPack().get_path("pkg")` | `ament_index_python.packages.get_package_share_directory("pkg")` |

`$(find pkg)/path` patterns in YAML config files are still expanded at runtime using the
`ament_index_python` API with the same regex substitution logic.

### Dynamic message/service import

Replaced exec-based import injection with proper `importlib`:
```python
# was: exec('from pkg.msg import Foo', globals())
pkg = importlib.import_module('pkg.msg')
cls = getattr(pkg, 'Foo')
globals()['Foo'] = cls  # keep for eval()-based callbacks
```

### KB update — update_type field

| ROS1 | ROS2 |
|---|---|
| `kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()` (bytes concat) | `kus.update_type.append(KnowledgeUpdateServiceArray.Request.ADD_KNOWLEDGE)` |

ROS2 maps `int8[]` fields to Python lists rather than raw bytes.

### Subscription with extra arguments

| ROS1 | ROS2 |
|---|---|
| `rospy.Subscriber(topic, Type, cb, extra_arg)` → `cb(msg, extra_arg)` | `create_subscription(Type, topic, lambda msg, a=arg: cb(msg, a), qos)` |

The lambda captures loop variables by value using default argument binding to avoid closure bugs.

### Configuration loading

The `topics`, `services`, and `functions` config (previously nested ROS params) is now loaded
from a YAML file. The node declares a single `config_path` string parameter:
```
ros2 run rosplan_sensing_interface sensing_interface --ros-args -p config_path:=/path/to/config.yaml
```
YAML structure is unchanged from ROS1 (`topics:`, `services:`, `functions:`).

### Service polling thread

`call_services()` runs in a `daemon=True` `threading.Thread`. Service calls from this thread use
`call_async()` + `rclpy.spin_until_future_complete(self._svc_node, ...)` where `_svc_node` is a
satellite `Node` not added to any executor (same pattern as `rosplan_action_interface`).

### Timing comparison

| ROS1 | ROS2 |
|---|---|
| `now = rospy.Time.now()` | `now = self.get_clock().now()` |
| `now - self.last_called_time[i] < rospy.Duration(t)` | `(now - self.last_called_time[i]).nanoseconds / 1e9 < t` |
| `self.last_called_time.append(rospy.Time(0))` | `self.last_called_time.append(rclpy.time.Time())` |

### Node identification in status callback

| ROS1 | ROS2 |
|---|---|
| `rospy.get_name()` | `self.get_name()` (method on Node) |

---

## 14. Launch Files — Pending

| ROS1 (XML) | ROS2 (Python) |
|---|---|
| `<launch>...</launch>` | `def generate_launch_description(): return LaunchDescription([...])` |
| `<node pkg="p" type="exec" name="n">` | `Node(package='p', executable='exec', name='n')` |
| `<param name="x" value="y">` | `parameters=[{'x': 'y'}]` inside `Node()` |
| `<arg name="x" default="y">` | `DeclareLaunchArgument('x', default_value='y')` |
| `$(find pkg)` | `get_package_share_directory('pkg')` |
| `$(arg name)` | `LaunchConfiguration('name')` |
| `<include file="...">` | `IncludeLaunchDescription(PythonLaunchDescriptionSource(...))` |
| `<remap from="a" to="b">` | `remappings=[('a', 'b')]` inside `Node()` |

---

## 14. rqt GUI Plugins (rosplan_rqt)

### ROS2 rqt plugin node access

In ROS1, `rospy` functions are global so widgets can call them directly. In ROS2, each rqt plugin
is given a managed `rclpy.Node` via `context.node` in the `Plugin.__init__` constructor. This node
is passed explicitly to every widget:

```python
class ROSPlanDispatcher(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self._widget = PlanViewWidget(context.node, self)
```

Widget constructors take `node` as first parameter and store it as `self._node`.

### Service calls from rqt widgets

Widgets make service calls from the Qt main thread (timers, button callbacks). Rather than
restructuring to be fully async, a satellite node pattern is used — the same pattern as in
`rosplan_action_interface` and `rosplan_sensing_interface`:

```python
self._svc_node = Node('rosplan_rqt_dispatcher_svc')
client = self._svc_node.create_client(SrvType, 'topic')

def _call_service(self, client, request, timeout_sec=5.0):
    if not client.wait_for_service(timeout_sec=timeout_sec):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self._svc_node, future, timeout_sec=timeout_sec)
    return future.result() if future.done() else None
```

### Package path resolution

| ROS1 | ROS2 |
|---|---|
| `rospkg.RosPack().get_path('pkg')` | `ament_index_python.packages.get_package_share_directory('pkg')` |

All UI file paths use `get_package_share_directory('rosplan_rqt')` to locate `.ui` files installed under `share/rosplan_rqt/resource/`.

### Python package location

ROS1 catkin used `src/rosplan_rqt/` with `catkin_python_setup()` and `package_dir={'': 'src'}`.
ROS2 `ament_python_install_package` requires the Python package at the repo root level:
`rosplan_rqt/rosplan_rqt/` (i.e., `<package_name>/<package_name>/`).

### plugin.xml library path

| ROS1 | ROS2 |
|---|---|
| `<library path="src">` | `<library path="rosplan_rqt">` |

The `type` attribute class paths remain unchanged (`rosplan_rqt.dispatcher.ROSPlanDispatcher` etc.).

### Subscriber unregistration

| ROS1 | ROS2 |
|---|---|
| `self._sub.unregister()` | `self._sub.destroy()` |

Used in `EsterelPlanViewerWidget` when the user clicks "Update subscriber".

### KnowledgeUpdateService request fields

| ROS1 | ROS2 |
|---|---|
| `rospy.ServiceProxy(name, KnowledgeUpdateService)(update_type, knowledge)` positional call | `req = KnowledgeUpdateService.Request(); req.update_type = ...; req.knowledge = ...` |
| `KnowledgeUpdateServiceRequest.ADD_GOAL` | `KnowledgeUpdateService.Request.ADD_GOAL` |

### Python 2 → Python 3 changes (rqt-specific)

- `from string import join, split` → removed; use `' '.join(iterable)` and `str.split()`
- `except SomeException, e:` → `except SomeException as e:`
- `print "text"` → `print('text')`

---

## 15. Launch Files (all packages)

All 9 ROS1 XML launch files were converted to ROS2 Python launch files.

### Conversion table

| ROS1 (XML) | ROS2 (Python) |
|---|---|
| `<arg name="X" default="Y"/>` | `DeclareLaunchArgument('X', default_value='Y')` |
| `<arg name="X"/>` (required) | `DeclareLaunchArgument('X')` with no `default_value` |
| `$(arg X)` | `LaunchConfiguration('X')` |
| `$(find pkg)` | `PathJoinSubstitution([FindPackageShare('pkg'), ...])` |
| `<node pkg="P" type="T" name="N">` | `Node(package='P', executable='T', name='N', output='screen')` |
| `<param name="K" value="V">` inside `<node>` | `parameters=[{'K': LaunchConfiguration('V')}]` in `Node()` |
| `<include file="...">` | `IncludeLaunchDescription(PythonLaunchDescriptionSource(...))` |
| Pass args to include: `<arg name="K" value="V"/>` | `launch_arguments={'K': LaunchConfiguration('V')}.items()` in `IncludeLaunchDescription` |

### Dynamic node name from argument

`name="rosplan_interface_$(arg pddl_action_name)"` → pass a list to `name=`:

```python
Node(
    name=['rosplan_interface_', LaunchConfiguration('pddl_action_name')],
    ...
)
```

### Dynamic executable from argument

`type="$(arg planner_interface)"` → `executable=LaunchConfiguration('planner_interface')`

### Path with `$(find pkg)` in default values

Default values that embed `$(find pkg)` use `PathJoinSubstitution` with `FindPackageShare`:

```python
DeclareLaunchArgument(
    'planner_command',
    default_value=[
        'timeout 10 ',
        PathJoinSubstitution([FindPackageShare('rosplan_planning_system'), 'common', 'bin', 'popf']),
        ' DOMAIN PROBLEM',
    ]
)
```

### `$(find pkg)` path resolution: share directory

In ROS1, `$(find pkg)` resolves to the package root (source tree), which may contain `common/`, `test/`, etc.
In ROS2, `FindPackageShare('pkg')` resolves to `share/pkg/` in the install tree. All data directories
(`common/bin/`, `test/pddl/`, `launch/`) must be installed to `share/` via `install(DIRECTORY ...)` in CMakeLists.

```cmake
install(DIRECTORY common/bin   DESTINATION share/${PROJECT_NAME}/common/bin)
install(DIRECTORY test/        DESTINATION share/${PROJECT_NAME}/test)
install(DIRECTORY launch/      DESTINATION share/${PROJECT_NAME}/launch)
```

### `<rosparam command="load">` → config file path parameter

The sensing interface launch used `<rosparam command="load" file="..."/>` to inject a whole YAML into the parameter server. In ROS2 there is no equivalent; instead the sensing interface accepts a `config_path` string parameter and loads the YAML file itself:

```python
Node(
    parameters=[{
        'config_path': PathJoinSubstitution([FindPackageShare('rosplan_sensing_interface'), 'example.yaml']),
    }]
)
```

---

## 16. Action Servers (rosplan_planning_system)

### Action server pattern

| ROS1 | ROS2 |
|---|---|
| `#include <actionlib/server/simple_action_server.h>` | `#include <rclcpp_action/rclcpp_action.hpp>` |
| `actionlib::SimpleActionServer<Action>` | `rclcpp_action::Server<Action>::SharedPtr` |
| Constructor: `SimpleActionServer(nh, "name", callback)` | `rclcpp_action::create_server<Action>(node, "name", goal_cb, cancel_cb, accepted_cb)` |
| Single `execute` callback | Three separate callbacks: `handle_goal`, `handle_cancel`, `handle_accepted` |
| `server_.setSucceeded(result)` | `goal_handle->succeed(result)` |
| `server_.setAborted(result)` | `goal_handle->abort(result)` |
| `server_.publishFeedback(feedback)` | `goal_handle->publish_feedback(feedback)` |
| `actionlib::SimpleActionClient<Action>` | `rclcpp_action::Client<Action>::SharedPtr` |
| `client.sendGoal(goal)` | `client->async_send_goal(goal, options)` |

**Accepted callback launches a detached thread:**
The `handle_accepted` callback must not block — it launches the plan execution in a detached thread:
```cpp
[this](std::shared_ptr<GoalHandleAction> gh) {
    std::thread([this, gh]() { executePlan(gh); }).detach();
}
```

### Blocking service calls from dispatch/parse threads

Service calls made from dispatch or plan-parsing threads (not executor threads) use a simple
future-wait pattern without a satellite node, since those threads do not hold an executor lock:

```cpp
template<typename ClientT, typename ReqT, typename ResT>
bool callServiceSync(std::shared_ptr<ClientT>& client,
                     std::shared_ptr<ReqT> req,
                     std::shared_ptr<ResT>& res)
{
    if (!client->wait_for_service(std::chrono::seconds(5))) return false;
    auto future = client->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) return false;
    res = future.get();
    return true;
}
```

This template is declared inline in the protected section of `PlanDispatcher` and `PlanParser`
base classes so all subclasses inherit it.

### Factory method: enum → string parameter

`ProblemGeneratorFactory::createProblemGenerator` originally took a `ProbGen` enum. In ROS2 the
parameter was changed to `const std::string&` (accepting `"pddl"`, `"rddl"`, `"chimp"`) to
remove the need for callers to include the factory header just to access the enum value.

### Inline helper methods in plan-parser headers

The `PDDLEsterelPlanParser` declares three short helper methods — `groundFormula`,
`satisfiesPrecondition`, and `domainFormulaMatches` — that were inline in the original ROS1
header. They access class member variables and must remain inline in the ROS2 header; they
cannot be moved to a `.cpp` file.

### Constructor error-exit guard in main()

Nodes that call `rclcpp::shutdown()` from their constructor (on configuration error) require a
guard in `main()` before `exec.add_node()`:

```cpp
auto node = std::make_shared<NodeType>();
if (!rclcpp::ok()) return 1;  // constructor called shutdown on error
rclcpp::executors::MultiThreadedExecutor exec;
exec.add_node(node);
exec.spin();
```

Without this guard, `exec.add_node()` throws `RCLError: the given context is not valid`.
Applied to: `ProblemInterface`, `KnowledgeBase`.

---

## 11. Integration Testing (Phase 11)

End-to-end testing of the full pipeline — KB → problem generation → POPF → parsing → dispatch —
revealed two additional migration issues.

### Knowledge base service namespace

ROS1 services named `"domain/name"` in the KB were reachable from planning nodes because ROS1
registers services globally using a full path derived from the node's namespace.  In ROS2 the same
relative name `"domain/name"` resolves to the path `/domain/name` (root namespace), while the
planning nodes construct client paths as `knowledge_base_node_name + "/domain/name"` =
`/rosplan_knowledge_base/domain/name`.  The mismatch caused every KB service call from the
problem interface, parsers, and dispatchers to time out.

**Fix:** All KB service names prefixed with `~/`:

```cpp
// Before
create_service<T>("domain/name", ...);

// After  
create_service<T>("~/domain/name", ...);
```

The `~/` prefix expands to `/<namespace>/<node_name>/` at runtime.  With node name
`rosplan_knowledge_base` and empty namespace the service path becomes
`/rosplan_knowledge_base/domain/name`, which matches the planning nodes' expectations.

Applied to all 19 KB service and publisher registrations in `KnowledgeBase.cpp`.

### CMake `install(DIRECTORY)` trailing-slash semantics

The planning system CMakeLists.txt installed the POPF binary with:

```cmake
install(DIRECTORY common/bin
  DESTINATION share/${PROJECT_NAME}/common/bin/
  USE_SOURCE_PERMISSIONS)
```

Without a trailing slash on the source directory, CMake copies the `bin/` directory *itself* into
the destination, producing the path `share/rosplan_planning_system/common/bin/bin/popf` (one extra
`bin/` level).  The planner node looked for the binary one level up and reported "No such file or
directory".

**Fix:** Add trailing slash to the source path so CMake copies the *contents* of `common/bin/`
rather than the directory itself:

```cmake
install(DIRECTORY common/bin/
  DESTINATION share/${PROJECT_NAME}/common/bin
  USE_SOURCE_PERMISSIONS)
```

### `ros2 service call` has no `--spin-time` flag in Humble

The ROS1 → ROS2 `rosservice call` migration dropped the blocking-with-timeout option.
`ros2 service call` in Humble only supports `-r/--rate`.  Integration test scripts that used
`ros2 service call --spin-time N ...` failed immediately with exit code 2 (unknown option).

**Fix:** Wrap calls with the POSIX `timeout` utility:

```bash
timeout 10 ros2 service call /service_name pkg/srv/Type "{}"
```

### Plan output filename

POPF's output is redirected to `<data_path>/plan.pddl` by the planner interface.  Test scripts
checking for `*.pddl.soln` (a convention from a different planner) found no file.  The check was
updated to look for `plan.pddl` and verify it contains the `;;;; Solution Found` marker.

---

## 12. Realistic End-to-End Test (Turtlebot Domain)

A full pipeline test was run with the turtlebot domain: 5 waypoints, robot kenny, goals to visit
all waypoints and return docked.  POPF produced a 12-action plan (undock → localise → 10 ×
goto_waypoint → dock); all 12 actions dispatched and executed, KB updated with all `visited` facts
and `docked` restored.  Several additional ROS2 async/QoS bugs were exposed and fixed.

### `RPActionInterface::setup()` future deadlock

**Root cause:** `setup()` is called from `main()` before `exec.spin()` starts.  It called
`async_send_request()` then `opFuture.wait_for(10s)` to fetch operator details from the KB.
Without a spinning executor, the response callback can never fire, so `wait_for` always times out.

**Fix (`rosplan_planning_system/src/ActionInterface/RPActionInterface.cpp`):**

```cpp
// Before (deadlock — wait_for without spin):
auto op_client = create_client<GetDomainOperatorDetailsService>(svc);
auto opFuture  = op_client->async_send_request(opReq);
if (opFuture.wait_for(std::chrono::seconds(10)) != std::future_status::ready) { ... }

// After (satellite node spins while waiting):
auto svc_node  = std::make_shared<rclcpp::Node>("rp_action_setup_svc_node");
auto op_client = svc_node->create_client<GetDomainOperatorDetailsService>(svc);
auto opFuture  = op_client->async_send_request(opReq);
if (rclcpp::spin_until_future_complete(svc_node, opFuture, std::chrono::seconds(10))
        != rclcpp::FutureReturnCode::SUCCESS) { ... }
```

The same pattern was applied to the predicate-details fetch and to the at-start/at-end KB update
calls in `dispatchCallback()`.  A persistent `kb_svc_node_` member (declared in the header) owns
the `update_knowledge_client_` so the satellite node stays alive for the lifetime of the action
interface node.

### Transient-local QoS mismatch on inter-node topics

**Root cause:** ROS2/DDS transient-local (latched) delivery requires BOTH publisher AND subscriber
to declare `rclcpp::QoS(n).transient_local()`.  A volatile subscriber connected to a
transient-local publisher does NOT receive the stored last message — it only receives messages
published while it is already subscribed.

Two subscribers had this bug:

| Subscriber | Topic | Publisher QoS | Subscriber QoS (before fix) |
|---|---|---|---|
| `PDDLSimplePlanParser` | `planner_output` | transient_local | default (volatile) |
| `SimplePlanDispatcher` | `complete_plan` | transient_local | default (volatile) |

Because components were started sequentially (not via a launch file), both subscribers connected
after the publisher had already published, and missed the stored message.

**Fix:** Add `.transient_local()` to both subscriptions:

```cpp
// PDDLSimplePlanParser.cpp
planner_output_sub_ = create_subscription<std_msgs::msg::String>(
    planner_topic, rclcpp::QoS(1).transient_local(), ...);

// SimplePlanDispatcher.cpp
plan_sub_ = create_subscription<rosplan_dispatch_msgs::msg::CompletePlan>(
    plan_topic, rclcpp::QoS(1).transient_local(), ...);
```

### Relative topic names and node-namespace resolution

The planner interface publishes on topic parameter `planner_output` (relative) which resolves to
`/planner_output` (root namespace, since no explicit node namespace is set).  A test that
subscribed to `/rosplan_planner_interface/planner_output` (incorrectly using the node name as a
namespace prefix) never matched.  The correct absolute path is `/planner_output`.

The same issue affected the `complete_plan` topic between the parser and dispatcher.

**Rule:** In ROS2, a relative topic name `foo` resolves to `/<node_namespace>/foo`.  With the
default namespace `/`, that is `/foo` — NOT `/<node_name>/foo`.  To get a node-namespaced
topic, use `~/foo` (tilde prefix), which expands to `/<node_namespace>/<node_name>/foo`.

### `DispatchService` vs `std_srvs/Empty`

`/dispatch_plan` is of type `rosplan_dispatch_msgs/srv/DispatchService` (empty request; response
has `bool success` and `bool goal_achieved`), not `std_srvs/srv/Empty`.  Calling with the wrong
type causes the CLI to wait indefinitely for a matching service that never appears.

```bash
# Wrong — waits forever, service type doesn't match:
ros2 service call /dispatch_plan std_srvs/srv/Empty "{}"

# Correct:
ros2 service call /dispatch_plan rosplan_dispatch_msgs/srv/DispatchService "{}"
```

---

## 19. Rebranding: `rosplan` → `ros2plan` (ROS2Plan repository)

After the functional migration was validated, the official ROS2 fork was rebranded from
`rosplan` to `ros2plan` in the standalone `~/git/ROS2Plan` repository, to make it internally
consistent and clearly distinct from the original ROSPlan. The change was applied as a
case-split rename so that attribution to the upstream project is preserved.

| Item | Before | After |
|---|---|---|
| Package directories & names | `rosplan_*` | `ros2plan_*` |
| `CMake project()` / `package.xml <name>` | `rosplan_*` | `ros2plan_*` |
| C++ namespace | `KCL_rosplan` | `KCL_ros2plan` |
| Includes / Python imports | `rosplan_*/...` | `ros2plan_*/...` |
| Message / service / action packages | `rosplan_*_msgs` | `ros2plan_*_msgs` (interface type strings change) |
| Runtime node names | `rosplan_knowledge_base`, … | `ros2plan_knowledge_base`, … |
| Service / topic namespaces | `/rosplan_*` | `/ros2plan_*` |
| rqt plugin classes & files | `ROSPlan*` | `ROS2Plan*` |
| Sensing class | `RosplanSensing` | `Ros2planSensing` |

**Technique (two passes):**
- **Pass A** — lowercase `rosplan` → `ros2plan` across all text files. The lowercase token only
  ever appears as a package / namespace / path / identifier / service string, never in
  attribution (which uses PascalCase `ROSPlan`), so it is safe to apply globally.
- **Pass B** — PascalCase `ROSPlan` → `ROS2Plan` and `Rosplan` → `Ros2plan`, scoped to
  `ros2plan_rqt/` and `ros2plan_sensing_interface/` only, and skipping any line containing the
  upstream URL `KCL-Planning/ROSPlan`.
- **Pass C** — the remaining cosmetic PascalCase `ROSPlan` mentions in *our* code rebranded to
  `ROS2Plan`: every `package.xml` `<description>`/`<maintainer>` (the `<url>` line guarded),
  the CHIMP planner-interface log strings (`"ROSPlan: (%s)"`), the RDDL generated-problem
  banner, code/test header comments, the `KnowledgeItem.msg` comment, and the rqt-menu
  references in `README.md`/`TUTORIAL.md`.

**Preserved (deliberately not renamed):** the upstream `<url>https://github.com/KCL-Planning/ROSPlan</url>`
tags, the LICENSE/README/CHANGELOG attribution lines, the migration guide
(`ros2_migration.md`, `md_to_pdf.py`), and third-party credits/identifiers in the CHIMP,
VAL, PROST/rddl_parser, ippc_server, and ppddl_parser dependency code.

**Validation:** clean `colcon build` of all 9 `ros2plan_*` packages; `ros2 interface list` shows
43 `ros2plan_*` types and zero `rosplan_*`; the `ros2plan_knowledge_base` node advertises its 25
`/ros2plan_knowledge_base/*` services; the **full turtlebot pipeline dispatches end to end** —
KB → state load → problem generation → POPF → parse → dispatch of an 11-action plan
(undock, localise, 8× goto_waypoint, dock), ending with `visited wp0..wp4` and `docked kenny`
in the knowledge base and `DispatchService` returning `goal_achieved=True`.

**Example scripts fixed while validating** (`docs/examples/turtlebot/`):
- `load_state.py` previously added the `visited` goals as *initial facts* too, leaving the goal
  already satisfied (empty plan). Now it seeds only `visited wp0` (the robot's start waypoint,
  which cannot be reached by `goto_waypoint`) as a fact; the rest are goals via `set_goals()`.
- `run_demo.sh` was rewritten to launch each node with `ros2 run` and explicit parameters,
  wired through the root-namespace topics the nodes actually use. Two pre-existing traps were
  worked around: (1) the planner must be triggered via the **`/planning_server` Empty service**
  (which reads node parameters) rather than the `/start_planning` **action** (which reads
  `planner_command`/paths from the action *goal*, empty here → empty command → no plan); and
  (2) `interfaced_planning_system.launch.py` passes `planner_command` as a substitution list
  that collapses to empty through the nested include — so `run_demo.sh` sets it directly.
