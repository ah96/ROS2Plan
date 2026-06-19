# ROS2Plan Tutorial

This tutorial walks you through using ROS2Plan to plan and execute tasks on a robot.
It covers the core pipeline from scratch, then shows how to write your own action
interfaces for a custom domain.

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Installation](#2-installation)
3. [Core Concepts](#3-core-concepts)
4. [Example: Turtlebot Waypoint Navigation](#4-example-turtlebot-waypoint-navigation)
   - 4.1 [The PDDL Domain](#41-the-pddl-domain)
   - 4.2 [Step 1 — Start the Knowledge Base](#42-step-1--start-the-knowledge-base)
   - 4.3 [Step 2 — Load the Initial State](#43-step-2--load-the-initial-state)
   - 4.4 [Step 3 — Generate the Problem File](#44-step-3--generate-the-problem-file)
   - 4.5 [Step 4 — Run the Planner](#45-step-4--run-the-planner)
   - 4.6 [Step 5 — Parse the Plan](#46-step-5--parse-the-plan)
   - 4.7 [Step 6 — Start Action Interfaces](#47-step-6--start-action-interfaces)
   - 4.8 [Step 7 — Dispatch the Plan](#48-step-7--dispatch-the-plan)
   - 4.9 [Running the Full Demo at Once](#49-running-the-full-demo-at-once)
5. [Writing a Custom Action Interface (Python)](#5-writing-a-custom-action-interface-python)
   - 5.1 [Service-based interface](#51-service-based-interface)
   - 5.2 [Action-based interface (actionlib style)](#52-action-based-interface-actionlib-style)
   - 5.3 [The action config YAML](#53-the-action-config-yaml)
6. [Writing a Custom Action Interface (C++)](#6-writing-a-custom-action-interface-c)
7. [The Sensing Interface](#7-the-sensing-interface)
8. [Inspecting the Knowledge Base](#8-inspecting-the-knowledge-base)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Build tool | colcon |
| C++ standard | 17 |
| Python | 3.10+ |

Install ROS 2 Humble by following the [official guide](https://docs.ros.org/en/humble/Installation.html).

---

## 2. Installation

```bash
# Create a colcon workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone ROS2Plan
git clone https://github.com/your-org/ROS2Plan.git

# Install ROS dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build (disable optional parsers if you don't need them)
colcon build --cmake-args -DROSPLAN_BUILD_RDDL=OFF -DROSPLAN_BUILD_PPDDL=OFF

# Source the workspace
source install/setup.bash
```

Add the source command to your shell profile so every terminal has it:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 3. Core Concepts

ROS2Plan has three main subsystems that work together in a pipeline:

```
Knowledge Base (KB)
    stores the domain model (types, predicates, operators)
    and the current problem state (instances, facts, goals)
    exposes everything as ROS 2 services

Planning System
    ├── Problem Interface  — queries KB, writes a PDDL problem file
    ├── Planner Interface  — calls an external planner (POPF, CHIMP, …)
    ├── Plan Parser        — converts the plan text into ActionDispatch messages
    └── Plan Dispatcher    — sends each action to the right action interface node

Action Interface nodes
    one per PDDL operator
    receives ActionDispatch, executes the real action, sends ActionFeedback
    updates KB on completion (at-start / at-end effects)

Sensing Interface (optional)
    periodically updates KB predicates from ROS 2 topic subscriptions
```

**Typical planning cycle:**

1. Load domain and initial state into the KB.
2. Call the Problem Interface service → PDDL problem file is generated.
3. Call the Planner Interface action → plan is published on a topic.
4. Call the Plan Parser service → plan becomes a sequence of `ActionDispatch` messages.
5. Call the Dispatcher service → actions are dispatched one by one to action nodes.
6. Each action node executes and reports success/failure via `ActionFeedback`.

---

## 4. Example: Turtlebot Waypoint Navigation

This example plans and executes a mission for a mobile robot (`kenny`) that must
visit five waypoints and return to its docking station.
All action nodes are *simulated* — no real robot is required.

The example files are in [docs/examples/turtlebot/](docs/examples/turtlebot/).

### 4.1 The PDDL Domain

The domain ([docs/examples/turtlebot/domain.pddl](docs/examples/turtlebot/domain.pddl))
defines four operators:

| Operator | Parameters | Effect |
|---|---|---|
| `undock` | `?v - robot, ?wp - waypoint` | robot becomes `undocked` |
| `localise` | `?v - robot` | robot becomes `localised` |
| `goto_waypoint` | `?v - robot, ?from ?to - waypoint` | `visited(?to)`, `robot_at(?v,?to)` |
| `dock` | `?v - robot, ?wp - waypoint` | robot becomes `docked` |

```pddl
(:durative-action goto_waypoint
    :parameters (?v - robot ?from ?to - waypoint)
    :duration ( = ?duration 1)
    :condition (and
        (at start (robot_at ?v ?from))
        (at start (localised ?v))
        (over all (undocked ?v)))
    :effect (and
        (at end (visited ?to))
        (at end (robot_at ?v ?to))
        (at start (not (robot_at ?v ?from))))
)
```

The **goal** is for all five waypoints to be visited and the robot to be docked.

---

### 4.2 Step 1 — Start the Knowledge Base

Open a terminal and launch the KB with the turtlebot domain:

```bash
# Terminal 1
DOMAIN=$(ros2 pkg prefix rosplan_planning_system)/share/rosplan_planning_system/test/pddl/turtlebot/domain.pddl

ros2 launch rosplan_knowledge_base rosplan_knowledge_base.launch.py \
    domain_path:="${DOMAIN}"
```

You should see output like:

```
[knowledgeBase]: Loading domain from: .../turtlebot/domain.pddl
[knowledgeBase]: Ready
```

Verify that the KB is advertising its services:

```bash
ros2 service list | grep rosplan_knowledge_base
# Should list ~19 services including:
# /rosplan_knowledge_base/domain/name
# /rosplan_knowledge_base/domain/predicates
# /rosplan_knowledge_base/update_array
# ...
```

---

### 4.3 Step 2 — Load the Initial State

The initial state consists of:
- 5 waypoint instances (`wp0` … `wp4`)
- 1 robot instance (`kenny`)
- Facts: `robot_at(kenny, wp0)`, `docked(kenny)`, `dock_at(wp0)`
- Goals: `visited(wp0)` … `visited(wp4)`, `docked(kenny)`

Use the provided loader script:

```bash
# Terminal 2
source ~/ros2_ws/install/setup.bash
python3 docs/examples/turtlebot/load_state.py --waypoints 5 --robot kenny
```

Expected output:

```
[turtlebot_state_loader]: Loaded 15 knowledge items (5 waypoints, robot: kenny)
[turtlebot_state_loader]: Set 6 goals
```

You can verify the state was loaded:

```bash
ros2 service call /rosplan_knowledge_base/state/instances \
    rosplan_knowledge_msgs/srv/GetInstanceService \
    "{type_name: 'waypoint'}"
```

---

### 4.4 Step 3 — Generate the Problem File

Start the Problem Interface node:

```bash
# Terminal 3
ros2 run rosplan_planning_system problemInterface
```

Then trigger problem generation:

```bash
timeout 15 ros2 service call \
    /rosplan_problem_interface/problem_generation_server \
    std_srvs/srv/Empty "{}"
```

The generated PDDL problem is published on the `/rosplan_problem_interface/problem`
topic (as a `std_msgs/String`) and also written to a temporary file.
You can inspect it:

```bash
ros2 topic echo /rosplan_problem_interface/problem --once
```

The problem should look like:

```pddl
(define (problem task)
(:domain turtlebot)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
    kenny - robot
)
(:init
    (robot_at kenny wp0)
    (docked kenny)
    (dock_at wp0)
)
(:goal (and
    (visited wp0)
    (visited wp1)
    ...
    (docked kenny)
)))
```

---

### 4.5 Step 4 — Run the Planner

Start the POPF planner interface:

```bash
# Terminal 4
DOMAIN=$(ros2 pkg prefix rosplan_planning_system)/share/rosplan_planning_system/test/pddl/turtlebot/domain.pddl
POPF=$(ros2 pkg prefix rosplan_planning_system)/share/rosplan_planning_system/common/bin/popf

ros2 run rosplan_planning_system popfPlannerInterface \
    --ros-args \
    -p domain_path:="${DOMAIN}" \
    -p planner_command:="timeout 30 ${POPF} DOMAIN PROBLEM"
```

Trigger planning with a ROS 2 action call:

```bash
ros2 action send_goal /rosplan_planner_interface/start_planning \
    rosplan_dispatch_msgs/action/Plan "{}"
```

POPF will print the plan to stdout and the planner interface will publish the plan text
on the `planner_output` topic. For the turtlebot domain the plan takes under 0.01 seconds
and contains 12 actions:

```
0.000: (undock kenny wp0)              [2.000]
2.001: (localise kenny)               [1.000]
3.002: (goto_waypoint kenny wp0 wp0)  [1.000]
4.003: (goto_waypoint kenny wp0 wp1)  [1.000]
...
12.011: (dock kenny wp0)              [3.000]
```

---

### 4.6 Step 5 — Parse the Plan

Start the plan parser. It subscribes to `planner_output` with
`transient_local` QoS so it receives the plan even if it starts after
the planner has already published:

```bash
# Terminal 5
ros2 run rosplan_planning_system PDDLSimplePlanParser \
    --ros-args -p planner_topic:=planner_output
```

Trigger parsing:

```bash
timeout 15 ros2 service call \
    /rosplan_parsing_interface/parse_plan \
    std_srvs/srv/Empty "{}"
```

The parser publishes a `CompletePlan` message (a sequence of `ActionDispatch` messages)
on the `complete_plan` topic.

---

### 4.7 Step 6 — Start Action Interfaces

Start one simulated action interface per PDDL operator.
Each simulated node waits for an `ActionDispatch` message, sleeps for
`action_duration` seconds, and reports success.

```bash
# Terminal 6 — open four sub-terminals or use separate terminals

ros2 launch rosplan_planning_system simulated_action.launch.py \
    pddl_action_name:=undock action_duration:=2.0 action_probability:=1.0

ros2 launch rosplan_planning_system simulated_action.launch.py \
    pddl_action_name:=dock action_duration:=3.0 action_probability:=1.0

ros2 launch rosplan_planning_system simulated_action.launch.py \
    pddl_action_name:=localise action_duration:=1.0 action_probability:=1.0

ros2 launch rosplan_planning_system simulated_action.launch.py \
    pddl_action_name:=goto_waypoint action_duration:=1.0 action_probability:=1.0
```

Each node prints:

```
[rosplan_interface_goto_waypoint]: Ready to receive
```

**Action probability** (`action_probability`) is a float in `[0, 1]`. Set it below `1.0`
to simulate random action failure and test replanning behaviour.

---

### 4.8 Step 7 — Dispatch the Plan

Start the dispatcher. Like the parser, it uses `transient_local` QoS so it receives
the `CompletePlan` immediately on startup:

```bash
# Terminal 7
ros2 run rosplan_planning_system SimplePlanDispatcher \
    --ros-args -p plan_topic:=complete_plan
```

Trigger dispatch:

```bash
timeout 120 ros2 service call \
    /dispatch_plan \
    rosplan_dispatch_msgs/srv/DispatchService "{}"
```

You will see each action dispatched and confirmed in the dispatcher terminal:

```
[SimplePlanDispatcher]: Dispatching action [undock; kenny, wp0]
[SimplePlanDispatcher]: Action [0, 0] succeeded
[SimplePlanDispatcher]: Dispatching action [localise; kenny]
...
[SimplePlanDispatcher]: Goal achieved.
```

Verify the final KB state:

```bash
ros2 service call /rosplan_knowledge_base/state/propositions \
    rosplan_knowledge_msgs/srv/GetAttributeService \
    "{attribute_name: 'visited'}"

ros2 service call /rosplan_knowledge_base/state/propositions \
    rosplan_knowledge_msgs/srv/GetAttributeService \
    "{attribute_name: 'docked'}"
```

Expected: `visited(wp0)` through `visited(wp4)` all present; `docked(kenny)` present.

---

### 4.9 Running the Full Demo at Once

A convenience script runs all seven steps automatically in the current terminal,
printing progress as each step completes:

```bash
chmod +x docs/examples/turtlebot/run_demo.sh
bash docs/examples/turtlebot/run_demo.sh --waypoints 5
```

---

## 5. Writing a Custom Action Interface (Python)

The `rosplan_action_interface` package provides base classes for three interface styles:

| Class | When to use |
|---|---|
| `ActionlibActionInterface` | Your execution node exposes a ROS 2 action server |
| `ServiceActionInterface` | Your execution node exposes a ROS 2 service |
| `FSMActionInterface` | You implement a custom finite-state machine |

All three share the same YAML-based configuration described in Section 5.3.

### 5.1 Service-based interface

Suppose you have a PDDL action `pick_up ?obj - object` implemented as a ROS 2 service
`/arm_controller/pick_up` of type `my_msgs/srv/Pick`.

**Package structure:**

```
my_robot_interfaces/
  my_robot_interfaces/
    pick_action.py
  config/
    actions.yaml
  setup.py
  package.xml
```

**`pick_action.py`:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rosplan_action_interface.ActionInterfaceManager import ActionInterfaceManager

def main():
    rclpy.init()
    node = ActionInterfaceManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**`config/actions.yaml`:**

```yaml
actions:
  - name: pick_up
    interface_type: service
    default_service: /arm_controller/pick_up
    default_service_type: my_msgs/srv/Pick
    # Fill request fields using PDDL parameter values:
    default_service_request:
        object_name: ($pddlparam obj)
    # Require 'success: true' in the response:
    default_service_response:
        success: True
```

**Running it:**

```bash
ros2 run my_robot_interfaces pick_action \
    --ros-args -p actions_config:=/path/to/config/actions.yaml
```

### 5.2 Action-based interface (actionlib style)

When your node exposes a ROS 2 action server (e.g., `move_base_msgs/MoveBase`):

```yaml
actions:
  - name: goto_waypoint
    interface_type: actionlib
    default_actionlib_topic: /move_base
    default_actionlib_msg_type: move_base_msgs/MoveBase
    default_actionlib_goal:
        target_pose.header.frame_id: "map"
        target_pose.pose.position.x: ($rosparam /waypoints/($pddlparam to)/x)
        target_pose.pose.position.y: ($rosparam /waypoints/($pddlparam to)/y)
        target_pose.pose.orientation.w: 1
```

The special terms in config strings:

| Term | Meaning |
|---|---|
| `($pddlparam NAME)` | Value of the PDDL action parameter named `NAME` |
| `($rosparam PATH)` | Value of the ROS 2 parameter at `PATH` |

**Parameter-specific overrides:**
You can specify different goals for specific parameter values:

```yaml
  - name: goto_waypoint
    interface_type: actionlib
    default_actionlib_topic: /move_base
    default_actionlib_msg_type: move_base_msgs/MoveBase
    default_actionlib_goal:
        target_pose.pose.position.x: 0.0
        target_pose.pose.position.y: 0.0
    pddl_parameters: [to]
    parameter_values:
      - values: [wp_home]
        actionlib_goal:
            target_pose.pose.position.x: 1.5
            target_pose.pose.position.y: 0.0
      - values: [wp_office]
        actionlib_goal:
            target_pose.pose.position.x: 5.0
            target_pose.pose.position.y: 3.2
```

### 5.3 The action config YAML

The `ActionInterfaceManager` node reads a YAML file whose path is given by the
`actions_config` parameter. Full structure:

```yaml
actions:
  - name: <pddl_action_name>          # must match operator name in domain
    interface_type: <actionlib|service|fsm>

    # ── actionlib ──────────────────────────────────────
    default_actionlib_topic: /topic
    default_actionlib_msg_type: pkg/MsgType
    default_actionlib_goal:            # field: value pairs
        field.subfield: value
    default_actionlib_result:          # expected result fields (optional)
        success: True

    # ── service ────────────────────────────────────────
    default_service: /topic
    default_service_type: pkg/srv/Type
    default_service_request:           # optional
        field: value
    default_service_response:          # expected response fields (optional)
        success: True

    # ── parameter-based overrides (both types) ─────────
    pddl_parameters: [param1, param2]
    parameter_values:
      - values: [val1, val2]
        actionlib_goal:   { ... }      # or service_request / service_response
```

---

## 6. Writing a Custom Action Interface (C++)

For performance-critical actions or when the action node is part of a larger
C++ node, you can subclass `RPActionInterface` directly.

**Minimal example — a `pick_up` action:**

```cpp
// pick_up_action.cpp
#include "rosplan_action_interface/RPActionInterface.h"

class PickUpAction : public KCL_rosplan::RPActionInterface {
public:
    PickUpAction() : RPActionInterface("pick_up_action") {}

    bool concreteCallback(
        const rosplan_dispatch_msgs::msg::ActionDispatch::ConstSharedPtr msg)
    {
        // Extract the PDDL parameter named "obj"
        std::string obj_name;
        for (const auto& kv : msg->parameters)
            if (kv.key == "obj") obj_name = kv.value;

        RCLCPP_INFO(get_logger(), "Picking up: %s", obj_name.c_str());

        // Call your hardware here...
        // Return true on success, false on failure.
        return true;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickUpAction>();

    // setup() queries the KB for operator details before spin starts.
    // It uses a satellite node internally — no deadlock.
    node->setup();
    if (!rclcpp::ok()) return 1;

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
```

**CMakeLists.txt additions:**

```cmake
find_package(rosplan_action_interface REQUIRED)
find_package(rosplan_dispatch_msgs REQUIRED)

add_executable(pick_up_action src/pick_up_action.cpp)
ament_target_dependencies(pick_up_action
    rclcpp
    rosplan_action_interface
    rosplan_dispatch_msgs
)
install(TARGETS pick_up_action DESTINATION lib/${PROJECT_NAME})
```

**Running:**

```bash
ros2 run my_robot_cpp_interfaces pick_up_action \
    --ros-args \
    -p knowledge_base:=rosplan_knowledge_base \
    -p pddl_action_name:=pick_up \
    -p action_dispatch_topic:=/rosplan_plan_dispatcher/action_dispatch \
    -p action_feedback_topic:=/rosplan_plan_dispatcher/action_feedback
```

The node subscribes to `action_dispatch_topic`, filters for messages that match
`pddl_action_name`, calls `concreteCallback()`, and publishes the result on
`action_feedback_topic`. KB effects (at-start / at-end) are applied automatically.

---

## 7. The Sensing Interface

The sensing interface keeps the KB updated from live sensor data.
It subscribes to ROS 2 topics and services and evaluates a one-line Python expression
to derive the truth value of a predicate.

**Config file (`sensing.yaml`):**

```yaml
topics:
    docked:                            # predicate name in domain
        params:
            - kenny                    # instantiated parameters
        topic: /battery_state
        msg_type: sensor_msgs/BatteryState
        operation: msg.power_supply_status == 4  # CHARGING

    robot_at:
        params:
            - kenny
            - '*'                      # wildcard — matched against waypoint list
        topic: /amcl_pose
        msg_type: geometry_msgs/PoseWithCovarianceStamped
        # No operation: a custom function in functions.py computes the waypoint

functions:
    - /path/to/my_sensing_functions.py
```

**Running:**

```bash
ros2 run rosplan_sensing_interface sensing_interface \
    --ros-args -p config_path:=/path/to/sensing.yaml
```

The sensing interface runs at a configurable rate (default 10 Hz) and calls
the KB `update_array` service to keep facts in sync with sensor data.

---

## 8. Inspecting the Knowledge Base

All KB data is accessible through ROS 2 services. Common queries:

```bash
# Domain name
ros2 service call /rosplan_knowledge_base/domain/name \
    rosplan_knowledge_msgs/srv/GetDomainNameService "{}"

# All predicate definitions
ros2 service call /rosplan_knowledge_base/domain/predicates \
    rosplan_knowledge_msgs/srv/GetDomainAttributeService "{}"

# All instances of a type
ros2 service call /rosplan_knowledge_base/state/instances \
    rosplan_knowledge_msgs/srv/GetInstanceService \
    "{type_name: 'waypoint'}"

# All true facts for a predicate
ros2 service call /rosplan_knowledge_base/state/propositions \
    rosplan_knowledge_msgs/srv/GetAttributeService \
    "{attribute_name: 'robot_at'}"

# Current goals
ros2 service call /rosplan_knowledge_base/state/goals \
    rosplan_knowledge_msgs/srv/GetAttributeService "{}"

# Add a fact manually
ros2 service call /rosplan_knowledge_base/update \
    rosplan_knowledge_msgs/srv/KnowledgeUpdateService \
    "{update_type: 0,
      knowledge: {
          knowledge_type: 1,
          attribute_name: 'visited',
          values: [{key: '?wp', value: 'wp3'}]}}"
```

Update type constants: `0 = ADD_KNOWLEDGE`, `2 = REMOVE_KNOWLEDGE`, `1 = ADD_GOAL`, `3 = REMOVE_GOAL`.

The rqt GUI provides a graphical KB inspector:

```bash
rqt --force-discover
# Plugins → ROSPlan → Problem Viewer
```

---

## 9. Troubleshooting

### KB services not found

```
[ERROR] KB service /rosplan_knowledge_base/domain/name not available
```

Check the KB node is running and the domain file path is correct:

```bash
ros2 node list | grep knowledge_base
ros2 service list | grep knowledge_base
```

### Parser prints "No new planner output received"

The parser subscribes to `planner_output` with `transient_local` QoS.
If it prints this message, either:
- The planner has not run yet (run Step 4 first), or
- The `planner_topic` parameter does not match the topic the planner publishes on.

Check with `ros2 topic list` and `ros2 topic echo /planner_output --once`.

### Dispatcher completes immediately with no actions dispatched

This means the dispatcher received an empty `CompletePlan`.
Ensure the parser ran successfully before the dispatcher started and that the
`plan_topic` parameter matches:

```bash
ros2 topic echo /complete_plan --once
```

### Action interface prints "could not call KB for operator details"

This happens if the KB is not running when the action interface starts.
Start the KB first and wait for it to print "Ready" before launching action nodes.

### `async_send_request` / future never completes

You are calling a service from inside an executor callback without using a satellite node.
See the [migration guide](ros2_migration.md#7-asynchronous-service-calls-and-executor-patterns)
for the correct pattern.

### POPF binary not found

```
[ERROR] planner call: No such file or directory
```

The binary must have execute permission and be at the installed path:

```bash
ls -la $(ros2 pkg prefix rosplan_planning_system)/share/rosplan_planning_system/common/bin/popf
chmod +x $(ros2 pkg prefix rosplan_planning_system)/share/rosplan_planning_system/common/bin/popf
```

If the path ends in `bin/bin/popf` instead of `bin/popf`, the CMake install rule has a
trailing-slash bug — see the [migration guide](ros2_migration.md#172-popf-binary-path-double-nesting).

### Plan dispatch service type error

```
Waiting for service /dispatch_plan to become available...
```

Always call the dispatch service with the correct type:

```bash
# Correct
ros2 service call /dispatch_plan rosplan_dispatch_msgs/srv/DispatchService "{}"

# Wrong — will hang indefinitely
ros2 service call /dispatch_plan std_srvs/srv/Empty "{}"
```
