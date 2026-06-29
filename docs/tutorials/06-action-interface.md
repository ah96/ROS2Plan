# Tutorial 6 — Action Interfaces

An **action interface** connects a PDDL operator to the code that actually performs it.
When the dispatcher sends an `ActionDispatch` for, say, `goto_waypoint`, the action
interface for `goto_waypoint` runs the real behaviour (drive the base, call a service, …),
reports success or failure, and the knowledge-base effects are applied.

[Tutorial 5](05-simulated-actions.md) used the built-in *simulated* interface. This
tutorial shows the two ways to write a real one:

- **Python, configuration-only** — map an operator to an existing ROS action or service
  with a YAML file, no code required;
- **C++, subclassing** — implement the behaviour directly in `concreteCallback`.

## Python: the action interface manager

The `ros2plan_action_interface` package provides a manager node that reads a YAML config
and creates one interface per PDDL operator. Three interface types are supported:

| `interface_type` | Use when the action is implemented as |
|---|---|
| `actionlib` | a ROS 2 action server |
| `service` | a ROS 2 service |
| `fsm` | a custom finite-state machine |

You run the manager and point it at your config:

```bash
ros2 run ros2plan_action_interface action_interface_manager --ros-args \
    -p actions_config:=/path/to/actions.yaml
```

### Example config

```yaml
actions:

    # An action implemented by an action server (e.g. navigation)
    - name: goto_waypoint
      interface_type: actionlib
      default_actionlib_topic: /move_base
      default_actionlib_msg_type: move_base_msgs/MoveBase
      default_actionlib_goal:
          target_pose.header.frame_id: "map"
          target_pose.pose.position.x: ($rosparam /waypoints/($pddlparam to))[0]
          target_pose.pose.position.y: ($rosparam /waypoints/($pddlparam to))[1]
          target_pose.pose.orientation.w: 1

    # An action implemented by a service
    - name: clear_costmap
      interface_type: service
      default_service: /move_base/clear_costmaps
      default_service_type: std_srvs/Empty
```

### Filling message fields from the plan

Two special terms let a config value depend on the dispatched action:

| Term | Resolves to |
|---|---|
| `($pddlparam NAME)` | the value of the PDDL parameter `NAME` in the dispatched action |
| `($rosparam PATH)` | the value of the ROS 2 parameter at `PATH` |

They can be nested — `($rosparam /waypoints/($pddlparam to))` looks up the parameter named
after the action's `to` argument. This is how a single `goto_waypoint` interface sends the
base to a different pose for every waypoint.

### Per-parameter overrides

To use different goal/request values for specific parameter combinations, add
`pddl_parameters` and `parameter_values`:

```yaml
    - name: goto_waypoint
      interface_type: actionlib
      default_actionlib_topic: /move_base
      default_actionlib_msg_type: move_base_msgs/MoveBase
      default_actionlib_goal:
          target_pose.pose.orientation.w: 1
      pddl_parameters: [to]
      parameter_values:
        - values: [wp0]
          actionlib_goal:
              target_pose.pose.position.x: 0.0
              target_pose.pose.position.y: 0.0
        - values: [wp1]
          actionlib_goal:
              target_pose.pose.position.x: 1.5
              target_pose.pose.position.y: 0.0
```

You can also require specific response/result fields with `default_service_response`,
`default_actionlib_result` (and their per-parameter variants); the action is reported as
*failed* if they do not match.

A complete annotated template ships at
`ros2plan_action_interface/config/example_config.yaml`.

## C++: subclassing `RPActionInterface`

For performance-critical actions or actions that are part of a larger C++ node, subclass
`KCL_ros2plan::RPActionInterface` and implement the pure-virtual `concreteCallback`. It is
called with the dispatched action; return `true` on success, `false` on failure.

```cpp
// pick_up_action.cpp
#include "ros2plan_action_interface/RPActionInterface.h"

class PickUpAction : public KCL_ros2plan::RPActionInterface {
public:
    PickUpAction() : RPActionInterface("pick_up_action") {}

    bool concreteCallback(
        const ros2plan_dispatch_msgs::msg::ActionDispatch::SharedPtr msg) override
    {
        // Read the PDDL parameter named "obj"
        std::string obj;
        for (const auto& kv : msg->parameters)
            if (kv.key == "obj") obj = kv.value;

        RCLCPP_INFO(get_logger(), "Picking up: %s", obj.c_str());

        // ... drive the hardware ...
        return true;   // false reports the action as failed
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickUpAction>();

    // setup() fetches this operator's details from the KB before spinning.
    // It uses a satellite node internally, so there is no pre-executor deadlock.
    node->setup();
    if (!rclcpp::ok()) return 1;

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
```

`CMakeLists.txt`:

```cmake
find_package(ros2plan_action_interface REQUIRED)
find_package(ros2plan_dispatch_msgs REQUIRED)

add_executable(pick_up_action src/pick_up_action.cpp)
ament_target_dependencies(pick_up_action
    rclcpp ros2plan_action_interface ros2plan_dispatch_msgs)
install(TARGETS pick_up_action DESTINATION lib/${PROJECT_NAME})
```

Run it like any action interface:

```bash
ros2 run my_pkg pick_up_action --ros-args \
    -p knowledge_base:=ros2plan_knowledge_base \
    -p pddl_action_name:=pick_up
```

The base class subscribes to the dispatch topic, filters for messages whose `name` matches
`pddl_action_name`, calls your `concreteCallback`, publishes feedback, and applies the
at-start / at-end knowledge-base effects automatically.

---

Next: [The Sensing Interface](07-sensing-interface.md) — update the knowledge base from
sensors.

Reference: message definitions in [Message types](../reference/message-types.md).
