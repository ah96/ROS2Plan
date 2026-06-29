# ROS2Plan

ROS2Plan is a port of the [ROSPlan](https://github.com/KCL-Planning/ROSPlan) task planning framework to **ROS2 Humble Hawksbill** on Ubuntu 22.04.  It preserves the full feature set of the original — PDDL/HDDL knowledge base, multiple planner adapters, plan dispatching, sensing interface, rqt GUI — while conforming to ROS2 conventions throughout.

## Packages

| Package | Description |
|---|---|
| `ros2plan_dependencies` | VAL PDDL validator, HDDL parser, POPF/CHIMP planner binaries |
| `ros2plan_knowledge_msgs` | Messages, services, and actions for the knowledge base |
| `ros2plan_dispatch_msgs` | Messages, services, and actions for plan dispatch |
| `ros2plan_knowledge_base` | PDDL/HDDL knowledge base node |
| `ros2plan_planning_system` | Problem generation, planner interfaces (POPF, CHIMP, …), plan parsing, plan dispatch |
| `ros2plan_action_interface` | Python base classes for action execution nodes |
| `ros2plan_sensing_interface` | Periodic knowledge base updates from sensor callbacks |
| `ros2plan_rqt` | rqt GUI plugins (dispatcher, plan viewer, action config) |
| `ros2plan` | Meta-package |

## Requirements

- ROS2 Humble Hawksbill
- Ubuntu 22.04 LTS
- C++17 (GCC 11)
- Python 3.10+
- colcon

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-org/ROS2Plan.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### Building the knowledge base

The RDDL and PPDDL sub-parsers are disabled by default (they have additional dependencies):

```bash
colcon build --packages-select ros2plan_knowledge_base \
    --cmake-args -DROS2PLAN_BUILD_RDDL=OFF -DROS2PLAN_BUILD_PPDDL=OFF
```

## Quick start

The fastest way to see the whole pipeline — knowledge base, problem generation, planning,
parsing, dispatch and four simulated action interfaces — is the turtlebot demo:

```bash
bash docs/examples/turtlebot/run_demo.sh --waypoints 5
```

It ends with the robot having visited every waypoint and re-docked
(`goal_achieved=True`). The script is also the clearest reference for how each node is
launched and wired together.

To run the steps yourself, the trigger services live in the **root** namespace, and the
planner is best triggered with the `/planning_server` service (which uses the node
parameters):

```bash
ros2 service call /problem_generation_server std_srvs/srv/Empty "{}"   # generate problem
ros2 service call /planning_server          std_srvs/srv/Empty "{}"    # run the planner
ros2 service call /parse_plan               std_srvs/srv/Empty "{}"    # parse the plan
ros2 service call /dispatch_plan ros2plan_dispatch_msgs/srv/DispatchService "{}"  # dispatch
```

See the [documentation and tutorials](docs/index.md) for the full step-by-step walkthrough.

## Architecture

```
Knowledge Base
    ├── domain (predicates, operators, functions)
    └── problem (instances, facts, goals)

Planning System
    ├── Problem Interface    → generates PDDL problem file
    ├── Planner Interface    → calls external planner (POPF, CHIMP, …)
    ├── Plan Parser          → parses plan into ActionDispatch messages
    └── Plan Dispatcher      → executes actions via action topics
            │
            ▼
    Action Interface nodes   (one per PDDL operator)
            │
            ▼
    Sensing Interface        (updates KB from sensor topics)
```

## Key ROS2 differences from ROSPlan

A full description of every change made during the port is in
[ros2_migration.md](ros2_migration.md) and the accompanying
[ros2_migration.pdf](ros2_migration.pdf).  A short summary:

- All nodes inherit `rclcpp::Node` and run under `MultiThreadedExecutor`
- Latched publishers use `rclcpp::QoS(1).transient_local()`; **subscribers must also declare
  `transient_local()`** to receive stored messages
- Service calls from setup code use a satellite node +
  `rclcpp::spin_until_future_complete` to avoid pre-executor deadlocks
- Launch files converted from XML to Python
- Python nodes use `rclpy`; complex config loaded from YAML files via a `config_path` parameter
- `hddl_parser` is built as a shared library to avoid linker symbol collisions

## Simulated action interfaces

For testing without real hardware, use the provided simulated action interface:

```bash
ros2 launch ros2plan_planning_system simulated_action.launch.py \
    pddl_action_name:=goto_waypoint \
    action_duration:=1.0 \
    action_probability:=1.0
```

## Sensing interface

```bash
ros2 run ros2plan_sensing_interface sensing_interface \
    --ros-args -p config_path:=/path/to/sensing_config.yaml
```

The YAML format is unchanged from ROS1.  See
`ros2plan_sensing_interface/config/sensing_example.yaml` for a template.

## rqt plugins

```bash
rqt --force-discover
```

The following plugins are available under *Plugins → ROS2Plan*:

- **Plan Dispatcher** — send service calls from the GUI
- **Action Dispatcher** — manually dispatch individual actions
- **Problem Viewer** — inspect current KB state
- **Esterel Plan Viewer** — graphical plan visualization
- **Action Config** — configure action interface parameters

## Generating the migration PDF

The migration documentation can be regenerated from the markdown source:

```bash
pip install weasyprint markdown
python3 md_to_pdf.py ros2_migration.md ros2_migration.pdf
```

## License

BSD 2-Clause — see [LICENSE](LICENSE).  
Original ROSPlan framework by the Planning group at King's College London.  
ROS2 port by Amar Halilovic.
