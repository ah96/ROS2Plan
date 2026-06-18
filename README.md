# ROS2Plan

ROS2Plan is a port of the [ROSPlan](https://github.com/KCL-Planning/ROSPlan) task planning framework to **ROS2 Humble Hawksbill** on Ubuntu 22.04.  It preserves the full feature set of the original — PDDL/HDDL knowledge base, multiple planner adapters, plan dispatching, sensing interface, rqt GUI — while conforming to ROS2 conventions throughout.

## Packages

| Package | Description |
|---|---|
| `rosplan_dependencies` | VAL PDDL validator, HDDL parser, POPF/CHIMP planner binaries |
| `rosplan_knowledge_msgs` | Messages, services, and actions for the knowledge base |
| `rosplan_dispatch_msgs` | Messages, services, and actions for plan dispatch |
| `rosplan_knowledge_base` | PDDL/HDDL knowledge base node |
| `rosplan_planning_system` | Problem generation, planner interfaces (POPF, CHIMP, …), plan parsing, plan dispatch |
| `rosplan_action_interface` | Python base classes for action execution nodes |
| `rosplan_sensing_interface` | Periodic knowledge base updates from sensor callbacks |
| `rosplan_rqt` | rqt GUI plugins (dispatcher, plan viewer, action config) |
| `rosplan` | Meta-package |

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
colcon build --packages-select rosplan_knowledge_base \
    --cmake-args -DROSPLAN_BUILD_RDDL=OFF -DROSPLAN_BUILD_PPDDL=OFF
```

## Quick start

### 1. Launch the knowledge base

```bash
ros2 launch rosplan_knowledge_base rosplan_knowledge_base.launch.py \
    domain_path:=/path/to/domain.pddl
```

### 2. Launch the full planning system

```bash
ros2 launch rosplan_planning_system interfaced_planning_system.launch.py \
    domain_path:=/path/to/domain.pddl \
    problem_path:=/path/to/problem.pddl \
    planner_command:="timeout 10 $(ros2 pkg prefix rosplan_planning_system)/share/rosplan_planning_system/common/bin/popf DOMAIN PROBLEM"
```

### 3. Trigger planning and dispatch

```bash
# Generate problem
ros2 service call /rosplan_problem_interface/problem_generation_server std_srvs/srv/Empty

# Run planner
ros2 action send_goal /rosplan_planner_interface/start_planning rosplan_dispatch_msgs/action/Plan "{}"

# Parse and dispatch
ros2 service call /rosplan_parsing_interface/parse_plan std_srvs/srv/Empty
ros2 service call /dispatch_plan rosplan_dispatch_msgs/srv/DispatchService "{}"
```

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
ros2 launch rosplan_planning_system simulated_action.launch.py \
    pddl_action_name:=goto_waypoint \
    action_duration:=1.0 \
    action_probability:=1.0
```

## Sensing interface

```bash
ros2 run rosplan_sensing_interface sensing_interface \
    --ros-args -p config_path:=/path/to/sensing_config.yaml
```

The YAML format is unchanged from ROS1.  See
`rosplan_sensing_interface/config/sensing_example.yaml` for a template.

## rqt plugins

```bash
rqt --force-discover
```

The following plugins are available under *Plugins → ROSPlan*:

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
