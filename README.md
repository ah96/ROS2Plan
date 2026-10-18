# ROS2Plan

ROS2Plan is a port of the [ROSPlan](https://github.com/KCL-Planning/ROSPlan) task planning
framework to **ROS 2 Lyrical Luth** on Ubuntu 26.04. It preserves the full feature set of
the original — a PDDL/HDDL/PPDDL/RDDL knowledge base, thirteen planner adapters, plan parsing
and dispatch, action and sensing interfaces, and the rqt GUI — while following ROS 2
conventions throughout.

> **Branches.** Each ROS 2 distribution has its own branch: this is `lyrical`. For ROS 2
> Humble on Ubuntu 22.04, use the [`humble`](https://github.com/ah96/ROS2Plan/tree/humble)
> branch.

## Packages

| Package | Description |
|---|---|
| `ros2plan_dependencies` | Parser libraries: VAL (PDDL), HDDL, PPDDL, RDDL, and the IPPC server |
| `ros2plan_knowledge_msgs` | Messages, services, and actions for the knowledge base |
| `ros2plan_dispatch_msgs` | Messages, services, and actions for plan dispatch |
| `ros2plan_knowledge_base` | Knowledge base node (PDDL, HDDL, PPDDL, RDDL) |
| `ros2plan_planning_system` | Problem generation, planner interfaces, plan parsing, plan dispatch, bundled planner binaries |
| `ros2plan_action_interface` | Action interface manager and Python base classes for action execution |
| `ros2plan_sensing_interface` | Periodic knowledge base updates from topics and services |
| `ros2plan_rqt` | rqt GUI plugins (dispatcher, plan viewer, action config) |
| `ros2plan` | Meta-package |

Planner adapters shipped with `ros2plan_planning_system`: POPF, FF, Metric-FF, FD, TFD, LPG,
UPM, PPRP, SMT, PANDA, CHIMP, RDDLSim and an online interface. POPF, Metric-FF, Contingent-FF,
PANDA and PROST binaries are bundled under `common/bin/`.

## Requirements

- ROS2 Lyrical Luth
- Ubuntu 26.04 LTS
- C++20 (GCC 15)
- Python 3.14
- CMake 3.10+
- Qt 6 / PyQt6 (for the rqt plugins; ships with Lyrical's rqt)
- colcon

## Installation

```bash
sudo apt install bison flex default-jdk libboost-filesystem-dev
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b lyrical https://github.com/ah96/ROS2Plan.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

`rosdep` also installs the runtime tools the nodes call out to, such as Graphviz (`dot`) for
the Esterel plan viewer.

Build with the **system** Python interpreter (`/usr/bin/python3`). The ROS 2 interface
generators need `empy` from the ROS installation, which a virtual environment will usually
shadow.

### Knowledge base: PDDL, HDDL, PPDDL and RDDL

The knowledge base supports four modelling languages — **PDDL**, **HDDL**, **PPDDL**
(probabilistic PDDL) and **RDDL** — and all four are built by a plain `colcon build` on the
stock Ubuntu 26.04 toolchain:

```bash
# PDDL + HDDL + PPDDL + RDDL
colcon build
```

PPDDL and RDDL can be turned off explicitly if you do not need probabilistic planning and
want a shorter build:

```bash
colcon build --cmake-args -DROS2PLAN_BUILD_RDDL=OFF -DROS2PLAN_BUILD_PPDDL=OFF
```

> The same flags apply to both
> `ros2plan_dependencies` and `ros2plan_knowledge_base`; the defaults are kept in sync so a
> plain `colcon build` of the whole workspace is consistent.

## Quick start

The fastest way to see the whole pipeline — knowledge base, problem generation, planning,
parsing, dispatch and four simulated action interfaces — is the turtlebot demo:

```bash
bash docs/examples/turtlebot/run_demo.sh --waypoints 5
```

It ends with the robot having visited every waypoint and re-docked
(`goal_achieved=True`). The script is also the clearest reference for how each node is
launched and wired together.

The demo runs on `ROS_DOMAIN_ID=77` so it cannot collide with other ROS 2 systems already
running on the default domain — ROS2Plan nodes use fixed node names, so two knowledge bases
on one domain are indistinguishable. Export a different `ROS_DOMAIN_ID` before running to
override it, and export the same value in any other shell you want to inspect the demo from.

To run the steps yourself, the trigger services live in the **root** namespace, and the
planner is best triggered with the `/planning_server` service (which uses the node
parameters):

```bash
ros2 service call /problem_generation_server std_srvs/srv/Empty "{}"   # generate problem
ros2 service call /planning_server          std_srvs/srv/Empty "{}"    # run the planner
ros2 service call /parse_plan               std_srvs/srv/Empty "{}"    # parse the plan
ros2 service call /dispatch_plan ros2plan_dispatch_msgs/srv/DispatchService "{}"  # dispatch
```

`docs/examples/turtlebot/` contains the demo script, the PDDL domain and a helper that
populates the knowledge base, so the sequence above can be run end to end from a clean
workspace.

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

A short summary of the port:

- All nodes inherit `rclcpp::Node` and run under `MultiThreadedExecutor`
- Latched publishers use `rclcpp::QoS(1).transient_local()`; **subscribers must also declare
  `transient_local()`** to receive stored messages
- Service calls from setup code use a satellite node +
  `rclcpp::spin_until_future_complete` to avoid pre-executor deadlocks
- Launch files converted from XML to Python
- Python nodes use `rclpy`; complex config loaded from YAML files via a `config_path` parameter
- `hddl_parser` is built as a shared library to avoid linker symbol collisions

## Differences between the `lyrical` and `humble` branches

The two branches share their sources apart from what the newer ROS 2 release requires. When
moving a fix from one branch to the other with `git cherry-pick`, these are the places where
the code differs:

| | `humble` | `lyrical` |
|---|---|---|
| Platform | Ubuntu 22.04, GCC 11, CMake 3.22, Python 3.10 | Ubuntu 26.04, GCC 15, CMake 4.2, Python 3.14 |
| C++ standard | C++17 | C++20 (required by `rclcpp`) |
| Linking ROS dependencies | `ament_target_dependencies()` | `target_link_libraries()` with the packages' CMake targets (`ament_target_dependencies()` was removed) |
| Service QoS | `rmw_qos_profile_services_default` | `rclcpp::ServicesQoS()` |
| VAL requirement check | `requires()` | `pddl_requires()` (`requires` is a C++20 keyword) |
| rqt GUI | PyQt5 | PyQt6 (scoped enums, `QMouseEvent.position()`) |

## Simulated action interfaces

For testing without real hardware, use the provided simulated action interface:

```bash
ros2 launch ros2plan_planning_system simulated_action.launch.py \
    pddl_action_name:=goto_waypoint \
    action_duration:=1.0 \
    action_probability:=1.0
```

## Action interfaces

Each PDDL operator needs a node that executes it. `ros2plan_action_interface` provides the
manager plus base classes for the three execution styles — ROS 2 action, service, and FSM:

```bash
ros2 run ros2plan_action_interface action_interface_manager --ros-args \
    -p actions_config:=$(ros2 pkg prefix ros2plan_action_interface)/share/ros2plan_action_interface/config/example_config.yaml
```

`example_config.yaml` documents the configuration format, including the `($pddlparam …)`
substitutions that pass PDDL action parameters into goal fields.

## Sensing interface

```bash
ros2 launch ros2plan_sensing_interface ros2plan_sensing_example.launch.py

# or point it at your own configuration
ros2 run ros2plan_sensing_interface sensing_interface \
    --ros-args -p config_path:=/path/to/sensing_config.yaml
```

The YAML format is unchanged from ROSPlan. `ros2plan_sensing_interface/example.yaml` is a
commented template covering topic-sensed and service-sensed predicates; the Python helper it
references (`example.py`) computes predicate values that cannot be expressed as a one-line
`operation`. A predicate whose message or service type is not installed is skipped with a
logged error rather than aborting the node — the shipped example uses `kobuki_msgs`, which has
no ROS 2 release, so its `docked`/`undocked` entries are skipped on a stock install.

## rqt plugins

```bash
rqt --force-discover
```

The following plugins are available under *Plugins → ROS2Plan*:

- **Plan Dispatcher** — send service calls from the GUI
- **Action Dispatcher** — manually dispatch individual actions
- **Problem Viewer** — inspect current KB state
- **Esterel Plan Viewer** — graphical plan visualization (needs Graphviz's `dot`)
- **Action Config** — configure action interface parameters

## Running the tests

```bash
export ROS_DOMAIN_ID=78
colcon test --executor sequential
colcon test-result --verbose
```

Both settings matter. The integration tests launch nodes under fixed names
(`ros2plan_knowledge_base` and friends), so any other ROS 2 system reachable on the same domain
is indistinguishable from the one under test; and the fixed service timeouts flake when several
packages' tests run concurrently on a small machine.

## License

BSD 2-Clause — see [LICENSE](LICENSE).  
Original ROSPlan framework by the Planning group at King's College London.  
ROS2 port by Amar Halilovic.
