# Installation

## Requirements

| Requirement | Version |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Build tool | colcon |
| C++ standard | 17 |
| Python | 3.10+ |

Install ROS 2 Humble first by following the
[official guide](https://docs.ros.org/en/humble/Installation.html), then source it:

```bash
source /opt/ros/humble/setup.bash
```

## Build

```bash
# 1. Create a colcon workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Get ROS2Plan
git clone https://github.com/KCL-Planning/ROS2Plan.git
#   (or copy the ros2plan_* packages into ~/ros2_ws/src)

# 3. Install ROS dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build

# 5. Source the workspace
source install/setup.bash
```

Add the source command to your shell profile so every new terminal has it:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Optional parsers

The RDDL and PPDDL parsers in `ros2plan_dependencies` and `ros2plan_knowledge_base` are
only needed for probabilistic planning, and they have extra toolchain requirements (the
RDDL parser needs GCC ≥ 12 or Clang). They are **off by default**. Enable them with:

```bash
colcon build --cmake-args -DROS2PLAN_BUILD_RDDL=ON -DROS2PLAN_BUILD_PPDDL=ON
```

> **Build environment note.** Build with the **system** Python interpreter
> (`/usr/bin/python3`). The ROS 2 interface generators (`rosidl`) need the `empy`
> module, which ships with the ROS install. If you build inside a virtual environment
> that does not expose the system site-packages, the `*_msgs` packages fail with
> `ModuleNotFoundError: No module named 'em'`. Do not activate such a virtualenv before
> running `colcon build`.

## Verify the install

```bash
# all nine packages are present
ros2 pkg list | grep ros2plan

# the message/service/action types are registered
ros2 interface list | grep ros2plan        # ~40 ros2plan_* types

# the knowledge base node runs
ros2 run ros2plan_knowledge_base knowledgeBase --ros-args \
    -p domain_path:=$(ros2 pkg prefix ros2plan_planning_system)/share/ros2plan_planning_system/test/pddl/turtlebot/domain.pddl
```

The knowledge base should print `KCL: (ros2plan_knowledge_base) Ready to receive`. Stop
it with `Ctrl-C`.

## The packages

| Package | Contents |
|---|---|
| `ros2plan_knowledge_base` | PDDL/HDDL knowledge base node and parsers |
| `ros2plan_planning_system` | problem generation, planner interfaces, parsing, dispatch |
| `ros2plan_action_interface` | Python base classes for action interfaces |
| `ros2plan_sensing_interface` | Python sensing interface |
| `ros2plan_knowledge_msgs` | knowledge-base message, service and action types |
| `ros2plan_dispatch_msgs` | planning and dispatch message, service and action types |
| `ros2plan_rqt` | rqt GUI plugins |
| `ros2plan_dependencies` | bundled planners and parsers (POPF, VAL, HDDL, RDDL, …) |
| `ros2plan` | metapackage |

Next: [The Knowledge Base](tutorials/01-knowledge-base.md).
