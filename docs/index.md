# ROS2Plan Documentation

ROS2Plan is a task-planning framework for ROS 2. It maintains a PDDL/HDDL knowledge
base, generates planning problems from the current world state, calls an external
planner, and dispatches the resulting plan to action interfaces that execute it on a
robot. It is a port of [ROSPlan](https://github.com/KCL-Planning/ROSPlan) to ROS 2
Humble.

These pages mirror the structure of the original ROSPlan documentation, updated for the
ROS 2 packages, node names, services and message types (`ros2plan_*`).

## Getting started

- [Installation](installation.md) — workspace setup, dependencies, building

## Tutorials

Work through these in order. Each one builds on the previous and all of them use the
same turtlebot waypoint-navigation example, whose files live in
[examples/turtlebot/](examples/turtlebot/).

1. [The Knowledge Base](tutorials/01-knowledge-base.md) — load a domain, add instances, facts and goals
2. [Problem Generation](tutorials/02-problem-generation.md) — turn the knowledge base into a PDDL problem
3. [Planning](tutorials/03-planning.md) — call POPF and get a plan
4. [Plan Execution](tutorials/04-plan-execution.md) — parse and dispatch the plan
5. [Simulated Actions](tutorials/05-simulated-actions.md) — run a plan without a real robot
6. [Action Interfaces](tutorials/06-action-interface.md) — connect actions to real components (Python and C++)
7. [The Sensing Interface](tutorials/07-sensing-interface.md) — keep the knowledge base in sync with sensors
8. [The rqt GUI](tutorials/08-rqt-gui.md) — inspect the system graphically

To run the whole pipeline end to end in one command, see the
[turtlebot demo script](examples/turtlebot/run_demo.sh) (covered in
[Plan Execution](tutorials/04-plan-execution.md#running-the-whole-pipeline)).

## Reference

- [Knowledge Base](reference/knowledge-base.md) — launch arguments and the full service list
- [Problem Interface](reference/problem-interface.md)
- [Planner Interface](reference/planner-interface.md)
- [Parsing Interface](reference/parsing-interface.md)
- [Plan Dispatch](reference/plan-dispatch.md)
- [Message types](reference/message-types.md) — `ros2plan_knowledge_msgs` and `ros2plan_dispatch_msgs`

## Architecture

```
                 ┌─────────────────────────────────────────────┐
                 │              Knowledge Base                  │
                 │  domain model + current state, as services   │
                 └───────────────▲───────────────┬─────────────┘
                          query  │               │ update
                                 │               ▼
   ┌─────────────────────────────┴───────────────────────────────┐
   │                       Planning System                        │
   │  Problem Interface → Planner Interface → Parser → Dispatcher │
   └───────────────────────────────┬─────────────────────────────┘
                                    │ ActionDispatch / ActionFeedback
                                    ▼
                 ┌─────────────────────────────────────────────┐
                 │              Action Interfaces               │
                 │   one per PDDL operator; execute the action  │
                 └─────────────────────────────────────────────┘
```

The Sensing Interface (optional) sits alongside the action interfaces and updates the
knowledge base from live topic and service data.

## Background

ROS2Plan targets **ROS 2 Humble** on **Ubuntu 22.04**, built with `ament_cmake` /
`ament_cmake_python` and C++17. For the details of how the framework was ported from
ROS 1, see the migration guide ([ros2_migration.md](../ros2_migration.md)).
