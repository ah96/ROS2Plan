# Reference — Planner Interface

Package `ros2plan_planning_system`, default node name `ros2plan_planner_interface`. Runs an
external planner as a subprocess and republishes the raw plan.

## Executables

| Executable | Planner |
|---|---|
| `popf_planner_interface` | POPF (temporal; bundled) |
| `ff_planner_interface` | FF |
| `fd_planner_interface` | Fast Downward |
| `tfd_planner_interface` | Temporal Fast Downward |
| `metricff_planner_interface` | Metric-FF |
| `lpg_planner_interface` | LPG |
| `chimp_planner_interface` | CHIMP (HTN; bundled) |
| `smt_planner_interface` | SMT-based |
| `online_planner_interface` | online/contingent |

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `planner_command` | `timeout 60 common/bin/popf -n DOMAIN PROBLEM` | command to run; `DOMAIN`/`PROBLEM` are replaced with the file paths |
| `domain_path` | — | PDDL domain file |
| `problem_path` | — | problem file (read, or written when `use_problem_topic`) |
| `data_path` | `common/` | planner working directory (`plan.pddl` is written here) |
| `use_problem_topic` | `false` | take the problem from `problem_topic` instead of `problem_path` |
| `problem_topic` | `problem_instance` | topic to read the problem from |
| `planner_topic` | `planner_output` | topic the raw plan is published on |

## Triggers (root namespace)

| Endpoint | Type | Reads settings from |
|---|---|---|
| `/planning_server` | `std_srvs/srv/Empty` | the node **parameters** |
| `/planning_server_params` | `ros2plan_dispatch_msgs/srv/PlanningService` | the **request** fields |
| `/start_planning` | `ros2plan_dispatch_msgs/action/Plan` | the **action goal** fields |

For interactive use prefer `/planning_server`. The `/start_planning` action takes its
domain/problem/command from the goal, so an empty goal produces an empty command and no
plan — see [Tutorial 3](../tutorials/03-planning.md).

## Topics

| Topic | Type | Direction |
|---|---|---|
| `planner_output` | `std_msgs/String` | published (`transient_local`) — the raw plan |
| `problem_instance` | `std_msgs/String` | subscribed when `use_problem_topic:=true` |

## Example

```bash
DOMAIN=$(ros2 pkg prefix ros2plan_planning_system)/share/ros2plan_planning_system/test/pddl/turtlebot/domain.pddl
POPF=$(ros2 pkg prefix ros2plan_planning_system)/share/ros2plan_planning_system/common/bin/popf

ros2 run ros2plan_planning_system popf_planner_interface --ros-args \
    -p use_problem_topic:=true -p domain_path:="$DOMAIN" \
    -p problem_path:=/tmp/problem.pddl -p data_path:=/tmp/ \
    -p planner_command:="timeout 30 $POPF DOMAIN PROBLEM"

ros2 service call /planning_server std_srvs/srv/Empty "{}"
```
