# Reference — Parsing Interface

Package `ros2plan_planning_system`, default node name `ros2plan_parsing_interface`. Turns
the planner's raw output into structured dispatch messages.

## Executables

| Executable | Output |
|---|---|
| `pddl_simple_plan_parser` | totally ordered `CompletePlan` (a list of `ActionDispatch`) |
| `pddl_esterel_plan_parser` | partially ordered `EsterelPlan` |

Pair the parser with the matching dispatcher: simple parser → simple dispatcher, esterel
parser → esterel dispatcher.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `knowledge_base` | `ros2plan_knowledge_base` | KB node name |
| `planner_topic` | `planner_output` | topic the raw plan is read from |
| `plan_topic` | `complete_plan` | topic the parsed plan is published on |

## Services (root namespace)

| Service | Type | Behaviour |
|---|---|---|
| `/parse_plan` | `std_srvs/srv/Empty` | parse the plan most recently received on `planner_topic` |
| `/parse_plan_from_file` | `ros2plan_dispatch_msgs/srv/ParsingService` | parse a plan from a file path in the request |

## Topics

| Topic | Type | Direction |
|---|---|---|
| `planner_output` | `std_msgs/String` | subscribed (`transient_local`) |
| `complete_plan` | `ros2plan_dispatch_msgs/CompletePlan` (simple) / `EsterelPlan` (esterel) | published (`transient_local`) |

## Example

```bash
ros2 run ros2plan_planning_system pddl_simple_plan_parser --ros-args \
    -p knowledge_base:=ros2plan_knowledge_base

ros2 service call /parse_plan std_srvs/srv/Empty "{}"
```

See [Tutorial 4](../tutorials/04-plan-execution.md).
