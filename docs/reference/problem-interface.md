# Reference — Problem Interface

Node: `problemInterface` (package `ros2plan_planning_system`), default node name
`ros2plan_problem_interface`. Generates a PDDL/RDDL/PPDDL problem from the current
knowledge-base state.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `knowledge_base` | `ros2plan_knowledge_base` | KB node name to query |
| `problem_path` | `""` | file the problem is written to |
| `planning_language` | `""` | `PDDL`, `RDDL` or `PPDDL` |
| `problem_topic` | `problem_instance` | topic the problem string is published on |

At least one of `problem_path` / `planning_language` must be set, or the node aborts.

## Services (root namespace)

| Service | Type | Behaviour |
|---|---|---|
| `/problem_generation_server` | `std_srvs/srv/Empty` | generate using the node parameters |
| `/problem_generation_server_params` | `ros2plan_dispatch_msgs/srv/ProblemService` | generate using fields supplied in the request |

## Topics

| Topic | Type | Direction |
|---|---|---|
| `problem_instance` | `std_msgs/String` | published (`transient_local`) — the generated problem |

## Example

```bash
ros2 run ros2plan_planning_system problemInterface --ros-args \
    -p knowledge_base:=ros2plan_knowledge_base \
    -p problem_path:=/tmp/problem.pddl \
    -p planning_language:=PDDL

ros2 service call /problem_generation_server std_srvs/srv/Empty "{}"
```

See [Tutorial 2](../tutorials/02-problem-generation.md).
