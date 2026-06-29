# Tutorial 3 — Planning

The **Planner Interface** wraps an external planner. It takes the domain and the problem,
runs the planner as a subprocess, and republishes the raw plan on a topic for the parser.
ROS2Plan ships several planner interfaces; this tutorial uses **POPF**, a temporal planner
bundled in `ros2plan_dependencies`.

This assumes the knowledge base ([Tutorial 1](01-knowledge-base.md)) and the Problem
Interface ([Tutorial 2](02-problem-generation.md)) are running and a problem has been
generated.

## 1. Start the POPF planner interface

The executable is `popf_planner_interface`. Its parameters:

| Parameter | Meaning |
|---|---|
| `planner_command` | shell command to run, with `DOMAIN` / `PROBLEM` placeholders |
| `domain_path` | PDDL domain file |
| `problem_path` | where to write the problem the planner reads |
| `data_path` | working directory for the planner (it writes `plan.pddl` here) |
| `use_problem_topic` | if `true`, take the problem from `/problem_instance` instead of a file |
| `planner_topic` | topic the raw plan is published on (default `planner_output`) |

```bash
DOMAIN=$(ros2 pkg prefix ros2plan_planning_system)/share/ros2plan_planning_system/test/pddl/turtlebot/domain.pddl
POPF=$(ros2 pkg prefix ros2plan_planning_system)/share/ros2plan_planning_system/common/bin/popf

ros2 run ros2plan_planning_system popf_planner_interface --ros-args \
    -p use_problem_topic:=true \
    -p domain_path:="$DOMAIN" \
    -p problem_path:=/tmp/problem.pddl \
    -p data_path:=/tmp/ \
    -p planner_command:="timeout 30 $POPF DOMAIN PROBLEM"
```

`DOMAIN` and `PROBLEM` in `planner_command` are **literal placeholder tokens**: the planner
interface substitutes them with `domain_path` and `problem_path` before running the
command. The output is redirected to `plan.pddl` in `data_path`.

## 2. Run the planner

There are two ways to trigger planning, and the difference matters:

| Trigger | Type | Where it reads its settings |
|---|---|---|
| `/planning_server` | `std_srvs/srv/Empty` | from the node **parameters** above |
| `/start_planning` | `ros2plan_dispatch_msgs/action/Plan` | from the **action goal** fields |

For an interactive run, use the **service** — it uses the parameters you just set:

```bash
ros2 service call /planning_server std_srvs/srv/Empty "{}"
```

```
KCL: (ros2plan_planner_interface) Problem received.
KCL: (ros2plan_planner_interface) Planning complete
```

> **Common pitfall.** Calling the `/start_planning` action with an empty goal (`"{}"`)
> sends empty `domain_path` / `problem_path` / `planner_command` fields, so the planner
> runs an empty command and reports *"Plan was unsolvable."* Use the action only when you
> fill in the goal fields explicitly; otherwise prefer `/planning_server`.

## 3. The plan

POPF prints the plan and the planner interface publishes it on
`/planner_output` (`std_msgs/String`, `transient_local` QoS). For the turtlebot problem:

```
0.000: (undock kenny wp0)              [2.000]
2.001: (localise kenny)                [1.000]
3.002: (goto_waypoint kenny wp0 wp1)   [1.000]
4.003: (goto_waypoint kenny wp1 wp0)   [1.000]
5.004: (goto_waypoint kenny wp0 wp2)   [1.000]
6.005: (goto_waypoint kenny wp2 wp0)   [1.000]
7.006: (goto_waypoint kenny wp0 wp3)   [1.000]
8.007: (goto_waypoint kenny wp3 wp0)   [1.000]
9.008: (goto_waypoint kenny wp0 wp4)   [1.000]
10.009: (goto_waypoint kenny wp4 wp0)  [1.000]
11.010: (dock kenny wp0)               [3.000]
```

Each line is `start_time: (action args) [duration]`. The robot undocks, localises, makes a
round trip to each unvisited waypoint, and docks again.

Check the published plan directly:

```bash
ros2 topic echo /planner_output --once
```

## Other planners

The same pattern works for the other interfaces — just change the executable and the
`planner_command`:

| Executable | Planner |
|---|---|
| `popf_planner_interface` | POPF (temporal) |
| `ff_planner_interface` | FF |
| `fd_planner_interface` | Fast Downward |
| `tfd_planner_interface` | Temporal Fast Downward |
| `chimp_planner_interface` | CHIMP (HTN) |
| `online_planner_interface` | online/contingent planning |

Only POPF and CHIMP are bundled; the others expect the planner binary to be installed and
referenced from `planner_command`.

---

Next: [Plan Execution](04-plan-execution.md) — parse and dispatch this plan.

Reference: [Planner Interface](../reference/planner-interface.md).
