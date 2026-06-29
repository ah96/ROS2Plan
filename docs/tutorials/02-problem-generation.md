# Tutorial 2 — Problem Generation

A planner needs a PDDL **problem**: the objects, the initial state and the goal. The
**Problem Interface** builds this automatically from whatever is currently in the
knowledge base, so you never write a problem file by hand.

This tutorial assumes the knowledge base from
[Tutorial 1](01-knowledge-base.md) is running with the turtlebot state loaded.

## 1. Start the Problem Interface

The node is `problemInterface` in `ros2plan_planning_system`. Its key parameters:

| Parameter | Meaning |
|---|---|
| `knowledge_base` | node name of the KB to query (default `ros2plan_knowledge_base`) |
| `problem_path` | file the generated problem is written to |
| `planning_language` | `PDDL`, `RDDL` or `PPDDL` |
| `problem_topic` | topic the problem string is published on (default `problem_instance`) |

```bash
ros2 run ros2plan_planning_system problemInterface --ros-args \
    -p knowledge_base:=ros2plan_knowledge_base \
    -p problem_path:=/tmp/problem.pddl \
    -p planning_language:=PDDL
```

```
KCL: (ros2plan_problem_interface) Ready to receive.
```

> If you start it with no parameters it aborts with *"Parameters problem_path and
> planning_language are both undefined."* — at least one of those must be set.

## 2. Generate the problem

Triggering generation is a single empty service call. The service lives in the **root**
namespace as `/problem_generation_server`:

```bash
ros2 service call /problem_generation_server std_srvs/srv/Empty "{}"
```

The Problem Interface queries the KB, writes the problem to `problem_path`, and publishes
it as a `std_msgs/String` on `/problem_instance` (with `transient_local` QoS, so a planner
that starts later still receives it):

```
KCL: (ros2plan_problem_interface) (problem.pddl) Generating problem file.
KCL: (ros2plan_problem_interface) (problem.pddl) The problem was generated.
```

## 3. Inspect the generated problem

```bash
cat /tmp/problem.pddl
```

For the turtlebot state from Tutorial 1 you get:

```pddl
(define (problem task)
(:domain turtlebot)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
    kenny - robot
)
(:init
    (robot_at kenny wp0)
    (visited wp0)
    (docked kenny)
    (dock_at wp0)
)
(:goal (and
    (visited wp0)
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (visited wp4)
    (docked kenny)
))
)
```

You can also read it off the topic:

```bash
ros2 topic echo /problem_instance --once
```

Notice the correspondence with the KB: every `(:object …)` comes from an `INSTANCE`, every
`(:init …)` line from a `FACT`, and every `(:goal …)` line from an `ADD_GOAL` update.
If the `(:goal …)` block were already entailed by `(:init …)`, the planner in the next
tutorial would have nothing to do.

## How it fits together

```
Knowledge Base ──query──> Problem Interface ──writes──> /tmp/problem.pddl
                                          └──publishes──> /problem_instance
```

The planner can take the problem either from the file (`problem_path`) or from the topic
(`use_problem_topic:=true`). The next tutorial uses the topic.

---

Next: [Planning](03-planning.md) — run POPF on this problem.

Reference: [Problem Interface](../reference/problem-interface.md).
