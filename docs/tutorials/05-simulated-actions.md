# Tutorial 5 — Simulated Actions

To develop and test a domain you usually do not want to drive real hardware. The
**simulated action interface** (`simulatedAction`) stands in for a real action: when an
action is dispatched, it waits for a configurable duration, then reports success or
failure with a configurable probability. The knowledge-base effects of the action are
applied exactly as they would be for a real interface, so the rest of the pipeline behaves
identically.

You run one `simulatedAction` node per PDDL operator.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `pddl_action_name` | — (required) | the operator this node handles |
| `action_duration` | `0.0` | seconds to simulate the action taking |
| `action_duration_stddev` | `0.0` | standard deviation of the duration (Gaussian) |
| `action_probability` | `1.0` | probability in `[0,1]` that the action succeeds |
| `knowledge_base` | `ros2plan_knowledge_base` | KB node name |
| `action_dispatch_topic` | `action_dispatch` | topic of incoming `ActionDispatch` |
| `action_feedback_topic` | `action_feedback` | topic for outgoing `ActionFeedback` |

## Start one node per operator

The turtlebot domain has four operators:

```bash
for action in undock dock localise goto_waypoint; do
  ros2 run ros2plan_planning_system simulatedAction --ros-args \
      -p knowledge_base:=ros2plan_knowledge_base \
      -p pddl_action_name:="$action" \
      -p action_duration:=0.5 \
      -p action_probability:=1.0 &
done
```

Each node prints when it is ready:

```
KCL: (ros2plan_interface_goto_waypoint) Ready to receive
```

## What a simulated action does

When the dispatcher publishes an `ActionDispatch` whose `name` matches `pddl_action_name`,
the node:

1. reads the action parameters from the message;
2. publishes `ActionFeedback` with status *action enabled*;
3. waits `action_duration` seconds (± `action_duration_stddev`);
4. draws a random number — with probability `action_probability` the action **succeeds**,
   otherwise it **fails**;
5. on success, applies the operator's effects to the knowledge base and reports *action
   achieved*; on failure, reports *action failed*.

The knowledge-base updates are what let the dispatcher confirm `goal_achieved` at the end
(see [Tutorial 4](04-plan-execution.md)).

## Simulating failure and replanning

Set `action_probability` below `1.0` to make actions fail at random. This is the easiest
way to exercise failure handling and replanning. For example, make `goto_waypoint` fail
half the time:

```bash
ros2 run ros2plan_planning_system simulatedAction --ros-args \
    -p knowledge_base:=ros2plan_knowledge_base \
    -p pddl_action_name:=goto_waypoint \
    -p action_duration:=0.5 \
    -p action_probability:=0.5 &
```

When an action fails, the dispatcher stops and `DispatchService` returns
`goal_achieved=False`. A higher-level controller would then regenerate the problem from
the current (partially executed) knowledge-base state and plan again — the same
problem-generation → planning → dispatch loop, now starting from where the robot actually
is.

## Using the launch file

There is also a launch file that wraps the node and its parameters:

```bash
ros2 launch ros2plan_planning_system simulated_action.launch.py \
    pddl_action_name:=goto_waypoint \
    action_duration:=1.0 \
    action_probability:=1.0
```

Note its `action_dispatch_topic` / `action_feedback_topic` defaults are namespaced under
the esterel dispatcher (`/ros2plan_plan_dispatcher/...`). When you drive the *simple*
dispatcher as in Tutorial 4, run the node with `ros2 run` so the topics default to the
root namespace and line up with what that dispatcher publishes.

---

Next: [Action Interfaces](06-action-interface.md) — replace the simulation with real
execution.

Reference: [Plan Dispatch](../reference/plan-dispatch.md).
