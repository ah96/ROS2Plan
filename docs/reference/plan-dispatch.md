# Reference — Plan Dispatch

Package `ros2plan_planning_system`. Sends each action of a parsed plan to the action
interface that handles it, in order, and waits for feedback.

## Executables

| Executable | Consumes | Notes |
|---|---|---|
| `pddl_simple_plan_dispatcher` | `CompletePlan` | totally ordered, sequential |
| `pddl_esterel_plan_dispatcher` | `EsterelPlan` | partial order; concurrent where the plan allows |
| `online_plan_dispatcher` | — | interleaves planning and execution |

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `knowledge_base` | `ros2plan_knowledge_base` | KB node name |
| `plan_topic` | `complete_plan` | topic the plan is read from |
| `action_dispatch_topic` | `action_dispatch` | topic `ActionDispatch` is published on |
| `action_feedback_topic` | `action_feedback` | topic `ActionFeedback` is read from |

## Services and actions (root namespace)

| Endpoint | Type | Behaviour |
|---|---|---|
| `/dispatch_plan` | `ros2plan_dispatch_msgs/srv/DispatchService` | dispatch the current plan; blocks until done |
| `/dispatch_plan_action` | `ros2plan_dispatch_msgs/action/NonBlockingDispatch` | non-blocking dispatch with feedback |
| `/cancel_dispatch` | `std_srvs/srv/Empty` | cancel an in-progress dispatch |

`DispatchService` returns `success` (dispatch ran to completion without an action failing)
and `goal_achieved` (every goal proposition is satisfied in the KB afterwards).

> Call `/dispatch_plan` with type `DispatchService`, not `Empty` — calling it with the
> wrong type makes the CLI wait forever for a service that does not exist.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `complete_plan` | `CompletePlan` / `EsterelPlan` | subscribed (`transient_local`) |
| `action_dispatch` | `ros2plan_dispatch_msgs/ActionDispatch` | published — actions to execute |
| `action_feedback` | `ros2plan_dispatch_msgs/ActionFeedback` | subscribed — results from action interfaces |

## Example

```bash
ros2 run ros2plan_planning_system pddl_simple_plan_dispatcher --ros-args \
    -p knowledge_base:=ros2plan_knowledge_base

ros2 service call /dispatch_plan ros2plan_dispatch_msgs/srv/DispatchService "{}"
```

See [Tutorial 4](../tutorials/04-plan-execution.md).
