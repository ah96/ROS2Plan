# Tutorial 4 — Plan Execution

A planner produces plain text. Executing it takes two more nodes:

- the **Plan Parser** turns the planner's output into a structured `CompletePlan`
  (a sequence of `ActionDispatch` messages);
- the **Plan Dispatcher** sends each action, in order and respecting the temporal
  schedule, to the action interface responsible for it, and waits for feedback.

This tutorial continues from [Tutorial 3](03-planning.md): the KB, Problem Interface and
Planner Interface are running and a plan has been produced on `/planner_output`.

## 1. Parse the plan

The simple parser is `pddl_simple_plan_parser`. It subscribes to `planner_output` and
publishes a `CompletePlan` on `complete_plan` (both with `transient_local` QoS).

```bash
ros2 run ros2plan_planning_system pddl_simple_plan_parser --ros-args \
    -p knowledge_base:=ros2plan_knowledge_base
```

Trigger parsing with the root-namespace service:

```bash
ros2 service call /parse_plan std_srvs/srv/Empty "{}"
```

Inspect the parsed plan:

```bash
ros2 topic echo /complete_plan --once
```

> There is also an **esterel** parser (`pddl_esterel_plan_parser`) that produces a partial
> order plan (an `EsterelPlan`) for the esterel dispatcher. The simple parser/dispatcher
> pair used here executes the plan as a totally ordered sequence, which is all the
> turtlebot example needs.

## 2. Start the action interfaces

The dispatcher sends each action to whichever node handles that PDDL operator. With no
action interfaces running, the dispatcher has nowhere to send actions. For this example we
use *simulated* action interfaces — one per operator — which just wait and report success.
This is covered in detail in [Tutorial 5](05-simulated-actions.md); for now:

```bash
for action in undock dock localise goto_waypoint; do
  ros2 run ros2plan_planning_system simulatedAction --ros-args \
      -p knowledge_base:=ros2plan_knowledge_base \
      -p pddl_action_name:="$action" \
      -p action_duration:=0.5 \
      -p action_probability:=1.0 &
done
```

## 3. Dispatch the plan

The simple dispatcher is `pddl_simple_plan_dispatcher`. It subscribes to `complete_plan`
and exposes `/dispatch_plan`:

```bash
ros2 run ros2plan_planning_system pddl_simple_plan_dispatcher --ros-args \
    -p knowledge_base:=ros2plan_knowledge_base
```

Dispatch — note the service **type** is `DispatchService`, not `Empty`:

```bash
ros2 service call /dispatch_plan ros2plan_dispatch_msgs/srv/DispatchService "{}"
```

You will see each action dispatched in order, and the knowledge base updated as the
at-start and at-end effects are applied:

```
KCL: (ros2plan_simple_plan_dispatcher) Dispatching action [0, undock(kenny, wp0), ...]
KCL: (ros2plan_simple_plan_dispatcher) Dispatching action [1, localise(kenny), ...]
...
KCL: (ros2plan_simple_plan_dispatcher) Dispatching action [10, dock(kenny, wp0), ...]
KCL: (ros2plan_simple_plan_dispatcher) Dispatch complete.
```

The service returns when dispatch finishes:

```
ros2plan_dispatch_msgs.srv.DispatchService_Response(success=True, goal_achieved=True)
```

`goal_achieved=True` means the dispatcher checked the goal against the knowledge base and
found every goal proposition satisfied.

## 4. Confirm the result

```bash
ros2 service call /ros2plan_knowledge_base/state/propositions \
    ros2plan_knowledge_msgs/srv/GetAttributeService "{predicate_name: 'visited'}"

ros2 service call /ros2plan_knowledge_base/state/propositions \
    ros2plan_knowledge_msgs/srv/GetAttributeService "{predicate_name: 'docked'}"
```

All of `visited(wp0)…visited(wp4)` are present and `docked(kenny)` is true — the mission is
complete.

## Running the whole pipeline

Doing every step by hand is good for understanding the system, but tedious. The example
script [run_demo.sh](../examples/turtlebot/run_demo.sh) runs all of it — knowledge base,
state loading, problem generation, planning, parsing, the four simulated actions and
dispatch — in a single terminal, and prints the final knowledge-base state:

```bash
bash docs/examples/turtlebot/run_demo.sh --waypoints 5
```

```
[1/6] Starting the knowledge base...
[2/6] Loading initial state and goals...
[3/6] Starting problem / planner / parser / dispatcher and action interfaces...
[4/6] Generating the problem and planning (POPF)...
[5/6] Parsing and dispatching the plan...
ros2plan_dispatch_msgs.srv.DispatchService_Response(success=True, goal_achieved=True)
[6/6] Final knowledge-base state:
  visited waypoints:
    wp0
    wp1
    wp2
    wp3
    wp4
  docked:
    robot value='kenny'
```

The script is the best reference for exactly how the nodes are wired together, including
the parameters each one needs.

---

Next: [Simulated Actions](05-simulated-actions.md) — how the simulated action interfaces
work, and how to test failure and replanning.

Reference: [Parsing Interface](../reference/parsing-interface.md),
[Plan Dispatch](../reference/plan-dispatch.md).
