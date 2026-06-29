# Tutorial 8 — The rqt GUI

`ros2plan_rqt` provides graphical plugins for driving and inspecting a running ROS2Plan
system from inside `rqt`. They are convenient alternatives to the `ros2 service call`
commands used in the previous tutorials.

## Launch rqt

With a ROS2Plan system running (at least the knowledge base), start rqt:

```bash
rqt --force-discover
```

`--force-discover` makes rqt re-scan for plugins; you only need it the first time after a
build. The plugins appear under **Plugins → ROS2Plan**.

## The plugins

| Plugin | What it does |
|---|---|
| **Plan Dispatcher** | Trigger problem generation, planning, parsing and dispatch with buttons instead of service calls, and watch dispatch progress. |
| **Action Dispatcher** | Dispatch a single PDDL action by hand — useful for testing one action interface in isolation. |
| **Problem Viewer** | Show the current knowledge-base state (instances, facts, goals) as it would appear in the generated problem. |
| **Esterel Plan Viewer** | Render an esterel (partial-order) plan as a graph, so you can see the temporal/causal structure of the plan. |
| **Action Config** | Edit action-interface configuration (the YAML from [Tutorial 6](06-action-interface.md)) in a form. |

Each plugin holds its own ROS 2 node and makes service calls to the knowledge base and the
planning-system nodes, so the names you started those nodes with must match (the defaults
`ros2plan_knowledge_base`, etc., are assumed).

## Typical use

A common workflow during domain development:

1. **Problem Viewer** — load a state into the KB (Tutorial 1) and confirm the instances,
   facts and goals are what you expect.
2. **Plan Dispatcher** — generate the problem and run the planner with one click; if it
   fails, the problem is usually wrong, so go back to step 1.
3. **Esterel Plan Viewer** — inspect the plan structure when using the esterel
   parser/dispatcher.
4. **Action Dispatcher** — fire individual actions to check that each action interface
   behaves correctly before running a full dispatch.

## If the plugins do not appear

- Make sure the workspace is sourced in the terminal you launched `rqt` from.
- Run `rqt --force-discover` once after building `ros2plan_rqt`.
- Check the package built: `ros2 pkg list | grep ros2plan_rqt`.
- ROS 2 Humble uses PyQt5; if rqt itself fails to start, confirm `python3-pyqt5` is
  installed.

---

This is the last tutorial. For exact node parameters, service names and message fields,
see the [reference pages](../index.md#reference).
