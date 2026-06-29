# Tutorial 1 — The Knowledge Base

The **knowledge base** (KB) is the centre of ROS2Plan. It holds two things:

- the **domain** — the planning model: types, predicates, functions and operators, parsed
  from a PDDL (or HDDL) domain file;
- the **state** — the current world: instances, facts, function values and goals.

Everything is exposed as ROS 2 services, so the rest of the system (and you) can query and
update it at runtime.

All commands below use the turtlebot example domain shipped with the framework.

## 1. Launch the knowledge base

The KB node is `knowledgeBase` in the `ros2plan_knowledge_base` package. Point it at a
domain file:

```bash
DOMAIN=$(ros2 pkg prefix ros2plan_planning_system)/share/ros2plan_planning_system/test/pddl/turtlebot/domain.pddl

ros2 run ros2plan_knowledge_base knowledgeBase --ros-args -p domain_path:="$DOMAIN"
```

or, equivalently, with the launch file (which also exposes `use_unknowns` and `kb_rate`):

```bash
ros2 launch ros2plan_knowledge_base ros2plan_knowledge_base.launch.py domain_path:="$DOMAIN"
```

It prints:

```
KCL: (ros2plan_knowledge_base) Parsing domain: .../turtlebot/domain.pddl.
KCL: (ros2plan_knowledge_base) Ready to receive
```

Leave it running and open a second terminal (remember to `source install/setup.bash`).
The node advertises its services under `/ros2plan_knowledge_base/`:

```bash
ros2 service list | grep ros2plan_knowledge_base
```

## 2. Inspect the domain

The domain services are read-only. None of them take request fields.

```bash
# Domain name
ros2 service call /ros2plan_knowledge_base/domain/name \
    ros2plan_knowledge_msgs/srv/GetDomainNameService "{}"
#   -> domain_name: turtlebot

# Types
ros2 service call /ros2plan_knowledge_base/domain/types \
    ros2plan_knowledge_msgs/srv/GetDomainTypeService "{}"

# Predicates
ros2 service call /ros2plan_knowledge_base/domain/predicates \
    ros2plan_knowledge_msgs/srv/GetDomainAttributeService "{}"

# Operators (the durative actions)
ros2 service call /ros2plan_knowledge_base/domain/operators \
    ros2plan_knowledge_msgs/srv/GetDomainOperatorService "{}"
```

## 3. Add the state: instances, facts and goals

State is built with two update services:

| Service | Type | Use |
|---|---|---|
| `/ros2plan_knowledge_base/update` | `KnowledgeUpdateService` | add/remove one `KnowledgeItem` |
| `/ros2plan_knowledge_base/update_array` | `KnowledgeUpdateServiceArray` | add/remove many at once |

The `update_type` field selects the operation:

| Value | Constant |
|---|---|
| `0` | `ADD_KNOWLEDGE` |
| `1` | `ADD_GOAL` |
| `2` | `REMOVE_KNOWLEDGE` |
| `3` | `REMOVE_GOAL` |
| `4` | `ADD_METRIC` |
| `5` | `REMOVE_METRIC` |

A [`KnowledgeItem`](../reference/message-types.md#knowledgeitem) describes one piece of
knowledge. Its `knowledge_type` is `0` (`INSTANCE`), `1` (`FACT`) or `2` (`FUNCTION`).

### Add an instance

```bash
# instance: kenny is a robot
ros2 service call /ros2plan_knowledge_base/update \
    ros2plan_knowledge_msgs/srv/KnowledgeUpdateService \
    "{update_type: 0, knowledge: {knowledge_type: 0, instance_type: 'robot', instance_name: 'kenny'}}"
```

### Add a fact

A fact is a predicate applied to instances. The `values` list maps each predicate
parameter (`key`) to an instance (`value`):

```bash
# fact: (robot_at kenny wp0)
ros2 service call /ros2plan_knowledge_base/update \
    ros2plan_knowledge_msgs/srv/KnowledgeUpdateService \
    "{update_type: 0, knowledge: {knowledge_type: 1, attribute_name: 'robot_at',
      values: [{key: 'v', value: 'kenny'}, {key: 'wp', value: 'wp0'}]}}"
```

### Add a goal

Same `KnowledgeItem`, but with `update_type: 1` (`ADD_GOAL`):

```bash
# goal: (visited wp1)
ros2 service call /ros2plan_knowledge_base/update \
    ros2plan_knowledge_msgs/srv/KnowledgeUpdateService \
    "{update_type: 1, knowledge: {knowledge_type: 1, attribute_name: 'visited',
      values: [{key: 'wp', value: 'wp1'}]}}"
```

## 4. Query the state

| Service | Type | Request |
|---|---|---|
| `state/instances` | `GetInstanceService` | `type_name` |
| `state/propositions` | `GetAttributeService` | `predicate_name` |
| `state/functions` | `GetAttributeService` | `predicate_name` |
| `state/goals` | `GetAttributeService` | `predicate_name` (empty = all) |

```bash
# all instances of type 'robot'
ros2 service call /ros2plan_knowledge_base/state/instances \
    ros2plan_knowledge_msgs/srv/GetInstanceService "{type_name: 'robot'}"

# all true 'robot_at' facts
ros2 service call /ros2plan_knowledge_base/state/propositions \
    ros2plan_knowledge_msgs/srv/GetAttributeService "{predicate_name: 'robot_at'}"

# all current goals
ros2 service call /ros2plan_knowledge_base/state/goals \
    ros2plan_knowledge_msgs/srv/GetAttributeService "{predicate_name: ''}"
```

> The query field is **`predicate_name`** (not `attribute_name`). Using the wrong field
> name makes `ros2 service call` fail with *"object has no attribute …"*.

## 5. Load a whole state at once

Adding facts one by one is tedious. The example loader
[load_state.py](../examples/turtlebot/load_state.py) batches the turtlebot initial state
and goals into a single `update_array` call. It is a good template for your own loaders —
note how it creates a *satellite node* purely for the blocking service call:

```python
self._svc_node = rclpy.create_node('state_loader_svc')
self._client = self._svc_node.create_client(
    KnowledgeUpdateServiceArray, f'/{self._kb}/update_array')
future = self._client.call_async(req)
rclpy.spin_until_future_complete(self._svc_node, future, timeout_sec=10.0)
```

Run it against the KB you started in step 1:

```bash
python3 docs/examples/turtlebot/load_state.py --waypoints 5 --robot kenny
```

```
[turtlebot_state_loader]: Loaded 10 knowledge items (5 waypoints, robot: kenny)
[turtlebot_state_loader]: Set 6 goals
```

This loads 5 waypoint instances, the robot `kenny`, the facts
`robot_at(kenny, wp0)`, `docked(kenny)`, `dock_at(wp0)`, `visited(wp0)`, and the goals
`visited(wp0..wp4)` and `docked(kenny)`.

> **Why `visited(wp0)` is a fact and a goal.** The robot starts on `wp0`, but a
> `goto_waypoint` action can never target the waypoint the robot is already on. So `wp0`
> is seeded as already visited; only `wp1..wp4` actually have to be reached. If you add
> *all* `visited` predicates as facts, the goal is already satisfied and the planner
> returns an empty plan.

## Clearing the state

To wipe all state (instances, facts and goals) without restarting the node:

```bash
ros2 service call /ros2plan_knowledge_base/clear std_srvs/srv/Empty "{}"
```

---

With a domain loaded and a state in the KB, the next step is to turn that state into a
PDDL problem: [Problem Generation](02-problem-generation.md).

For the complete service list and launch arguments, see the
[Knowledge Base reference](../reference/knowledge-base.md).
