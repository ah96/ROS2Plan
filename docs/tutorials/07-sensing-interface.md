# Tutorial 7 — The Sensing Interface

So far the knowledge base has only been updated by action effects. On a real robot you also
want predicates to reflect live sensor data — whether the robot is docked, where it is,
whether a door is open. The **sensing interface** does this: it subscribes to topics and
calls services, evaluates a short Python expression on each message, and writes the
resulting truth values back into the knowledge base.

## How it works

You give the sensing interface a YAML config describing, for each predicate:

- the **topic** or **service** that carries the relevant data;
- the **message/service type**;
- the predicate **parameters** it applies to (fully instantiated, or `'*'` wildcard);
- an **operation** — a Python expression that turns the message into a truth value.

The node runs at `main_rate` Hz (default 10 Hz) and calls the KB's `update_array` service to
keep the facts in sync.

## Config format

```yaml
# Optional: Python files providing helper functions used by 'operation' fields.
functions:
    - $(find ros2plan_sensing_interface)/example.py

# Predicates sensed from topics.
topics:
    docked:
        params:
            - kenny                          # fully instantiated parameter(s)
        topic: /mobile_base/sensors/core
        msg_type: kobuki_msgs/SensorState
        operation: msg.charger != msg.DISCHARGING   # -> bool

    robot_at:
        params:
            - kenny
            - '*'                            # wildcard: matched against waypoint instances
        topic: /amcl_pose
        msg_type: geometry_msgs/PoseWithCovarianceStamped
        # No 'operation' here: a function in the 'functions' file computes the waypoint.

    is_busy:
        params:
            - '*'
        topic: /isbusy_mock
        msg_type: diagnostic_msgs/KeyValue
        operation: '[(msg.key, msg.value == "yes")]'   # -> list of (instance, bool)

# Predicates sensed by polling a service.
services:
    docked:
        params:
            - kenny
        service: /dock_query
        srv_type: std_srvs/SetBool
        time_between_calls: 10               # seconds
        request: req.data = True             # build the request
        operation: res.success               # -> bool
```

The `operation` is evaluated with the incoming message bound to `msg` (for topics) or the
response bound to `res` (for services). It returns either:

- a single `bool`, when every parameter is instantiated; or
- a list of `(instance, bool)` tuples, when a parameter is the `'*'` wildcard — one truth
  value per matching instance.

Helper functions listed under `functions` are imported and made available to the
`operation` expressions, which is how more complex sensing (e.g. nearest-waypoint
computation) is done without inlining it in YAML.

## Run it

```bash
ros2 run ros2plan_sensing_interface sensing_interface --ros-args \
    -p config_path:=/path/to/sensing.yaml
```

```
KCL: (ros2plan_sensing_interface) Ready.
```

The node subscribes to each configured topic, sets up timed clients for each configured
service, and from then on updates the knowledge base whenever sensor data changes. You can
watch the effect with the state queries from [Tutorial 1](01-knowledge-base.md):

```bash
ros2 service call /ros2plan_knowledge_base/state/propositions \
    ros2plan_knowledge_msgs/srv/GetAttributeService "{predicate_name: 'docked'}"
```

A complete template ships at `ros2plan_sensing_interface/example.yaml`.

## Sensing vs. action effects

Both update the knowledge base, but they answer different questions:

- **action effects** record what the plan *intends* to be true after an action;
- **sensing** records what is *actually* true according to the robot's sensors.

When they disagree — an action reported success but the sensor says otherwise — that is the
signal to regenerate the problem and replan from the real state.

---

Next: [The rqt GUI](08-rqt-gui.md) — inspect all of this graphically.

Reference: [Knowledge Base](../reference/knowledge-base.md).
