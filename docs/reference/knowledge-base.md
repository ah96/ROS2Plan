# Reference — Knowledge Base

Node: `knowledgeBase` (package `ros2plan_knowledge_base`), default node name
`ros2plan_knowledge_base`. All services are advertised under
`/ros2plan_knowledge_base/`.

## Launch / parameters

| Parameter | Default | Meaning |
|---|---|---|
| `domain_path` | turtlebot domain | PDDL/HDDL domain file to parse |
| `problem_path` | `""` | optional PDDL problem file to seed the initial state |
| `use_unknowns` | `false` | allow unknown (`?`) predicate values for conditional planning |
| `kb_rate` | `100.0` | Hz at which timed initial literals are processed |

```bash
ros2 run ros2plan_knowledge_base knowledgeBase --ros-args -p domain_path:=/path/domain.pddl
# or
ros2 launch ros2plan_knowledge_base ros2plan_knowledge_base.launch.py domain_path:=/path/domain.pddl
```

## Domain services (read-only)

| Service | Type |
|---|---|
| `domain/name` | `GetDomainNameService` |
| `domain/types` | `GetDomainTypeService` |
| `domain/predicates` | `GetDomainAttributeService` |
| `domain/predicate_details` | `GetDomainPredicateDetailsService` |
| `domain/functions` | `GetDomainAttributeService` |
| `domain/operators` | `GetDomainOperatorService` |
| `domain/operator_details` | `GetDomainOperatorDetailsService` |

## State services (read-only)

| Service | Type | Request field |
|---|---|---|
| `state/instances` | `GetInstanceService` | `type_name` |
| `state/propositions` | `GetAttributeService` | `predicate_name` |
| `state/functions` | `GetAttributeService` | `predicate_name` |
| `state/goals` | `GetAttributeService` | `predicate_name` (empty = all) |
| `state/timed_knowledge` | `GetAttributeService` | `predicate_name` |
| `state/metric` | `GetMetricService` | — |

> The request field for proposition/function/goal queries is **`predicate_name`**.

## Update services

| Service | Type | Use |
|---|---|---|
| `update` | `KnowledgeUpdateService` | add/remove one `KnowledgeItem` |
| `update_array` | `KnowledgeUpdateServiceArray` | add/remove many at once |
| `update_sensed_predicates` | `KnowledgeUpdateServiceArray` | used by the sensing interface |
| `update_constraints_oneof` | `KnowledgeUpdateServiceArray` | one-of constraints |
| `query_state` | `KnowledgeQueryService` | test whether a set of items currently holds |
| `clear` | `std_srvs/srv/Empty` | wipe all instances, facts and goals |

`update_type` values (both update services):

| Value | Constant |
|---|---|
| `0` | `ADD_KNOWLEDGE` |
| `1` | `ADD_GOAL` |
| `2` | `REMOVE_KNOWLEDGE` |
| `3` | `REMOVE_GOAL` |
| `4` | `ADD_METRIC` |
| `5` | `REMOVE_METRIC` |

All service and message types are in the `ros2plan_knowledge_msgs` package; see
[Message types](message-types.md).

## Service namespace

The knowledge base advertises its services with a private (`~/`) prefix, so they resolve
to `/ros2plan_knowledge_base/<name>`. The planning-system nodes that talk to the KB take a
`knowledge_base` parameter (default `ros2plan_knowledge_base`) and build these names
accordingly. This is different from the planning-system trigger services
(`/problem_generation_server`, `/planning_server`, `/parse_plan`, `/dispatch_plan`), which
live in the **root** namespace.
