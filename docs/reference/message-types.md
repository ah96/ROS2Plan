# Reference — Message Types

ROS2Plan defines its interfaces in two packages: `ros2plan_knowledge_msgs` (everything to
do with the knowledge base) and `ros2plan_dispatch_msgs` (planning and dispatch).

```bash
ros2 interface list | grep ros2plan          # all types
ros2 interface show ros2plan_knowledge_msgs/msg/KnowledgeItem
```

## ros2plan_knowledge_msgs

### KnowledgeItem

The central message — one piece of knowledge (an instance, a fact, a function or a goal).

```
uint8 INSTANCE=0  FACT=1  FUNCTION=2  EXPRESSION=3  INEQUALITY=4
uint8 knowledge_type

builtin_interfaces/Time initial_time   # when this becomes true (TILs/TIFs)
bool is_negative                       # the knowledge is explicitly false

# INSTANCE
string instance_type
string instance_name

# FACT / FUNCTION
string attribute_name                  # predicate or function name
diagnostic_msgs/KeyValue[] values      # parameter -> instance bindings

# FUNCTION
float64 function_value
uint8 assign_op                        # AP_ASSIGN=0, AP_INCREASE=1, ...

# EXPRESSION / INEQUALITY
string optimization
ros2plan_knowledge_msgs/ExprComposite expr
ros2plan_knowledge_msgs/DomainInequality ineq
```

### Other messages

| Message | Purpose |
|---|---|
| `DomainFormula` | a predicate/function/operator signature (name + typed parameters) |
| `DomainOperator` | a full operator: formula + conditions + effects |
| `DomainAssignment`, `DomainInequality` | numeric effects and constraints |
| `ExprBase`, `ExprComposite` | numeric expression trees |
| `ProbabilisticEffect` | probabilistic action effects (PPDDL/RDDL) |
| `StatusUpdate` | knowledge-base change notifications |

### Services

| Service | Request → Response |
|---|---|
| `GetDomainNameService` | — → `domain_name` |
| `GetDomainTypeService` | — → types |
| `GetDomainAttributeService` | — → `DomainFormula[] items` (predicates or functions) |
| `GetDomainPredicateDetailsService` / `GetDomainOperatorDetailsService` | name → details |
| `GetDomainOperatorService` | — → `DomainFormula[] operators` |
| `GetInstanceService` | `type_name` → `string[] instances` |
| `GetAttributeService` | `predicate_name` → `KnowledgeItem[] attributes` |
| `GetMetricService` | — → the optimization metric |
| `KnowledgeQueryService` | `KnowledgeItem[]` → does it hold? |
| `KnowledgeUpdateService` | `update_type`, `KnowledgeItem` → `success` |
| `KnowledgeUpdateServiceArray` | `update_type[]`, `KnowledgeItem[]` → `success` |

`update_type`: `ADD_KNOWLEDGE=0`, `ADD_GOAL=1`, `REMOVE_KNOWLEDGE=2`, `REMOVE_GOAL=3`,
`ADD_METRIC=4`, `REMOVE_METRIC=5`.

## ros2plan_dispatch_msgs

### Messages

```
# ActionDispatch — one action to execute
int32 action_id
int32 plan_id
string name
diagnostic_msgs/KeyValue[] parameters   # PDDL parameter -> instance
float32 duration
float32 dispatch_time
```

```
# ActionFeedback — result of an action
int32 ACTION_ENABLED=1  ACTION_SUCCEEDED_TO_GOAL_STATE=2  ACTION_FAILED=10  ...
int32 action_id
int32 plan_id
int32 status
diagnostic_msgs/KeyValue[] information
```

| Message | Purpose |
|---|---|
| `CompletePlan` | `ActionDispatch[] plan` — a totally ordered plan |
| `EsterelPlan`, `EsterelPlanNode`, `EsterelPlanEdge` | a partial-order plan graph |

### Services

| Service | Request → Response |
|---|---|
| `DispatchService` | — → `success`, `goal_achieved` |
| `ParsingService` | plan file → parsed plan |
| `PlanningService` | domain/problem/command → `plan_found` |
| `ProblemService` | problem-generation parameters → result |
| `GetPlanningParams` | — → current planning parameters |

### Actions

| Action | Goal → Result → Feedback |
|---|---|
| `Plan` | `domain_path`, `problem_path`, `data_path`, `planner_command`, `use_problem_topic`, `start_action_id` → `result_status` → `current_phase`, `current_status`, `percent_completed` |
| `NonBlockingDispatch` | non-blocking plan dispatch with progress feedback |
