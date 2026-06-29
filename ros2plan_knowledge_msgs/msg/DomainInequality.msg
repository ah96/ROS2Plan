# A message used to store the numeric effects of an action
# Can be grounded or ungrounded

uint8 GREATER   = 0
uint8 GREATEREQ = 1
uint8 LESS      = 2
uint8 LESSEQ    = 3
uint8 EQUALS    = 4

uint8 comparison_type

ros2plan_knowledge_msgs/ExprComposite lhs
ros2plan_knowledge_msgs/ExprComposite rhs

bool grounded
