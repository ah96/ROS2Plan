#ifndef KCL_knowledgecomparitor
#define KCL_knowledgecomparitor

#include <vector>
#include "ros2plan_knowledge_msgs/msg/knowledge_item.hpp"
#include "ros2plan_knowledge_msgs/msg/domain_inequality.hpp"
#include "ros2plan_knowledge_msgs/msg/expr_base.hpp"
#include "ros2plan_knowledge_msgs/msg/expr_composite.hpp"

namespace KCL_ros2plan {

    class KnowledgeComparitor
    {
    public:
        static double getValue(const ros2plan_knowledge_msgs::msg::ExprBase &expr, const std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> &modelFunctions);
        static double evaluateOperation(const double &lhs, const double &rhs, const ros2plan_knowledge_msgs::msg::ExprBase &op);
        static double evaluateExpression(const ros2plan_knowledge_msgs::msg::ExprComposite &a, const std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> &modelFunctions);
        static bool inequalityTrue(const ros2plan_knowledge_msgs::msg::KnowledgeItem &a, const std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> &modelFunctions);
        static bool containsKnowledge(const ros2plan_knowledge_msgs::msg::KnowledgeItem &a, const ros2plan_knowledge_msgs::msg::KnowledgeItem &b);
        static bool containsInstance(const ros2plan_knowledge_msgs::msg::KnowledgeItem &a, std::string &name);
    };
}
#endif
