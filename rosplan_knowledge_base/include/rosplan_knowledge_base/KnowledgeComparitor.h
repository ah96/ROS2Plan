#ifndef KCL_knowledgecomparitor
#define KCL_knowledgecomparitor

#include <vector>
#include "rosplan_knowledge_msgs/msg/knowledge_item.hpp"
#include "rosplan_knowledge_msgs/msg/domain_inequality.hpp"
#include "rosplan_knowledge_msgs/msg/expr_base.hpp"
#include "rosplan_knowledge_msgs/msg/expr_composite.hpp"

namespace KCL_rosplan {

    class KnowledgeComparitor
    {
    public:
        static double getValue(const rosplan_knowledge_msgs::msg::ExprBase &expr, const std::vector<rosplan_knowledge_msgs::msg::KnowledgeItem> &modelFunctions);
        static double evaluateOperation(const double &lhs, const double &rhs, const rosplan_knowledge_msgs::msg::ExprBase &op);
        static double evaluateExpression(const rosplan_knowledge_msgs::msg::ExprComposite &a, const std::vector<rosplan_knowledge_msgs::msg::KnowledgeItem> &modelFunctions);
        static bool inequalityTrue(const rosplan_knowledge_msgs::msg::KnowledgeItem &a, const std::vector<rosplan_knowledge_msgs::msg::KnowledgeItem> &modelFunctions);
        static bool containsKnowledge(const rosplan_knowledge_msgs::msg::KnowledgeItem &a, const rosplan_knowledge_msgs::msg::KnowledgeItem &b);
        static bool containsInstance(const rosplan_knowledge_msgs::msg::KnowledgeItem &a, std::string &name);
    };
}
#endif
