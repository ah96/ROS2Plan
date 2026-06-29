//
// Created by gerard on 21/09/18.
//

#ifndef ROS2PLAN_KNOWLEDGE_BASE_RDDLEXPRUTILS_H
#define ROS2PLAN_KNOWLEDGE_BASE_RDDLEXPRUTILS_H

#define NOT_IMPLEMENTED_EXPR ROS_WARN_STREAM(__FILE__ << ":" << __LINE__ << ": Unknown or unsupported operand type for the expression.")

#include <ros2plan_knowledge_msgs/ExprComposite.h>
#include <ros2plan_knowledge_msgs/ExprBase.h>
#include "RDDLParser.h"

class RDDLExprUtils {
private:
    // Gets a list of the expression of the quantifier (sumation or product) instantiated with all the variables of the quantifier
    static void getQuantExprRec(const Quantifier* expr, int param_id, std::map<std::string, std::string>& assign,
                                std::vector<ros2plan_knowledge_msgs::ExprComposite>& result);

    // Moves elements of b to the end of a
    static inline void join(std::vector<ros2plan_knowledge_msgs::ExprBase>& a, std::vector<ros2plan_knowledge_msgs::ExprBase>& b);

    // Joints a list of instantiated expressions to a single polish notation opertion
    static ros2plan_knowledge_msgs::ExprComposite joinQuantOperator(std::vector<ros2plan_knowledge_msgs::ExprComposite> &operands,
                                                                   ros2plan_knowledge_msgs::ExprBase::_op_type op_type, int op_idx=0);
public:
    /* Convert to ROS2Plan expression */
    static ros2plan_knowledge_msgs::ExprComposite getExpression(const Evaluatable* expr);

    static ros2plan_knowledge_msgs::ExprComposite getExpression(const Connective* expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::ExprComposite getExpression(const Quantifier* expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::ExprComposite getExpression(const Negation *expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::ExprComposite getExpression(const NumericConstant *expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::ExprComposite getExpression(const ParametrizedVariable *expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::ExprComposite getExpression(const Parameter *expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::ExprComposite getExpression(const IfThenElseExpression *expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::ExprComposite getExpression(const LogicalExpression *expr, const std::map<std::string, std::string>& assign);
};


#endif //ROS2PLAN_KNOWLEDGE_BASE_RDDLEXPRUTILS_H
