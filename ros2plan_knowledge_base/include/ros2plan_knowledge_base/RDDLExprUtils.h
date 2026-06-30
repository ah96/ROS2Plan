//
// Created by gerard on 21/09/18.
//

#ifndef ROS2PLAN_KNOWLEDGE_BASE_RDDLEXPRUTILS_H
#define ROS2PLAN_KNOWLEDGE_BASE_RDDLEXPRUTILS_H

#include <rclcpp/rclcpp.hpp>

#define NOT_IMPLEMENTED_EXPR RCLCPP_WARN_STREAM(rclcpp::get_logger("RDDLExprUtils"), __FILE__ << ":" << __LINE__ << ": Unknown or unsupported operand type for the expression.")

#include <ros2plan_knowledge_msgs/msg/expr_composite.hpp>
#include <ros2plan_knowledge_msgs/msg/expr_base.hpp>
#include "RDDLParser.h"

class RDDLExprUtils {
private:
    // Gets a list of the expression of the quantifier (sumation or product) instantiated with all the variables of the quantifier
    static void getQuantExprRec(const Quantifier* expr, int param_id, std::map<std::string, std::string>& assign,
                                std::vector<ros2plan_knowledge_msgs::msg::ExprComposite>& result);

    // Moves elements of b to the end of a
    static inline void join(std::vector<ros2plan_knowledge_msgs::msg::ExprBase>& a, std::vector<ros2plan_knowledge_msgs::msg::ExprBase>& b);

    // Joints a list of instantiated expressions to a single polish notation opertion
    static ros2plan_knowledge_msgs::msg::ExprComposite joinQuantOperator(std::vector<ros2plan_knowledge_msgs::msg::ExprComposite> &operands,
                                                                   uint8_t op_type, int op_idx=0);
public:
    /* Convert to ROS2Plan expression */
    static ros2plan_knowledge_msgs::msg::ExprComposite getExpression(const Evaluatable* expr);

    static ros2plan_knowledge_msgs::msg::ExprComposite getExpression(const Connective* expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::msg::ExprComposite getExpression(const Quantifier* expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::msg::ExprComposite getExpression(const Negation *expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::msg::ExprComposite getExpression(const NumericConstant *expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::msg::ExprComposite getExpression(const ParametrizedVariable *expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::msg::ExprComposite getExpression(const Parameter *expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::msg::ExprComposite getExpression(const IfThenElseExpression *expr, const std::map<std::string, std::string>& assign);
    static ros2plan_knowledge_msgs::msg::ExprComposite getExpression(const LogicalExpression *expr, const std::map<std::string, std::string>& assign);
};


#endif //ROS2PLAN_KNOWLEDGE_BASE_RDDLEXPRUTILS_H
