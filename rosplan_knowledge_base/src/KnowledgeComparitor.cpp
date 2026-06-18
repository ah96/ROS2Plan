#include "rosplan_knowledge_base/KnowledgeComparitor.h"

#include <boost/algorithm/string.hpp>
#include <rclcpp/rclcpp.hpp>

namespace KCL_rosplan {

    using KI  = rosplan_knowledge_msgs::msg::KnowledgeItem;
    using EB  = rosplan_knowledge_msgs::msg::ExprBase;
    using EC  = rosplan_knowledge_msgs::msg::ExprComposite;
    using DI  = rosplan_knowledge_msgs::msg::DomainInequality;

    bool KnowledgeComparitor::inequalityTrue(const KI &a, const std::vector<KI> &modelFunctions) {

        double lhs = KnowledgeComparitor::evaluateExpression(a.ineq.lhs, modelFunctions);
        double rhs = KnowledgeComparitor::evaluateExpression(a.ineq.rhs, modelFunctions);

        switch(a.ineq.comparison_type) {
            case DI::GREATER:   return lhs > rhs;
            case DI::GREATEREQ: return lhs >= rhs;
            case DI::LESS:      return lhs < rhs;
            case DI::LESSEQ:    return lhs <= rhs;
            case DI::EQUALS:    return lhs == rhs;
        }
        RCLCPP_ERROR(rclcpp::get_logger("rosplan_knowledge_base"), "KCL: Knowledge Comparitor does not recognise comparison type.");
        return false;
    }

    double KnowledgeComparitor::evaluateExpression(const EC &expr, const std::vector<KI> &modelFunctions) {

        std::vector<EB> tokens = expr.tokens;

        bool pending_operand = false;
        std::vector<EB> operator_stack;
        std::vector<EB> operand_stack;
        for (size_t i = 0; i < tokens.size(); i++) {
            EB token = tokens[i];
            switch (token.expr_type) {
                case EB::OPERATOR:
                    operator_stack.push_back(token);
                    pending_operand = false;
                    break;
                case EB::CONSTANT:
                case EB::FUNCTION:
                case EB::SPECIAL:
                    if (pending_operand) {
                        while (operand_stack.size() > 0) {
                            EB lhs = token;
                            EB rhs = operand_stack.back();
                            operand_stack.pop_back();
                            EB op = operator_stack.back();
                            operator_stack.back();
                            double value = KnowledgeComparitor::evaluateOperation(
                                    KnowledgeComparitor::getValue(lhs, modelFunctions),
                                    KnowledgeComparitor::getValue(rhs, modelFunctions),
                                    op);
                            token.expr_type = EB::CONSTANT;
                            token.constant = value;
                        }
                    }
                    pending_operand = true;
                    operand_stack.push_back(token);
                    break;
            }
        }

        EB result = operand_stack.back();
        operand_stack.pop_back();
        return getValue(result, modelFunctions);
    }

    double KnowledgeComparitor::evaluateOperation(const double &lhs, const double &rhs, const EB &op) {

        switch (op.op) {
            case EB::ADD:    return lhs + rhs;
            case EB::SUB:    return lhs - rhs;
            case EB::MUL:    return lhs * rhs;
            case EB::DIV:    return lhs / rhs;
            case EB::UMINUS: return -lhs;
        }
        RCLCPP_ERROR(rclcpp::get_logger("rosplan_knowledge_base"), "KCL: Knowledge Comparitor does not recognise operator type.");
        return 0;
    }

    double KnowledgeComparitor::getValue(const EB &expr, const std::vector<KI> &modelFunctions) {
        switch (expr.expr_type) {
            case EB::CONSTANT:
                return expr.constant;
            case EB::FUNCTION:
                for (auto fit = modelFunctions.begin(); fit != modelFunctions.end(); fit++) {
                    if (expr.function.name == fit->attribute_name &&
                        expr.function.typed_parameters.size() == fit->values.size()) {
                        bool match = true;
                        for (size_t i = 0; i < fit->values.size(); i++) {
                            if ("" != expr.function.typed_parameters[i].value &&
                                !(boost::iequals(expr.function.typed_parameters[i].value, fit->values[i].value))) {
                                match = false;
                            }
                        }
                        if (match) return fit->function_value;
                    }
                }
                return 0;
        }
        RCLCPP_ERROR(rclcpp::get_logger("rosplan_knowledge_base"), "KCL: Knowledge Comparitor cannot get value for base expression.");
        return 0;
    }

    bool KnowledgeComparitor::containsKnowledge(const KI &a, const KI &b) {

        int matches = 0;
        if (a.knowledge_type != b.knowledge_type) return false;

        switch (a.knowledge_type) {

        case KI::INSTANCE:
            {
                if (0 != a.instance_type.compare(b.instance_type)) return false;
                if (a.instance_name != "" && !boost::iequals(a.instance_name, b.instance_name)) return false;
                return true;
            }
            break;

        case KI::FUNCTION:

        case KI::FACT:
            {
                if (a.attribute_name == "") return true;
                if (!boost::iequals(a.attribute_name, b.attribute_name)) return false;
                if (a.is_negative != b.is_negative) return false;
                if (a.values.size() != b.values.size()) return false;
                for (size_t i = 0; i < a.values.size(); i++) {
                    if ("" == a.values[i].value) {
                        ++matches;
                    } else if (boost::iequals(a.values[i].value, b.values[i].value)) {
                        ++matches;
                    }
                }
                return (matches == static_cast<int>(a.values.size()));
            }
            break;
        }
        return false;
    }

    bool KnowledgeComparitor::containsInstance(const KI &a, std::string &name) {

        if (0 == a.instance_name.compare(name))
            return true;

        for (size_t i = 0; i < a.values.size(); i++) {
            if (boost::iequals(a.values[i].value, name))
                return true;
        }

        return false;
    }

} // namespace KCL_rosplan
