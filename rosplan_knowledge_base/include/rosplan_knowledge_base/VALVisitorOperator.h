#ifndef KCL_valvisitor_operator
#define KCL_valvisitor_operator

#include <sstream>
#include <string>
#include <vector>

#include "VALfiles/ptree.h"
#include "VALfiles/VisitController.h"

#include "rosplan_knowledge_msgs/msg/domain_formula.hpp"
#include "rosplan_knowledge_msgs/msg/domain_inequality.hpp"
#include "rosplan_knowledge_msgs/msg/domain_operator.hpp"
#include "rosplan_knowledge_msgs/msg/expr_base.hpp"
#include "rosplan_knowledge_msgs/msg/expr_composite.hpp"
#include "rosplan_knowledge_msgs/msg/knowledge_item.hpp"

namespace KCL_rosplan {

    class VALVisitorOperator : public VAL1_2::VisitController
    {
    public:
        rosplan_knowledge_msgs::msg::ExprComposite last_expr;
        rosplan_knowledge_msgs::msg::DomainFormula  last_prop;
        rosplan_knowledge_msgs::msg::DomainFormula  last_func;
        rosplan_knowledge_msgs::msg::DomainOperator msg;

        virtual void visit_proposition(VAL1_2::proposition *);
        virtual void visit_operator_(VAL1_2::operator_ *);

        virtual void visit_simple_goal(VAL1_2::simple_goal *);
        virtual void visit_qfied_goal(VAL1_2::qfied_goal *);
        virtual void visit_conj_goal(VAL1_2::conj_goal *);
        virtual void visit_disj_goal(VAL1_2::disj_goal *);
        virtual void visit_timed_goal(VAL1_2::timed_goal *);
        virtual void visit_imply_goal(VAL1_2::imply_goal *);
        virtual void visit_neg_goal(VAL1_2::neg_goal *);

        virtual void visit_assignment(VAL1_2::assignment * e);
        virtual void visit_simple_effect(VAL1_2::simple_effect * e);
        virtual void visit_forall_effect(VAL1_2::forall_effect * e);
        virtual void visit_cond_effect(VAL1_2::cond_effect * e);
        virtual void visit_timed_effect(VAL1_2::timed_effect * e);
        virtual void visit_effect_lists(VAL1_2::effect_lists * e);

        virtual void visit_comparison(VAL1_2::comparison * c);

        virtual void visit_plus_expression(VAL1_2::plus_expression * s);
        virtual void visit_minus_expression(VAL1_2::minus_expression * s);
        virtual void visit_mul_expression(VAL1_2::mul_expression * s);
        virtual void visit_div_expression(VAL1_2::div_expression * s);
        virtual void visit_uminus_expression(VAL1_2::uminus_expression * s);
        virtual void visit_int_expression(VAL1_2::int_expression * s);
        virtual void visit_float_expression(VAL1_2::float_expression * s);
        virtual void visit_special_val_expr(VAL1_2::special_val_expr * s);
        virtual void visit_func_term(VAL1_2::func_term * s);

        virtual void visit_derivation_rule(VAL1_2::derivation_rule * o);
    };

} // namespace KCL_rosplan
#endif
