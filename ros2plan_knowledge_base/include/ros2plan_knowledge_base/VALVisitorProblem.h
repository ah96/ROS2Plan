#ifndef KCL_valvisitor_problem
#define KCL_valvisitor_problem

#include <sstream>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

#include "VALfiles/ptree.h"
#include "VALfiles/VisitController.h"

#include "ros2plan_knowledge_base/KnowledgeComparitor.h"

#include "ros2plan_knowledge_msgs/msg/domain_formula.hpp"
#include "ros2plan_knowledge_msgs/msg/knowledge_item.hpp"
#include "ros2plan_knowledge_msgs/msg/expr_composite.hpp"
#include "ros2plan_knowledge_msgs/msg/expr_base.hpp"

namespace KCL_ros2plan {

    class VALVisitorProblem : public VAL1_2::VisitController
    {
    private:
        bool   problem_cond_neg;
        bool   problem_eff_neg;
        double problem_eff_time;

        VAL1_2::domain*  domain;
        VAL1_2::problem* problem;

        bool effects_read;

    public:
        VALVisitorProblem(VAL1_2::domain* inputDomain, VAL1_2::problem* inputProblem)
        {
            domain  = inputDomain;
            problem = inputProblem;

            problem_eff_time = 0;
            effects_read     = false;
            problem_cond_neg = false;
            problem_eff_neg  = false;
        }

        ros2plan_knowledge_msgs::msg::DomainFormula  last_prop;
        ros2plan_knowledge_msgs::msg::ExprComposite  last_expr;
        ros2plan_knowledge_msgs::msg::DomainFormula  last_func_term;

        std::map<std::string, std::vector<std::string>> instances;
        std::map<std::string, std::vector<std::string>> constants;
        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> facts;
        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> functions;
        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> goals;
        ros2plan_knowledge_msgs::msg::KnowledgeItem metric;

        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> timed_initial_literals;

        std::map<std::string, std::vector<std::string>> returnInstances();
        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> returnFacts();
        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> returnFunctions();
        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> returnGoals();
        ros2plan_knowledge_msgs::msg::KnowledgeItem returnMetric();
        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> returnTimedKnowledge();

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
        virtual void visit_timed_initial_literal(VAL1_2::timed_initial_literal * s);
        virtual void visit_effect_lists(VAL1_2::effect_lists * e);

        virtual void visit_comparison(VAL1_2::comparison * c);
        virtual void visit_derivation_rule(VAL1_2::derivation_rule * o);
        virtual void visit_metric_spec(VAL1_2::metric_spec * s);

        virtual void visit_plus_expression(VAL1_2::plus_expression * s);
        virtual void visit_minus_expression(VAL1_2::minus_expression * s);
        virtual void visit_mul_expression(VAL1_2::mul_expression * s);
        virtual void visit_div_expression(VAL1_2::div_expression * s);
        virtual void visit_uminus_expression(VAL1_2::uminus_expression * s);
        virtual void visit_int_expression(VAL1_2::int_expression * s);
        virtual void visit_float_expression(VAL1_2::float_expression * s);
        virtual void visit_special_val_expr(VAL1_2::special_val_expr * s);
        virtual void visit_func_term(VAL1_2::func_term * s);
    };

} // namespace KCL_ros2plan
#endif
