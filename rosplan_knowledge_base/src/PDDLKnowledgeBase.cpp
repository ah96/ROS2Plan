#include "rosplan_knowledge_base/PDDLKnowledgeBase.h"

#include <diagnostic_msgs/msg/key_value.hpp>
#include <fstream>

namespace KCL_rosplan {

    namespace kmsrv = rosplan_knowledge_msgs::srv;
    namespace kmmsg = rosplan_knowledge_msgs::msg;

    /*-----------------*/
    /* fetching domain */
    /*-----------------*/

    void PDDLKnowledgeBase::getDomainName(
        const std::shared_ptr<kmsrv::GetDomainNameService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetDomainNameService::Response> res)
    {
        if (!domain_parser.domain_parsed) return;
        res->domain_name = domain_parser.domain->name;
    }

    void PDDLKnowledgeBase::getTypes(
        const std::shared_ptr<kmsrv::GetDomainTypeService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetDomainTypeService::Response> res)
    {
        if (!domain_parser.domain_parsed) return;

        VAL1_2::pddl_type_list* types = domain_parser.domain->types;
        if (types) {
            for (auto ci = types->begin(); ci != types->end(); ci++) {
                const VAL1_2::pddl_type* type = *ci;
                res->types.push_back(type->getName());
                res->super_types.push_back(type->type ? type->type->getName() : "");
            }
        }
    }

    void PDDLKnowledgeBase::getPredicates(
        const std::shared_ptr<kmsrv::GetDomainAttributeService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetDomainAttributeService::Response> res)
    {
        VAL1_2::pred_decl_list* predicates = domain_parser.domain->predicates;
        if (predicates) {
            for (auto ci = predicates->begin(); ci != predicates->end(); ci++) {
                const VAL1_2::pred_decl* predicate = *ci;
                kmmsg::DomainFormula formula;
                formula.name = predicate->getPred()->symbol::getName();
                for (auto vi = predicate->getArgs()->begin(); vi != predicate->getArgs()->end(); vi++) {
                    const VAL1_2::var_symbol* var = *vi;
                    diagnostic_msgs::msg::KeyValue param;
                    param.key   = var->pddl_typed_symbol::getName();
                    param.value = (var->type != nullptr) ? var->type->getName() : "object";
                    formula.typed_parameters.push_back(param);
                }
                res->items.push_back(formula);
            }
        }
    }

    void PDDLKnowledgeBase::getFunctionPredicates(
        const std::shared_ptr<kmsrv::GetDomainAttributeService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetDomainAttributeService::Response> res)
    {
        VAL1_2::func_decl_list* functions = domain_parser.domain->functions;
        if (functions) {
            for (auto ci = functions->begin(); ci != functions->end(); ci++) {
                const VAL1_2::func_decl* function = *ci;
                kmmsg::DomainFormula formula;
                formula.name = function->getFunction()->symbol::getName();
                for (auto vi = function->getArgs()->begin(); vi != function->getArgs()->end(); vi++) {
                    const VAL1_2::var_symbol* var = *vi;
                    diagnostic_msgs::msg::KeyValue param;
                    param.key   = var->pddl_typed_symbol::getName();
                    param.value = (var->type != nullptr) ? var->type->getName() : "object";
                    formula.typed_parameters.push_back(param);
                }
                res->items.push_back(formula);
            }
        }
    }

    void PDDLKnowledgeBase::getOperators(
        const std::shared_ptr<kmsrv::GetDomainOperatorService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetDomainOperatorService::Response> res)
    {
        VAL1_2::operator_list* operators = domain_parser.domain->ops;
        for (auto ci = operators->begin(); ci != operators->end(); ci++) {
            const VAL1_2::operator_* op = *ci;
            kmmsg::DomainFormula formula;
            formula.name = op->name->symbol::getName();
            for (auto vi = op->parameters->begin(); vi != op->parameters->end(); vi++) {
                const VAL1_2::var_symbol* var = *vi;
                diagnostic_msgs::msg::KeyValue param;
                param.key   = var->pddl_typed_symbol::getName();
                param.value = (var->type != nullptr) ? var->type->getName() : "object";
                formula.typed_parameters.push_back(param);
            }
            res->operators.push_back(formula);
        }
    }

    void PDDLKnowledgeBase::getOperatorDetails(
        const std::shared_ptr<kmsrv::GetDomainOperatorDetailsService::Request> req,
        std::shared_ptr<kmsrv::GetDomainOperatorDetailsService::Response> res)
    {
        VALVisitorOperator op_visitor;
        VAL1_2::operator_list* operators = domain_parser.domain->ops;
        for (auto ci = operators->begin(); ci != operators->end(); ci++) {
            if ((*ci)->name->symbol::getName() == req->name) {
                op_visitor.visit_operator_(*ci);
                res->op = op_visitor.msg;
                return;
            }
        }
    }

    void PDDLKnowledgeBase::getPredicateDetails(
        const std::shared_ptr<kmsrv::GetDomainPredicateDetailsService::Request> req,
        std::shared_ptr<kmsrv::GetDomainPredicateDetailsService::Response> res)
    {
        VALVisitorPredicate pred_visitor;
        VAL1_2::pred_decl_list* predicates = domain_parser.domain->predicates;
        for (auto ci = predicates->begin(); ci != predicates->end(); ci++) {
            if ((*ci)->getPred()->symbol::getName() == req->name) {
                pred_visitor.visit_pred_decl(*ci);
                res->predicate = pred_visitor.msg;
                if (sensed_predicates.find(req->name) == sensed_predicates.end()) {
                    sensed_predicates[req->name] = false;
                }
                res->is_sensed = sensed_predicates[req->name];
                return;
            }
        }
    }

    /*-------------------------------------*/
    /* add initial state to knowledge base */
    /*-------------------------------------*/

    void PDDLKnowledgeBase::addConstants() {
        if (!domain_parser.domain_parsed) return;
        VAL1_2::const_symbol_list* c = domain_parser.domain->constants;
        if (c) {
            for (auto it = c->begin(); it != c->end(); it++) {
                const VAL1_2::const_symbol* object = *it;
                domain_constants[object->type->getName()].push_back(
                    object->pddl_typed_symbol::getName());
            }
        }
    }

    void PDDLKnowledgeBase::addInitialState() {
        VALVisitorProblem problem_visitor(domain_parser.domain, problem_parser.problem);
        model_instances = problem_visitor.returnInstances();
        model_facts     = problem_visitor.returnFacts();
        model_functions = problem_visitor.returnFunctions();
        model_goals     = problem_visitor.returnGoals();
        model_timed_initial_literals = problem_visitor.returnTimedKnowledge();
        if (problem_parser.problem->metric) {
            model_metric = problem_visitor.returnMetric();
        }
    }

    void PDDLKnowledgeBase::parseDomain(const std::string& domain_file_path,
                                        const std::string& problem_file_path)
    {
        std::ifstream file_check;
        VAL1_2::analysis* an_analysis = new VAL1_2::analysis;
        domain_parser.val_analysis  = an_analysis;
        problem_parser.val_analysis = an_analysis;

        RCLCPP_INFO(get_logger(), "KCL: (%s) Parsing domain", get_name());
        domain_parser.domain_parsed = false;

        file_check.open(domain_file_path.c_str());
        if (!file_check.good()) {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) Domain file does not exist: (%s).",
                         get_name(), domain_file_path.c_str());
            rclcpp::shutdown();
            return;
        }
        file_check.close();

        VAL1_2::domain* domain = domain_parser.parseDomain(domain_file_path);
        if (domain) {
            addConstants();
        } else {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) There were syntax errors in the domain file.", get_name());
            rclcpp::shutdown();
            return;
        }

        if (problem_file_path != "") {
            RCLCPP_INFO(get_logger(), "KCL: (%s) Parsing initial state", get_name());
            problem_parser.problem_parsed = false;
            file_check.open(problem_file_path.c_str());
            if (!file_check.good()) {
                RCLCPP_WARN(get_logger(), "KCL: (%s) Initial state file does not exist.", get_name());
            } else {
                file_check.close();
                VAL1_2::problem* problem = problem_parser.parseProblem(problem_file_path);
                if (problem) {
                    addInitialState();
                } else {
                    RCLCPP_WARN(get_logger(), "KCL: (%s) There were syntax errors in the problem file.", get_name());
                }
            }
        }
    }

} // namespace KCL_rosplan
