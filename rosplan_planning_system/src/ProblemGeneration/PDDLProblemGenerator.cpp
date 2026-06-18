#include "rosplan_planning_system/ProblemGeneration/PDDLProblemGenerator.h"

namespace KCL_rosplan {

    PDDLProblemGenerator::PDDLProblemGenerator(const std::string& kb, rclcpp::Node::SharedPtr svc_node)
        : ProblemGenerator(kb, svc_node)
    {
        // Create all clients once (cached)
        get_name_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetDomainNameService>(domain_name_service);
        get_types_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetDomainTypeService>(domain_type_service);
        get_instances_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetInstanceService>(state_instance_service);
        get_domain_props_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetDomainAttributeService>(domain_predicate_service);
        get_domain_funcs_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetDomainAttributeService>(domain_function_service);
        get_props_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetAttributeService>(state_proposition_service);
        get_funcs_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetAttributeService>(state_function_service);
        get_tils_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetAttributeService>(state_timed_knowledge_service);
        get_goals_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetAttributeService>(state_goal_service);
        get_metric_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetMetricService>(state_metric_service);
    }

    // Helper macro for synchronous service call
    #define CALL_SERVICE(client, request, response, error_msg) \
    do { \
        if (!client->wait_for_service(std::chrono::seconds(5))) { \
            RCLCPP_ERROR(svc_node_->get_logger(), "KCL: (PDDLProblemGenerator) Service unavailable: %s", error_msg); \
        } else { \
            auto future = client->async_send_request(request); \
            auto status = rclcpp::spin_until_future_complete(svc_node_, future, std::chrono::seconds(5)); \
            if (status == rclcpp::FutureReturnCode::SUCCESS) { \
                response = future.get(); \
            } else { \
                RCLCPP_ERROR(svc_node_->get_logger(), "KCL: (PDDLProblemGenerator) Failed to call service: %s", error_msg); \
            } \
        } \
    } while(0)

    /*--------*/
    /* header */
    /*--------*/

    void PDDLProblemGenerator::makeHeader(std::ofstream &pFile) {
        // get domain name
        auto nameReq = std::make_shared<rosplan_knowledge_msgs::srv::GetDomainNameService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainNameService::Response> nameRes;
        CALL_SERVICE(get_name_client_, nameReq, nameRes, domain_name_service.c_str());

        std::string domain_name = nameRes ? nameRes->domain_name : "unknown";
        pFile << "(define (problem task)" << std::endl;
        pFile << "(:domain " << domain_name << ")" << std::endl;

        pFile << "(:objects" << std::endl;

        // get types
        auto typeReq = std::make_shared<rosplan_knowledge_msgs::srv::GetDomainTypeService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainTypeService::Response> typeRes;
        CALL_SERVICE(get_types_client_, typeReq, typeRes, domain_type_service.c_str());

        if (typeRes) {
            for (size_t t = 0; t < typeRes->types.size(); t++) {
                auto instReq = std::make_shared<rosplan_knowledge_msgs::srv::GetInstanceService::Request>();
                instReq->type_name = typeRes->types[t];
                std::shared_ptr<rosplan_knowledge_msgs::srv::GetInstanceService::Response> instRes;
                CALL_SERVICE(get_instances_client_, instReq, instRes, state_instance_service.c_str());

                if (instRes && !instRes->instances.empty()) {
                    pFile << "    ";
                    for (size_t i = 0; i < instRes->instances.size(); i++) {
                        pFile << instRes->instances[i] << " ";
                    }
                    pFile << "- " << typeRes->types[t] << std::endl;
                }
            }
        }

        pFile << ")" << std::endl;
    }

    /*---------------*/
    /* initial state */
    /*---------------*/

    void PDDLProblemGenerator::makeInitialState(std::ofstream &pFile) {

        // note the time now for TILs
        rclcpp::Time time = rclcpp::Clock().now() + rclcpp::Duration::from_seconds(1.0);

        pFile << "(:init" << std::endl;

        // get domain predicates
        auto domainAttrReq = std::make_shared<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Response> domainPropsRes;
        CALL_SERVICE(get_domain_props_client_, domainAttrReq, domainPropsRes, domain_predicate_service.c_str());

        if (domainPropsRes) {
            for (auto& df : domainPropsRes->items) {
                // propositions
                auto attrReq = std::make_shared<rosplan_knowledge_msgs::srv::GetAttributeService::Request>();
                attrReq->predicate_name = df.name;
                std::shared_ptr<rosplan_knowledge_msgs::srv::GetAttributeService::Response> attrRes;
                CALL_SERVICE(get_props_client_, attrReq, attrRes, state_proposition_service.c_str());

                if (attrRes) {
                    for (auto& attr : attrRes->attributes) {
                        pFile << "    (";
                        if (attr.is_negative) pFile << "not (";
                        pFile << attr.attribute_name;
                        for (auto& v : attr.values) {
                            pFile << " " << v.value;
                        }
                        pFile << ")";
                        if (attr.is_negative) pFile << ")";
                        pFile << std::endl;
                    }
                }
                pFile << std::endl;

                // TILs
                auto tilReq = std::make_shared<rosplan_knowledge_msgs::srv::GetAttributeService::Request>();
                tilReq->predicate_name = df.name;
                std::shared_ptr<rosplan_knowledge_msgs::srv::GetAttributeService::Response> tilRes;
                CALL_SERVICE(get_tils_client_, tilReq, tilRes, state_timed_knowledge_service.c_str());

                if (tilRes && !tilRes->attributes.empty()) {
                    for (auto& attr : tilRes->attributes) {
                        rclcpp::Time attr_time(attr.initial_time.sec, attr.initial_time.nanosec);
                        double dt = (attr_time - time).seconds();
                        pFile << "    (at " << dt << " (";
                        if (attr.is_negative) pFile << "not (";
                        pFile << attr.attribute_name;
                        for (auto& v : attr.values) {
                            pFile << " " << v.value;
                        }
                        pFile << "))";
                        if (attr.is_negative) pFile << ")";
                        pFile << std::endl;
                    }
                }
                pFile << std::endl;
            }
        }

        // get domain functions
        auto domainFuncReq = std::make_shared<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Response> domainFuncsRes;
        CALL_SERVICE(get_domain_funcs_client_, domainFuncReq, domainFuncsRes, domain_function_service.c_str());

        if (domainFuncsRes) {
            for (auto& df : domainFuncsRes->items) {
                auto funcReq = std::make_shared<rosplan_knowledge_msgs::srv::GetAttributeService::Request>();
                funcReq->predicate_name = df.name;
                std::shared_ptr<rosplan_knowledge_msgs::srv::GetAttributeService::Response> funcRes;
                CALL_SERVICE(get_funcs_client_, funcReq, funcRes, state_function_service.c_str());

                if (funcRes) {
                    for (auto& attr : funcRes->attributes) {
                        pFile << "    (";
                        rclcpp::Time attr_time(attr.initial_time.sec, attr.initial_time.nanosec);
                        if (time < attr_time) {
                            pFile << "at " << (attr_time - time).seconds() << " (";
                        }
                        pFile << "= (";
                        pFile << attr.attribute_name;
                        for (auto& v : attr.values) {
                            pFile << " " << v.value;
                        }
                        pFile << ") " << attr.function_value << ")";
                        if (time < attr_time) {
                            pFile << ")";
                        }
                        pFile << std::endl;
                    }
                }
                pFile << std::endl;
            }
        }

        pFile << ")" << std::endl;
    }

    /*------*/
    /* goal */
    /*------*/

    void PDDLProblemGenerator::makeGoals(std::ofstream &pFile) {
        pFile << "(:goal (and" << std::endl;

        auto goalReq = std::make_shared<rosplan_knowledge_msgs::srv::GetAttributeService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetAttributeService::Response> goalRes;
        CALL_SERVICE(get_goals_client_, goalReq, goalRes, state_goal_service.c_str());

        if (goalRes) {
            for (auto& attr : goalRes->attributes) {
                if (attr.knowledge_type == rosplan_knowledge_msgs::msg::KnowledgeItem::FACT) {
                    if (attr.is_negative) {
                        pFile << "    (not(" << attr.attribute_name;
                    } else {
                        pFile << "    (" << attr.attribute_name;
                    }
                    for (auto& v : attr.values) {
                        pFile << " " << v.value;
                    }
                    if (attr.is_negative) pFile << ")";
                    pFile << ")" << std::endl;
                }
                if (attr.knowledge_type == rosplan_knowledge_msgs::msg::KnowledgeItem::INEQUALITY) {
                    if (attr.is_negative) pFile << "    (not(";
                    else pFile << "    (";
                    switch (attr.ineq.comparison_type) {
                        case 0: pFile << "> "; break;
                        case 1: pFile << ">= "; break;
                        case 2: pFile << "<"; break;
                        case 3: pFile << "<= "; break;
                        case 4: pFile << "="; break;
                    }
                    printExpression(pFile, const_cast<rosplan_knowledge_msgs::msg::ExprComposite&>(attr.ineq.lhs));
                    printExpression(pFile, const_cast<rosplan_knowledge_msgs::msg::ExprComposite&>(attr.ineq.rhs));
                    if (attr.is_negative) pFile << ")";
                    pFile << ")" << std::endl;
                }
            }
        }
        pFile << "))" << std::endl;
    }

    /*--------*/
    /* metric */
    /*--------*/

    void PDDLProblemGenerator::makeMetric(std::ofstream &pFile) {
        auto metricReq = std::make_shared<rosplan_knowledge_msgs::srv::GetMetricService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetMetricService::Response> metricRes;
        CALL_SERVICE(get_metric_client_, metricReq, metricRes, state_metric_service.c_str());

        if (metricRes) {
            auto& metric = metricRes->metric;
            if (metric.knowledge_type == rosplan_knowledge_msgs::msg::KnowledgeItem::EXPRESSION) {
                pFile << "(:metric " << metric.optimization;
                printExpression(pFile, const_cast<rosplan_knowledge_msgs::msg::ExprComposite&>(metric.expr));
                pFile << ")" << std::endl;
            }
        }
    }

    void PDDLProblemGenerator::printExpression(std::ofstream &pFile,
                                               rosplan_knowledge_msgs::msg::ExprComposite &e)
    {
        auto& tokens = e.tokens;
        bool second_operand = false;
        int depth = 0;
        for (auto& token : tokens) {
            pFile << " ";
            switch (token.expr_type) {
            case rosplan_knowledge_msgs::msg::ExprBase::OPERATOR:
                switch (token.op) {
                    case rosplan_knowledge_msgs::msg::ExprBase::ADD: pFile << "(+"; break;
                    case rosplan_knowledge_msgs::msg::ExprBase::SUB: pFile << "(-"; break;
                    case rosplan_knowledge_msgs::msg::ExprBase::MUL: pFile << "(*"; break;
                    case rosplan_knowledge_msgs::msg::ExprBase::DIV: pFile << "(/"; break;
                }
                second_operand = false;
                depth++;
                break;
            case rosplan_knowledge_msgs::msg::ExprBase::CONSTANT:
                pFile << token.constant;
                break;
            case rosplan_knowledge_msgs::msg::ExprBase::FUNCTION:
                pFile << "(" << token.function.name;
                for (auto& p : token.function.typed_parameters) {
                    pFile << " " << p.value;
                }
                pFile << ")";
                break;
            case rosplan_knowledge_msgs::msg::ExprBase::SPECIAL:
                switch (token.special_type) {
                    case rosplan_knowledge_msgs::msg::ExprBase::HASHT:      pFile << "#t"; break;
                    case rosplan_knowledge_msgs::msg::ExprBase::TOTAL_TIME: pFile << "(total-time)"; break;
                    case rosplan_knowledge_msgs::msg::ExprBase::DURATION:   pFile << "?duration"; break;
                }
                break;
            }
            if (second_operand && token.expr_type != rosplan_knowledge_msgs::msg::ExprBase::OPERATOR) {
                second_operand = true;
                pFile << ")";
                depth--;
            }
            if (token.expr_type != rosplan_knowledge_msgs::msg::ExprBase::OPERATOR) {
                second_operand = true;
            }
        }
        while (depth > 0) {
            pFile << ")";
            depth--;
        }
    }

    void PDDLProblemGenerator::makeProblem(std::ofstream &pFile) {
        makeHeader(pFile);
        makeInitialState(pFile);
        makeGoals(pFile);
        makeMetric(pFile);
        pFile << ")" << std::endl;
    }

} // close namespace
