#include "ros2plan_planning_system/ProblemGeneration/CHIMPProblemGenerator.h"

namespace KCL_ros2plan {

    // Helper macro for synchronous service call via satellite node
    #define CALL_SVC(client, request, response, err_msg) \
    do { \
        if (!client->wait_for_service(std::chrono::seconds(5))) { \
            RCLCPP_ERROR(svc_node_->get_logger(), "KCL: (CHIMPProblemGenerator) Service unavailable: %s", err_msg); \
        } else { \
            auto _future = client->async_send_request(request); \
            auto _status = rclcpp::spin_until_future_complete(svc_node_, _future, std::chrono::seconds(5)); \
            if (_status == rclcpp::FutureReturnCode::SUCCESS) { \
                response = _future.get(); \
            } else { \
                RCLCPP_ERROR(svc_node_->get_logger(), "KCL: (CHIMPProblemGenerator) Failed to call service: %s", err_msg); \
            } \
        } \
    } while(0)

    CHIMPProblemGenerator::CHIMPProblemGenerator(const std::string& kb, rclcpp::Node::SharedPtr svc_node)
        : ProblemGenerator(kb, svc_node)
    {
        get_instances_client_    = svc_node_->create_client<ros2plan_knowledge_msgs::srv::GetInstanceService>(state_instance_service);
        get_domain_props_client_ = svc_node_->create_client<ros2plan_knowledge_msgs::srv::GetDomainAttributeService>(domain_predicate_service);
        get_props_client_        = svc_node_->create_client<ros2plan_knowledge_msgs::srv::GetAttributeService>(state_proposition_service);
        get_tils_client_         = svc_node_->create_client<ros2plan_knowledge_msgs::srv::GetAttributeService>(state_timed_knowledge_service);
        get_goals_client_        = svc_node_->create_client<ros2plan_knowledge_msgs::srv::GetAttributeService>(state_goal_service);
    }

    /*--------*/
    /* header */
    /*--------*/

    void CHIMPProblemGenerator::addInstances(CHIMPProblem& problem) {
        for (const std::string& instance : queryAllInstances()) {
            problem.addArgumentSymbol(instance);
        }
    }

    std::vector<std::string> CHIMPProblemGenerator::queryAllInstances() {
        auto req = std::make_shared<ros2plan_knowledge_msgs::srv::GetInstanceService::Request>();
        std::shared_ptr<ros2plan_knowledge_msgs::srv::GetInstanceService::Response> res;
        CALL_SVC(get_instances_client_, req, res, state_instance_service.c_str());
        if (res) return res->instances;
        return {};
    }

    static std::vector<std::string> extractArgs(const ros2plan_knowledge_msgs::msg::KnowledgeItem& item) {
        std::vector<std::string> args;
        for (auto& v : item.values) {
            args.push_back(v.value);
        }
        return args;
    }

    static std::vector<CHIMPFluent> knowledgeItemsToFluents(
        const std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem>& items,
        CHIMPFluent::FluentType type,
        rclcpp::Logger logger)
    {
        std::vector<CHIMPFluent> fluents;
        for (auto& item : items) {
            if (item.is_negative) {
                RCLCPP_ERROR(logger,
                    "KCL: (CHIMPProblemGenerator) CHIMP does not support negative predicates: %s",
                    item.attribute_name.c_str());
            } else {
                CHIMPFluent fl{type, item.attribute_name, extractArgs(item)};
                fluents.push_back(fl);
            }
        }
        return fluents;
    }

    /*---------------*/
    /* initial state */
    /*---------------*/

    void CHIMPProblemGenerator::addInitialState(CHIMPProblem& problem) {
        auto domAttrReq = std::make_shared<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Request>();
        std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Response> domAttrRes;
        CALL_SVC(get_domain_props_client_, domAttrReq, domAttrRes, domain_predicate_service.c_str());
        if (!domAttrRes) return;

        for (auto& df : domAttrRes->items) {
            // propositions
            auto propReq = std::make_shared<ros2plan_knowledge_msgs::srv::GetAttributeService::Request>();
            propReq->predicate_name = df.name;
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Response> propRes;
            CALL_SVC(get_props_client_, propReq, propRes, state_proposition_service.c_str());
            if (propRes) {
                problem.addFluents(knowledgeItemsToFluents(propRes->attributes,
                                                           CHIMPFluent::FluentType::STATE,
                                                           svc_node_->get_logger()));
            }

            // TILs
            auto tilReq = std::make_shared<ros2plan_knowledge_msgs::srv::GetAttributeService::Request>();
            tilReq->predicate_name = df.name;
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Response> tilRes;
            CALL_SVC(get_tils_client_, tilReq, tilRes, state_timed_knowledge_service.c_str());
            if (tilRes) {
                problem.addFluents(knowledgeItemsToFluents(tilRes->attributes,
                                                           CHIMPFluent::FluentType::STATE,
                                                           svc_node_->get_logger()));
            }
        }
    }

    /*------*/
    /* goal */
    /*------*/

    void CHIMPProblemGenerator::addGoals(CHIMPProblem& problem) {
        auto goalReq = std::make_shared<ros2plan_knowledge_msgs::srv::GetAttributeService::Request>();
        std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Response> goalRes;
        CALL_SVC(get_goals_client_, goalReq, goalRes, state_goal_service.c_str());
        if (!goalRes) return;

        auto fluents = knowledgeItemsToFluents(goalRes->attributes,
                                               CHIMPFluent::FluentType::TASK,
                                               svc_node_->get_logger());
        for (auto& fl : fluents) {
            problem.addTask(fl);
        }
    }

    void CHIMPProblemGenerator::makeProblem(std::ofstream &pFile) {
        CHIMPProblem problem;
        addInstances(problem);
        addInitialState(problem);
        addGoals(problem);
        problem.generateProblem(pFile);
    }

} // close namespace
