#include "ros2plan_action_interface/RPActionInterface.h"

namespace KCL_ros2plan {

    RPActionInterface::RPActionInterface(const std::string& node_name)
        : rclcpp::Node(node_name),
          action_success(false),
          action_cancelled(false)
    {
        srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    }

    void RPActionInterface::setup() {

        // set action name
        declare_parameter("pddl_action_name", std::string(""));
        declare_parameter("knowledge_base", std::string("ros2plan_knowledge_base"));
        declare_parameter("action_feedback_topic", std::string("action_feedback"));
        declare_parameter("action_dispatch_topic", std::string("action_dispatch"));

        params.name = get_parameter("pddl_action_name").as_string();
        std::string kb = get_parameter("knowledge_base").as_string();

        // Satellite node for blocking KB calls in setup() (executor not yet running)
        auto svc_node = std::make_shared<rclcpp::Node>("rp_action_setup_svc_node");

        // fetch action params from KB
        std::string op_details_svc = "/" + kb + "/domain/operator_details";
        auto op_client = svc_node->create_client<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService>(op_details_svc);

        if (!op_client->wait_for_service(std::chrono::seconds(20))) {
            RCLCPP_ERROR(get_logger(), "KCL: (RPActionInterface) KB operator_details service unavailable: %s",
                         op_details_svc.c_str());
            return;
        }
        auto opReq = std::make_shared<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Request>();
        opReq->name = params.name;
        auto opFuture = op_client->async_send_request(opReq);
        if (rclcpp::spin_until_future_complete(svc_node, opFuture, std::chrono::seconds(10))
                != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "KCL: (RPActionInterface) could not call KB for operator details: %s",
                         params.name.c_str());
            return;
        }
        auto opRes = opFuture.get();
        params = opRes->op.formula;
        op     = opRes->op;

        // collect predicate names from operator
        std::vector<std::string> predicateNames;
        for (auto& e : op.at_start_add_effects) predicateNames.push_back(e.name);
        for (auto& e : op.at_start_del_effects) predicateNames.push_back(e.name);
        for (auto& e : op.at_end_add_effects)   predicateNames.push_back(e.name);
        for (auto& e : op.at_end_del_effects)   predicateNames.push_back(e.name);
        for (auto& e : op.at_start_simple_condition) predicateNames.push_back(e.name);
        for (auto& e : op.over_all_simple_condition) predicateNames.push_back(e.name);
        for (auto& e : op.at_end_simple_condition)   predicateNames.push_back(e.name);
        for (auto& e : op.at_start_neg_condition) predicateNames.push_back(e.name);
        for (auto& e : op.over_all_neg_condition) predicateNames.push_back(e.name);
        for (auto& e : op.at_end_neg_condition)   predicateNames.push_back(e.name);

        // fetch and store predicate details
        std::string pred_details_svc = "/" + kb + "/domain/predicate_details";
        auto pred_client = svc_node->create_client<ros2plan_knowledge_msgs::srv::GetDomainPredicateDetailsService>(pred_details_svc);
        if (!pred_client->wait_for_service(std::chrono::seconds(20))) {
            RCLCPP_ERROR(get_logger(), "KCL: (RPActionInterface) KB predicate_details service unavailable");
            return;
        }

        for (auto& name : predicateNames) {
            if (predicates.find(name) != predicates.end()) continue;
            if (name == "=" || name == ">" || name == "<" || name == ">=" || name == "<=") continue;

            auto predReq = std::make_shared<ros2plan_knowledge_msgs::srv::GetDomainPredicateDetailsService::Request>();
            predReq->name = name;
            auto predFuture = pred_client->async_send_request(predReq);
            if (rclcpp::spin_until_future_complete(svc_node, predFuture, std::chrono::seconds(5))
                    != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(get_logger(), "KCL: (RPActionInterface) could not call KB for predicate details: %s",
                             name.c_str());
                return;
            }
            auto predRes = predFuture.get();
            if (predRes->is_sensed) {
                sensed_predicates.insert(std::make_pair(name, predRes->predicate));
            } else {
                predicates.insert(std::make_pair(name, predRes->predicate));
            }
        }

        // PDDL action parameters publisher (latched, 1 Hz)
        std::string pddl_pub_topic = "/" + kb + "/pddl_action_parameters";
        pddl_action_parameters_pub_ = create_publisher<ros2plan_knowledge_msgs::msg::DomainFormula>(
            pddl_pub_topic, rclcpp::QoS(10).transient_local());

        param_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { pddl_action_parameters_pub_->publish(params); });

        // Action feedback publisher
        std::string aft = get_parameter("action_feedback_topic").as_string();
        action_feedback_pub_ = create_publisher<ros2plan_dispatch_msgs::msg::ActionFeedback>(
            aft, rclcpp::QoS(10).transient_local());

        // KB update client — created on a persistent satellite node so update calls
        // inside dispatchCallback() can be spun without blocking the executor.
        kb_svc_node_ = std::make_shared<rclcpp::Node>("rp_action_kb_svc_node");
        update_knowledge_client_ = kb_svc_node_->create_client<ros2plan_knowledge_msgs::srv::KnowledgeUpdateServiceArray>(
            "/" + kb + "/update_array");

        // Action dispatch subscription
        std::string adt = get_parameter("action_dispatch_topic").as_string();
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_cbg_;
        dispatch_sub_ = create_subscription<ros2plan_dispatch_msgs::msg::ActionDispatch>(
            adt, 1000,
            [this](const ros2plan_dispatch_msgs::msg::ActionDispatch::SharedPtr msg) { dispatchCallback(msg); },
            sub_opts);

        RCLCPP_INFO(get_logger(), "KCL: (%s) Ready to receive", params.name.c_str());
    }

    void RPActionInterface::dispatchCallback(const ros2plan_dispatch_msgs::msg::ActionDispatch::SharedPtr msg) {

        if (msg->name == "cancel_action") {
            action_cancelled = true;
            return;
        }
        if (msg->name != params.name) return;
        RCLCPP_INFO(get_logger(), "KCL: (%s) action received", params.name.c_str());

        action_cancelled = false;

        // check PDDL parameters
        std::vector<bool> found(params.typed_parameters.size(), false);
        std::map<std::string, std::string> boundParameters;
        for (size_t j = 0; j < params.typed_parameters.size(); j++) {
            for (size_t i = 0; i < msg->parameters.size(); i++) {
                if (params.typed_parameters[j].key == msg->parameters[i].key) {
                    boundParameters[msg->parameters[i].key] = msg->parameters[i].value;
                    found[j] = true;
                    break;
                }
            }
            if (!found[j]) {
                RCLCPP_INFO(get_logger(),
                    "KCL: (%s) aborting action dispatch; malformed parameters, missing %s",
                    params.name.c_str(), params.typed_parameters[j].key.c_str());
                return;
            }
        }

        // send feedback (enabled)
        ros2plan_dispatch_msgs::msg::ActionFeedback fb;
        fb.action_id = msg->action_id;
        fb.status = ros2plan_dispatch_msgs::msg::ActionFeedback::ACTION_ENABLED;
        action_feedback_pub_->publish(fb);

        {
            // update KB for at-start effects
            auto updatePredSrv = std::make_shared<ros2plan_knowledge_msgs::srv::KnowledgeUpdateServiceArray::Request>();

            for (auto& e : op.at_start_del_effects) {
                if (sensed_predicates.find(e.name) != sensed_predicates.end()) continue;
                ros2plan_knowledge_msgs::msg::KnowledgeItem item;
                item.knowledge_type = ros2plan_knowledge_msgs::msg::KnowledgeItem::FACT;
                item.attribute_name = e.name;
                for (size_t j = 0; j < e.typed_parameters.size(); j++) {
                    diagnostic_msgs::msg::KeyValue pair;
                    pair.key   = predicates[e.name].typed_parameters[j].key;
                    pair.value = boundParameters[e.typed_parameters[j].key];
                    item.values.push_back(pair);
                }
                updatePredSrv->knowledge.push_back(item);
                updatePredSrv->update_type.push_back(
                    ros2plan_knowledge_msgs::srv::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
            }

            for (auto& e : op.at_start_add_effects) {
                if (sensed_predicates.find(e.name) != sensed_predicates.end()) continue;
                ros2plan_knowledge_msgs::msg::KnowledgeItem item;
                item.knowledge_type = ros2plan_knowledge_msgs::msg::KnowledgeItem::FACT;
                item.attribute_name = e.name;
                for (size_t j = 0; j < e.typed_parameters.size(); j++) {
                    diagnostic_msgs::msg::KeyValue pair;
                    pair.key   = predicates[e.name].typed_parameters[j].key;
                    pair.value = boundParameters[e.typed_parameters[j].key];
                    item.values.push_back(pair);
                }
                updatePredSrv->knowledge.push_back(item);
                updatePredSrv->update_type.push_back(
                    ros2plan_knowledge_msgs::srv::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
            }

            if (!updatePredSrv->knowledge.empty()) {
                auto future = update_knowledge_client_->async_send_request(updatePredSrv);
                if (rclcpp::spin_until_future_complete(kb_svc_node_, future, std::chrono::seconds(5))
                        != rclcpp::FutureReturnCode::SUCCESS) {
                    RCLCPP_INFO(get_logger(), "KCL: (%s) failed to update PDDL model in knowledge base",
                                params.name.c_str());
                }
            }
        }

        // call concrete implementation
        action_success = concreteCallback(msg);
        if (action_cancelled) {
            action_success = false;
            RCLCPP_INFO(get_logger(), "KCL: (%s) an old action that was cancelled is stopping now",
                        params.name.c_str());
            return;
        }

        if (action_success) {

            RCLCPP_INFO(get_logger(), "KCL: (%s) action completed successfully", params.name.c_str());

            auto updatePredSrv = std::make_shared<ros2plan_knowledge_msgs::srv::KnowledgeUpdateServiceArray::Request>();

            for (auto& e : op.at_end_del_effects) {
                if (sensed_predicates.find(e.name) != sensed_predicates.end()) continue;
                ros2plan_knowledge_msgs::msg::KnowledgeItem item;
                item.knowledge_type = ros2plan_knowledge_msgs::msg::KnowledgeItem::FACT;
                item.attribute_name = e.name;
                for (size_t j = 0; j < e.typed_parameters.size(); j++) {
                    diagnostic_msgs::msg::KeyValue pair;
                    pair.key   = predicates[e.name].typed_parameters[j].key;
                    pair.value = boundParameters[e.typed_parameters[j].key];
                    item.values.push_back(pair);
                }
                updatePredSrv->knowledge.push_back(item);
                updatePredSrv->update_type.push_back(
                    ros2plan_knowledge_msgs::srv::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
            }

            for (auto& e : op.at_end_add_effects) {
                if (sensed_predicates.find(e.name) != sensed_predicates.end()) continue;
                ros2plan_knowledge_msgs::msg::KnowledgeItem item;
                item.knowledge_type = ros2plan_knowledge_msgs::msg::KnowledgeItem::FACT;
                item.attribute_name = e.name;
                for (size_t j = 0; j < e.typed_parameters.size(); j++) {
                    diagnostic_msgs::msg::KeyValue pair;
                    pair.key   = predicates[e.name].typed_parameters[j].key;
                    pair.value = boundParameters[e.typed_parameters[j].key];
                    item.values.push_back(pair);
                }
                updatePredSrv->knowledge.push_back(item);
                updatePredSrv->update_type.push_back(
                    ros2plan_knowledge_msgs::srv::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
            }

            if (!updatePredSrv->knowledge.empty()) {
                auto future = update_knowledge_client_->async_send_request(updatePredSrv);
                if (rclcpp::spin_until_future_complete(kb_svc_node_, future, std::chrono::seconds(5))
                        != rclcpp::FutureReturnCode::SUCCESS) {
                    RCLCPP_INFO(get_logger(), "KCL: (%s) failed to update PDDL model in knowledge base",
                                params.name.c_str());
                }
            }

            fb.status = ros2plan_dispatch_msgs::msg::ActionFeedback::ACTION_SUCCEEDED_TO_GOAL_STATE;
            action_feedback_pub_->publish(fb);

        } else {

            fb.status = ros2plan_dispatch_msgs::msg::ActionFeedback::ACTION_FAILED;
            action_feedback_pub_->publish(fb);
        }
    }

} // close namespace
