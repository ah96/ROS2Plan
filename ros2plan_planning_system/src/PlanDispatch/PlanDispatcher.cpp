//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#include "ros2plan_planning_system/PlanDispatch/PlanDispatcher.h"

namespace KCL_ros2plan {

    PlanDispatcher::PlanDispatcher(const std::string& node_name)
        : rclcpp::Node(node_name),
          dispatching(false),
          plan_received(false),
          replan_requested(false),
          dispatch_paused(false),
          plan_cancelled(false),
          mission_start_time(0.0),
          as_active_(false)
    {
        declare_parameter("knowledge_base", std::string("ros2plan_knowledge_base"));
        declare_parameter("action_dispatch_topic", std::string("action_dispatch"));
        declare_parameter("action_feedback_topic", std::string("action_feedback"));

        kb_                   = get_parameter("knowledge_base").as_string();
        action_dispatch_topic = get_parameter("action_dispatch_topic").as_string();
        action_feedback_topic = get_parameter("action_feedback_topic").as_string();

        // Callback groups
        srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // KB service clients
        queryKnowledgeClient_ = create_client<ros2plan_knowledge_msgs::srv::KnowledgeQueryService>(
            "/" + kb_ + "/query_state");
        queryDomainClient_ = create_client<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService>(
            "/" + kb_ + "/domain/operator_details");
        get_goals_client_ = create_client<ros2plan_knowledge_msgs::srv::GetAttributeService>(
            "/" + kb_ + "/state/goals");

        // Publishers
        action_dispatch_publisher_ = create_publisher<ros2plan_dispatch_msgs::msg::ActionDispatch>(
            action_dispatch_topic,
            rclcpp::QoS(1).transient_local());
        action_feedback_publisher_ = create_publisher<ros2plan_dispatch_msgs::msg::ActionFeedback>(
            action_feedback_topic,
            rclcpp::QoS(1).transient_local());

        // Action server
        dispatch_action_server_ = rclcpp_action::create_server<NonBlockingDispatchAction>(
            this,
            "dispatch_plan_action",
            [this](const rclcpp_action::GoalUUID& /*uuid*/,
                   std::shared_ptr<const NonBlockingDispatchAction::Goal> /*goal*/) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](std::shared_ptr<GoalHandleNonBlockingDispatch> /*gh*/) {
                cancelDispatch();
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](std::shared_ptr<GoalHandleNonBlockingDispatch> gh) {
                std::thread([this, gh]() { dispatchPlanActionlib(gh); }).detach();
            });

        // Services
        dispatch_plan_srv_ = create_service<ros2plan_dispatch_msgs::srv::DispatchService>(
            "dispatch_plan",
            [this](const std::shared_ptr<ros2plan_dispatch_msgs::srv::DispatchService::Request> req,
                   std::shared_ptr<ros2plan_dispatch_msgs::srv::DispatchService::Response> res) {
                dispatchPlanService(req, res);
            },
            rclcpp::ServicesQoS(),
            srv_cbg_);

        cancel_dispatch_srv_ = create_service<std_srvs::srv::Empty>(
            "cancel_dispatch",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                   std::shared_ptr<std_srvs::srv::Empty::Response> res) {
                cancelDispatchService(req, res);
            },
            rclcpp::ServicesQoS(),
            srv_cbg_);
    }

    void PlanDispatcher::publishFeedback(const ros2plan_dispatch_msgs::msg::ActionFeedback& fb) {
        std::lock_guard<std::mutex> lock(as_mutex_);
        if (as_active_ && current_goal_handle_) {
            auto afeedback = std::make_shared<NonBlockingDispatchAction::Feedback>();
            afeedback->feedback = fb;
            current_goal_handle_->publish_feedback(afeedback);
        }
        action_feedback_publisher_->publish(fb);
    }

    bool PlanDispatcher::cancelDispatch() {
        RCLCPP_INFO(get_logger(), "KCL: (%s) Cancel plan command received.", get_name());
        {
            std::lock_guard<std::mutex> lock(as_mutex_);
            if (as_active_ && current_goal_handle_) {
                auto result = std::make_shared<NonBlockingDispatchAction::Result>();
                result->success = false;
                result->goal_achieved = false;
                current_goal_handle_->abort(result);
                as_active_ = false;
                current_goal_handle_.reset();
            }
        }
        plan_cancelled = true;
        return true;
    }

    void PlanDispatcher::cancelDispatchService(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
    {
        cancelDispatch();
    }

    void PlanDispatcher::dispatchPlanService(
        const std::shared_ptr<ros2plan_dispatch_msgs::srv::DispatchService::Request> /*req*/,
        std::shared_ptr<ros2plan_dispatch_msgs::srv::DispatchService::Response> res)
    {
        if (dispatching) {
            res->success = false;
            res->goal_achieved = false;
            return;
        }
        dispatching = true;
        bool success = dispatchPlan(mission_start_time, rclcpp::Clock().now().seconds());
        dispatching = false;
        reset();
        res->success = success;
        res->goal_achieved = goalAchieved();
    }

    void PlanDispatcher::dispatchPlanActionlib(std::shared_ptr<GoalHandleNonBlockingDispatch> goal_handle) {
        {
            std::lock_guard<std::mutex> lock(as_mutex_);
            if (as_active_ || dispatching) {
                RCLCPP_WARN(get_logger(),
                    "KCL: (%s) Got a new dispatch request but a plan is already being dispatched! Ignoring.",
                    get_name());
                auto result = std::make_shared<NonBlockingDispatchAction::Result>();
                result->success = false;
                result->goal_achieved = false;
                goal_handle->abort(result);
                return;
            }
            as_active_ = true;
            current_goal_handle_ = goal_handle;
        }

        dispatching = true;
        bool success = dispatchPlan(mission_start_time, rclcpp::Clock().now().seconds());
        dispatching = false;
        reset();

        auto result = std::make_shared<NonBlockingDispatchAction::Result>();
        result->success = success;
        result->goal_achieved = goalAchieved();

        {
            std::lock_guard<std::mutex> lock(as_mutex_);
            if (as_active_ && current_goal_handle_) {
                current_goal_handle_->succeed(result);
                as_active_ = false;
                current_goal_handle_.reset();
            }
        }
    }

    void PlanDispatcher::reset() {
        replan_requested  = false;
        dispatch_paused   = false;
        plan_cancelled    = false;
        action_received.clear();
        action_completed.clear();
        plan_received = false;
        dispatching   = false;
    }

    bool PlanDispatcher::checkPreconditions(ros2plan_dispatch_msgs::msg::ActionDispatch msg) {

        auto opReq = std::make_shared<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Request>();
        opReq->name = msg.name;
        std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Response> opRes;
        if (!callServiceSync(queryDomainClient_, opReq, opRes)) {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) could not call KB for operator details: %s",
                         get_name(), msg.name.c_str());
            return false;
        }

        auto& op = opRes->op;

        // Build positive query
        auto positiveQuery = std::make_shared<ros2plan_knowledge_msgs::srv::KnowledgeQueryService::Request>();
        for (auto& cit : op.at_start_simple_condition) {
            ros2plan_knowledge_msgs::msg::KnowledgeItem condition;
            condition.knowledge_type = ros2plan_knowledge_msgs::msg::KnowledgeItem::FACT;
            condition.attribute_name = cit.name;
            for (size_t i = 0; i < cit.typed_parameters.size(); i++) {
                diagnostic_msgs::msg::KeyValue param;
                param.key = cit.typed_parameters[i].key;
                for (size_t j = 0; j < msg.parameters.size() && j < op.formula.typed_parameters.size(); j++) {
                    if (op.formula.typed_parameters[j].key == cit.typed_parameters[i].key) {
                        param.value = msg.parameters[j].value;
                    }
                }
                condition.values.push_back(param);
            }
            positiveQuery->knowledge.push_back(condition);
        }

        // Build negative query
        auto negativeQuery = std::make_shared<ros2plan_knowledge_msgs::srv::KnowledgeQueryService::Request>();
        for (auto& cit : op.at_start_neg_condition) {
            ros2plan_knowledge_msgs::msg::KnowledgeItem condition;
            condition.knowledge_type = ros2plan_knowledge_msgs::msg::KnowledgeItem::FACT;
            condition.attribute_name = cit.name;
            for (size_t i = 0; i < cit.typed_parameters.size(); i++) {
                diagnostic_msgs::msg::KeyValue param;
                param.key = cit.typed_parameters[i].key;
                for (size_t j = 0; j < msg.parameters.size() && j < op.formula.typed_parameters.size(); j++) {
                    if (op.formula.typed_parameters[j].key == cit.typed_parameters[i].key) {
                        param.value = msg.parameters[j].value;
                    }
                }
                condition.values.push_back(param);
            }
            negativeQuery->knowledge.push_back(condition);
        }

        bool neg_preconditions = false;

        if (!negativeQuery->knowledge.empty()) {
            std::shared_ptr<ros2plan_knowledge_msgs::srv::KnowledgeQueryService::Response> negRes;
            if (callServiceSync(queryKnowledgeClient_, negativeQuery, negRes)) {
                for (size_t i = 0; i < negRes->results.size(); i++) {
                    if (negRes->results[i]) {
                        neg_preconditions = true;
                        RCLCPP_INFO(get_logger(), "Negative precondition not achieved: not (%s)",
                                    negativeQuery->knowledge[i].attribute_name.c_str());
                    }
                }
            } else {
                RCLCPP_ERROR(get_logger(), "KCL: (%s) Failed to call service query_state", get_name());
            }
        }

        if (!positiveQuery->knowledge.empty()) {
            std::shared_ptr<ros2plan_knowledge_msgs::srv::KnowledgeQueryService::Response> posRes;
            if (callServiceSync(queryKnowledgeClient_, positiveQuery, posRes)) {
                if (!posRes->all_true) {
                    for (auto& kit : posRes->false_knowledge) {
                        RCLCPP_INFO(get_logger(), "KCL: (%s) Precondition not achieved: %s",
                                    get_name(), kit.attribute_name.c_str());
                    }
                }
                return posRes->all_true && !neg_preconditions;
            } else {
                RCLCPP_ERROR(get_logger(), "KCL: (%s) Failed to call service query_state", get_name());
            }
        }

        return !neg_preconditions;
    }

    bool PlanDispatcher::goalAchieved() {
        auto goalReq = std::make_shared<ros2plan_knowledge_msgs::srv::GetAttributeService::Request>();
        std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Response> goalRes;
        if (!callServiceSync(get_goals_client_, goalReq, goalRes)) {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) Failed to call service get_goals", get_name());
            return false;
        }
        if (goalRes->attributes.empty()) return true;

        auto queryReq = std::make_shared<ros2plan_knowledge_msgs::srv::KnowledgeQueryService::Request>();
        queryReq->knowledge = goalRes->attributes;
        std::shared_ptr<ros2plan_knowledge_msgs::srv::KnowledgeQueryService::Response> queryRes;
        if (!callServiceSync(queryKnowledgeClient_, queryReq, queryRes)) {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) Failed to call service query_state", get_name());
            return false;
        }
        return queryRes->all_true;
    }

} // close namespace
