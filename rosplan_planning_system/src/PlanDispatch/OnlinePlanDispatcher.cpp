//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#include "rosplan_planning_system/PlanDispatch/OnlinePlanDispatcher.h"
#include <regex>

namespace KCL_rosplan {

    OnlinePlanDispatcher::OnlinePlanDispatcher()
        : PlanDispatcher("rosplan_online_plan_dispatcher"),
          current_action(0),
          server_port_(3234),
          compute_rewards_(false)
    {
        declare_parameter("ippc_server_port", 3234);
        declare_parameter("compute_rewards", false);
        declare_parameter("plan_topic", std::string("complete_plan"));

        server_port_      = get_parameter("ippc_server_port").as_int();
        compute_rewards_  = get_parameter("compute_rewards").as_bool();
        std::string plan_topic = get_parameter("plan_topic").as_string();
        std::string feedback_topic = get_parameter("action_feedback_topic").as_string();

        queryPropositionsClient_ = create_client<rosplan_knowledge_msgs::srv::GetAttributeService>(
            "/" + kb_ + "/state/propositions");
        get_rddl_params_client_ = create_client<rosplan_knowledge_msgs::srv::GetRDDLParams>(
            "/" + kb_ + "/state/rddl_parameters");
        get_immediate_reward_client_ = create_client<rosplan_knowledge_msgs::srv::GetRDDLImmediateReward>(
            "/" + kb_ + "/state/get_immediate_reward");
        get_planner_params_client_ = create_client<rosplan_dispatch_msgs::srv::GetPlanningParams>(
            "/rosplan_planner_interface/get_planning_params");

        plan_publisher_ = create_publisher<rosplan_dispatch_msgs::msg::CompletePlan>(
            plan_topic,
            rclcpp::QoS(1).transient_local());

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_cbg_;
        feedback_sub_ = create_subscription<rosplan_dispatch_msgs::msg::ActionFeedback>(
            feedback_topic, 1000,
            [this](const rosplan_dispatch_msgs::msg::ActionFeedback::SharedPtr msg) { feedbackCallback(msg); },
            sub_opts);

        reset();

        RCLCPP_INFO(get_logger(), "KCL: (%s) Ready to receive.", get_name());
    }

    void OnlinePlanDispatcher::reset() {
        PlanDispatcher::reset();
        current_action = 0;
    }

    void OnlinePlanDispatcher::dispatchPlanService(
        const std::shared_ptr<rosplan_dispatch_msgs::srv::DispatchService::Request> req,
        std::shared_ptr<rosplan_dispatch_msgs::srv::DispatchService::Response> res)
    {
        mission_start_time = rclcpp::Clock().now().seconds();
        PlanDispatcher::dispatchPlanService(req, res);
    }

    void OnlinePlanDispatcher::dispatchPlanActionlib(std::shared_ptr<GoalHandleNonBlockingDispatch> goal_handle) {
        mission_start_time = rclcpp::Clock().now().seconds();
        PlanDispatcher::dispatchPlanActionlib(goal_handle);
    }

    bool OnlinePlanDispatcher::dispatchPlan(double missionStartTime, double planStartTime) {
        RCLCPP_INFO(get_logger(), "KCL: (%s) Dispatching online plan", get_name());

        auto paramsReq = std::make_shared<rosplan_knowledge_msgs::srv::GetRDDLParams::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetRDDLParams::Response> paramsRes;
        if (!callServiceSync(get_rddl_params_client_, paramsReq, paramsRes)) {
            RCLCPP_ERROR(get_logger(),
                "KCL: (%s) Failed to call service to get RDDL parameters! Are you using a RDDL KnowledgeBase?",
                get_name());
            rclcpp::shutdown();
            return false;
        }

        auto plannerParamsReq = std::make_shared<rosplan_dispatch_msgs::srv::GetPlanningParams::Request>();
        std::shared_ptr<rosplan_dispatch_msgs::srv::GetPlanningParams::Response> plannerParamsRes;
        if (!callServiceSync(get_planner_params_client_, plannerParamsReq, plannerParamsRes)) {
            RCLCPP_ERROR(get_logger(),
                "KCL: (%s) Failed to call service to get planner parameters! Is the OnlinePlanningInterface running?",
                get_name());
            rclcpp::shutdown();
            return false;
        }

        if (!plannerParamsRes->planner_ready) {
            RCLCPP_WARN(get_logger(),
                "KCL: (%s) Planner is not ready! Did you call the plan_server to start the online planner thread?",
                get_name());
            return false;
        }

        RCLCPP_INFO(get_logger(), "KCL: (%s) Starting IPPC server on port %d and waiting for connections!",
                    get_name(), server_port_);
        XMLServer_t ippcserver;
        std::string client = ippcserver.start_session(
            server_port_, plannerParamsRes->domain_path, plannerParamsRes->problem_path);
        RCLCPP_INFO(get_logger(), "KCL: (%s) Planner %s connected! Starting planning round",
                    get_name(), client.c_str());
        ippcserver.start_round();
        float planning_result;
        rclcpp::Duration total_planning_time = rclcpp::Duration::from_seconds(0);
        std::string action;
        bool first_action = true;
        std::regex commas(",(\\S)");

        for (int horizon = 0; horizon < paramsRes->horizon; ++horizon) {
            while (rclcpp::ok() && dispatch_paused) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (plan_cancelled) break;

            auto stateReq = std::make_shared<rosplan_knowledge_msgs::srv::GetAttributeService::Request>();
            std::shared_ptr<rosplan_knowledge_msgs::srv::GetAttributeService::Response> stateRes;
            if (!callServiceSync(queryPropositionsClient_, stateReq, stateRes)) {
                RCLCPP_ERROR(get_logger(), "KCL: (%s) Failed to call service to get current state.", get_name());
            }

            try {
                rclcpp::Time start = rclcpp::Clock().now();
                double reward = 0;
                if (compute_rewards_ && !first_action) {
                    auto rwdReq = std::make_shared<rosplan_knowledge_msgs::srv::GetRDDLImmediateReward::Request>();
                    if (action.empty()) action = "noop";
                    std::istringstream ss(action);
                    std::string a;
                    while (std::getline(ss, a, ';')) {
                        std_msgs::msg::String msg;
                        std::regex_replace(std::back_inserter(msg.data), a.begin(), a.end(), commas, ", $1");
                        rwdReq->action.push_back(msg);
                    }
                    std::shared_ptr<rosplan_knowledge_msgs::srv::GetRDDLImmediateReward::Response> rwdRes;
                    if (!callServiceSync(get_immediate_reward_client_, rwdReq, rwdRes)) {
                        RCLCPP_ERROR(get_logger(), "KCL: (%s) Failed to call service to get immediate reward.", get_name());
                    } else {
                        reward = rwdRes->reward;
                    }
                } else {
                    first_action = false;
                }
                action = ippcserver.get_action(stateRes ? stateRes->attributes
                                                        : std::vector<rosplan_knowledge_msgs::msg::KnowledgeItem>{},
                                               planning_result, reward);
                total_planning_time = total_planning_time + (rclcpp::Clock().now() - start);
            } catch (std::runtime_error& e) {
                RCLCPP_ERROR(get_logger(), "KCL: (%s) %s", get_name(), e.what());
                break;
            }

            if (action.empty()) continue;
            RCLCPP_INFO(get_logger(), "KCL: (%s) Received new action from planner: %s", get_name(), action.c_str());

            std::vector<rosplan_dispatch_msgs::msg::ActionDispatch> action_msgs = toActionMsg(action, current_action);
            for (auto& it : action_msgs) {
                dispatchOneAction(it, missionStartTime, planStartTime);
                current_plan.plan.push_back(it);
                plan_publisher_->publish(current_plan);
                ++current_action;
                action_received[current_action]  = false;
                action_completed[current_action] = false;
            }
        }

        ippcserver.end_round();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ippcserver.end_session();

        RCLCPP_INFO(get_logger(), "KCL: (%s) Dispatch complete.", get_name());
        RCLCPP_INFO(get_logger(), "KCL: (%s) Total planning time: %f seconds",
                    get_name(), total_planning_time.seconds());
        return true;
    }

    void OnlinePlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::msg::ActionFeedback::SharedPtr msg) {

        RCLCPP_INFO(get_logger(), "KCL: (%s) Feedback received [%i, %d]",
                    get_name(), msg->action_id, msg->status);

        if (current_action != static_cast<unsigned int>(msg->action_id)) {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) Unexpected action ID: %d; current action: %d",
                         get_name(), msg->action_id, current_action);
        }

        if (!action_received[msg->action_id]
            && msg->status == rosplan_dispatch_msgs::msg::ActionFeedback::ACTION_ENABLED)
        {
            action_received[msg->action_id] = true;
        } else if (!action_completed[msg->action_id]
                   && msg->status == rosplan_dispatch_msgs::msg::ActionFeedback::ACTION_SUCCEEDED_TO_GOAL_STATE)
        {
            action_completed[msg->action_id] = true;
        } else if (!action_completed[msg->action_id]
                   && msg->status == rosplan_dispatch_msgs::msg::ActionFeedback::ACTION_FAILED)
        {
            replan_requested = true;
            action_completed[msg->action_id] = true;
        } else if (!action_completed[msg->action_id]
                   && msg->status == rosplan_dispatch_msgs::msg::ActionFeedback::ACTION_PRECONDITION_FALSE)
        {
            replan_requested = true;
            action_completed[msg->action_id] = true;
        }
    }

    void OnlinePlanDispatcher::dispatchOneAction(
        rosplan_dispatch_msgs::msg::ActionDispatch current_action_msg,
        double missionStartTime, double planStartTime)
    {
        if (!checkPreconditions(current_action_msg)) {
            RCLCPP_WARN(get_logger(), "KCL: (%s) Preconditions not achieved [%i, %s]",
                        get_name(), current_action_msg.action_id, current_action_msg.name.c_str());

            rosplan_dispatch_msgs::msg::ActionFeedback fb;
            fb.action_id = current_action_msg.action_id;
            fb.status = rosplan_dispatch_msgs::msg::ActionFeedback::ACTION_PRECONDITION_FALSE;
            publishFeedback(fb);

            replan_requested = true;
        } else {
            RCLCPP_INFO(get_logger(), "KCL: (%s) Dispatching action [%i, %s, %f, %f]",
                        get_name(),
                        current_action_msg.action_id,
                        current_action_msg.name.c_str(),
                        (current_action_msg.dispatch_time + planStartTime - missionStartTime),
                        current_action_msg.duration);

            action_dispatch_publisher_->publish(current_action_msg);

            rosplan_dispatch_msgs::msg::ActionFeedback fb;
            fb.action_id = current_action_msg.action_id;
            fb.status = rosplan_dispatch_msgs::msg::ActionFeedback::ACTION_DISPATCHED_TO_GOAL_STATE;
            publishFeedback(fb);

            double late_print = rclcpp::Clock().now().seconds() - (current_action_msg.dispatch_time + planStartTime);
            if (late_print > 0.1) {
                RCLCPP_INFO(get_logger(), "KCL: (%s) Action [%i] is %f second(s) late",
                            get_name(), current_action_msg.action_id, late_print);
            }
        }

        while (rclcpp::ok() && !action_completed[current_action]) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::vector<rosplan_dispatch_msgs::msg::ActionDispatch> OnlinePlanDispatcher::toActionMsg(
        std::string resp_actions, int action_id)
    {
        std::vector<rosplan_dispatch_msgs::msg::ActionDispatch> ret;
        std::regex action_name_params_rgx("(.+)\\((.*)\\)|,?\\s?(.+)");
        std::smatch match;

        std::istringstream action_list(resp_actions);
        std::string action_str;
        while (std::getline(action_list, action_str, ';')) {
            rosplan_dispatch_msgs::msg::ActionDispatch action_msg;
            action_msg.action_id = action_id;
            if (std::regex_search(action_str, match, action_name_params_rgx)) {
                std::string action_name = match[1].str().empty() ? match[3].str() : match[1].str();
                action_msg.name = action_name;

                auto opReq = std::make_shared<rosplan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Request>();
                opReq->name = action_name;
                std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Response> opRes;
                if (!callServiceSync(queryDomainClient_, opReq, opRes)) {
                    RCLCPP_ERROR(get_logger(), "KCL: (%s) could not call KB for operator details: %s",
                                 get_name(), action_name.c_str());
                    return ret;
                }

                action_msg.parameters = opRes->op.formula.typed_parameters;
                std::istringstream p(match[2].str());
                std::string param;
                size_t param_idx = 0;
                while (std::getline(p, param, ',') && param_idx < action_msg.parameters.size()) {
                    action_msg.parameters[param_idx].value = param;
                    ++param_idx;
                }
            }
            action_msg.duration      = 0.001;
            action_msg.dispatch_time = rclcpp::Clock().now().seconds();
            ret.push_back(action_msg);
            ++action_id;
        }

        return ret;
    }

} // close namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_rosplan::OnlinePlanDispatcher>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
