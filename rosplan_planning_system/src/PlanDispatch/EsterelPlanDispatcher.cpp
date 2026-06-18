#include "rosplan_planning_system/PlanDispatch/EsterelPlanDispatcher.h"

namespace KCL_rosplan {

    EsterelPlanDispatcher::EsterelPlanDispatcher()
        : PlanDispatcher("rosplan_esterel_plan_dispatcher"),
          display_edge_type_(false),
          finished_execution(true),
          state_changed(false),
          timeout_actions(true),
          action_timeout_fraction(0)
    {
        declare_parameter("plan_topic", std::string("complete_plan"));
        declare_parameter("plan_graph_topic", std::string("plan_graph"));
        declare_parameter("display_edge_type", false);
        declare_parameter("timeout_actions", true);
        declare_parameter("action_timeout_fraction", 0.0);

        timeout_actions          = get_parameter("timeout_actions").as_bool();
        action_timeout_fraction  = get_parameter("action_timeout_fraction").as_double();
        display_edge_type_       = get_parameter("display_edge_type").as_bool();

        std::string plan_topic        = get_parameter("plan_topic").as_string();
        std::string plan_graph_topic  = get_parameter("plan_graph_topic").as_string();
        std::string feedback_topic    = get_parameter("action_feedback_topic").as_string();

        plan_graph_publisher_ = create_publisher<std_msgs::msg::String>(
            plan_graph_topic,
            rclcpp::QoS(1000).transient_local());

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_cbg_;

        plan_sub_ = create_subscription<rosplan_dispatch_msgs::msg::EsterelPlan>(
            plan_topic, 1,
            [this](const rosplan_dispatch_msgs::msg::EsterelPlan::SharedPtr msg) { planCallback(msg); },
            sub_opts);

        feedback_sub_ = create_subscription<rosplan_dispatch_msgs::msg::ActionFeedback>(
            feedback_topic, 1000,
            [this](const rosplan_dispatch_msgs::msg::ActionFeedback::SharedPtr msg) { feedbackCallback(msg); },
            sub_opts);

        RCLCPP_INFO(get_logger(), "KCL: (%s) Ready to receive.", get_name());
    }

    void EsterelPlanDispatcher::reset() {

        for (auto& node : current_plan.nodes) {
            if (node.node_type == rosplan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START
                && action_dispatched[node.action.action_id]
                && !action_completed[node.action.action_id])
            {
                RCLCPP_INFO(get_logger(), "KCL: (%s) Preempting action [%i, %s]",
                            get_name(), node.action.action_id, node.action.name.c_str());
                auto cancelMsg = node.action;
                cancelMsg.name = "cancel_action";
                action_dispatch_publisher_->publish(cancelMsg);
            }
        }

        PlanDispatcher::reset();
        finished_execution = true;
    }

    void EsterelPlanDispatcher::planCallback(const rosplan_dispatch_msgs::msg::EsterelPlan::SharedPtr plan) {
        if (finished_execution) {
            RCLCPP_INFO(get_logger(), "KCL: (%s) Plan received.", get_name());
            plan_received = true;
            mission_start_time = rclcpp::Clock().now().seconds();
            current_plan = *plan;
            printPlan();
        } else {
            RCLCPP_INFO(get_logger(), "KCL: (%s) Plan received, but current execution not yet finished.", get_name());
        }
    }

    bool EsterelPlanDispatcher::dispatchPlan(double missionStartTime, double planStartTime) {

        RCLCPP_INFO(get_logger(), "KCL: (%s) Dispatching plan.", get_name());

        replan_requested = false;
        plan_cancelled   = false;

        initialise();

        finished_execution = false;
        state_changed = false;
        bool plan_started = false;

        while (rclcpp::ok() && !finished_execution) {

            while (rclcpp::ok() && dispatch_paused) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (plan_cancelled) {
                RCLCPP_INFO(get_logger(), "KCL: (%s) Plan cancelled.", get_name());
                break;
            }

            finished_execution = true;
            state_changed = false;

            for (auto& node : current_plan.nodes) {

                if (node.node_type == rosplan_dispatch_msgs::msg::EsterelPlanNode::PLAN_START && !plan_started) {
                    double NOW = rclcpp::Clock().now().seconds();
                    node_real_dispatch_time.insert(std::make_pair(node.node_id, NOW));
                    for (int eid : node.edges_out) edge_active[eid] = true;
                    finished_execution = false;
                    state_changed = true;
                    plan_started = true;
                }

                if (node.node_type != rosplan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START
                    && node.node_type != rosplan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END)
                    continue;

                if (action_dispatched[node.action.action_id] && !action_completed[node.action.action_id])
                    finished_execution = false;

                bool edges_activate_action = true;
                for (int eid : node.edges_in) {
                    if (!edge_active[eid]) { edges_activate_action = false; break; }
                }
                if (!edges_activate_action) continue;

                if (node.node_type == rosplan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END
                    && action_completed[node.action.action_id])
                {
                    RCLCPP_INFO(get_logger(), "KCL: (%s) %i: action %s completed",
                                get_name(), node.action.action_id, node.action.name.c_str());
                    finished_execution = false;
                    state_changed = true;
                    for (int eid : node.edges_in)  edge_active[eid] = false;
                    for (int eid : node.edges_out) edge_active[eid] = true;
                }

                if (timeout_actions) {
                    bool times_activate_action = true;
                    for (int eid : node.edges_in) {
                        auto& edge = current_plan.edges[eid];
                        float min_dt = node_real_dispatch_time[edge.source_ids[0]] + edge.duration_lower_bound - planStartTime;
                        float max_dt = node_real_dispatch_time[edge.source_ids[0]] + edge.duration_upper_bound - planStartTime;
                        min_dt = min_dt - action_timeout_fraction * min_dt;
                        max_dt = max_dt + action_timeout_fraction * max_dt;

                        double NOW = rclcpp::Clock().now().seconds();
                        if (NOW - planStartTime < min_dt) {
                            times_activate_action = false;
                            finished_execution = false;
                            break;
                        }
                        if (NOW - planStartTime > max_dt) {
                            if (node.node_type != rosplan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END
                                || !action_completed[node.action.action_id])
                            {
                                replan_requested = true;
                                times_activate_action = false;
                                RCLCPP_INFO(get_logger(), "KCL: (%s) Deadline passed, %s: %f > %f.",
                                            get_name(), node.name.c_str(), NOW - planStartTime, max_dt);
                            }
                        }
                    }
                    if (!times_activate_action) continue;
                }

                if (node.node_type == rosplan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START
                    && !action_dispatched[node.action.action_id])
                {
                    finished_execution = false;
                    bool condition_activate_action = false;
                    if (edges_activate_action) {
                        condition_activate_action = checkPreconditions(node.action);
                    }
                    if (!condition_activate_action && timeout_actions) {
                        replan_requested = true;
                    }
                    if (condition_activate_action) {
                        action_dispatched[node.action.action_id] = true;
                        action_received[node.action.action_id]   = false;
                        action_completed[node.action.action_id]  = false;

                        RCLCPP_INFO(get_logger(), "KCL: (%s) Dispatching action [%i, %s, %f, %f]",
                                    get_name(),
                                    node.action.action_id,
                                    node.action.name.c_str(),
                                    (node.action.dispatch_time + planStartTime - missionStartTime),
                                    node.action.duration);

                        action_dispatch_publisher_->publish(node.action);

                        double NOW = rclcpp::Clock().now().seconds();
                        node_real_dispatch_time.insert(std::make_pair(node.node_id, NOW));
                        state_changed = true;

                        for (int eid : node.edges_in)  edge_active[eid] = false;
                        for (int eid : node.edges_out) edge_active[eid] = true;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if (state_changed) printPlan();

            if (replan_requested) {
                RCLCPP_INFO(get_logger(), "KCL: (%s) Replan requested.", get_name());
                reset();
                return false;
            }
        }

        RCLCPP_INFO(get_logger(), "KCL: (%s) Dispatch complete.", get_name());
        reset();
        return true;
    }

    void EsterelPlanDispatcher::initialise() {
        node_real_dispatch_time.clear();
        for (auto& node : current_plan.nodes) {
            action_dispatched[node.action.action_id] = false;
            action_received[node.action.action_id]   = false;
            action_completed[node.action.action_id]  = false;
        }
        for (auto& edge : current_plan.edges) {
            edge_active[edge.edge_id] = false;
        }
    }

    void EsterelPlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::msg::ActionFeedback::SharedPtr msg) {

        RCLCPP_INFO(get_logger(), "KCL: (%s) Feedback received [%i, %d]",
                    get_name(), msg->action_id, msg->status);

        if (!action_received[msg->action_id]
            && msg->status == rosplan_dispatch_msgs::msg::ActionFeedback::ACTION_ENABLED)
        {
            action_received[msg->action_id] = true;
            state_changed = true;
        }

        if (!action_completed[msg->action_id]
            && msg->status == rosplan_dispatch_msgs::msg::ActionFeedback::ACTION_SUCCEEDED_TO_GOAL_STATE)
        {
            if (!action_received[msg->action_id]) {
                RCLCPP_WARN(get_logger(), "KCL: (%s) Action not yet dispatched, ignoring feedback", get_name());
            } else {
                for (auto& node : current_plan.nodes) {
                    if (node.action.action_id == msg->action_id
                        && node.node_type == rosplan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END)
                    {
                        double NOW = rclcpp::Clock().now().seconds();
                        node_real_dispatch_time.insert(std::make_pair(node.node_id, NOW));
                    }
                }
                action_completed[msg->action_id] = true;
                state_changed = true;
            }
        }

        if (!action_completed[msg->action_id]
            && msg->status == rosplan_dispatch_msgs::msg::ActionFeedback::ACTION_FAILED)
        {
            if (!action_received[msg->action_id]) {
                RCLCPP_WARN(get_logger(), "KCL: (%s) Action not yet dispatched, ignoring feedback", get_name());
            } else {
                replan_requested = true;
                state_changed = true;
                action_completed[msg->action_id] = true;
            }
        }
    }

    bool EsterelPlanDispatcher::printPlan() {

        std::stringstream dest;
        dest << "digraph plan {" << std::endl;

        for (auto& node : current_plan.nodes) {
            std::stringstream params;
            if (node.node_type != rosplan_dispatch_msgs::msg::EsterelPlanNode::PLAN_START) {
                for (auto& p : node.action.parameters) params << p.value << ",";
                params.seekp(-1, params.cur); params << ')';
                dest << node.node_id << "[ label=\"" << node.name << "\n(" << params.str();
            } else {
                dest << node.node_id << "[ label=\"" << node.name;
            }

            switch (node.node_type) {
            case rosplan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START:
                if (action_received[node.action.action_id])
                    dest << "\",style=filled,fillcolor=darkolivegreen,fontcolor=white];";
                else if (action_dispatched[node.action.action_id])
                    dest << "\",style=filled,fillcolor=darkgoldenrod2];";
                else
                    dest << "\"];";
                break;
            case rosplan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END:
                if (action_completed[node.action.action_id])
                    dest << "\",style=filled,fillcolor=darkolivegreen,fontcolor=white];";
                else if (action_dispatched[node.action.action_id])
                    dest << "\",style=filled,fillcolor=darkgoldenrod2];";
                else
                    dest << "\"];";
                break;
            case rosplan_dispatch_msgs::msg::EsterelPlanNode::PLAN_START:
                dest << "\",style=filled,fillcolor=black,fontcolor=white];";
                break;
            default:
                dest << "\"];";
                break;
            }
            dest << std::endl;
        }

        for (auto& edge : current_plan.edges) {
            for (auto sink : edge.sink_ids) {
                for (auto src : edge.source_ids) {
                    dest << "\"" << src << "\" -> \"" << sink << "\"";
                    if (edge.duration_upper_bound == std::numeric_limits<double>::max()) {
                        dest << " [ label=\"[" << edge.duration_lower_bound << ", inf]\"";
                    } else {
                        dest << " [ label=\"[" << edge.duration_lower_bound << ", " << edge.duration_upper_bound << "]\"";
                    }

                    std::string edge_color = "black";
                    if (display_edge_type_) {
                        if (edge.edge_type == rosplan_dispatch_msgs::msg::EsterelPlanEdge::CONDITION_EDGE)
                            edge_color = "green";
                        else if (edge.edge_type == rosplan_dispatch_msgs::msg::EsterelPlanEdge::INTERFERENCE_EDGE)
                            edge_color = "blue";
                        else if (edge.edge_type == rosplan_dispatch_msgs::msg::EsterelPlanEdge::START_END_ACTION_EDGE)
                            edge_color = "red";
                    } else {
                        edge_color = edge_active[edge.edge_id] ? "red" : "black";
                    }

                    dest << " , penwidth=2, color=\"" << edge_color << "\"]" << std::endl;
                }
            }
        }

        dest << "}" << std::endl;

        std_msgs::msg::String msg;
        msg.data = dest.str();
        plan_graph_publisher_->publish(msg);
        return true;
    }

} // close namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_rosplan::EsterelPlanDispatcher>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
