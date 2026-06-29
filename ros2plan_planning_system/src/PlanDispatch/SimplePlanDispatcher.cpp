#include "ros2plan_planning_system/PlanDispatch/SimplePlanDispatcher.h"

namespace KCL_ros2plan {

    SimplePlanDispatcher::SimplePlanDispatcher()
        : PlanDispatcher("ros2plan_simple_plan_dispatcher"),
          current_action(0)
    {
        declare_parameter("plan_topic", std::string("complete_plan"));

        std::string plan_topic     = get_parameter("plan_topic").as_string();
        std::string feedback_topic = get_parameter("action_feedback_topic").as_string();

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_cbg_;

        plan_sub_ = create_subscription<ros2plan_dispatch_msgs::msg::CompletePlan>(
            plan_topic, rclcpp::QoS(1).transient_local(),
            [this](const ros2plan_dispatch_msgs::msg::CompletePlan::SharedPtr msg) { planCallback(msg); },
            sub_opts);

        feedback_sub_ = create_subscription<ros2plan_dispatch_msgs::msg::ActionFeedback>(
            feedback_topic, 1000,
            [this](const ros2plan_dispatch_msgs::msg::ActionFeedback::SharedPtr msg) { feedbackCallback(msg); },
            sub_opts);

        reset();

        RCLCPP_INFO(get_logger(), "KCL: (%s) Ready to receive.", get_name());
    }

    void SimplePlanDispatcher::reset() {
        PlanDispatcher::reset();
        current_action = 0;
    }

    void SimplePlanDispatcher::planCallback(const ros2plan_dispatch_msgs::msg::CompletePlan::SharedPtr plan) {
        RCLCPP_INFO(get_logger(), "KCL: (%s) Plan received.", get_name());
        plan_received = true;
        mission_start_time = rclcpp::Clock().now().seconds();
        current_plan = *plan;
    }

    bool SimplePlanDispatcher::dispatchPlan(double missionStartTime, double planStartTime) {

        RCLCPP_INFO(get_logger(), "KCL: (%s) Dispatching plan", get_name());

        replan_requested = false;

        while (rclcpp::ok() && current_plan.plan.size() > current_action) {

            // loop while dispatch is paused
            while (rclcpp::ok() && dispatch_paused) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // cancel plan
            if (plan_cancelled) break;

            ros2plan_dispatch_msgs::msg::ActionDispatch currentMessage = current_plan.plan[current_action];

            if (!checkPreconditions(currentMessage)) {

                RCLCPP_INFO(get_logger(), "KCL: (%s) Preconditions not achieved [%i, %s]",
                            get_name(), currentMessage.action_id, currentMessage.name.c_str());

                ros2plan_dispatch_msgs::msg::ActionFeedback fb;
                fb.action_id = currentMessage.action_id;
                fb.status = ros2plan_dispatch_msgs::msg::ActionFeedback::ACTION_PRECONDITION_FALSE;
                publishFeedback(fb);

                replan_requested = true;

            } else {

                std::string params = "(";
                for (size_t i = 0; i < currentMessage.parameters.size(); ++i) {
                    if (i > 0) params += ", ";
                    params += currentMessage.parameters[i].value;
                }
                params += ")";

                RCLCPP_INFO(get_logger(), "KCL: (%s) Dispatching action [%i, %s%s, %f, %f]",
                            get_name(),
                            currentMessage.action_id,
                            currentMessage.name.c_str(),
                            params.c_str(),
                            (currentMessage.dispatch_time + planStartTime - missionStartTime),
                            currentMessage.duration);

                action_dispatch_publisher_->publish(currentMessage);

                ros2plan_dispatch_msgs::msg::ActionFeedback fb;
                fb.action_id = currentMessage.action_id;
                fb.status = ros2plan_dispatch_msgs::msg::ActionFeedback::ACTION_DISPATCHED_TO_GOAL_STATE;
                publishFeedback(fb);

                double late_print = rclcpp::Clock().now().seconds() - (currentMessage.dispatch_time + planStartTime);
                if (late_print > 0.1) {
                    RCLCPP_INFO(get_logger(), "KCL: (%s) Action [%i] is %f second(s) late",
                                get_name(), currentMessage.action_id, late_print);
                }

                // wait for action to complete
                while (rclcpp::ok() && !action_completed[current_action]) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }

            current_action++;
            action_received[current_action]  = false;
            action_completed[current_action] = false;

            if (replan_requested) return false;
        }

        RCLCPP_INFO(get_logger(), "KCL: (%s) Dispatch complete.", get_name());
        return true;
    }

    void SimplePlanDispatcher::feedbackCallback(const ros2plan_dispatch_msgs::msg::ActionFeedback::SharedPtr msg) {

        RCLCPP_INFO(get_logger(), "KCL: (%s) Feedback received [%i, %d]",
                    get_name(), msg->action_id, msg->status);

        if (current_action != static_cast<unsigned int>(msg->action_id)) {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) Unexpected action ID: %d; current action: %d",
                         get_name(), msg->action_id, current_action);
        }

        if (!action_received[msg->action_id] &&
            msg->status == ros2plan_dispatch_msgs::msg::ActionFeedback::ACTION_ENABLED)
        {
            action_received[msg->action_id] = true;
        }

        if (!action_completed[msg->action_id] &&
            msg->status == ros2plan_dispatch_msgs::msg::ActionFeedback::ACTION_SUCCEEDED_TO_GOAL_STATE)
        {
            action_completed[msg->action_id] = true;
        }

        if (!action_completed[msg->action_id] &&
            msg->status == ros2plan_dispatch_msgs::msg::ActionFeedback::ACTION_FAILED)
        {
            replan_requested = true;
            action_completed[msg->action_id] = true;
        }
    }

} // close namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_ros2plan::SimplePlanDispatcher>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
