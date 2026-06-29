#ifndef KCL_planner_interface
#define KCL_planner_interface

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

#include "ros2plan_dispatch_msgs/srv/planning_service.hpp"
#include "ros2plan_dispatch_msgs/action/plan.hpp"

/**
 * This file contains an interface to the planner.
 */
namespace KCL_ros2plan {

    using PlanAction = ros2plan_dispatch_msgs::action::Plan;
    using GoalHandlePlan = rclcpp_action::ServerGoalHandle<PlanAction>;

    class PlannerInterface : public rclcpp::Node
    {
    protected:

        rclcpp::CallbackGroup::SharedPtr srv_cbg_;
        rclcpp::CallbackGroup::SharedPtr sub_cbg_;

        rclcpp_action::Server<PlanAction>::SharedPtr plan_server_;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr planning_server_default_;
        rclcpp::Service<ros2plan_dispatch_msgs::srv::PlanningService>::SharedPtr planning_server_params_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr problem_sub_;

        /* params */
        bool use_problem_topic;
        std::string planner_command;
        std::string domain_path;
        std::string problem_path;
        std::string problem_name;
        std::string data_path;

        /* planner outputs */
        std::string planner_output;

        /* problem subscription */
        std::string problem_instance;
        bool problem_instance_received;
        double problem_instance_time;

        /* planning */
        virtual bool runPlanner() = 0;

    public:

        explicit PlannerInterface(const std::string& node_name = "ros2plan_planner_interface");

        void problemCallback(const std_msgs::msg::String::SharedPtr problemInstance);

        void runPlanningServerDefault(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res);

        void runPlanningServerParams(
            const std::shared_ptr<ros2plan_dispatch_msgs::srv::PlanningService::Request> req,
            std::shared_ptr<ros2plan_dispatch_msgs::srv::PlanningService::Response> res);

        void runPlanningServerAction(std::shared_ptr<GoalHandlePlan> goal_handle);

        bool runPlanningServer(std::string domainPath, std::string problemPath,
                               std::string dataPath, std::string plannerCommand,
                               bool useProblemTopic);

        /* ROS interface */
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plan_publisher_;
    };

} // close namespace

#endif
