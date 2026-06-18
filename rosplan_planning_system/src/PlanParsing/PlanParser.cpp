#include "rosplan_planning_system/PlanParsing/PlanParser.h"

namespace KCL_rosplan {

    PlanParser::PlanParser(const std::string& node_name)
        : rclcpp::Node(node_name),
          planner_output_received(false),
          planner_output_time(0.0)
    {
    }

    void PlanParser::plannerCallback(const std_msgs::msg::String::SharedPtr plannerOutput) {
        RCLCPP_INFO(get_logger(), "KCL: (%s) Planner output received.", get_name());
        planner_output_received = true;
        planner_output_time = rclcpp::Clock().now().seconds();
        planner_output = plannerOutput->data;
    }

    void PlanParser::parsePlanFromFile(
        const std::shared_ptr<rosplan_dispatch_msgs::srv::ParsingService::Request> req,
        std::shared_ptr<rosplan_dispatch_msgs::srv::ParsingService::Response> res)
    {
        RCLCPP_INFO(get_logger(), "KCL: (%s) Parsing plan from file.", get_name());

        reset();

        std::stringstream ss;
        std::ifstream source;
        source.open(req->plan_path.c_str());
        ss << source.rdbuf();
        planner_output = ss.str();
        source.close();

        planner_output_received = true;

        preparePlan();
        publishPlan();

        planner_output_received = false;

        res->plan_parsed = true;
    }

    void PlanParser::parsePlan(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
    {
        if (planner_output_received) {
            RCLCPP_INFO(get_logger(), "KCL: (%s) Parsing planner output.", get_name());
            reset();
            preparePlan();
            publishPlan();
            planner_output_received = false;
        } else {
            RCLCPP_INFO(get_logger(), "KCL: (%s) No new planner output received; nothing to parse.", get_name());
        }
    }

} // close namespace
