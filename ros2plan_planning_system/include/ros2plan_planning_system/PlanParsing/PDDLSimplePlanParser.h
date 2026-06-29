/**
 * This file parses standard PDDL output and generates a list of ActionDispatch messages.
 */
#include "PlanParser.h"

#include "ros2plan_dispatch_msgs/msg/action_dispatch.hpp"
#include "ros2plan_dispatch_msgs/msg/complete_plan.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_operator_details_service.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include <string>
#include <sstream>

#ifndef KCL_pddl_simple_plan_parser
#define KCL_pddl_simple_plan_parser

namespace KCL_ros2plan {

    class PDDLSimplePlanParser : public PlanParser
    {
    private:

        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService>::SharedPtr get_operator_details_client_;

    protected:

        /* post process plan */
        void processPDDLParameters(ros2plan_dispatch_msgs::msg::ActionDispatch &msg, std::vector<std::string> &params);

        /* virtual methods */
        void reset();
        void preparePlan();
        void publishPlan();

    public:

        PDDLSimplePlanParser();
        ~PDDLSimplePlanParser() = default;

        /* ROS interface */
        rclcpp::Publisher<ros2plan_dispatch_msgs::msg::CompletePlan>::SharedPtr plan_publisher_;
    };

} // close namespace

#endif
