//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#ifndef ROSPLAN_PLANNING_SYSTEM_ONLINEPLANDISPATCHER_H
#define ROSPLAN_PLANNING_SYSTEM_ONLINEPLANDISPATCHER_H

#include "PlanDispatcher.h"
#include "rosplan_dependencies/ippc_server.h"
#include "rosplan_knowledge_msgs/srv/get_attribute_service.hpp"
#include "rosplan_knowledge_msgs/srv/get_rddl_params.hpp"
#include "rosplan_knowledge_msgs/srv/get_rddl_immediate_reward.hpp"
#include "rosplan_dispatch_msgs/srv/get_planning_params.hpp"
#include <regex>

namespace KCL_rosplan
{

    class OnlinePlanDispatcher : public PlanDispatcher {
    private:
        rosplan_dispatch_msgs::msg::CompletePlan current_plan;

        rclcpp::Client<rosplan_knowledge_msgs::srv::GetAttributeService>::SharedPtr queryPropositionsClient_;
        rclcpp::Client<rosplan_knowledge_msgs::srv::GetRDDLParams>::SharedPtr get_rddl_params_client_;
        rclcpp::Client<rosplan_dispatch_msgs::srv::GetPlanningParams>::SharedPtr get_planner_params_client_;
        rclcpp::Client<rosplan_knowledge_msgs::srv::GetRDDLImmediateReward>::SharedPtr get_immediate_reward_client_;

        rclcpp::Subscription<rosplan_dispatch_msgs::msg::ActionFeedback>::SharedPtr feedback_sub_;

        int current_action;
        int server_port_;
        bool compute_rewards_;

        rclcpp::Publisher<rosplan_dispatch_msgs::msg::CompletePlan>::SharedPtr plan_publisher_;

        std::vector<rosplan_dispatch_msgs::msg::ActionDispatch> toActionMsg(std::string action_str, int action_id);

    public:

        OnlinePlanDispatcher();
        ~OnlinePlanDispatcher() = default;

        void reset() override;

        void dispatchPlanService(
            const std::shared_ptr<rosplan_dispatch_msgs::srv::DispatchService::Request> req,
            std::shared_ptr<rosplan_dispatch_msgs::srv::DispatchService::Response> res) override;

        void dispatchPlanActionlib(std::shared_ptr<GoalHandleNonBlockingDispatch> goal_handle);

        bool dispatchPlan(double missionStartTime, double planStartTime) override;

        void dispatchOneAction(rosplan_dispatch_msgs::msg::ActionDispatch current_action_msg,
                               double missionStartTime, double planStartTime);

        void feedbackCallback(const rosplan_dispatch_msgs::msg::ActionFeedback::SharedPtr msg) override;
    };
}


#endif //ROSPLAN_PLANNING_SYSTEM_ONLINEPLANDISPATCHER_H
