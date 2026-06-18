/*
 * This file describes the class used to dispatch a Simple (sequential) PDDL plan.
 */

#ifndef KCL_simple_dispatcher
#define KCL_simple_dispatcher

#include "PlanDispatcher.h"
#include "rosplan_dispatch_msgs/msg/complete_plan.hpp"
#include "rosplan_knowledge_msgs/msg/domain_operator.hpp"
#include <map>


namespace KCL_rosplan
{

    class SimplePlanDispatcher : public PlanDispatcher
    {
    private:

        rosplan_dispatch_msgs::msg::CompletePlan current_plan;

        rclcpp::Subscription<rosplan_dispatch_msgs::msg::CompletePlan>::SharedPtr plan_sub_;
        rclcpp::Subscription<rosplan_dispatch_msgs::msg::ActionFeedback>::SharedPtr feedback_sub_;

        int current_action;

    public:

        SimplePlanDispatcher();
        ~SimplePlanDispatcher() = default;

        void planCallback(const rosplan_dispatch_msgs::msg::CompletePlan::SharedPtr plan);

        void reset() override;

        bool dispatchPlan(double missionStartTime, double planStartTime) override;

        void feedbackCallback(const rosplan_dispatch_msgs::msg::ActionFeedback::SharedPtr msg) override;
    };
}

#endif
