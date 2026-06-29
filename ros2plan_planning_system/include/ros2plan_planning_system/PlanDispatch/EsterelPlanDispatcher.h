#ifndef KCL_esterel_dispatcher
#define KCL_esterel_dispatcher

#include <stdlib.h>
#include <map>
#include <iostream>
#include <string>

#include "PlanDispatcher.h"

#include "ros2plan_knowledge_msgs/msg/knowledge_item.hpp"
#include "ros2plan_dispatch_msgs/msg/esterel_plan.hpp"
#include "ros2plan_dispatch_msgs/msg/esterel_plan_node.hpp"
#include "ros2plan_dispatch_msgs/msg/esterel_plan_edge.hpp"

#include "std_msgs/msg/string.hpp"


namespace KCL_ros2plan
{
    class EsterelPlanDispatcher : public PlanDispatcher
    {
    private:

        int getNodeFromActionID(int action_id, bool end_node) {
            for (const auto& ci : current_plan.nodes) {
                if (ci.action.action_id == action_id) {
                    if (!end_node && ci.node_type == ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START)
                        return ci.node_id;
                    else if (end_node && ci.node_type == ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END)
                        return ci.node_id;
                }
            }
            return -1;
        }

        void initialise();

        ros2plan_dispatch_msgs::msg::EsterelPlan current_plan;

        std::map<int,bool> edge_active;
        std::map<int,bool> action_dispatched;

        bool state_changed;
        bool finished_execution;

        std::map<int,double> node_real_dispatch_time;

        bool printPlan();
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plan_graph_publisher_;

        rclcpp::Subscription<ros2plan_dispatch_msgs::msg::EsterelPlan>::SharedPtr plan_sub_;
        rclcpp::Subscription<ros2plan_dispatch_msgs::msg::ActionFeedback>::SharedPtr feedback_sub_;

        bool display_edge_type_;

        bool timeout_actions;
        double action_timeout_fraction;

    public:

        EsterelPlanDispatcher();
        ~EsterelPlanDispatcher() = default;

        void planCallback(const ros2plan_dispatch_msgs::msg::EsterelPlan::SharedPtr plan);

        void reset() override;

        bool dispatchPlan(double missionStartTime, double planStartTime) override;

        void feedbackCallback(const ros2plan_dispatch_msgs::msg::ActionFeedback::SharedPtr msg) override;
    };
}

#endif
