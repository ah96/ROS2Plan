/**
 * This file describes a class that dispatches a plan.
 */

#ifndef KCL_dispatcher
#define KCL_dispatcher

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rosplan_dispatch_msgs/msg/complete_plan.hpp"
#include "rosplan_dispatch_msgs/msg/action_feedback.hpp"
#include "rosplan_dispatch_msgs/msg/action_dispatch.hpp"
#include "rosplan_dispatch_msgs/action/non_blocking_dispatch.hpp"
#include "rosplan_dispatch_msgs/srv/dispatch_service.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rosplan_knowledge_msgs/srv/get_domain_operator_details_service.hpp"
#include "rosplan_knowledge_msgs/srv/knowledge_query_service.hpp"
#include "rosplan_knowledge_msgs/srv/get_attribute_service.hpp"
#include "rosplan_knowledge_msgs/msg/knowledge_item.hpp"

namespace KCL_rosplan
{
    using NonBlockingDispatchAction = rosplan_dispatch_msgs::action::NonBlockingDispatch;
    using GoalHandleNonBlockingDispatch = rclcpp_action::ServerGoalHandle<NonBlockingDispatchAction>;

    class PlanDispatcher : public rclcpp::Node
    {
    protected:
        rclcpp::CallbackGroup::SharedPtr srv_cbg_;
        rclcpp::CallbackGroup::SharedPtr sub_cbg_;

        rclcpp_action::Server<NonBlockingDispatchAction>::SharedPtr dispatch_action_server_;
        rclcpp::Service<rosplan_dispatch_msgs::srv::DispatchService>::SharedPtr dispatch_plan_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr cancel_dispatch_srv_;

        double mission_start_time;

        std::string action_dispatch_topic;
        std::string action_feedback_topic;

        /* dispatch state */
        bool plan_received;
        std::map<int,bool> action_received;
        std::map<int,bool> action_completed;

        /* dispatch modes */
        bool dispatch_paused;
        bool replan_requested;
        bool plan_cancelled;
        bool dispatching;

        /* action publishers */
        rclcpp::Publisher<rosplan_dispatch_msgs::msg::ActionDispatch>::SharedPtr action_dispatch_publisher_;
        rclcpp::Publisher<rosplan_dispatch_msgs::msg::ActionFeedback>::SharedPtr action_feedback_publisher_;

        /* check preconditions are true */
        bool checkPreconditions(rosplan_dispatch_msgs::msg::ActionDispatch msg);
        rclcpp::Client<rosplan_knowledge_msgs::srv::KnowledgeQueryService>::SharedPtr queryKnowledgeClient_;
        rclcpp::Client<rosplan_knowledge_msgs::srv::GetDomainOperatorDetailsService>::SharedPtr queryDomainClient_;
        rclcpp::Client<rosplan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_goals_client_;

        bool goalAchieved();

        template<typename ClientT, typename ReqT, typename ResT>
        bool callServiceSync(std::shared_ptr<ClientT>& client,
                             std::shared_ptr<ReqT> req,
                             std::shared_ptr<ResT>& res)
        {
            if (!client->wait_for_service(std::chrono::seconds(5))) return false;
            auto future = client->async_send_request(req);
            auto status = future.wait_for(std::chrono::seconds(5));
            if (status != std::future_status::ready) return false;
            res = future.get();
            return true;
        }

        std::string kb_;

        /* action server state */
        bool as_active_;
        std::shared_ptr<GoalHandleNonBlockingDispatch> current_goal_handle_;
        std::mutex as_mutex_;

    public:
        explicit PlanDispatcher(const std::string& node_name);
        virtual ~PlanDispatcher() = default;

        bool cancelDispatch();
        void cancelDispatchService(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res);

        virtual void reset();

        virtual bool dispatchPlan(double missionStartTime, double planStartTime) = 0;

        virtual void dispatchPlanService(
            const std::shared_ptr<rosplan_dispatch_msgs::srv::DispatchService::Request> req,
            std::shared_ptr<rosplan_dispatch_msgs::srv::DispatchService::Response> res);

        void dispatchPlanActionlib(std::shared_ptr<GoalHandleNonBlockingDispatch> goal_handle);

        void publishFeedback(const rosplan_dispatch_msgs::msg::ActionFeedback& fb);

        virtual void feedbackCallback(const rosplan_dispatch_msgs::msg::ActionFeedback::SharedPtr msg) = 0;
    };
}

#endif
