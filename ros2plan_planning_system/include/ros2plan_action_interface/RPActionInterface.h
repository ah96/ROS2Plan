#include <rclcpp/rclcpp.hpp>

#include "ros2plan_dispatch_msgs/msg/action_feedback.hpp"
#include "ros2plan_dispatch_msgs/msg/action_dispatch.hpp"
#include "ros2plan_knowledge_msgs/msg/domain_formula.hpp"
#include "ros2plan_knowledge_msgs/msg/knowledge_item.hpp"
#include "ros2plan_knowledge_msgs/srv/knowledge_update_service.hpp"
#include "ros2plan_knowledge_msgs/srv/knowledge_update_service_array.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_operator_details_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_predicate_details_service.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#ifndef KCL_action_interface
#define KCL_action_interface

/**
 * This file defines the RPActionInterface header.
 * This header defines a standard action interface for ROS2Plan.
 * This interface will link a PDDL action to some implementation, most
 * commonly as an action client.
 */
namespace KCL_ros2plan {

    class RPActionInterface : public rclcpp::Node
    {
    protected:

        rclcpp::CallbackGroup::SharedPtr srv_cbg_;
        rclcpp::CallbackGroup::SharedPtr sub_cbg_;

        std::map<std::string, ros2plan_knowledge_msgs::msg::DomainFormula> predicates;
        std::map<std::string, ros2plan_knowledge_msgs::msg::DomainFormula> sensed_predicates;

        /* PDDL info and publisher */
        ros2plan_knowledge_msgs::msg::DomainFormula params;
        ros2plan_knowledge_msgs::msg::DomainOperator op;
        rclcpp::Publisher<ros2plan_knowledge_msgs::msg::DomainFormula>::SharedPtr pddl_action_parameters_pub_;
        rclcpp::TimerBase::SharedPtr param_timer_;

        /* action feedback to planning system */
        rclcpp::Publisher<ros2plan_dispatch_msgs::msg::ActionFeedback>::SharedPtr action_feedback_pub_;

        /* dispatch subscription */
        rclcpp::Subscription<ros2plan_dispatch_msgs::msg::ActionDispatch>::SharedPtr dispatch_sub_;

        /* satellite node for blocking KB calls (setup + dispatch KB updates) */
        rclcpp::Node::SharedPtr kb_svc_node_;

        /* service handle to PDDL knowledge base */
        rclcpp::Client<ros2plan_knowledge_msgs::srv::KnowledgeUpdateServiceArray>::SharedPtr update_knowledge_client_;

        /* action status */
        bool action_success;
        bool action_cancelled;

    public:

        explicit RPActionInterface(const std::string& node_name = "ros2plan_action_interface");

        /* setup called from constructor (fetches operator details, creates clients) */
        void setup();

        /* listen to and process action_dispatch topic */
        void dispatchCallback(const ros2plan_dispatch_msgs::msg::ActionDispatch::SharedPtr msg);

        /* perform or call real action implementation */
        virtual bool concreteCallback(const ros2plan_dispatch_msgs::msg::ActionDispatch::SharedPtr msg) = 0;
    };
}
#endif
