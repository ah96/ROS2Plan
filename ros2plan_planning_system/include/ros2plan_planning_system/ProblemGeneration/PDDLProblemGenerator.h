/**
 * This class is responsible for generating the PDDL instance.
 * This is done by using the objects requested from Knowledge services.
 */
#include "ProblemGenerator.h"

#include "ros2plan_knowledge_msgs/srv/get_domain_name_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_type_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_instance_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_metric_service.hpp"
#include "ros2plan_knowledge_msgs/msg/expr_composite.hpp"
#include "ros2plan_knowledge_msgs/msg/expr_base.hpp"
#include "ros2plan_knowledge_msgs/msg/knowledge_item.hpp"

#ifndef KCL_PDDLproblemgenerator
#define KCL_PDDLproblemgenerator

namespace KCL_ros2plan {

    class PDDLProblemGenerator : public ProblemGenerator {
    private:
        // Cached service clients (created once in constructor)
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainNameService>::SharedPtr get_name_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainTypeService>::SharedPtr get_types_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetInstanceService>::SharedPtr get_instances_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainAttributeService>::SharedPtr get_domain_props_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainAttributeService>::SharedPtr get_domain_funcs_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_props_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_funcs_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_tils_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_goals_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetMetricService>::SharedPtr get_metric_client_;

        void makeHeader(std::ofstream &pFile);
        void makeInitialState(std::ofstream &pFile);
        void makeGoals(std::ofstream &pFile);
        void makeMetric(std::ofstream &pFile);
        void printExpression(std::ofstream &pFile, ros2plan_knowledge_msgs::msg::ExprComposite &e);

        void makeProblem(std::ofstream &pFile);

    public:
        PDDLProblemGenerator(const std::string& kb, rclcpp::Node::SharedPtr svc_node);
    };

} // close namespace

#endif
