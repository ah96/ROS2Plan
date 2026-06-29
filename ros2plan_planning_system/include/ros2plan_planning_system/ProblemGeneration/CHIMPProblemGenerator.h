#ifndef CHIMPPROBLEMGENERATOR_H
#define CHIMPPROBLEMGENERATOR_H

/**
 * This class is responsible for generating the CHIMP problem instance (pdl).
 * This is done by using the objects requested from Knowledge services.
 */
#include "CHIMPProblem.h"
#include "ProblemGenerator.h"

#include "ros2plan_knowledge_msgs/srv/get_instance_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_attribute_service.hpp"

#include <string>
#include <vector>

namespace KCL_ros2plan
{
class CHIMPProblemGenerator : public ProblemGenerator
{
private:
    rclcpp::Client<ros2plan_knowledge_msgs::srv::GetInstanceService>::SharedPtr get_instances_client_;
    rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainAttributeService>::SharedPtr get_domain_props_client_;
    rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainAttributeService>::SharedPtr get_domain_funcs_client_;
    rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_props_client_;
    rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_tils_client_;
    rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_goals_client_;

    void addInstances(CHIMPProblem& problem);
    std::vector<std::string> queryAllInstances();

    void addInitialState(CHIMPProblem& problem);
    void addGoals(CHIMPProblem& problem);

    void makeProblem(std::ofstream &pFile);

public:
    CHIMPProblemGenerator(const std::string &kb, rclcpp::Node::SharedPtr svc_node);
};
}  // namespace KCL_ros2plan

#endif
