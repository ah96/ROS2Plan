//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 26/09/18.
//

#ifndef ROS2PLAN_PLANNING_SYSTEM_RDDLPROBLEMGENERATOR_H
#define ROS2PLAN_PLANNING_SYSTEM_RDDLPROBLEMGENERATOR_H

#include "ProblemGenerator.h"
#include "ros2plan_knowledge_msgs/srv/get_domain_operator_details_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_operator_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_rddl_params.hpp"
#include "ros2plan_knowledge_msgs/srv/get_enumerable_type_service.hpp"
#include "ros2plan_knowledge_msgs/srv/reload_rddl_domain_problem.hpp"
#include "ros2plan_knowledge_msgs/srv/get_rddl_fluent_type.hpp"
#include "ros2plan_knowledge_msgs/srv/get_instance_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_type_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/msg/knowledge_item.hpp"

#include <set>
#include <map>
#include <iomanip>

namespace KCL_ros2plan {
    class RDDLProblemGenerator : public ProblemGenerator {
    private:
        inline void makeProblem(std::ofstream &pFile) override;

        void makeNonFluents(std::ofstream &pFile, const std::set<std::string>& nonfluents, const std::map<std::string, std::string> &fluent_types);
        void makeInstance(std::ofstream &pFile, const std::set<std::string>& fluents, const std::map<std::string, std::string> &fluent_types);

        std::map<std::string, std::vector<std::string>> getInstances();
        void getAllFluents(const std::vector<std::string> &pred_funcs, std::set<std::string> &found_fluents,
                           std::set<std::string> &found_nonfluents);
        std::vector<std::string> getPredicatesFunctions();
        void printGenericFluentElement(std::ofstream &pFile,
                                       std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem>::iterator fluent,
                                       const std::map<std::string, std::string> &fluent_types);
        void printGenericFluentList(std::ofstream &pFile, const std::set<std::string>& fluentlist,
                                    const std::map<std::string, std::string> &fluent_types);
        void getFluentTypes(const std::set<std::string> &fluents, const std::set<std::string> &nonfluents,
                            std::map<std::string, std::string> &fluent_types);

        std::string _domain_name;
        std::string _non_fluents_name;

        // Cached clients
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainNameService>::SharedPtr get_name_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::ReloadRDDLDomainProblem>::SharedPtr reload_domain_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetRDDLFluentType>::SharedPtr get_fluent_type_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetInstanceService>::SharedPtr get_instances_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainTypeService>::SharedPtr get_types_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService>::SharedPtr get_op_details_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainOperatorService>::SharedPtr get_op_list_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainAttributeService>::SharedPtr get_domain_props_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainAttributeService>::SharedPtr get_domain_funcs_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_props_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_funcs_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetEnumerableTypeService>::SharedPtr get_enumerable_type_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetRDDLParams>::SharedPtr get_rddl_params_client_;

        std::map<std::string, std::vector<std::string>> _enumeration_types;

    public:
        RDDLProblemGenerator(const std::string& kb, rclcpp::Node::SharedPtr svc_node);
        void generateProblemFile(std::string &problemPath) override;
    };
}

#endif //ROS2PLAN_PLANNING_SYSTEM_RDDLPROBLEMGENERATOR_H
