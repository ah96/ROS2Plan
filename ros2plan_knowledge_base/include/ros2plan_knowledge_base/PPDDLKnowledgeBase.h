//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 30/09/18.
//

#ifndef ROS2PLAN_KNOWLEDGE_BASE_PPDDLKNOWLEDGEBASE_H
#define ROS2PLAN_KNOWLEDGE_BASE_PPDDLKNOWLEDGEBASE_H

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include <fstream>

#include <diagnostic_msgs/msg/key_value.hpp>

#include "KnowledgeBase.h"
#include "PPDDLParser.h"
#include "PPDDLUtils.h"

namespace KCL_ros2plan {

    class PPDDLKnowledgeBase : public KnowledgeBase
    {
    private:
        std::vector<diagnostic_msgs::msg::KeyValue> getTypedParams(const ppddl_parser::TypeList& params);

        /* parsing domain using VAL */
        PPDDLParser domain_parser;
    public:
        PPDDLKnowledgeBase() : KnowledgeBase("ros2plan_knowledge_base") {}
        ~PPDDLKnowledgeBase() override = default;

        /* parse domain and problem files */
        void parseDomain(const std::string& domain_file_path, const std::string& problem_file_path) override;

        /* add the initial state to the knowledge base */
        void addInitialState() override;
        void addConstants() override;

        /* service methods for fetching the domain details */
        void getDomainName(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainNameService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainNameService::Response> res) override;
        void getTypes(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainTypeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainTypeService::Response> res) override;
        void getPredicates(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Response> res) override;
        void getFunctionPredicates(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Response> res) override;
        void getOperators(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorService::Response> res) override;
        void getOperatorDetails(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Response> res) override;
        void getPredicateDetails(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainPredicateDetailsService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainPredicateDetailsService::Response> res) override;
    };
}


#endif //ROS2PLAN_KNOWLEDGE_BASE_PPDDLKNOWLEDGEBASE_H
