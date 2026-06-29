//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 30/09/18.
//

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "ros2plan_knowledge_msgs/KnowledgeUpdateService.h"
#include "ros2plan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "ros2plan_knowledge_msgs/KnowledgeQueryService.h"

#include "ros2plan_knowledge_msgs/GetDomainNameService.h"
#include "ros2plan_knowledge_msgs/GetDomainTypeService.h"
#include "ros2plan_knowledge_msgs/GetDomainAttributeService.h"
#include "ros2plan_knowledge_msgs/GetDomainOperatorService.h"
#include "ros2plan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "ros2plan_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "ros2plan_knowledge_msgs/DomainFormula.h"

#include "ros2plan_knowledge_msgs/GetAttributeService.h"
#include "ros2plan_knowledge_msgs/GetInstanceService.h"
#include "ros2plan_knowledge_msgs/GetMetricService.h"
#include "ros2plan_knowledge_msgs/KnowledgeItem.h"

#include "KnowledgeBase.h"
#include "PPDDLParser.h"
#include "PPDDLUtils.h"

#ifndef ROS2PLAN_KNOWLEDGE_BASE_PPDDLKNOWLEDGEBASE_H
#define ROS2PLAN_KNOWLEDGE_BASE_PPDDLKNOWLEDGEBASE_H

namespace KCL_ros2plan {

    class PPDDLKnowledgeBase : public KnowledgeBase
    {
    private:
        std::vector<diagnostic_msgs::KeyValue> getTypedParams(const ppddl_parser::TypeList& params);

        /* parsing domain using VAL */
        PPDDLParser domain_parser;
    public:
        PPDDLKnowledgeBase(ros::NodeHandle& n) : KnowledgeBase(n) {};
        ~PPDDLKnowledgeBase() = default;

        /* parse domain and problem files */
        inline void parseDomain(const std::string& domain_file_path, const std::string& problem_file_path) override;

        /* add the initial state to the knowledge base */
        void addInitialState() override;
        void addConstants() override;

        /* service methods for fetching the domain details */
        bool getDomainName(ros2plan_knowledge_msgs::GetDomainNameService::Request  &req, ros2plan_knowledge_msgs::GetDomainNameService::Response &res) override;
        bool getTypes(ros2plan_knowledge_msgs::GetDomainTypeService::Request  &req, ros2plan_knowledge_msgs::GetDomainTypeService::Response &res) override;
        bool getPredicates(ros2plan_knowledge_msgs::GetDomainAttributeService::Request  &req, ros2plan_knowledge_msgs::GetDomainAttributeService::Response &res) override;
        bool getFunctionPredicates(ros2plan_knowledge_msgs::GetDomainAttributeService::Request  &req, ros2plan_knowledge_msgs::GetDomainAttributeService::Response &res) override;
        bool getOperators(ros2plan_knowledge_msgs::GetDomainOperatorService::Request  &req, ros2plan_knowledge_msgs::GetDomainOperatorService::Response &res) override;
        bool getOperatorDetails(ros2plan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, ros2plan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) override;
        bool getPredicateDetails(ros2plan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, ros2plan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) override;
    };
}


#endif //ROS2PLAN_KNOWLEDGE_BASE_PPDDLKNOWLEDGEBASE_H
