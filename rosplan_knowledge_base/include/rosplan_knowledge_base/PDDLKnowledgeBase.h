#ifndef KCL_PDDL_Knowledgebase
#define KCL_PDDL_Knowledgebase

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include <fstream>

#include "KnowledgeBase.h"
#include "KnowledgeComparitor.h"
#include "PDDLDomainParser.h"
#include "PDDLProblemParser.h"
#include "VALVisitorOperator.h"
#include "VALVisitorPredicate.h"
#include "VALVisitorProblem.h"

namespace KCL_rosplan {

    class PDDLKnowledgeBase : public KnowledgeBase {
    private:
        PDDLDomainParser  domain_parser;
        PDDLProblemParser problem_parser;

    public:
        PDDLKnowledgeBase() : KnowledgeBase("rosplan_knowledge_base") {}
        ~PDDLKnowledgeBase() override = default;

        void parseDomain(const std::string& domain_file_path, const std::string& problem_file_path) override;

        void addInitialState() override;
        void addConstants() override;

        void getDomainName(
            const std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainNameService::Request> req,
            std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainNameService::Response> res) override;
        void getTypes(
            const std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainTypeService::Request> req,
            std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainTypeService::Response> res) override;
        void getPredicates(
            const std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Request> req,
            std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Response> res) override;
        void getFunctionPredicates(
            const std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Request> req,
            std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Response> res) override;
        void getOperators(
            const std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainOperatorService::Request> req,
            std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainOperatorService::Response> res) override;
        void getOperatorDetails(
            const std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Request> req,
            std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Response> res) override;
        void getPredicateDetails(
            const std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainPredicateDetailsService::Request> req,
            std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainPredicateDetailsService::Response> res) override;
    };

} // namespace KCL_rosplan
#endif
