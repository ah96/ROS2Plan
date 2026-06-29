#ifndef KCL_HDDL_Knowledgebase
#define KCL_HDDL_Knowledgebase

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "hddl_parser.h"
#include "KnowledgeBase.h"

namespace KCL_ros2plan {

    class HDDLKnowledgeBase : public KnowledgeBase {
    public:
        HDDLKnowledgeBase() : KnowledgeBase("ros2plan_knowledge_base"), problem_file_path_("") {}
        ~HDDLKnowledgeBase() override = default;

        void parseDomain(const std::string& domain_file_path, const std::string& problem_file_path) override;

        void addInitialState() override;
        void addConstants() override;

        void getDomainName(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainNameService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainNameService::Response> res) override;
        void getTypes(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainTypeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainTypeService::Response> res) override;
        void getPredicates(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Response> res) override;
        void getPredicateDetails(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainPredicateDetailsService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainPredicateDetailsService::Response> res) override;
        void getFunctionPredicates(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Response> res) override;
        void getOperators(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorService::Response> res) override;
        void getOperatorDetails(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Response> res) override;

    private:
        HDDLParser  hddl_parser_;
        std::string problem_file_path_;
    };

} // namespace KCL_ros2plan
#endif
