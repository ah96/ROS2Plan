#ifndef KCL_knowledgebase
#define KCL_knowledgebase

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include <fstream>

#include "std_srvs/srv/empty.hpp"

#include "ros2plan_knowledge_msgs/srv/knowledge_update_service.hpp"
#include "ros2plan_knowledge_msgs/srv/knowledge_update_service_array.hpp"
#include "ros2plan_knowledge_msgs/srv/knowledge_query_service.hpp"
#include "ros2plan_knowledge_msgs/srv/set_named_bool.hpp"

#include "ros2plan_knowledge_msgs/srv/get_domain_name_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_type_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_operator_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_operator_details_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_predicate_details_service.hpp"
#include "ros2plan_knowledge_msgs/msg/domain_formula.hpp"
#include "ros2plan_knowledge_msgs/msg/status_update.hpp"

#include "ros2plan_knowledge_msgs/srv/get_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_instance_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_metric_service.hpp"
#include "ros2plan_knowledge_msgs/msg/knowledge_item.hpp"

#include "KnowledgeComparitor.h"

namespace KCL_ros2plan {

    class KnowledgeBase : public rclcpp::Node
    {
    private:
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetDomainNameService>::SharedPtr domainServer1;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetDomainTypeService>::SharedPtr domainServer2;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetDomainAttributeService>::SharedPtr domainServer3;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetDomainAttributeService>::SharedPtr domainServer4;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetDomainOperatorService>::SharedPtr domainServer5;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService>::SharedPtr domainServer6;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetDomainPredicateDetailsService>::SharedPtr domainServer7;

        rclcpp::Service<ros2plan_knowledge_msgs::srv::KnowledgeQueryService>::SharedPtr queryServer;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::SetNamedBool>::SharedPtr senseServer;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr updateServer0;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::KnowledgeUpdateService>::SharedPtr updateServer1;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::KnowledgeUpdateServiceArray>::SharedPtr updateServer2;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::KnowledgeUpdateServiceArray>::SharedPtr updateServer3;

        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetInstanceService>::SharedPtr stateServer1;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr stateServer2;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr stateServer3;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr stateServer4;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr stateServer5;
        rclcpp::Service<ros2plan_knowledge_msgs::srv::GetMetricService>::SharedPtr stateServer6;

        rclcpp::TimerBase::SharedPtr til_timer_;

    protected:

        // callback group for all service servers (prevents re-entrant calls);
        // shared with subclasses (e.g. RDDLKnowledgeBase) that register extra services
        rclcpp::CallbackGroup::SharedPtr srv_cbg_;

        rclcpp::Publisher<ros2plan_knowledge_msgs::msg::StatusUpdate>::SharedPtr status_pub_;

        /* adding items to the knowledge base */
        void addKnowledge(ros2plan_knowledge_msgs::msg::KnowledgeItem &msg);
        void addMissionGoal(ros2plan_knowledge_msgs::msg::KnowledgeItem &msg);
        void addMissionMetric(ros2plan_knowledge_msgs::msg::KnowledgeItem &msg);

        /* removing items from the knowledge base */
        void removeKnowledge(ros2plan_knowledge_msgs::msg::KnowledgeItem &msg);
        virtual void removeFact(const ros2plan_knowledge_msgs::msg::KnowledgeItem &msg);

        void removeMissionGoal(ros2plan_knowledge_msgs::msg::KnowledgeItem &msg);
        void removeMissionMetric(ros2plan_knowledge_msgs::msg::KnowledgeItem &msg);

        /* PDDL model (persistent state) */
        std::map<std::string, std::vector<std::string>> domain_constants;

        /* PDDL model (current state) */
        std::map<std::string, std::vector<std::string>> model_constants;
        std::map<std::string, std::vector<std::string>> model_instances;
        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> model_facts;
        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> model_functions;
        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> model_goals;
        ros2plan_knowledge_msgs::msg::KnowledgeItem model_metric;

        /* timed initial literals */
        std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem> model_timed_initial_literals;

        /* conditional planning */
        std::vector<std::vector<ros2plan_knowledge_msgs::msg::KnowledgeItem>> model_oneof_constraints;

        /* sensing information */
        std::map<std::string, bool> sensed_predicates;

        double _kb_rate;

    public:

        explicit KnowledgeBase(const std::string& node_name = "ros2plan_knowledge_base");
        ~KnowledgeBase() override = default;

        bool use_unknowns;

        /* parse domain and problem files */
        virtual void parseDomain(const std::string& domain_file_path, const std::string& problem_file_path) = 0;

        /* add the initial state to the knowledge base */
        virtual void addInitialState() = 0;
        virtual void addConstants() = 0;

        /* service callbacks for querying the model */
        void queryKnowledge(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::KnowledgeQueryService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::KnowledgeQueryService::Response> res);

        /* service callbacks for fetching the current state */
        void getInstances(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetInstanceService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetInstanceService::Response> res);
        void getPropositions(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Response> res);
        void getFunctions(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Response> res);
        void getGoals(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Response> res);
        void getMetric(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetMetricService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetMetricService::Response> res);
        void getTimedKnowledge(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Response> res);

        /* service callbacks for adding/removing items from the current state */
        void updateKnowledgeArray(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::KnowledgeUpdateServiceArray::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::KnowledgeUpdateServiceArray::Response> res);
        void updateKnowledge(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::KnowledgeUpdateService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::KnowledgeUpdateService::Response> res);
        void clearKnowledge(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res);

        /* service callbacks for fetching domain details */
        void getSubtypes(std::string &type, std::vector<std::string> &subtypes);
        virtual void getDomainName(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainNameService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainNameService::Response> res) = 0;
        virtual void getTypes(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainTypeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainTypeService::Response> res) = 0;
        virtual void getPredicates(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Response> res) = 0;
        virtual void getFunctionPredicates(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Response> res) = 0;
        virtual void getOperators(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorService::Response> res) = 0;
        virtual void getOperatorDetails(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Response> res) = 0;
        virtual void getPredicateDetails(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainPredicateDetailsService::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainPredicateDetailsService::Response> res) = 0;

        /* service callbacks for conditional planning */
        void updateKnowledgeConstraintsOneOf(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::KnowledgeUpdateServiceArray::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::KnowledgeUpdateServiceArray::Response> res);

        /* service callbacks for sensed predicates */
        void setSensedPredicate(
            const std::shared_ptr<ros2plan_knowledge_msgs::srv::SetNamedBool::Request> req,
            std::shared_ptr<ros2plan_knowledge_msgs::srv::SetNamedBool::Response> res);

        /* publish status update */
        void publishStatusUpdate();

        /* timed initial literals timer callback */
        void checkTimedInitialLiterals();
    };

} // namespace KCL_ros2plan
#endif
