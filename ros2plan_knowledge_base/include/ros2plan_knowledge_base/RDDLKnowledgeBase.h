#ifndef KCL_RDDL_Knowledgebase
#define KCL_RDDL_Knowledgebase

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include <fstream>

#include <diagnostic_msgs/msg/key_value.hpp>

#include "ros2plan_knowledge_msgs/srv/get_enumerable_type_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_rddl_params.hpp"
#include "ros2plan_knowledge_msgs/srv/get_rddl_fluent_type.hpp"
#include "ros2plan_knowledge_msgs/srv/get_rddl_immediate_reward.hpp"
#include "ros2plan_knowledge_msgs/srv/set_float.hpp"
#include "ros2plan_knowledge_msgs/srv/set_int.hpp"
#include "ros2plan_knowledge_msgs/srv/reload_rddl_domain_problem.hpp"

#include "KnowledgeBase.h"
#include "KnowledgeComparitor.h"
#include "RDDLTaskParser.h"

#include "VALVisitorOperator.h"
#include "VALVisitorPredicate.h"
#include "VALVisitorProblem.h"


namespace KCL_ros2plan {

	class RDDLKnowledgeBase : public KnowledgeBase
	{
	private:
		/* Converts RDDL params to ROS-like params */
		std::vector<diagnostic_msgs::msg::KeyValue> getTypedParams(const std::vector<Parameter*>& params);

		/* Get object instances */
        std::map<std::string, std::vector<std::string> > getInstances(const std::map<std::string, Object*>& instance_objects);

        /* Get facts and functions from the task */
        void loadFactsAndFunctions();

		/* Get the metric from the task */
        void loadMetric();

        /* Just fills the header for a fact */
        ros2plan_knowledge_msgs::msg::KnowledgeItem fillKI(const ParametrizedVariable* var, const std::vector<Parameter *> &params, double initialValue);
        /* Adds all the functions and facts that have a default value in the initial state with their grounded parameters */
        void fillKIAddAllGroundedParameters(const ParametrizedVariable *var,
											std::map<std::string, ros2plan_knowledge_msgs::msg::KnowledgeItem> &factsfuncs,
											ros2plan_knowledge_msgs::msg::KnowledgeItem &item,
											std::string ground_params = "", int param_index = 0);

		/* parsing domain */
		RDDLTaskParser domain_parser;
		std::string domain_path_; // Path to the domain file
		int _horizon;
		int _max_nondef_actions;
		float _discount_factor;

		/* Services to set and get parameters */
		rclcpp::Service<ros2plan_knowledge_msgs::srv::GetRDDLParams>::SharedPtr _getParamsService;
		rclcpp::Service<ros2plan_knowledge_msgs::srv::SetFloat>::SharedPtr _setRDDLDiscountFactorSrv;
		rclcpp::Service<ros2plan_knowledge_msgs::srv::SetInt>::SharedPtr _setRDDLHorizonSrv;
		rclcpp::Service<ros2plan_knowledge_msgs::srv::SetInt>::SharedPtr _setRDDLMaxNonDefSrv;
		rclcpp::Service<ros2plan_knowledge_msgs::srv::GetRDDLImmediateReward>::SharedPtr _setImmediateRewardsSrv;
		rclcpp::Service<ros2plan_knowledge_msgs::srv::ReloadRDDLDomainProblem>::SharedPtr _reloadDomainStructureSrv;
		rclcpp::Service<ros2plan_knowledge_msgs::srv::GetEnumerableTypeService>::SharedPtr _getEnumtypesSrv;
		rclcpp::Service<ros2plan_knowledge_msgs::srv::GetRDDLFluentType>::SharedPtr _getFluentTypeSrv;

		void getRDDLParams(
			const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetRDDLParams::Request> req,
			std::shared_ptr<ros2plan_knowledge_msgs::srv::GetRDDLParams::Response> res);
		void setRDDLDiscountFactor(
			const std::shared_ptr<ros2plan_knowledge_msgs::srv::SetFloat::Request> req,
			std::shared_ptr<ros2plan_knowledge_msgs::srv::SetFloat::Response> res);
		void setRDDLHorizon(
			const std::shared_ptr<ros2plan_knowledge_msgs::srv::SetInt::Request> req,
			std::shared_ptr<ros2plan_knowledge_msgs::srv::SetInt::Response> res);
		void setRDDLMAxNonDefActions(
			const std::shared_ptr<ros2plan_knowledge_msgs::srv::SetInt::Request> req,
			std::shared_ptr<ros2plan_knowledge_msgs::srv::SetInt::Response> res);
		void computeImmediateReward(
			const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetRDDLImmediateReward::Request> req,
			std::shared_ptr<ros2plan_knowledge_msgs::srv::GetRDDLImmediateReward::Response> res);
		void reloadDomain(
			const std::shared_ptr<ros2plan_knowledge_msgs::srv::ReloadRDDLDomainProblem::Request> req,
			std::shared_ptr<ros2plan_knowledge_msgs::srv::ReloadRDDLDomainProblem::Response> res);
		void getEnumTypes(
			const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetEnumerableTypeService::Request> req,
			std::shared_ptr<ros2plan_knowledge_msgs::srv::GetEnumerableTypeService::Response> res);
		void getFluentType(
			const std::shared_ptr<ros2plan_knowledge_msgs::srv::GetRDDLFluentType::Request> req,
			std::shared_ptr<ros2plan_knowledge_msgs::srv::GetRDDLFluentType::Response> res);
        void removeFact(const ros2plan_knowledge_msgs::msg::KnowledgeItem &msg) override;
	public:
		RDDLKnowledgeBase();
		~RDDLKnowledgeBase() override = default;


		/* parse domain and probelm files */
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
#endif
