#ifndef KCL_RDDL_Knowledgebase
#define KCL_RDDL_Knowledgebase

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "std_srvs/Empty.h"

#include "ros2plan_knowledge_msgs/KnowledgeUpdateService.h"
#include "ros2plan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "ros2plan_knowledge_msgs/KnowledgeQueryService.h"

#include "ros2plan_knowledge_msgs/GetDomainNameService.h"
#include "ros2plan_knowledge_msgs/GetDomainTypeService.h"
#include "ros2plan_knowledge_msgs/GetDomainAttributeService.h"
#include "ros2plan_knowledge_msgs/GetDomainOperatorService.h"
#include "ros2plan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "ros2plan_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "ros2plan_knowledge_msgs/GetEnumerableTypeService.h"
#include "ros2plan_knowledge_msgs/DomainFormula.h"

#include "ros2plan_knowledge_msgs/GetAttributeService.h"
#include "ros2plan_knowledge_msgs/GetInstanceService.h"
#include "ros2plan_knowledge_msgs/GetMetricService.h"
#include "ros2plan_knowledge_msgs/KnowledgeItem.h"
#include "ros2plan_knowledge_msgs/GetRDDLParams.h"
#include "ros2plan_knowledge_msgs/GetRDDLFluentType.h"
#include "ros2plan_knowledge_msgs/GetRDDLImmediateReward.h"
#include "ros2plan_knowledge_msgs/SetFloat.h"
#include "ros2plan_knowledge_msgs/SetInt.h"
#include "ros2plan_knowledge_msgs/ReloadRDDLDomainProblem.h"

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
		std::vector<diagnostic_msgs::KeyValue> getTypedParams(const std::vector<Parameter*>& params);

		/* Get object instances */
        std::map<std::string, std::vector<std::string> > getInstances(const std::map<std::string, Object*>& instance_objects);

        /* Get facts and functions from the task */
        void loadFactsAndFunctions();

		/* Get the metric from the task */
        void loadMetric();

        /* Just fills the header for a fact */
        ros2plan_knowledge_msgs::KnowledgeItem fillKI(const ParametrizedVariable* var, const std::vector<Parameter *> &params, double initialValue);
        /* Adds all the functions and facts that have a default value in the initial state with their grounded parameters */
        void fillKIAddAllGroundedParameters(const ParametrizedVariable *var,
											std::map<std::string, ros2plan_knowledge_msgs::KnowledgeItem> &factsfuncs,
											ros2plan_knowledge_msgs::KnowledgeItem &item,
											std::string ground_params = "", int param_index = 0);

		/* parsing domain */
		RDDLTaskParser domain_parser;
		std::string domain_path_; // Path to the domain file
		int _horizon;
		int _max_nondef_actions;
		float _discount_factor;

		/* Services to set and get parameters */
		ros::ServiceServer _getParamsService;
		ros::ServiceServer _setRDDLDiscountFactorSrv;
		ros::ServiceServer _setRDDLHorizonSrv;
		ros::ServiceServer _setRDDLMaxNonDefSrv;
		ros::ServiceServer _setImmediateRewardsSrv;
		ros::ServiceServer _reloadDomainStructureSrv;
		ros::ServiceServer _getEnumtypesSrv;
		ros::ServiceServer _getFluentTypeSrv;

		bool getRDDLParams(ros2plan_knowledge_msgs::GetRDDLParams::Request &req, ros2plan_knowledge_msgs::GetRDDLParams::Response &res);
		bool setRDDLDiscountFactor(ros2plan_knowledge_msgs::SetFloat::Request &req, ros2plan_knowledge_msgs::SetFloat::Response &res);
		bool setRDDLHorizon(ros2plan_knowledge_msgs::SetInt::Request &req, ros2plan_knowledge_msgs::SetInt::Response &res);
		bool setRDDLMAxNonDefActions(ros2plan_knowledge_msgs::SetInt::Request &req, ros2plan_knowledge_msgs::SetInt::Response &res);
		bool computeImmediateReward(ros2plan_knowledge_msgs::GetRDDLImmediateReward::Request &req, ros2plan_knowledge_msgs::GetRDDLImmediateReward::Response &res);
		bool reloadDomain(ros2plan_knowledge_msgs::ReloadRDDLDomainProblem::Request &req, ros2plan_knowledge_msgs::ReloadRDDLDomainProblem::Response &res);
		bool getEnumTypes(ros2plan_knowledge_msgs::GetEnumerableTypeService::Request &req, ros2plan_knowledge_msgs::GetEnumerableTypeService::Response &res);
		bool getFluentType(ros2plan_knowledge_msgs::GetRDDLFluentType::Request &req, ros2plan_knowledge_msgs::GetRDDLFluentType::Response &res);
        void removeFact(const ros2plan_knowledge_msgs::KnowledgeItem &msg) override;
	public:
		RDDLKnowledgeBase(ros::NodeHandle& n);
		~RDDLKnowledgeBase() = default;


		/* parse domain and probelm files */
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
#endif
