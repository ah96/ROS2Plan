
#include "ros2plan_knowledge_base/RDDLTaskParser.h"

/* implementation of RDDLTaskParser.h */
namespace KCL_ros2plan {

	/*----------------*/
	/* parsing domain */
	/*----------------*/

	/**
	 * parse the domain file
	 */
    RDDLTask *RDDLTaskParser::parseTask(const std::string &domainPath, const std::string &instancePath, bool reload) {
		// only parse domain once
		if (domain_parsed and not reload) return rddlTask;
        RCLCPP_INFO(rclcpp::get_logger("RDDLTaskParser"), "KCL: Parsing domain: %s.", domainPath.c_str());
        if (not instancePath.empty())
            RCLCPP_INFO(rclcpp::get_logger("RDDLTaskParser"), "KCL: Parsing initial state: %s", instancePath.c_str());
        else RCLCPP_INFO(rclcpp::get_logger("RDDLTaskParser"), "KCL: Parsing initial state");

        // parse domain
        rddlTask = RDDLParser::parseRDDLTask(domainPath, instancePath);
        if (not rddlTask) {
            RCLCPP_ERROR(rclcpp::get_logger("RDDLTaskParser"), "KCL: Failed to open domain file.");
            return nullptr;
        }
        domain_name = rddlTask->domainName;
        problem_name = rddlTask->name;

        RCLCPP_INFO(rclcpp::get_logger("RDDLTaskParser"), "KCL: Pre-processing domain...");
        uninstantiated_SACs = rddlTask->SACs;
        Instantiator instantiator(rddlTask);
        instantiator.instantiate(false);
        Preprocessor preprocessor(rddlTask, false); // ipc2018 to false
        preprocessor.preprocess(false);
        RCLCPP_INFO(rclcpp::get_logger("RDDLTaskParser"), "KCL: RDDL domain is ready!");

        domain_parsed = true;

		return rddlTask;
	}

} // close namespace
