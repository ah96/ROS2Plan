//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 30/09/18.
//

#include "ros2plan_knowledge_base/PPDDLParser.h"

namespace KCL_ros2plan {
    PPDDLDomainPtr PPDDLParser::parseDomainProblem(const std::string &domainPath, const std::string &problemPath) {
        // only parse domain once
        if (domain_parsed) return domain;
        RCLCPP_INFO(rclcpp::get_logger("PPDDLParser"), "KCL: Parsing domain: %s.", domainPath.c_str());
        RCLCPP_INFO(rclcpp::get_logger("PPDDLParser"), "KCL: Parsing initial state");

        // parse domain
        try {
            std::vector<std::string> problem_vect;
            if (not problemPath.empty()) problem_vect.push_back(problemPath);
            domain = PPDDLInterface::Domain(domainPath, problem_vect)._getWrappedDomain();
        }
        catch (std::runtime_error e) {
            std::string filetype = "domain";
            std::string err(e.what());
            if (err.find("Problem") != std::string::npos or err.find("problem") != std::string::npos) filetype = "problem";
            RCLCPP_ERROR(rclcpp::get_logger("PPDDLParser"), "KCL: Failed to open %s file.", filetype.c_str());
            return nullptr;
        }
        if (not domain) {
            RCLCPP_ERROR(rclcpp::get_logger("PPDDLParser"), "KCL: Failed to open domain file.");
            return nullptr;
        }

        domain_name =  domain->name();
        problem = ppddl_parser::Problem::begin()->second;
        problem_name = problem->name();
        domain_parsed = true;

        return domain;
    }
}
