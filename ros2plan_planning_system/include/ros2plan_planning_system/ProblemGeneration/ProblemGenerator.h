/**
 * This class is responsible for generating the PDDL instance.
 * This is done by using the objects requested from Knowledge services.
 */
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

#include "ros2plan_knowledge_msgs/srv/get_domain_name_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_type_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_instance_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_metric_service.hpp"

#ifndef KCL_ProblemGenerator
#define KCL_ProblemGenerator

namespace KCL_ros2plan {

    class ProblemGenerator
    {
    protected:

        std::string domain_name_service;
        std::string domain_type_service;
        std::string domain_predicate_service;
        std::string domain_function_service;
        std::string domain_operator_details_service;
        std::string domain_operator_list_service;

        std::string state_instance_service;
        std::string state_proposition_service;
        std::string state_function_service;
        std::string state_timed_knowledge_service;
        std::string state_goal_service;
        std::string state_metric_service;

        // satellite node for service calls (not in any executor — safe for spin_until_future_complete)
        rclcpp::Node::SharedPtr svc_node_;

        virtual void makeProblem(std::ofstream &pFile) = 0;

        // params
        std::string knowledge_base;

    public:

        ProblemGenerator(const std::string& kb, rclcpp::Node::SharedPtr svc_node);
        // Virtual: generators are owned through ProblemGeneratorPtr (a base-class unique_ptr), so
        // without it a subclass's service clients were never destroyed, kept the ROS context
        // alive past main(), and crashed the DDS threads at exit.
        virtual ~ProblemGenerator() = default;

        virtual void generateProblemFile(std::string &problemPath);
    };

} // close namespace

#endif
