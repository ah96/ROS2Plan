//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 26/09/18.
//

#ifndef ROS2PLAN_PLANNING_SYSTEM_PROBLEMGENERATORFACTORY_H
#define ROS2PLAN_PLANNING_SYSTEM_PROBLEMGENERATORFACTORY_H

#include "ProblemGenerator.h"
#include "PDDLProblemGenerator.h"
#include "RDDLProblemGenerator.h"
#include "CHIMPProblemGenerator.h"

namespace KCL_ros2plan {
    typedef std::unique_ptr<ProblemGenerator> ProblemGeneratorPtr;

    class ProblemGeneratorFactory {
    public:
        static ProblemGeneratorPtr createProblemGenerator(const std::string& pg_type,
                                                           const std::string& kb,
                                                           rclcpp::Node::SharedPtr svc_node);
    };
}

#endif //ROS2PLAN_PLANNING_SYSTEM_PROBLEMGENERATORFACTORY_H
