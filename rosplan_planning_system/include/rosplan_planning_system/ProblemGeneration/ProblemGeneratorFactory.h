//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 26/09/18.
//

#ifndef ROSPLAN_PLANNING_SYSTEM_PROBLEMGENERATORFACTORY_H
#define ROSPLAN_PLANNING_SYSTEM_PROBLEMGENERATORFACTORY_H

#include "ProblemGenerator.h"
#include "PDDLProblemGenerator.h"
#include "RDDLProblemGenerator.h"
#include "CHIMPProblemGenerator.h"

namespace KCL_rosplan {
    typedef std::unique_ptr<ProblemGenerator> ProblemGeneratorPtr;

    class ProblemGeneratorFactory {
    public:
        static ProblemGeneratorPtr createProblemGenerator(const std::string& pg_type,
                                                           const std::string& kb,
                                                           rclcpp::Node::SharedPtr svc_node);
    };
}

#endif //ROSPLAN_PLANNING_SYSTEM_PROBLEMGENERATORFACTORY_H
