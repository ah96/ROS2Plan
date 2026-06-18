//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 26/09/18.
//

#include "rosplan_planning_system/ProblemGeneration/ProblemGeneratorFactory.h"
#include "rosplan_planning_system/ProblemGeneration/PDDLProblemGenerator.h"
#include "rosplan_planning_system/ProblemGeneration/RDDLProblemGenerator.h"
#include "rosplan_planning_system/ProblemGeneration/CHIMPProblemGenerator.h"
#include <algorithm>

namespace KCL_rosplan {

    ProblemGeneratorPtr ProblemGeneratorFactory::createProblemGenerator(
        const std::string& pg_type_str,
        const std::string& kb,
        rclcpp::Node::SharedPtr svc_node)
    {
        std::string type = pg_type_str;
        std::transform(type.begin(), type.end(), type.begin(), ::tolower);

        if (type == "pddl" || type == "ppddl") {
            return ProblemGeneratorPtr(new PDDLProblemGenerator(kb, svc_node));
        } else if (type == "rddl") {
            return ProblemGeneratorPtr(new RDDLProblemGenerator(kb, svc_node));
        } else if (type == "chimp") {
            return ProblemGeneratorPtr(new CHIMPProblemGenerator(kb, svc_node));
        } else {
            RCLCPP_ERROR(svc_node->get_logger(),
                "KCL: (ProblemGeneratorFactory) Unknown Problem Generator type: %s", type.c_str());
            rclcpp::shutdown();
            return nullptr;
        }
    }

} // close namespace
