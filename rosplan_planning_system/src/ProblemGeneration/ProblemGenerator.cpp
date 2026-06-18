#include "rosplan_planning_system/ProblemGeneration/ProblemGenerator.h"

namespace KCL_rosplan {

    void ProblemGenerator::generateProblemFile(std::string &problemPath) {
        if (knowledge_base.empty()) {
            RCLCPP_ERROR(svc_node_->get_logger(), "KCL: (%s) Knowledge base is not set!",
                         svc_node_->get_name());
        }
        std::ofstream pFile;
        pFile.open(problemPath.c_str());
        makeProblem(pFile);
        pFile.close();
    }

    ProblemGenerator::ProblemGenerator(const std::string &kb, rclcpp::Node::SharedPtr svc_node)
        : knowledge_base(kb), svc_node_(svc_node)
    {
        std::stringstream ss;

        ss << "/" << knowledge_base << "/domain/name";
        domain_name_service = ss.str(); ss.str("");

        ss << "/" << knowledge_base << "/domain/types";
        domain_type_service = ss.str(); ss.str("");

        ss << "/" << knowledge_base << "/domain/predicates";
        domain_predicate_service = ss.str(); ss.str("");

        ss << "/" << knowledge_base << "/domain/operators";
        domain_operator_list_service = ss.str(); ss.str("");

        ss << "/" << knowledge_base << "/domain/operator_details";
        domain_operator_details_service = ss.str(); ss.str("");

        ss << "/" << knowledge_base << "/domain/functions";
        domain_function_service = ss.str(); ss.str("");

        ss << "/" << knowledge_base << "/state/instances";
        state_instance_service = ss.str(); ss.str("");

        ss << "/" << knowledge_base << "/state/propositions";
        state_proposition_service = ss.str(); ss.str("");

        ss << "/" << knowledge_base << "/state/functions";
        state_function_service = ss.str(); ss.str("");

        ss << "/" << knowledge_base << "/state/timed_knowledge";
        state_timed_knowledge_service = ss.str(); ss.str("");

        ss << "/" << knowledge_base << "/state/goals";
        state_goal_service = ss.str(); ss.str("");

        ss << "/" << knowledge_base << "/state/metric";
        state_metric_service = ss.str(); ss.str("");
    }

} // close namespace
