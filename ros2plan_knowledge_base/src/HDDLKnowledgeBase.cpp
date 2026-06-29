#include "ros2plan_knowledge_base/HDDLKnowledgeBase.h"

namespace KCL_ros2plan {

    namespace kmsrv = ros2plan_knowledge_msgs::srv;
    namespace kmmsg = ros2plan_knowledge_msgs::msg;

    void HDDLKnowledgeBase::parseDomain(const std::string& domain_file_path,
                                        const std::string& problem_file_path)
    {
        problem_file_path_ = problem_file_path;

        RCLCPP_INFO(get_logger(), "KCL: Parsing hddl domain : %s", domain_file_path.c_str());
        hddl_parser_.parse(domain_file_path);

        if (!hddl_parser_.parsing_ok()) {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) There were syntax errors in the domain file.", get_name());
            rclcpp::shutdown();
            return;
        }

        if (problem_file_path_ != "") addInitialState();
        RCLCPP_INFO(get_logger(), "KCL: successfully parsed hddl domain!");
    }

    void HDDLKnowledgeBase::addInitialState() {
        RCLCPP_WARN(get_logger(), "Initial state not yet implemented, cannot load: %s",
                    problem_file_path_.c_str());
    }

    void HDDLKnowledgeBase::addConstants() {}

    void HDDLKnowledgeBase::getDomainName(
        const std::shared_ptr<kmsrv::GetDomainNameService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetDomainNameService::Response> res)
    {
        res->domain_name = hddl_parser_.domain_.domain_name_;
    }

    void HDDLKnowledgeBase::getTypes(
        const std::shared_ptr<kmsrv::GetDomainTypeService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetDomainTypeService::Response> res)
    {
        for (auto it = hddl_parser_.domain_.domain_types_.begin();
             it != hddl_parser_.domain_.domain_types_.end(); it++) {
            res->types.push_back(it->first);
            res->super_types.push_back(it->second);
        }
    }

    void HDDLKnowledgeBase::getPredicates(
        const std::shared_ptr<kmsrv::GetDomainAttributeService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetDomainAttributeService::Response> res)
    {
        for (auto it = hddl_parser_.domain_.domain_predicates_.begin();
             it != hddl_parser_.domain_.domain_predicates_.end(); it++) {
            kmmsg::DomainFormula df;
            df.name = it->name;
            for (auto pit = it->pred_params.params.begin();
                 pit != it->pred_params.params.end(); pit++) {
                diagnostic_msgs::msg::KeyValue kv;
                kv.key   = *pit;
                kv.value = it->pred_params.params_map[*pit];
                df.typed_parameters.push_back(kv);
            }
            res->items.push_back(df);
        }
    }

    void HDDLKnowledgeBase::getPredicateDetails(
        const std::shared_ptr<kmsrv::GetDomainPredicateDetailsService::Request> req,
        std::shared_ptr<kmsrv::GetDomainPredicateDetailsService::Response> res)
    {
        for (auto it = hddl_parser_.domain_.domain_predicates_.begin();
             it != hddl_parser_.domain_.domain_predicates_.end(); it++) {
            if (it->name == req->name) {
                kmmsg::DomainFormula df;
                df.name = it->name;
                for (auto pit = it->pred_params.params.begin();
                     pit != it->pred_params.params.end(); pit++) {
                    diagnostic_msgs::msg::KeyValue kv;
                    kv.key   = *pit;
                    kv.value = it->pred_params.params_map[*pit];
                    df.typed_parameters.push_back(kv);
                }
                res->predicate = df;
                return;
            }
        }
    }

    void HDDLKnowledgeBase::getFunctionPredicates(
        const std::shared_ptr<kmsrv::GetDomainAttributeService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetDomainAttributeService::Response> /*res*/)
    {
        RCLCPP_WARN(get_logger(), "Functions are not supported in hddl, returning empty response");
    }

    void HDDLKnowledgeBase::getOperators(
        const std::shared_ptr<kmsrv::GetDomainOperatorService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetDomainOperatorService::Response> res)
    {
        for (auto it = hddl_parser_.domain_.domain_actions_.begin();
             it != hddl_parser_.domain_.domain_actions_.end(); it++) {
            kmmsg::DomainFormula df;
            df.name = it->name;
            for (auto pit = it->action_params.params.begin();
                 pit != it->action_params.params.end(); pit++) {
                diagnostic_msgs::msg::KeyValue kv;
                kv.key   = *pit;
                kv.value = it->action_params.params_map[*pit];
                df.typed_parameters.push_back(kv);
            }
            res->operators.push_back(df);
        }
    }

    void HDDLKnowledgeBase::getOperatorDetails(
        const std::shared_ptr<kmsrv::GetDomainOperatorDetailsService::Request> req,
        std::shared_ptr<kmsrv::GetDomainOperatorDetailsService::Response> res)
    {
        std::string request = req->name;
        if (!request.empty() && request.front() == '!') request.erase(0, 1);

        for (auto it = hddl_parser_.domain_.domain_actions_.begin();
             it != hddl_parser_.domain_.domain_actions_.end(); it++) {

            if (it->name != request) continue;

            kmmsg::DomainOperator dop;
            kmmsg::DomainFormula df;
            df.name = it->name;
            for (auto pit = it->action_params.params.begin();
                 pit != it->action_params.params.end(); pit++) {
                diagnostic_msgs::msg::KeyValue kv;
                kv.key   = *pit;
                kv.value = it->action_params.params_map[*pit];
                df.typed_parameters.push_back(kv);
            }
            dop.formula = df;

            for (auto pit = it->preconditions.begin(); pit != it->preconditions.end(); pit++) {
                kmmsg::DomainFormula pdf;
                pdf.name = pit->name;
                for (auto pait = pit->pred_params.params.begin();
                     pait != pit->pred_params.params.end(); pait++) {
                    diagnostic_msgs::msg::KeyValue kv;
                    kv.key   = *pait;
                    kv.value = it->action_params.params_map[*pait];
                    pdf.typed_parameters.push_back(kv);
                }
                if (pit->negated) dop.at_start_neg_condition.push_back(pdf);
                else              dop.at_start_simple_condition.push_back(pdf);
            }

            for (auto eit = it->effects.begin(); eit != it->effects.end(); eit++) {
                kmmsg::DomainFormula edf;
                edf.name = eit->name;
                for (auto pait = eit->pred_params.params.begin();
                     pait != eit->pred_params.params.end(); pait++) {
                    diagnostic_msgs::msg::KeyValue kv;
                    kv.key   = *pait;
                    kv.value = it->action_params.params_map[*pait];
                    edf.typed_parameters.push_back(kv);
                }
                if (eit->negated) dop.at_end_del_effects.push_back(edf);
                else              dop.at_end_add_effects.push_back(edf);
            }

            res->op = dop;
            return;
        }

        RCLCPP_ERROR(get_logger(), "operator '%s' not found", request.c_str());
    }

} // namespace KCL_ros2plan
