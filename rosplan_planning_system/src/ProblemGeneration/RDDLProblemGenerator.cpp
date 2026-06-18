//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 26/09/18.
//

#include "rosplan_planning_system/ProblemGeneration/RDDLProblemGenerator.h"
#include <iomanip>

namespace KCL_rosplan {

    // Helper macro for synchronous service call via satellite node
    #define CALL_SVC(client, request, response, err_msg) \
    do { \
        if (!client->wait_for_service(std::chrono::seconds(5))) { \
            RCLCPP_ERROR(svc_node_->get_logger(), "KCL: (RDDLProblemGenerator) Service unavailable: %s", err_msg); \
        } else { \
            auto _future = client->async_send_request(request); \
            auto _status = rclcpp::spin_until_future_complete(svc_node_, _future, std::chrono::seconds(5)); \
            if (_status == rclcpp::FutureReturnCode::SUCCESS) { \
                response = _future.get(); \
            } else { \
                RCLCPP_ERROR(svc_node_->get_logger(), "KCL: (RDDLProblemGenerator) Failed to call service: %s", err_msg); \
            } \
        } \
    } while(0)

    RDDLProblemGenerator::RDDLProblemGenerator(const std::string& kb, rclcpp::Node::SharedPtr svc_node)
        : ProblemGenerator(kb, svc_node)
    {
        std::string reload_svc  = "/" + kb + "/reload_rddl_domain";
        std::string fluent_type_svc = "/" + kb + "/domain/fluent_type";
        std::string enum_type_svc   = "/" + kb + "/domain/enumerable_type";
        std::string rddl_params_svc = "/" + kb + "/state/rddl_parameters";

        get_name_client_       = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetDomainNameService>(domain_name_service);
        get_types_client_      = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetDomainTypeService>(domain_type_service);
        get_instances_client_  = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetInstanceService>(state_instance_service);
        get_domain_props_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetDomainAttributeService>(domain_predicate_service);
        get_domain_funcs_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetDomainAttributeService>(domain_function_service);
        get_op_list_client_    = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetDomainOperatorService>(domain_operator_list_service);
        get_op_details_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetDomainOperatorDetailsService>(domain_operator_details_service);
        get_props_client_      = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetAttributeService>(state_proposition_service);
        get_funcs_client_      = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetAttributeService>(state_function_service);
        reload_domain_client_  = svc_node_->create_client<rosplan_knowledge_msgs::srv::ReloadRDDLDomainProblem>(reload_svc);
        get_fluent_type_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetRDDLFluentType>(fluent_type_svc);
        get_enumerable_type_client_  = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetEnumerableTypeService>(enum_type_svc);
        get_rddl_params_client_ = svc_node_->create_client<rosplan_knowledge_msgs::srv::GetRDDLParams>(rddl_params_svc);

        // Get domain name
        auto nameReq = std::make_shared<rosplan_knowledge_msgs::srv::GetDomainNameService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainNameService::Response> nameRes;
        CALL_SVC(get_name_client_, nameReq, nameRes, domain_name_service.c_str());
        if (nameRes) {
            _domain_name = nameRes->domain_name;
        } else {
            RCLCPP_ERROR(svc_node_->get_logger(),
                "KCL: (RDDLProblemGenerator) Failed to get domain name. Is the Knowledge Base running?");
        }
        _non_fluents_name = "nf_" + _domain_name + "__generate_instance";
    }

    void RDDLProblemGenerator::generateProblemFile(std::string &problemPath) {
        if (knowledge_base.empty()) {
            RCLCPP_ERROR(svc_node_->get_logger(), "KCL: (RDDLProblemGenerator) Knowledge base is not set!");
        }
        std::ofstream pFile;
        pFile.open(problemPath.c_str());
        makeProblem(pFile);
        pFile.close();

        auto reloadReq = std::make_shared<rosplan_knowledge_msgs::srv::ReloadRDDLDomainProblem::Request>();
        reloadReq->problem_path = problemPath;
        std::shared_ptr<rosplan_knowledge_msgs::srv::ReloadRDDLDomainProblem::Response> reloadRes;
        CALL_SVC(reload_domain_client_, reloadReq, reloadRes, "reload_rddl_domain");
        if (!reloadRes || !reloadRes->success) {
            RCLCPP_ERROR(svc_node_->get_logger(), "KCL: (RDDLProblemGenerator) Failed to reload RDDL domain/problem.");
        }
    }

    void RDDLProblemGenerator::makeProblem(std::ofstream &pFile) {
        std::vector<std::string> pred_funcs = getPredicatesFunctions();
        std::set<std::string> fluents, non_fluents;
        getAllFluents(pred_funcs, fluents, non_fluents);
        std::map<std::string, std::string> fluent_types;
        getFluentTypes(fluents, non_fluents, fluent_types);
        pFile << "// Auto-generated problem file by ROSPlan" << std::endl;
        makeNonFluents(pFile, non_fluents, fluent_types);
        makeInstance(pFile, fluents, fluent_types);
    }

    void RDDLProblemGenerator::makeNonFluents(std::ofstream &pFile,
                                               const std::set<std::string>& nonfluents,
                                               const std::map<std::string, std::string>& fluent_types)
    {
        pFile << "non-fluents " << _non_fluents_name << " {" << std::endl;
        pFile << "\tdomain = " << _domain_name << ";" << std::endl;
        pFile << "\tobjects {" << std::endl;

        std::map<std::string, std::vector<std::string>> instances = getInstances();
        for (auto it = instances.begin(); it != instances.end(); ++it) {
            if (!it->second.empty()) {
                if ((*it->second.begin())[0] == '@') {
                    auto enumReq = std::make_shared<rosplan_knowledge_msgs::srv::GetEnumerableTypeService::Request>();
                    enumReq->type_name = it->first;
                    std::shared_ptr<rosplan_knowledge_msgs::srv::GetEnumerableTypeService::Response> enumRes;
                    CALL_SVC(get_enumerable_type_client_, enumReq, enumRes, "enumerable_type");
                    if (enumRes) {
                        _enumeration_types[it->first] = enumRes->values;
                    }
                } else {
                    pFile << "\t\t" << it->first << ": " << "{";
                    bool comma = false;
                    for (auto& inst : it->second) {
                        if (comma) pFile << ", ";
                        else comma = true;
                        pFile << inst;
                    }
                    pFile << "};" << std::endl;
                }
            }
        }
        pFile << "\t};" << std::endl;

        pFile << "\tnon-fluents {" << std::endl;
        printGenericFluentList(pFile, nonfluents, fluent_types);
        pFile << "\t};" << std::endl;
        pFile << "}" << std::endl;
    }

    void RDDLProblemGenerator::makeInstance(std::ofstream &pFile,
                                             const std::set<std::string>& fluents,
                                             const std::map<std::string, std::string>& fluent_types)
    {
        pFile << "instance " << _domain_name << "__generated_instance {" << std::endl;
        pFile << "\tdomain = " << _domain_name << ";" << std::endl;
        pFile << "\tnon-fluents = " << _non_fluents_name << ";" << std::endl;

        pFile << "\tinit-state {" << std::endl;
        printGenericFluentList(pFile, fluents, fluent_types);
        pFile << "\t};" << std::endl;

        auto paramsReq = std::make_shared<rosplan_knowledge_msgs::srv::GetRDDLParams::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetRDDLParams::Response> paramsRes;
        CALL_SVC(get_rddl_params_client_, paramsReq, paramsRes, "state/rddl_parameters");

        if (paramsRes) {
            pFile << "\tmax-nondef-actions = " << paramsRes->max_nondef_actions << ";" << std::endl;
            pFile << "\thorizon = " << paramsRes->horizon << ";" << std::endl;
            pFile << "\tdiscount = " << std::fixed << std::setprecision(2) << paramsRes->discount_factor << ";" << std::endl;
        }
        pFile << "}" << std::endl;
    }

    std::map<std::string, std::vector<std::string>> RDDLProblemGenerator::getInstances() {
        std::map<std::string, std::vector<std::string>> inst;

        auto typeReq = std::make_shared<rosplan_knowledge_msgs::srv::GetDomainTypeService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainTypeService::Response> typeRes;
        CALL_SVC(get_types_client_, typeReq, typeRes, domain_type_service.c_str());

        if (!typeRes) return inst;

        for (size_t t = 0; t < typeRes->types.size(); ++t) {
            auto instReq = std::make_shared<rosplan_knowledge_msgs::srv::GetInstanceService::Request>();
            instReq->type_name = typeRes->types[t];
            std::shared_ptr<rosplan_knowledge_msgs::srv::GetInstanceService::Response> instRes;
            CALL_SVC(get_instances_client_, instReq, instRes, state_instance_service.c_str());

            if (instRes && !instRes->instances.empty()) {
                inst[instReq->type_name] = instRes->instances;
            }
        }
        return inst;
    }

    void RDDLProblemGenerator::getAllFluents(const std::vector<std::string>& pred_funcs,
                                              std::set<std::string>& found_fluents,
                                              std::set<std::string>& found_nonfluents)
    {
        auto opListReq = std::make_shared<rosplan_knowledge_msgs::srv::GetDomainOperatorService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainOperatorService::Response> opListRes;
        CALL_SVC(get_op_list_client_, opListReq, opListRes, domain_operator_list_service.c_str());

        if (!opListRes) return;

        for (auto& op : opListRes->operators) {
            auto opDetReq = std::make_shared<rosplan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Request>();
            opDetReq->name = op.name;
            std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Response> opDetRes;
            CALL_SVC(get_op_details_client_, opDetReq, opDetRes, domain_operator_details_service.c_str());

            if (!opDetRes) continue;

            for (auto& e : opDetRes->op.at_start_add_effects) found_fluents.insert(e.name);
            for (auto& e : opDetRes->op.at_start_del_effects) found_fluents.insert(e.name);
            for (auto& e : opDetRes->op.at_end_add_effects)   found_fluents.insert(e.name);
            for (auto& e : opDetRes->op.at_end_del_effects)   found_fluents.insert(e.name);
            for (auto& e : opDetRes->op.at_start_assign_effects) found_fluents.insert(e.lhs.name);
            for (auto& e : opDetRes->op.at_end_assign_effects)   found_fluents.insert(e.lhs.name);
            for (auto& prob : opDetRes->op.probabilistic_effects) {
                for (auto& e : prob.add_effects) found_fluents.insert(e.name);
                for (auto& e : prob.del_effects) found_fluents.insert(e.name);
                for (auto& e : prob.assign_effects) found_fluents.insert(e.lhs.name);
            }
        }

        for (auto& pf : pred_funcs) {
            if (found_fluents.find(pf) == found_fluents.end()) found_nonfluents.insert(pf);
        }
    }

    void RDDLProblemGenerator::getFluentTypes(const std::set<std::string>& fluents,
                                               const std::set<std::string>& nonfluents,
                                               std::map<std::string, std::string>& fluent_types)
    {
        auto querySet = [this, &fluent_types](const std::set<std::string>& names) {
            for (auto& name : names) {
                auto req = std::make_shared<rosplan_knowledge_msgs::srv::GetRDDLFluentType::Request>();
                req->fluent_name = name;
                std::shared_ptr<rosplan_knowledge_msgs::srv::GetRDDLFluentType::Response> res;
                CALL_SVC(get_fluent_type_client_, req, res, "domain/fluent_type");
                if (res) fluent_types[name] = res->type;
            }
        };
        querySet(fluents);
        querySet(nonfluents);
    }

    std::vector<std::string> RDDLProblemGenerator::getPredicatesFunctions() {
        std::vector<std::string> pred_funcs;

        auto propReq = std::make_shared<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Response> propRes;
        CALL_SVC(get_domain_props_client_, propReq, propRes, domain_predicate_service.c_str());

        auto funcReq = std::make_shared<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetDomainAttributeService::Response> funcRes;
        CALL_SVC(get_domain_funcs_client_, funcReq, funcRes, domain_function_service.c_str());

        if (propRes) for (auto& item : propRes->items) pred_funcs.push_back(item.name);
        if (funcRes) for (auto& item : funcRes->items) pred_funcs.push_back(item.name);
        return pred_funcs;
    }

    void RDDLProblemGenerator::printGenericFluentElement(
        std::ofstream &pFile,
        std::vector<rosplan_knowledge_msgs::msg::KnowledgeItem>::iterator pfit,
        const std::map<std::string, std::string>& fluent_types)
    {
        pFile << "\t\t" << pfit->attribute_name;
        if (!pfit->values.empty()) {
            pFile << "(";
            bool comma = false;
            for (auto& v : pfit->values) {
                if (comma) pFile << ", ";
                else comma = true;
                pFile << v.value;
            }
            pFile << ")";
        }
        pFile << " = ";
        if (pfit->knowledge_type == rosplan_knowledge_msgs::msg::KnowledgeItem::FACT) {
            pFile << ((pfit->is_negative == 0) ? "true" : "false");
        } else {
            auto enum_type = _enumeration_types.find(fluent_types.at(pfit->attribute_name));
            if (enum_type != _enumeration_types.end()) {
                pFile << enum_type->second[static_cast<size_t>(pfit->function_value)];
            } else {
                pFile << pfit->function_value;
            }
        }
        pFile << ";" << std::endl;
    }

    void RDDLProblemGenerator::printGenericFluentList(
        std::ofstream &pFile,
        const std::set<std::string>& fluentlist,
        const std::map<std::string, std::string>& fluent_types)
    {
        // Get all propositions
        auto propReq = std::make_shared<rosplan_knowledge_msgs::srv::GetAttributeService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetAttributeService::Response> propRes;
        CALL_SVC(get_props_client_, propReq, propRes, state_proposition_service.c_str());

        // Get all functions
        auto funcReq = std::make_shared<rosplan_knowledge_msgs::srv::GetAttributeService::Request>();
        std::shared_ptr<rosplan_knowledge_msgs::srv::GetAttributeService::Response> funcRes;
        CALL_SVC(get_funcs_client_, funcReq, funcRes, state_function_service.c_str());

        if (propRes) {
            for (auto pit = propRes->attributes.begin(); pit != propRes->attributes.end(); ++pit) {
                for (auto& fl : fluentlist) {
                    if (pit->attribute_name == fl) printGenericFluentElement(pFile, pit, fluent_types);
                }
            }
        }
        if (funcRes) {
            for (auto pit = funcRes->attributes.begin(); pit != funcRes->attributes.end(); ++pit) {
                for (auto& fl : fluentlist) {
                    if (pit->attribute_name == fl) printGenericFluentElement(pFile, pit, fluent_types);
                }
            }
        }
    }

} // close namespace
