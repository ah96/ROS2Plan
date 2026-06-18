#include "rosplan_knowledge_base/KnowledgeBase.h"
#include "rosplan_knowledge_base/KnowledgeBaseFactory.h"

namespace KCL_rosplan {

    using KI  = rosplan_knowledge_msgs::msg::KnowledgeItem;
    namespace kmsrv = rosplan_knowledge_msgs::srv;

    /*-------------*/
    /* constructor */
    /*-------------*/

    KnowledgeBase::KnowledgeBase(const std::string& node_name)
        : rclcpp::Node(node_name)
    {
        srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        using namespace std::placeholders;
        auto sopt = rcl_service_options_t{};
        sopt = rcl_service_get_default_options();

        // domain info
        domainServer1 = create_service<kmsrv::GetDomainNameService>(
            "~/domain/name", std::bind(&KnowledgeBase::getDomainName, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        domainServer2 = create_service<kmsrv::GetDomainTypeService>(
            "~/domain/types", std::bind(&KnowledgeBase::getTypes, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        domainServer3 = create_service<kmsrv::GetDomainAttributeService>(
            "~/domain/predicates", std::bind(&KnowledgeBase::getPredicates, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        domainServer4 = create_service<kmsrv::GetDomainAttributeService>(
            "~/domain/functions", std::bind(&KnowledgeBase::getFunctionPredicates, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        domainServer5 = create_service<kmsrv::GetDomainOperatorService>(
            "~/domain/operators", std::bind(&KnowledgeBase::getOperators, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        domainServer6 = create_service<kmsrv::GetDomainOperatorDetailsService>(
            "~/domain/operator_details", std::bind(&KnowledgeBase::getOperatorDetails, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        domainServer7 = create_service<kmsrv::GetDomainPredicateDetailsService>(
            "~/domain/predicate_details", std::bind(&KnowledgeBase::getPredicateDetails, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);

        // query
        queryServer = create_service<kmsrv::KnowledgeQueryService>(
            "~/query_state", std::bind(&KnowledgeBase::queryKnowledge, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);

        // update
        updateServer0 = create_service<std_srvs::srv::Empty>(
            "~/clear", std::bind(&KnowledgeBase::clearKnowledge, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        updateServer1 = create_service<kmsrv::KnowledgeUpdateService>(
            "~/update", std::bind(&KnowledgeBase::updateKnowledge, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        updateServer2 = create_service<kmsrv::KnowledgeUpdateServiceArray>(
            "~/update_array", std::bind(&KnowledgeBase::updateKnowledgeArray, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        updateServer3 = create_service<kmsrv::KnowledgeUpdateServiceArray>(
            "~/update_constraints_oneof",
            std::bind(&KnowledgeBase::updateKnowledgeConstraintsOneOf, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);

        // state fetch
        stateServer1 = create_service<kmsrv::GetInstanceService>(
            "~/state/instances", std::bind(&KnowledgeBase::getInstances, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        stateServer2 = create_service<kmsrv::GetAttributeService>(
            "~/state/propositions", std::bind(&KnowledgeBase::getPropositions, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        stateServer3 = create_service<kmsrv::GetAttributeService>(
            "~/state/functions", std::bind(&KnowledgeBase::getFunctions, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        stateServer4 = create_service<kmsrv::GetAttributeService>(
            "~/state/timed_knowledge", std::bind(&KnowledgeBase::getTimedKnowledge, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        stateServer5 = create_service<kmsrv::GetAttributeService>(
            "~/state/goals", std::bind(&KnowledgeBase::getGoals, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);
        stateServer6 = create_service<kmsrv::GetMetricService>(
            "~/state/metric", std::bind(&KnowledgeBase::getMetric, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);

        // sensed predicates
        senseServer = create_service<kmsrv::SetNamedBool>(
            "~/update_sensed_predicates", std::bind(&KnowledgeBase::setSensedPredicate, this, _1, _2),
            rmw_qos_profile_services_default, srv_cbg_);

        // publisher (latched — transient_local keeps last message for late subscribers)
        status_pub_ = create_publisher<rosplan_knowledge_msgs::msg::StatusUpdate>(
            "~/status/update", rclcpp::QoS(100).transient_local());

        // parameters
        declare_parameter("kb_rate", 100.0);
        _kb_rate = get_parameter("kb_rate").as_double();

        // timer for timed initial literals
        auto period = std::chrono::duration<double>(1.0 / _kb_rate);
        til_timer_ = create_wall_timer(period,
            std::bind(&KnowledgeBase::checkTimedInitialLiterals, this));
    }

    /*------------------------------*/
    /* timed initial literals timer */
    /*------------------------------*/

    void KnowledgeBase::checkTimedInitialLiterals() {
        auto t = now();
        auto tit = model_timed_initial_literals.begin();
        for (; tit != model_timed_initial_literals.end();) {
            if (rclcpp::Time(tit->initial_time) <= t) {
                if (tit->is_negative) {
                    tit->is_negative = false;
                    removeKnowledge(*tit);
                } else {
                    addKnowledge(*tit);
                }
                model_timed_initial_literals.erase(tit);
            } else {
                tit++;
            }
        }
    }

    /*-----------------*/
    /* knowledge query */
    /*-----------------*/

    void KnowledgeBase::queryKnowledge(
        const std::shared_ptr<kmsrv::KnowledgeQueryService::Request> req,
        std::shared_ptr<kmsrv::KnowledgeQueryService::Response> res)
    {
        res->all_true = true;
        for (auto iit = req->knowledge.begin(); iit != req->knowledge.end(); iit++) {

            bool present = false;
            switch (iit->knowledge_type) {

            case KI::INSTANCE:
                {
                    auto sit = find(model_instances[iit->instance_type].begin(),
                                    model_instances[iit->instance_type].end(),
                                    iit->instance_name);
                    present = (sit != model_instances[iit->instance_type].end());
                    if (!present) {
                        sit = find(domain_constants[iit->instance_type].begin(),
                                   domain_constants[iit->instance_type].end(),
                                   iit->instance_name);
                        present = (sit != domain_constants[iit->instance_type].end());
                    }
                }
                break;

            case KI::FUNCTION:
                for (auto pit = model_functions.begin(); pit != model_functions.end(); pit++) {
                    if (KnowledgeComparitor::containsKnowledge(*iit, *pit)) {
                        present = true;
                        break;
                    }
                }
                break;

            case KI::FACT:
                for (auto pit = model_facts.begin(); pit != model_facts.end(); pit++) {
                    if (KnowledgeComparitor::containsKnowledge(*iit, *pit)) {
                        present = true;
                        break;
                    }
                }
                break;

            case KI::INEQUALITY:
                present = KnowledgeComparitor::inequalityTrue(*iit, model_functions);
                break;
            }

            if (!present) {
                res->all_true = false;
                res->results.push_back(false);
                res->false_knowledge.push_back(*iit);
            } else {
                res->results.push_back(true);
            }
        }
    }

    /*--------------------*/
    /* adding constraints */
    /*--------------------*/

    void KnowledgeBase::updateKnowledgeConstraintsOneOf(
        const std::shared_ptr<kmsrv::KnowledgeUpdateServiceArray::Request> req,
        std::shared_ptr<kmsrv::KnowledgeUpdateServiceArray::Response> res)
    {
        int count = 0;
        for (size_t i = 0; i < req->knowledge.size(); i++) {
            for (auto pit = model_facts.begin(); pit != model_facts.end(); pit++) {
                if (KnowledgeComparitor::containsKnowledge(req->knowledge[i], *pit)) count++;
            }
        }
        if (count > 1) RCLCPP_WARN(get_logger(), "KCL: (%s) Warning: more than one Knowledge Item is true in new OneOf constraint!", get_name());
        model_oneof_constraints.push_back(req->knowledge);
        res->success = true;
    }

    /*------------------*/
    /* knowledge update */
    /*------------------*/

    void KnowledgeBase::updateKnowledge(
        const std::shared_ptr<kmsrv::KnowledgeUpdateService::Request> req,
        std::shared_ptr<kmsrv::KnowledgeUpdateService::Response> res)
    {
        auto t = now();

        switch (req->update_type) {

        case kmsrv::KnowledgeUpdateService::Request::ADD_KNOWLEDGE:
            if (rclcpp::Time(req->knowledge.initial_time) > t) {
                model_timed_initial_literals.push_back(req->knowledge);
            } else {
                if (rclcpp::Time(req->knowledge.initial_time).seconds() == 0.0) {
                    req->knowledge.initial_time = t;
                }
                addKnowledge(req->knowledge);
            }
            break;

        case kmsrv::KnowledgeUpdateService::Request::ADD_GOAL:
            addMissionGoal(req->knowledge);
            break;

        case kmsrv::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE:
            removeKnowledge(req->knowledge);
            break;

        case kmsrv::KnowledgeUpdateService::Request::REMOVE_GOAL:
            removeMissionGoal(req->knowledge);
            break;

        case kmsrv::KnowledgeUpdateService::Request::ADD_METRIC:
            addMissionMetric(req->knowledge);
            break;

        case kmsrv::KnowledgeUpdateService::Request::REMOVE_METRIC:
            removeMissionMetric(req->knowledge);
            break;
        }

        res->success = true;
    }

    void KnowledgeBase::updateKnowledgeArray(
        const std::shared_ptr<kmsrv::KnowledgeUpdateServiceArray::Request> req,
        std::shared_ptr<kmsrv::KnowledgeUpdateServiceArray::Response> res)
    {
        res->success = true;

        auto single_req = std::make_shared<kmsrv::KnowledgeUpdateService::Request>();
        auto single_res = std::make_shared<kmsrv::KnowledgeUpdateService::Response>();

        for (size_t i = 0; i < req->update_type.size() && i < req->knowledge.size(); i++) {
            single_req->update_type = req->update_type[i];
            single_req->knowledge   = req->knowledge[i];
            updateKnowledge(single_req, single_res);
            res->success = res->success && single_res->success;
        }

        publishStatusUpdate();
    }

    /*----------------*/
    /* removing items */
    /*----------------*/

    void KnowledgeBase::clearKnowledge(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
    {
        KI empty;
        RCLCPP_INFO(get_logger(), "KCL: (%s) Removing whole model", get_name());
        model_timed_initial_literals.clear();
        model_constants.clear();
        model_instances.clear();
        model_facts.clear();
        model_functions.clear();
        model_goals.clear();
        model_metric = empty;
    }

    void KnowledgeBase::removeKnowledge(KI &msg) {

        auto t = now();
        if (rclcpp::Time(msg.initial_time) > t) {
            for (auto pit = model_timed_initial_literals.begin();
                 pit != model_timed_initial_literals.end();) {
                if (msg.initial_time == pit->initial_time &&
                    KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                    RCLCPP_INFO(get_logger(), "KCL: (%s) Removing timed attribute (%s)",
                                get_name(), msg.attribute_name.c_str());
                    model_timed_initial_literals.erase(pit);
                    return;
                } else {
                    pit++;
                }
            }
        }

        switch (msg.knowledge_type) {

        case KI::INSTANCE:
            {
                for (auto iit = model_instances[msg.instance_type].begin();
                     iit != model_instances[msg.instance_type].end();) {
                    std::string name = *iit;
                    if (name.compare(msg.instance_name) == 0 || msg.instance_name.compare("") == 0) {
                        RCLCPP_INFO(get_logger(), "KCL: (%s) Removing instance (%s, %s)",
                                    get_name(), msg.instance_type.c_str(),
                                    (msg.instance_name.compare("") == 0) ? "ALL" : msg.instance_name.c_str());
                        iit = model_instances[msg.instance_type].erase(iit);
                        if (iit != model_instances[msg.instance_type].begin()) iit--;

                        for (auto pit = model_facts.begin(); pit != model_facts.end();) {
                            if (KnowledgeComparitor::containsInstance(*pit, name)) {
                                RCLCPP_INFO(get_logger(), "KCL: (%s) Removing domain attribute (%s)",
                                            get_name(), pit->attribute_name.c_str());
                                pit = model_facts.erase(pit);
                            } else {
                                pit++;
                            }
                        }
                        for (auto pit = model_functions.begin(); pit != model_functions.end();) {
                            if (KnowledgeComparitor::containsInstance(*pit, name)) {
                                RCLCPP_INFO(get_logger(), "KCL: (%s) Removing domain attribute (%s)",
                                            get_name(), pit->attribute_name.c_str());
                                pit = model_functions.erase(pit);
                            } else {
                                pit++;
                            }
                        }
                    } else {
                        iit++;
                    }
                }
            }
            break;

        case KI::FUNCTION:
            for (auto pit = model_functions.begin(); pit != model_functions.end();) {
                if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                    RCLCPP_INFO(get_logger(), "KCL: (%s) Removing domain attribute (%s)",
                                get_name(), msg.attribute_name.c_str());
                    pit = model_functions.erase(pit);
                } else {
                    pit++;
                }
            }
            break;

        case KI::FACT:
            removeFact(msg);
            break;
        }
    }

    void KnowledgeBase::removeFact(const KI &msg) {
        for (auto pit = model_facts.begin(); pit != model_facts.end();) {
            if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                RCLCPP_INFO(get_logger(), "KCL: (%s) Removing Fact (%s,%i)",
                            get_name(), msg.attribute_name.c_str(), msg.is_negative);
                pit = model_facts.erase(pit);
            } else {
                pit++;
            }
        }
    }

    void KnowledgeBase::removeMissionGoal(KI &msg) {
        for (auto git = model_goals.begin(); git != model_goals.end();) {
            if (KnowledgeComparitor::containsKnowledge(msg, *git)) {
                RCLCPP_INFO(get_logger(), "KCL: (%s) Removing goal (%s)",
                            get_name(), git->attribute_name.c_str());
                git = model_goals.erase(git);
            } else {
                git++;
            }
        }
    }

    void KnowledgeBase::removeMissionMetric(KI &/*msg*/) {
        KI empty;
        RCLCPP_INFO(get_logger(), "KCL: (%s) Removing metric", get_name());
        model_metric = empty;
    }

    /*--------------*/
    /* adding items */
    /*--------------*/

    void KnowledgeBase::addKnowledge(KI &msg) {

        switch (msg.knowledge_type) {

        case KI::INSTANCE:
            {
                auto iit = find(domain_constants[msg.instance_type].begin(),
                                domain_constants[msg.instance_type].end(),
                                msg.instance_name);
                if (iit == domain_constants[msg.instance_type].end()) {
                    iit = find(model_instances[msg.instance_type].begin(),
                               model_instances[msg.instance_type].end(),
                               msg.instance_name);
                    if (iit == model_instances[msg.instance_type].end()) {
                        RCLCPP_INFO(get_logger(), "KCL: (%s) Adding instance (%s, %s)",
                                    get_name(), msg.instance_type.c_str(), msg.instance_name.c_str());
                        model_instances[msg.instance_type].push_back(msg.instance_name);
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "KCL: (%s) instance (%s, %s) already exists as a constant.",
                                get_name(), msg.instance_type.c_str(), msg.instance_name.c_str());
                }
            }
            break;

        case KI::FACT:
            {
                std::string param_str;
                for (size_t i = 0; i < msg.values.size(); ++i) param_str += " " + msg.values[i].value;

                for (auto pit = model_facts.begin(); pit != model_facts.end(); pit++) {
                    if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                        RCLCPP_WARN(get_logger(), "KCL: (%s) fact (%s%s) already exists",
                                    get_name(), msg.attribute_name.c_str(), param_str.c_str());
                        return;
                    }
                    msg.is_negative = 1 - msg.is_negative;
                    if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                        RCLCPP_INFO(get_logger(), "KCL: (%s) Setting fact (%s%s) is_negative=%i",
                                    get_name(), msg.attribute_name.c_str(), param_str.c_str(), (1 - msg.is_negative));
                        pit->is_negative = 1 - pit->is_negative;
                        return;
                    }
                    msg.is_negative = 1 - msg.is_negative;
                }

                RCLCPP_INFO(get_logger(), "KCL: (%s) Adding fact (%s%s, %i)",
                            get_name(), msg.attribute_name.c_str(), param_str.c_str(), msg.is_negative);
                model_facts.push_back(msg);
            }
            break;

        case KI::FUNCTION:
            {
                std::string param_str;
                for (size_t i = 0; i < msg.values.size(); ++i) param_str += " " + msg.values[i].value;

                bool function_exists = false;
                for (auto pit = model_functions.begin(); pit != model_functions.end(); pit++) {
                    if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                        switch (msg.assign_op) {
                        case KI::AP_ASSIGN:
                            RCLCPP_INFO(get_logger(), "KCL: (%s) Updating function (= (%s%s) %f)",
                                        get_name(), msg.attribute_name.c_str(), param_str.c_str(), msg.function_value);
                            pit->function_value = msg.function_value;
                            break;
                        case KI::AP_SCALE_UP:
                        case KI::AP_INCREASE:
                            pit->function_value += msg.function_value;
                            RCLCPP_INFO(get_logger(), "KCL: (%s) Increasing function (= (%s%s) %f)",
                                        get_name(), msg.attribute_name.c_str(), param_str.c_str(), pit->function_value);
                            break;
                        case KI::AP_SCALE_DOWN:
                        case KI::AP_DECREASE:
                            pit->function_value -= msg.function_value;
                            RCLCPP_INFO(get_logger(), "KCL: (%s) Decreasing function (= (%s%s) %f)",
                                        get_name(), msg.attribute_name.c_str(), param_str.c_str(), pit->function_value);
                            break;
                        case KI::AP_ASSIGN_CTS:
                            RCLCPP_WARN(get_logger(), "KCL: (%s) Continuous numeric effects not implemented.", get_name());
                            break;
                        }
                        function_exists = true;
                        break;
                    }
                }
                if (!function_exists && msg.assign_op == KI::AP_ASSIGN) {
                    RCLCPP_INFO(get_logger(), "KCL: (%s) Adding function (= (%s%s) %f)",
                                get_name(), msg.attribute_name.c_str(), param_str.c_str(), msg.function_value);
                    model_functions.push_back(msg);
                } else if (!function_exists) {
                    RCLCPP_INFO(get_logger(), "KCL: (%s) Ignoring function update (%s%s) function does not exist",
                                get_name(), msg.attribute_name.c_str(), param_str.c_str());
                }
            }
            break;
        }
    }

    void KnowledgeBase::addMissionGoal(KI &msg) {
        std::string param_str;
        for (size_t i = 0; i < msg.values.size(); ++i) param_str += " " + msg.values[i].value;

        for (auto pit = model_goals.begin(); pit != model_goals.end(); pit++) {
            if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                RCLCPP_WARN(get_logger(), "KCL: (%s) Goal (%s%s) already posted",
                            get_name(), msg.attribute_name.c_str(), param_str.c_str());
                return;
            }
        }
        RCLCPP_INFO(get_logger(), "KCL: (%s) Adding mission goal (%s%s)",
                    get_name(), msg.attribute_name.c_str(), param_str.c_str());
        model_goals.push_back(msg);
    }

    void KnowledgeBase::addMissionMetric(KI &msg) {
        RCLCPP_INFO(get_logger(), "KCL: (%s) Adding mission metric", get_name());
        model_metric = msg;
    }

    /*----------------*/
    /* fetching items */
    /*----------------*/

    void KnowledgeBase::getSubtypes(std::string &type, std::vector<std::string> &subtypes) {
        auto treq = std::make_shared<kmsrv::GetDomainTypeService::Request>();
        auto tres = std::make_shared<kmsrv::GetDomainTypeService::Response>();
        getTypes(treq, tres);

        std::map<std::string, std::string> typemap;
        for (size_t i = 0; i < tres->types.size(); i++)
            typemap[tres->types[i]] = tres->super_types[i];

        bool finished = false;
        while (!finished) {
            finished = true;
            for (size_t i = 0; i < tres->types.size(); i++) {
                if (tres->super_types[i] == type) subtypes.push_back(tres->types[i]);
                typemap[tres->types[i]] = typemap[tres->super_types[i]];
                if ("" != typemap[tres->types[i]]) finished = false;
            }
        }
        subtypes.push_back(type);
    }

    void KnowledgeBase::getInstances(
        const std::shared_ptr<kmsrv::GetInstanceService::Request> req,
        std::shared_ptr<kmsrv::GetInstanceService::Response> res)
    {
        if ("" == req->type_name) {
            for (auto iit = model_instances.begin(); iit != model_instances.end(); iit++) {
                for (size_t j = 0; j < iit->second.size(); j++)
                    res->instances.push_back(iit->second[j]);
            }
            if (req->include_constants) {
                for (auto iit = domain_constants.begin(); iit != domain_constants.end(); iit++) {
                    for (size_t j = 0; j < iit->second.size(); j++)
                        res->instances.push_back(iit->second[j]);
                }
            }
        } else {
            std::vector<std::string> alltypes;
            if (req->include_subtypes) {
                getSubtypes(req->type_name, alltypes);
            } else {
                alltypes.push_back(req->type_name);
            }

            for (auto tit = alltypes.begin(); tit != alltypes.end(); tit++) {
                auto iit = model_instances.find(*tit);
                if (iit != model_instances.end()) {
                    for (size_t j = 0; j < iit->second.size(); j++)
                        res->instances.push_back(iit->second[j]);
                }
                if (req->include_constants) {
                    iit = domain_constants.find(*tit);
                    if (iit != domain_constants.end()) {
                        for (size_t j = 0; j < iit->second.size(); j++)
                            res->instances.push_back(iit->second[j]);
                    }
                }
            }
        }
    }

    void KnowledgeBase::getPropositions(
        const std::shared_ptr<kmsrv::GetAttributeService::Request> req,
        std::shared_ptr<kmsrv::GetAttributeService::Response> res)
    {
        for (size_t i = 0; i < model_facts.size(); i++) {
            if (0 == req->predicate_name.compare(model_facts[i].attribute_name) ||
                "" == req->predicate_name) {
                res->attributes.push_back(model_facts[i]);
            }
        }
    }

    void KnowledgeBase::getFunctions(
        const std::shared_ptr<kmsrv::GetAttributeService::Request> req,
        std::shared_ptr<kmsrv::GetAttributeService::Response> res)
    {
        for (size_t i = 0; i < model_functions.size(); i++) {
            if (0 == req->predicate_name.compare(model_functions[i].attribute_name) ||
                "" == req->predicate_name) {
                res->attributes.push_back(model_functions[i]);
            }
        }
    }

    void KnowledgeBase::getTimedKnowledge(
        const std::shared_ptr<kmsrv::GetAttributeService::Request> req,
        std::shared_ptr<kmsrv::GetAttributeService::Response> res)
    {
        for (auto tit = model_timed_initial_literals.begin();
             tit != model_timed_initial_literals.end(); tit++) {
            if (0 == req->predicate_name.compare(tit->attribute_name) ||
                "" == req->predicate_name) {
                res->attributes.push_back(*tit);
            }
        }
    }

    void KnowledgeBase::getGoals(
        const std::shared_ptr<kmsrv::GetAttributeService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetAttributeService::Response> res)
    {
        for (size_t i = 0; i < model_goals.size(); i++)
            res->attributes.push_back(model_goals[i]);
    }

    void KnowledgeBase::getMetric(
        const std::shared_ptr<kmsrv::GetMetricService::Request> /*req*/,
        std::shared_ptr<kmsrv::GetMetricService::Response> res)
    {
        res->metric = model_metric;
    }

    void KnowledgeBase::setSensedPredicate(
        const std::shared_ptr<kmsrv::SetNamedBool::Request> req,
        std::shared_ptr<kmsrv::SetNamedBool::Response> res)
    {
        sensed_predicates[req->name] = req->value;
        res->success = true;
    }

    /*-----------------*/
    /* status update   */
    /*-----------------*/

    void KnowledgeBase::publishStatusUpdate() {
        rosplan_knowledge_msgs::msg::StatusUpdate update_msg;
        update_msg.last_update_time = now();
        status_pub_->publish(update_msg);
    }

} // namespace KCL_rosplan

/*-------------*/
/* main method */
/*-------------*/

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Use a temporary node to read the domain_path parameter before choosing the KB type.
    std::string domain_path, problem_path;
    bool use_unknowns = false;
    {
        auto tmp = std::make_shared<rclcpp::Node>("rosplan_knowledge_base",
            rclcpp::NodeOptions().allow_undeclared_parameters(true));
        tmp->declare_parameter("domain_path",  std::string("common/domain.pddl"));
        tmp->declare_parameter("problem_path", std::string(""));
        tmp->declare_parameter("use_unknowns", false);
        domain_path  = tmp->get_parameter("domain_path").as_string();
        problem_path = tmp->get_parameter("problem_path").as_string();
        use_unknowns = tmp->get_parameter("use_unknowns").as_bool();
    }

    std::string extension = (domain_path.size() > 5)
        ? domain_path.substr(domain_path.find_last_of('.'))
        : "";

    KCL_rosplan::KnowledgeBaseFactory::KB kb_type;
    if (extension == ".pddl") {
        kb_type = KCL_rosplan::KnowledgeBaseFactory::PDDL;
        RCLCPP_INFO(rclcpp::get_logger("rosplan_knowledge_base"),
                    "KCL: Starting a PDDL Knowledge Base");
    } else if (extension == ".ppddl") {
#ifdef ROSPLAN_BUILD_PPDDL
        kb_type = KCL_rosplan::KnowledgeBaseFactory::PPDDL;
        VAL1_2::parse_category::recoverWriteController();
        RCLCPP_INFO(rclcpp::get_logger("rosplan_knowledge_base"),
                    "KCL: Starting a PPDDL Knowledge Base");
#else
        RCLCPP_ERROR(rclcpp::get_logger("rosplan_knowledge_base"),
                     "KCL: PPDDL support not compiled in (rebuild with -DROSPLAN_BUILD_PPDDL=ON)");
        rclcpp::shutdown();
        return 1;
#endif
    } else if (extension == ".rddl") {
#ifdef ROSPLAN_BUILD_RDDL
        kb_type = KCL_rosplan::KnowledgeBaseFactory::RDDL;
        VAL1_2::parse_category::recoverWriteController();
        RCLCPP_INFO(rclcpp::get_logger("rosplan_knowledge_base"),
                    "KCL: Starting a RDDL Knowledge Base");
#else
        RCLCPP_ERROR(rclcpp::get_logger("rosplan_knowledge_base"),
                     "KCL: RDDL support not compiled in (rebuild with -DROSPLAN_BUILD_RDDL=ON)");
        rclcpp::shutdown();
        return 1;
#endif
    } else if (extension == ".hddl") {
        kb_type = KCL_rosplan::KnowledgeBaseFactory::HDDL;
        RCLCPP_INFO(rclcpp::get_logger("rosplan_knowledge_base"),
                    "KCL: Starting a HDDL Knowledge Base");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rosplan_knowledge_base"),
                     "KCL: Unexpected domain file extension '%s' (expected .pddl/.rddl/.hddl)",
                     extension.c_str());
        rclcpp::shutdown();
        return 1;
    }

    auto kb = KCL_rosplan::KnowledgeBaseFactory::createKB(kb_type);
    kb->parseDomain(domain_path, problem_path);
    kb->use_unknowns = use_unknowns;

    if (!rclcpp::ok()) {
        return 1;
    }

    RCLCPP_INFO(kb->get_logger(), "KCL: (%s) Ready to receive", kb->get_name());

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(kb);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
