#include "ros2plan_planning_system/PlanParsing/PDDLEsterelPlanParser.h"

namespace KCL_ros2plan {

    PDDLEsterelPlanParser::PDDLEsterelPlanParser()
        : PlanParser("ros2plan_parsing_interface"),
          epsilon_time(0.1)
    {
        declare_parameter("knowledge_base", std::string("ros2plan_knowledge_base"));
        declare_parameter("plan_topic", std::string("complete_plan"));
        declare_parameter("planner_topic", std::string("planner_output"));
        declare_parameter("epsilon_time", 0.1);

        epsilon_time          = get_parameter("epsilon_time").as_double();
        std::string kb        = get_parameter("knowledge_base").as_string();
        std::string plan_topic = get_parameter("plan_topic").as_string();
        std::string planner_topic = get_parameter("planner_topic").as_string();

        // Create service clients
        get_predicate_client_        = create_client<ros2plan_knowledge_msgs::srv::GetDomainAttributeService>("/" + kb + "/domain/predicates");
        get_propositions_client_     = create_client<ros2plan_knowledge_msgs::srv::GetAttributeService>("/" + kb + "/state/propositions");
        get_tils_client_             = create_client<ros2plan_knowledge_msgs::srv::GetAttributeService>("/" + kb + "/state/timed_knowledge");
        get_functions_client_        = create_client<ros2plan_knowledge_msgs::srv::GetAttributeService>("/" + kb + "/state/functions");
        get_operator_details_client_ = create_client<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService>("/" + kb + "/domain/operator_details");

        // Latched publisher
        plan_publisher_ = create_publisher<ros2plan_dispatch_msgs::msg::EsterelPlan>(
            plan_topic,
            rclcpp::QoS(1).transient_local());

        // Callback groups
        srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Subscription
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_cbg_;
        planner_output_sub_ = create_subscription<std_msgs::msg::String>(
            planner_topic, 1,
            [this](const std_msgs::msg::String::SharedPtr msg) { plannerCallback(msg); },
            sub_opts);

        // Services
        parse_plan_srv_ = create_service<std_srvs::srv::Empty>(
            "parse_plan",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                   std::shared_ptr<std_srvs::srv::Empty::Response> res) {
                parsePlan(req, res);
            },
            rclcpp::ServicesQoS(),
            srv_cbg_);

        parse_plan_from_file_srv_ = create_service<ros2plan_dispatch_msgs::srv::ParsingService>(
            "parse_plan_from_file",
            [this](const std::shared_ptr<ros2plan_dispatch_msgs::srv::ParsingService::Request> req,
                   std::shared_ptr<ros2plan_dispatch_msgs::srv::ParsingService::Response> res) {
                parsePlanFromFile(req, res);
            },
            rclcpp::ServicesQoS(),
            srv_cbg_);

        RCLCPP_INFO(get_logger(), "KCL: (%s) Ready to receive.", get_name());
    }

    void PDDLEsterelPlanParser::reset() {
        action_list.clear();
        til_list.clear();
        last_plan.nodes.clear();
        last_plan.edges.clear();
    }

    void PDDLEsterelPlanParser::publishPlan() {
        plan_publisher_->publish(last_plan);
    }

    /*----------------------*/
    /* Post processing plan */
    /*----------------------*/

    void PDDLEsterelPlanParser::preparePlan() {

        ros2plan_dispatch_msgs::msg::EsterelPlanNode plan_start;
        plan_start.node_type = ros2plan_dispatch_msgs::msg::EsterelPlanNode::PLAN_START;
        plan_start.node_id = last_plan.nodes.size();
        plan_start.name = "plan_start";
        last_plan.nodes.push_back(plan_start);

        int curr, next;
        std::string line;
        std::istringstream planfile(planner_output);

        size_t planFreeActionID = 0;

        while (std::getline(planfile, line)) {

            if (line.length() < 2)
                break;

            if (line.find("[", 0) == std::string::npos
                    || line.find("]", 0) == std::string::npos
                    || line.find("(", 0) == std::string::npos
                    || line.find(")", 0) == std::string::npos
                    || line.find(":", 0) == std::string::npos)
                continue;

            ros2plan_dispatch_msgs::msg::ActionDispatch msg;

            msg.action_id = planFreeActionID;
            planFreeActionID++;

            curr = line.find(":");
            double dispatchTime = (double)atof(line.substr(0, curr).c_str());
            msg.dispatch_time = dispatchTime;

            curr = line.find("(") + 1;
            bool paramsExist = (line.find(" ", curr) < line.find(")", curr));

            if (paramsExist) {
                next = line.find(" ", curr);
                std::string name = line.substr(curr, next - curr);
                msg.name = name;

                std::vector<std::string> params;
                curr = next + 1;
                next = line.find(")", curr);
                int at = curr;
                while (at < next) {
                    int cc  = line.find(" ", curr);
                    int cc1 = line.find(")", curr);
                    curr = cc < cc1 ? cc : cc1;
                    std::string param = line.substr(at, curr - at);
                    params.push_back(param);
                    ++curr;
                    at = curr;
                }
                processPDDLParameters(msg, params);
            } else {
                next = line.find(")", curr);
                std::string name = line.substr(curr, next - curr);
                msg.name = name;

                auto req = std::make_shared<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Request>();
                req->name = msg.name;
                std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Response> res;
                if (callServiceSync(get_operator_details_client_, req, res)) {
                    action_details[msg.action_id] = res->op;
                } else {
                    RCLCPP_ERROR(get_logger(), "KCL: (%s) could not call KB for operator details: %s",
                                 get_name(), msg.name.c_str());
                }
            }

            curr = line.find("[", curr) + 1;
            next = line.find("]", curr);
            msg.duration = (double)atof(line.substr(curr, next - curr).c_str());

            action_list.push_back(msg);

            ros2plan_dispatch_msgs::msg::EsterelPlanNode node_start;
            node_start.node_type = ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START;
            node_start.node_id = last_plan.nodes.size();
            node_start.action = msg;
            std::stringstream ss;
            ss << msg.name << "_start";
            node_start.name = ss.str();
            last_plan.nodes.push_back(node_start);

            ros2plan_dispatch_msgs::msg::EsterelPlanNode node_end;
            node_end.node_type = ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END;
            node_end.node_id = last_plan.nodes.size();
            node_end.action = msg;
            ss.str("");
            ss << msg.name << "_end";
            node_end.name = ss.str();
            last_plan.nodes.push_back(node_end);
        }

        fetchTILs();
        createGraph();
    }

    void PDDLEsterelPlanParser::processPDDLParameters(
        ros2plan_dispatch_msgs::msg::ActionDispatch& msg,
        std::vector<std::string>& params)
    {
        auto req = std::make_shared<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Request>();
        req->name = msg.name;
        std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Response> res;
        if (!callServiceSync(get_operator_details_client_, req, res)) {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) could not call KB for operator details: %s",
                         get_name(), msg.name.c_str());
            return;
        }

        action_details[msg.action_id] = res->op;

        auto& opParams = res->op.formula.typed_parameters;
        for (size_t i = 0; i < opParams.size() && i < params.size(); i++) {
            diagnostic_msgs::msg::KeyValue pair;
            pair.key   = opParams[i].key;
            pair.value = params[i];
            msg.parameters.push_back(pair);
        }

        groundFormula(action_details[msg.action_id].at_start_add_effects,    opParams, params);
        groundFormula(action_details[msg.action_id].at_start_del_effects,    opParams, params);
        groundFormula(action_details[msg.action_id].at_end_add_effects,      opParams, params);
        groundFormula(action_details[msg.action_id].at_end_del_effects,      opParams, params);
        groundFormula(action_details[msg.action_id].at_start_simple_condition, opParams, params);
        groundFormula(action_details[msg.action_id].at_end_simple_condition,   opParams, params);
        groundFormula(action_details[msg.action_id].over_all_simple_condition, opParams, params);
        groundFormula(action_details[msg.action_id].at_start_neg_condition,    opParams, params);
        groundFormula(action_details[msg.action_id].at_end_neg_condition,      opParams, params);
        groundFormula(action_details[msg.action_id].over_all_neg_condition,    opParams, params);
    }

    void PDDLEsterelPlanParser::fetchTILs() {

        auto predReq = std::make_shared<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Request>();
        std::shared_ptr<ros2plan_knowledge_msgs::srv::GetDomainAttributeService::Response> predRes;
        if (!callServiceSync(get_predicate_client_, predReq, predRes)) {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) could not call KB for predicate list", get_name());
            return;
        }

        rclcpp::Time time = rclcpp::Clock().now();

        for (auto& df : predRes->items) {
            auto tilReq = std::make_shared<ros2plan_knowledge_msgs::srv::GetAttributeService::Request>();
            tilReq->predicate_name = df.name;
            std::shared_ptr<ros2plan_knowledge_msgs::srv::GetAttributeService::Response> tilRes;
            if (!callServiceSync(get_tils_client_, tilReq, tilRes)) {
                RCLCPP_ERROR(get_logger(), "KCL: (%s) could not call KB for TILs (%s)",
                             get_name(), df.name.c_str());
                continue;
            }

            for (auto& attr : tilRes->attributes) {
                rclcpp::Time attr_time(attr.initial_time.sec, attr.initial_time.nanosec);
                if (time > attr_time) continue;
                double key = (attr_time - time).seconds();
                til_list.insert(std::make_pair(key, attr));
            }
        }
    }

    void PDDLEsterelPlanParser::createGraph() {

        std::multimap<double, int> nodes;

        for (auto& node : last_plan.nodes) {
            double t = 0;
            switch (node.node_type) {
            case ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START:
                t = node.action.dispatch_time;
                break;
            case ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END:
                t = node.action.dispatch_time + node.action.duration;
                break;
            default:
                t = 0;
                break;
            }
            nodes.insert(std::make_pair(t, node.node_id));
        }

        for (auto nit = nodes.begin(); nit != nodes.end(); ++nit) {

            ros2plan_dispatch_msgs::msg::EsterelPlanNode* node = &last_plan.nodes[nit->second];

            switch (node->node_type) {
            case ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END:
                makeEdge(node->node_id - 1, node->node_id,
                         node->action.duration, node->action.duration,
                         ros2plan_dispatch_msgs::msg::EsterelPlanEdge::START_END_ACTION_EDGE);
                // FALLTHROUGH intentional
            case ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START: {

                ros2plan_knowledge_msgs::msg::DomainOperator& op = action_details[node->action.action_id];

                bool edge_created = false;

                if (node->node_type == ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START) {
                    for (auto& c : op.at_start_simple_condition)
                        if (addConditionEdge(nodes, nit, c, false, false)) edge_created = true;
                    for (auto& c : op.over_all_simple_condition)
                        if (addConditionEdge(nodes, nit, c, false, true)) edge_created = true;
                    for (auto& c : op.at_start_neg_condition)
                        if (addConditionEdge(nodes, nit, c, true, false)) edge_created = true;
                    for (auto& c : op.over_all_neg_condition)
                        if (addConditionEdge(nodes, nit, c, true, true)) edge_created = true;
                } else {
                    for (auto& c : op.at_end_simple_condition)
                        if (addConditionEdge(nodes, nit, c, false, false)) edge_created = true;
                    for (auto& c : op.at_end_neg_condition)
                        if (addConditionEdge(nodes, nit, c, true, false)) edge_created = true;
                }

                if (addInterferenceEdges(nodes, nit)) edge_created = true;

                if (!edge_created) {
                    makeEdge(0, node->node_id, ros2plan_dispatch_msgs::msg::EsterelPlanEdge::CONDITION_EDGE);
                }
                break;
            }
            default:
                break;
            }
        }
    }

    bool PDDLEsterelPlanParser::addConditionEdge(
        std::multimap<double, int>& node_map,
        std::multimap<double, int>::iterator& current_node,
        ros2plan_knowledge_msgs::msg::DomainFormula& condition,
        bool negative_condition,
        bool overall_condition)
    {
        std::multimap<double, ros2plan_knowledge_msgs::msg::KnowledgeItem>::const_reverse_iterator tit;
        tit = til_list.rbegin();

        while (tit != til_list.rend() && tit->first > current_node->first) {
            if (satisfiesPrecondition(condition, tit->second, !negative_condition)) {
                if (overall_condition) {
                    makeEdge(0, current_node->second + 1, 0, tit->first - epsilon_time,
                             ros2plan_dispatch_msgs::msg::EsterelPlanEdge::CONDITION_EDGE);
                } else {
                    makeEdge(0, current_node->second, 0, tit->first - epsilon_time,
                             ros2plan_dispatch_msgs::msg::EsterelPlanEdge::CONDITION_EDGE);
                }
            }
            ++tit;
        }

        std::multimap<double, int>::const_reverse_iterator rit(current_node);
        for (; rit != node_map.rend(); ++rit) {
            while (tit != til_list.rend() && tit->first > rit->first) {
                if (satisfiesPrecondition(condition, tit->second, negative_condition)) {
                    makeEdge(0, current_node->second, tit->first, std::numeric_limits<double>::max(),
                             ros2plan_dispatch_msgs::msg::EsterelPlanEdge::CONDITION_EDGE);
                    return true;
                }
                ++tit;
            }
            if (satisfiesPrecondition(condition, last_plan.nodes[rit->second], negative_condition)) {
                makeEdge(rit->second, current_node->second,
                         ros2plan_dispatch_msgs::msg::EsterelPlanEdge::CONDITION_EDGE);
                return true;
            }
        }

        return false;
    }

    bool PDDLEsterelPlanParser::addInterferenceEdges(
        std::multimap<double, int>& node_map,
        std::multimap<double, int>::iterator& current_node)
    {
        bool edge_added = false;

        ros2plan_dispatch_msgs::msg::EsterelPlanNode* node = &last_plan.nodes[current_node->second];

        std::multimap<double, int>::const_reverse_iterator rit(current_node);
        for (; rit != node_map.rend(); ++rit) {

            ros2plan_dispatch_msgs::msg::EsterelPlanNode* prenode = &last_plan.nodes[rit->second];

            bool interferes = false;

            ros2plan_knowledge_msgs::msg::DomainOperator& op = action_details[prenode->action.action_id];

            if (prenode->node_type == ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START) {
                for (auto& c : op.at_start_simple_condition)
                    interferes = interferes || satisfiesPrecondition(c, *node, true);
                for (auto& c : op.at_start_neg_condition)
                    interferes = interferes || satisfiesPrecondition(c, *node, false);
                for (auto& c : op.at_start_add_effects)
                    interferes = interferes || satisfiesPrecondition(c, *node, true);
                for (auto& c : op.at_start_del_effects)
                    interferes = interferes || satisfiesPrecondition(c, *node, false);
            } else if (prenode->node_type == ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END) {
                for (auto& c : op.at_end_simple_condition)
                    interferes = interferes || satisfiesPrecondition(c, *node, true);
                for (auto& c : op.at_end_neg_condition)
                    interferes = interferes || satisfiesPrecondition(c, *node, false);
                for (auto& c : op.over_all_simple_condition)
                    interferes = interferes || satisfiesPrecondition(c, *node, true);
                for (auto& c : op.over_all_neg_condition)
                    interferes = interferes || satisfiesPrecondition(c, *node, false);
                for (auto& c : op.at_end_add_effects)
                    interferes = interferes || satisfiesPrecondition(c, *node, true);
                for (auto& c : op.at_end_del_effects)
                    interferes = interferes || satisfiesPrecondition(c, *node, false);
            }

            if (interferes) {
                makeEdge(prenode->node_id, node->node_id,
                         ros2plan_dispatch_msgs::msg::EsterelPlanEdge::INTERFERENCE_EDGE);
                edge_added = true;
            }
        }

        return edge_added;
    }

    void PDDLEsterelPlanParser::makeEdge(int source_node_id, int sink_node_id,
                                          double lower_bound, double upper_bound, int edge_type)
    {
        if (lower_bound == 0) lower_bound = 0.001;
        if (upper_bound == 0) upper_bound = 0.001;

        for (int edgeID : last_plan.nodes[source_node_id].edges_out) {
            if (edgeID < 0 || edgeID >= static_cast<int>(last_plan.edges.size())) continue;
            auto& edge = last_plan.edges[edgeID];
            auto nit = std::find(edge.sink_ids.begin(), edge.sink_ids.end(), sink_node_id);
            if (nit != edge.sink_ids.end()) {
                if (lower_bound > edge.duration_lower_bound) edge.duration_lower_bound = lower_bound;
                if (upper_bound < edge.duration_upper_bound) edge.duration_upper_bound = upper_bound;
                return;
            }
        }

        ros2plan_dispatch_msgs::msg::EsterelPlanEdge newEdge;
        newEdge.edge_type = edge_type;
        newEdge.edge_id = last_plan.edges.size();
        std::stringstream ss;
        ss << "edge_" << newEdge.edge_id;
        newEdge.edge_name = ss.str();
        newEdge.signal_type = 0;
        newEdge.source_ids.push_back(source_node_id);
        newEdge.sink_ids.push_back(sink_node_id);
        newEdge.duration_lower_bound = lower_bound;
        newEdge.duration_upper_bound = upper_bound;

        last_plan.edges.push_back(newEdge);
        last_plan.nodes[source_node_id].edges_out.push_back(newEdge.edge_id);
        last_plan.nodes[sink_node_id].edges_in.push_back(newEdge.edge_id);
    }

} // close namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_ros2plan::PDDLEsterelPlanParser>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
