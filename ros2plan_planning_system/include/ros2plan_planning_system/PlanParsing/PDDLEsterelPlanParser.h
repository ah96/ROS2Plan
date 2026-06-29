#include "PlanParser.h"

#include "ros2plan_dispatch_msgs/msg/action_dispatch.hpp"
#include "ros2plan_dispatch_msgs/msg/esterel_plan.hpp"
#include "ros2plan_dispatch_msgs/msg/esterel_plan_node.hpp"
#include "ros2plan_dispatch_msgs/msg/esterel_plan_edge.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_operator_details_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/msg/domain_operator.hpp"
#include "ros2plan_knowledge_msgs/msg/domain_formula.hpp"
#include "ros2plan_knowledge_msgs/msg/knowledge_item.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include <stdlib.h>
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <algorithm>
#include <limits>

#ifndef KCL_pddl_esterel_plan_parser
#define KCL_pddl_esterel_plan_parser

namespace KCL_ros2plan {

    class PDDLEsterelPlanParser : public PlanParser
    {
    private:

        bool satisfiesPrecondition(ros2plan_knowledge_msgs::msg::DomainFormula& condition,
                                   const ros2plan_dispatch_msgs::msg::EsterelPlanNode& node,
                                   bool negative_condition)
        {
            if (!negative_condition) {
                if (node.node_type == ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START) {
                    for (auto cit = action_details[node.action.action_id].at_start_add_effects.begin();
                         cit != action_details[node.action.action_id].at_start_add_effects.end(); ++cit)
                        if (domainFormulaMatches(condition, *cit)) return true;
                } else if (node.node_type == ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END) {
                    for (auto cit = action_details[node.action.action_id].at_end_add_effects.begin();
                         cit != action_details[node.action.action_id].at_end_add_effects.end(); ++cit)
                        if (domainFormulaMatches(condition, *cit)) return true;
                }
            } else {
                if (node.node_type == ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_START) {
                    for (auto cit = action_details[node.action.action_id].at_start_del_effects.begin();
                         cit != action_details[node.action.action_id].at_start_del_effects.end(); ++cit)
                        if (domainFormulaMatches(condition, *cit)) return true;
                } else if (node.node_type == ros2plan_dispatch_msgs::msg::EsterelPlanNode::ACTION_END) {
                    for (auto cit = action_details[node.action.action_id].at_end_del_effects.begin();
                         cit != action_details[node.action.action_id].at_end_del_effects.end(); ++cit)
                        if (domainFormulaMatches(condition, *cit)) return true;
                }
            }
            return false;
        }

        bool satisfiesPrecondition(ros2plan_knowledge_msgs::msg::DomainFormula& condition,
                                   const ros2plan_knowledge_msgs::msg::KnowledgeItem& effect,
                                   bool negative_condition)
        {
            ros2plan_knowledge_msgs::msg::DomainFormula eff;
            eff.name = effect.attribute_name;
            for (const auto& v : effect.values) {
                diagnostic_msgs::msg::KeyValue pair;
                pair.key = v.key;
                pair.value = v.value;
                eff.typed_parameters.push_back(pair);
            }
            return (negative_condition == effect.is_negative) && domainFormulaMatches(condition, eff);
        }

        void makeEdge(int source_node_id, int sink_node_id, int edge_type) {
            makeEdge(source_node_id, sink_node_id, 0, std::numeric_limits<double>::max(), edge_type);
        }

        void groundFormula(std::vector<ros2plan_knowledge_msgs::msg::DomainFormula>& formula,
                           std::vector<diagnostic_msgs::msg::KeyValue>& opParams,
                           std::vector<std::string>& params)
        {
            for (auto& df : formula) {
                for (auto& tp : df.typed_parameters) {
                    bool found = false;
                    for (size_t i = 0; i < opParams.size(); ++i) {
                        if (opParams[i].key == tp.key) {
                            tp.value = params[i];
                            found = true;
                            break;
                        }
                    }
                    if (!found) tp.value = tp.key;
                }
            }
        }

        bool domainFormulaMatches(ros2plan_knowledge_msgs::msg::DomainFormula& a,
                                  ros2plan_knowledge_msgs::msg::DomainFormula& b)
        {
            if (a.name != b.name) return false;
            if (a.typed_parameters.size() != b.typed_parameters.size()) return false;
            for (size_t i = 0; i < a.typed_parameters.size(); ++i)
                if (a.typed_parameters[i].value != b.typed_parameters[i].value) return false;
            return true;
        }

        ros2plan_dispatch_msgs::msg::EsterelPlan last_plan;
        std::map<int, ros2plan_knowledge_msgs::msg::DomainOperator> action_details;
        double epsilon_time;

        /* clients */
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainAttributeService>::SharedPtr get_predicate_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_propositions_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_tils_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetAttributeService>::SharedPtr get_functions_client_;
        rclcpp::Client<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService>::SharedPtr get_operator_details_client_;

        /* TILs */
        std::multimap<double, ros2plan_knowledge_msgs::msg::KnowledgeItem> til_list;

    protected:

        void processPDDLParameters(ros2plan_dispatch_msgs::msg::ActionDispatch &msg, std::vector<std::string> &params);
        void fetchTILs();
        void createGraph();
        bool addInterferenceEdges(std::multimap<double,int> &node_map, std::multimap<double,int>::iterator &current_node);
        void makeEdge(int source_node_id, int sink_node_id, double lower_bound, double upper_bound, int edge_type);
        bool addConditionEdge(
            std::multimap<double,int> &node_map, std::multimap<double,int>::iterator &current_node,
            ros2plan_knowledge_msgs::msg::DomainFormula &condition, bool negative_condition, bool overall_condition);

        void reset();
        void preparePlan();
        void publishPlan();

    public:

        PDDLEsterelPlanParser();
        ~PDDLEsterelPlanParser() = default;

        rclcpp::Publisher<ros2plan_dispatch_msgs::msg::EsterelPlan>::SharedPtr plan_publisher_;
    };

} // close namespace

#endif
