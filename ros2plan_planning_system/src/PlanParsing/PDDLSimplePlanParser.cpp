#include "ros2plan_planning_system/PlanParsing/PDDLSimplePlanParser.h"

namespace KCL_ros2plan {

    PDDLSimplePlanParser::PDDLSimplePlanParser()
        : PlanParser("ros2plan_plan_parser")
    {
        declare_parameter("knowledge_base", std::string("ros2plan_knowledge_base"));
        declare_parameter("plan_topic", std::string("complete_plan"));
        declare_parameter("planner_topic", std::string("planner_output"));

        std::string kb         = get_parameter("knowledge_base").as_string();
        std::string plan_topic = get_parameter("plan_topic").as_string();
        std::string planner_topic = get_parameter("planner_topic").as_string();

        std::string op_details_svc = "/" + kb + "/domain/operator_details";
        get_operator_details_client_ = create_client<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService>(op_details_svc);

        // Latched publisher
        plan_publisher_ = create_publisher<ros2plan_dispatch_msgs::msg::CompletePlan>(
            plan_topic,
            rclcpp::QoS(1).transient_local());

        // Callback groups
        srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Subscription to planner output — must match planner's transient_local publisher
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_cbg_;
        planner_output_sub_ = create_subscription<std_msgs::msg::String>(
            planner_topic, rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg) { plannerCallback(msg); },
            sub_opts);

        // Services
        parse_plan_srv_ = create_service<std_srvs::srv::Empty>(
            "parse_plan",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                   std::shared_ptr<std_srvs::srv::Empty::Response> res) {
                parsePlan(req, res);
            },
            rmw_qos_profile_services_default,
            srv_cbg_);

        parse_plan_from_file_srv_ = create_service<ros2plan_dispatch_msgs::srv::ParsingService>(
            "parse_plan_from_file",
            [this](const std::shared_ptr<ros2plan_dispatch_msgs::srv::ParsingService::Request> req,
                   std::shared_ptr<ros2plan_dispatch_msgs::srv::ParsingService::Response> res) {
                parsePlanFromFile(req, res);
            },
            rmw_qos_profile_services_default,
            srv_cbg_);

        RCLCPP_INFO(get_logger(), "KCL: (%s) Ready to receive.", get_name());
    }

    void PDDLSimplePlanParser::reset() {
        action_list.clear();
    }

    void PDDLSimplePlanParser::publishPlan() {
        ros2plan_dispatch_msgs::msg::CompletePlan msg;
        msg.plan = action_list;
        plan_publisher_->publish(msg);
    }

    void PDDLSimplePlanParser::preparePlan() {

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
            }

            curr = line.find("[", curr) + 1;
            next = line.find("]", curr);
            msg.duration = (double)atof(line.substr(curr, next - curr).c_str());

            action_list.push_back(msg);
        }
    }

    void PDDLSimplePlanParser::processPDDLParameters(
        ros2plan_dispatch_msgs::msg::ActionDispatch& msg,
        std::vector<std::string>& params)
    {
        if (!get_operator_details_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) KB operator details service unavailable.", get_name());
            return;
        }
        auto req = std::make_shared<ros2plan_knowledge_msgs::srv::GetDomainOperatorDetailsService::Request>();
        req->name = msg.name;
        auto future = get_operator_details_client_->async_send_request(req);
        auto status = future.wait_for(std::chrono::seconds(5));
        if (status != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "KCL: (%s) Could not call KB for operator details: %s",
                         get_name(), msg.name.c_str());
            return;
        }
        auto res = future.get();
        auto& opParams = res->op.formula.typed_parameters;
        for (size_t i = 0; i < opParams.size() && i < params.size(); i++) {
            diagnostic_msgs::msg::KeyValue pair;
            pair.key   = opParams[i].key;
            pair.value = params[i];
            msg.parameters.push_back(pair);
        }
    }

} // close namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_ros2plan::PDDLSimplePlanParser>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
