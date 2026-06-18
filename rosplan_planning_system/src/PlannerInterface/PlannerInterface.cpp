#include "rosplan_planning_system/PlannerInterface/PlannerInterface.h"
#include <thread>

namespace KCL_rosplan {

    PlannerInterface::PlannerInterface(const std::string& node_name)
        : rclcpp::Node(node_name),
          problem_instance_received(false),
          problem_instance_time(0.0),
          use_problem_topic(false)
    {
        srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // declare params
        declare_parameter("planner_topic", std::string("planner_output"));
        declare_parameter("problem_topic", std::string("problem_instance"));
        declare_parameter("use_problem_topic", false);
        declare_parameter("domain_path", std::string("common/domain.pddl"));
        declare_parameter("problem_path", std::string("common/problem.pddl"));
        declare_parameter("data_path", std::string("common/"));
        declare_parameter("planner_command", std::string("timeout 60 common/bin/popf -n DOMAIN PROBLEM"));

        // publishing raw planner output
        std::string plannerTopic = get_parameter("planner_topic").as_string();
        plan_publisher_ = create_publisher<std_msgs::msg::String>(
            plannerTopic, rclcpp::QoS(1).transient_local());

        // subscribe to problem instance
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_cbg_;
        std::string problemTopic = get_parameter("problem_topic").as_string();
        problem_sub_ = create_subscription<std_msgs::msg::String>(
            problemTopic, 1,
            [this](const std_msgs::msg::String::SharedPtr msg) { problemCallback(msg); },
            sub_opts);

        // planning services
        planning_server_default_ = create_service<std_srvs::srv::Empty>(
            "planning_server",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                   std::shared_ptr<std_srvs::srv::Empty::Response> res) {
                runPlanningServerDefault(req, res);
            },
            rmw_qos_profile_services_default,
            srv_cbg_);

        planning_server_params_ = create_service<rosplan_dispatch_msgs::srv::PlanningService>(
            "planning_server_params",
            [this](const std::shared_ptr<rosplan_dispatch_msgs::srv::PlanningService::Request> req,
                   std::shared_ptr<rosplan_dispatch_msgs::srv::PlanningService::Response> res) {
                runPlanningServerParams(req, res);
            },
            rmw_qos_profile_services_default,
            srv_cbg_);

        // action server
        plan_server_ = rclcpp_action::create_server<PlanAction>(
            this,
            "start_planning",
            [](const rclcpp_action::GoalUUID&, std::shared_ptr<const PlanAction::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](std::shared_ptr<GoalHandlePlan>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](std::shared_ptr<GoalHandlePlan> goal_handle) {
                std::thread([this, goal_handle]() { runPlanningServerAction(goal_handle); }).detach();
            });

        RCLCPP_INFO(get_logger(), "KCL: (%s) Ready to receive", get_name());
    }

    /*----------------------*/
    /* Problem subscription */
    /*----------------------*/

    void PlannerInterface::problemCallback(const std_msgs::msg::String::SharedPtr problemInstance) {
        RCLCPP_INFO(get_logger(), "KCL: (%s) Problem received.", get_name());
        problem_instance_received = true;
        problem_instance_time = rclcpp::Clock().now().seconds();
        problem_instance = problemInstance->data;
    }

    /*--------------------*/
    /* Planning interface */
    /*--------------------*/

    void PlannerInterface::runPlanningServerDefault(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
    {
        // defaults
        use_problem_topic = false;
        data_path = "common/";
        domain_path = "common/domain.pddl";
        problem_path = "common/problem.pddl";
        planner_command = "timeout 60 common/bin/popf -n DOMAIN PROBLEM";

        // load params
        use_problem_topic = get_parameter("use_problem_topic").as_bool();
        domain_path = get_parameter("domain_path").as_string();
        data_path = get_parameter("data_path").as_string();
        problem_path = get_parameter("problem_path").as_string();
        planner_command = get_parameter("planner_command").as_string();

        runPlanningServer(domain_path, problem_path, data_path, planner_command, use_problem_topic);
    }

    void PlannerInterface::runPlanningServerParams(
        const std::shared_ptr<rosplan_dispatch_msgs::srv::PlanningService::Request> req,
        std::shared_ptr<rosplan_dispatch_msgs::srv::PlanningService::Response> res)
    {
        res->plan_found = runPlanningServer(req->domain_path, req->problem_path,
                                             req->data_path, req->planner_command,
                                             req->use_problem_topic);
    }

    void PlannerInterface::runPlanningServerAction(std::shared_ptr<GoalHandlePlan> goal_handle) {
        auto goal = goal_handle->get_goal();
        if (runPlanningServer(goal->domain_path, goal->problem_path, goal->data_path,
                              goal->planner_command, goal->use_problem_topic)) {
            auto result = std::make_shared<PlanAction::Result>();
            goal_handle->succeed(result);
        } else {
            auto result = std::make_shared<PlanAction::Result>();
            goal_handle->abort(result);
        }
    }

    bool PlannerInterface::runPlanningServer(std::string domainPath, std::string problemPath,
                                             std::string dataPath, std::string plannerCommand,
                                             bool useProblemTopic)
    {
        // save parameters
        data_path = dataPath;
        domain_path = domainPath;
        problem_path = problemPath;
        planner_command = plannerCommand;
        use_problem_topic = useProblemTopic;

        // check if data_path ends in "/"
        if (!data_path.empty() && data_path.back() != '/') {
            data_path += '/';
        }

        // set problem name for logging
        std::size_t lastDivide = problem_path.find_last_of("/\\");
        if (lastDivide != std::string::npos) {
            problem_name = problem_path.substr(lastDivide + 1);
        } else {
            problem_name = problem_path;
        }

        if (use_problem_topic && !problem_instance_received) {
            RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Problem was not published yet.",
                        get_name(), problem_name.c_str());
            return false;
        }

        bool success = runPlanner();

        if (success) {
            std_msgs::msg::String planMsg;
            planMsg.data = planner_output;
            plan_publisher_->publish(planMsg);
        }

        return success;
    }

} // close namespace
