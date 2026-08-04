#include "ros2plan_planning_system/ProblemGeneration/ProblemInterface.h"
#include "ros2plan_planning_system/ProblemGeneration/ProblemGeneratorFactory.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>
#include <algorithm>

namespace KCL_ros2plan {

    ProblemInterface::ProblemInterface()
        : rclcpp::Node("ros2plan_problem_interface")
    {
        // declare and get params
        declare_parameter("knowledge_base", std::string("ros2plan_knowledge_base"));
        declare_parameter("problem_path", std::string(""));
        declare_parameter("planning_language", std::string(""));
        declare_parameter("problem_topic", std::string("problem_instance"));

        knowledge_base   = get_parameter("knowledge_base").as_string();
        problem_path     = get_parameter("problem_path").as_string();
        std::string planning_lang = get_parameter("planning_language").as_string();
        std::string problem_topic = get_parameter("problem_topic").as_string();

        if (problem_path.empty() && planning_lang.empty()) {
            RCLCPP_ERROR(get_logger(),
                "KCL: (%s) Parameters problem_path and planning_language are both undefined. At least one must be defined.",
                get_name());
            rclcpp::shutdown();
            return;
        }

        // Determine problem generator type
        std::transform(planning_lang.begin(), planning_lang.end(), planning_lang.begin(), ::tolower);
        std::string pg_type_str = "pddl";

        if (!planning_lang.empty()) {
            if (planning_lang == "pddl" || planning_lang == "ppddl") pg_type_str = "pddl";
            else if (planning_lang == "rddl")  pg_type_str = "rddl";
            else if (planning_lang == "chimp") pg_type_str = "chimp";
            else {
                RCLCPP_WARN(get_logger(),
                    "KCL: (%s) Unexpected planning language %s. Defaulting to PDDL.",
                    get_name(), planning_lang.c_str());
            }
        }

        if (!problem_path.empty()) {
            std::string extension = (problem_path.size() > 5)
                                    ? problem_path.substr(problem_path.find_last_of('.'))
                                    : "";
            if (extension == ".pddl" || extension == ".ppddl") pg_type_str = "pddl";
            else if (extension == ".rddl") pg_type_str = "rddl";
            else if (extension == ".pdl")  pg_type_str = "chimp";
            else {
                RCLCPP_ERROR(get_logger(),
                    "KCL: (%s) Unexpected problem file extension %s.",
                    get_name(), extension.c_str());
                rclcpp::shutdown();
                return;
            }
        } else {
            RCLCPP_ERROR(get_logger(),
                "KCL: (%s) problem_path is not specified. Please provide a problem file path.",
                get_name());
            rclcpp::shutdown();
            return;
        }

        // Create callback groups
        srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create satellite node for deadlock-safe service calls (not added to any executor).
        // use_global_arguments(false) keeps the name below: launch_ros passes the node name
        // as a process-wide `-r __node:=` remap, which would otherwise rename this node too
        // and leave two nodes sharing one name in the graph.
        svc_node_ = std::make_shared<rclcpp::Node>(
            "problem_interface_svc_node",
            rclcpp::NodeOptions().use_global_arguments(false));

        // Create the problem generator
        problem_generator_ = ProblemGeneratorFactory::createProblemGenerator(pg_type_str, knowledge_base, svc_node_);

        // Publisher (latched)
        problem_publisher_ = create_publisher<std_msgs::msg::String>(
            problem_topic,
            rclcpp::QoS(1).transient_local());

        // Service: generate problem file (default)
        problem_generation_service_ = create_service<std_srvs::srv::Empty>(
            "problem_generation_server",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                   std::shared_ptr<std_srvs::srv::Empty::Response> res) {
                runProblemServerDefault(req, res);
            },
            rmw_qos_profile_services_default,
            srv_cbg_);

        // Service: generate problem file (params)
        problem_generation_params_service_ = create_service<ros2plan_dispatch_msgs::srv::ProblemService>(
            "problem_generation_server_params",
            [this](const std::shared_ptr<ros2plan_dispatch_msgs::srv::ProblemService::Request> req,
                   std::shared_ptr<ros2plan_dispatch_msgs::srv::ProblemService::Response> res) {
                runProblemServerParams(req, res);
            },
            rmw_qos_profile_services_default,
            srv_cbg_);

        RCLCPP_INFO(get_logger(), "KCL: (%s) Ready to receive.", get_name());
    }

    void ProblemInterface::runProblemServerDefault(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
    {
        // problem_path already set from params
        runProblemServer(problem_path);
    }

    void ProblemInterface::runProblemServerParams(
        const std::shared_ptr<ros2plan_dispatch_msgs::srv::ProblemService::Request> req,
        std::shared_ptr<ros2plan_dispatch_msgs::srv::ProblemService::Response> res)
    {
        RCLCPP_INFO(get_logger(), "KCL: (%s) Problem server params request received.", get_name());
        bool success = runProblemServer(req->problem_path);
        if (req->problem_string_response) {
            std::ifstream problemIn(req->problem_path.c_str());
            if (problemIn) {
                res->problem_string = std::string(
                    std::istreambuf_iterator<char>(problemIn),
                    std::istreambuf_iterator<char>());
            }
        }
        res->problem_generated = success;
    }

    bool ProblemInterface::runProblemServer(const std::string& problemPath) {
        // set problem name for logging
        std::string problem_name;
        std::size_t lastDivide = problemPath.find_last_of("/\\");
        if (lastDivide != std::string::npos)
            problem_name = problemPath.substr(lastDivide + 1);
        else
            problem_name = problemPath;

        RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Generating problem file.",
                    get_name(), problem_name.c_str());

        std::string mutablePath = problemPath;
        problem_generator_->generateProblemFile(mutablePath);

        RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) The problem was generated.",
                    get_name(), problem_name.c_str());

        // publish problem
        std::ifstream problemIn(problemPath.c_str());
        if (problemIn) {
            std_msgs::msg::String problemMsg;
            problemMsg.data = std::string(
                std::istreambuf_iterator<char>(problemIn),
                std::istreambuf_iterator<char>());
            problem_publisher_->publish(problemMsg);
        }

        return true;
    }

} // close namespace

int main(int argc, char **argv) {
    srand(static_cast<unsigned>(time(0)));
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_ros2plan::ProblemInterface>();
    if (!rclcpp::ok()) return 1;
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
