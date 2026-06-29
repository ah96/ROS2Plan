//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#include "ros2plan_planning_system/PlannerInterface/OnlinePlannerInterface.h"

namespace KCL_ros2plan {

    OnlinePlannerInterface::OnlinePlannerInterface()
        : PlannerInterface("ros2plan_planner_interface"),
          planner_running_(false)
    {
        get_planner_params_ = create_service<ros2plan_dispatch_msgs::srv::GetPlanningParams>(
            "get_planning_params",
            [this](const std::shared_ptr<ros2plan_dispatch_msgs::srv::GetPlanningParams::Request> req,
                   std::shared_ptr<ros2plan_dispatch_msgs::srv::GetPlanningParams::Response> res) {
                getPlannerParams(req, res);
            },
            rmw_qos_profile_services_default,
            srv_cbg_);
    }

    OnlinePlannerInterface::~OnlinePlannerInterface() {
        if (planner_.joinable()) planner_.join();
    }

    std::string OnlinePlannerInterface::runCommand(std::string cmd) {
        planner_running_ = true;
        std::string data;
        FILE *stream;
        char buffer[1000];
        stream = popen(cmd.c_str(), "r");
        while (fgets(buffer, 1000, stream) != NULL)
            data.append(buffer);
        pclose(stream);
        planner_running_ = false;
        return data;
    }

    bool OnlinePlannerInterface::runPlanner() {
        if (planner_running_) {
            RCLCPP_INFO(get_logger(), "KCL: (%s) A plan is already being computed.", get_name());
            return false;
        } else if (planner_.joinable()) {
            planner_.join();
        }

        if (use_problem_topic && problem_instance_received) {
            RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Writing problem to file.", get_name(), problem_name.c_str());
            std::ofstream dest;
            dest.open(problem_path.c_str());
            dest << problem_instance;
            dest.close();
        }

        std::string str = planner_command;
        std::size_t dit = str.find("DOMAIN");
        if (dit != std::string::npos) str.replace(dit, 6, domain_path);
        std::size_t pit = str.find("PROBLEM");
        if (pit != std::string::npos) str.replace(pit, 7, problem_path);
        std::string commandString = str + " > " + data_path + "plan.pddl";

        RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Running: %s in a new thread",
                    get_name(), problem_name.c_str(), commandString.c_str());

        planner_ = std::thread(&OnlinePlannerInterface::runCommand, this, commandString);
        return true;
    }

    void OnlinePlannerInterface::getPlannerParams(
        const std::shared_ptr<ros2plan_dispatch_msgs::srv::GetPlanningParams::Request> /*req*/,
        std::shared_ptr<ros2plan_dispatch_msgs::srv::GetPlanningParams::Response> res)
    {
        res->domain_path = domain_path;
        res->problem_path = problem_path;
        res->planner_ready = planner_running_;
    }

} // close namespace

int main(int argc, char **argv) {
    srand(static_cast<unsigned>(time(0)));
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_ros2plan::OnlinePlannerInterface>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
