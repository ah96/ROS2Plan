#include "ros2plan_planning_system/PlannerInterface/CHIMPPlannerInterface.h"

namespace KCL_ros2plan {

    CHIMPPlannerInterface::CHIMPPlannerInterface()
        : PlannerInterface("ros2plan_planner_interface")
    {
    }

    std::string CHIMPPlannerInterface::runCommand(std::string cmd) {
        std::string data;
        FILE *stream;
        char buffer[1000];
        stream = popen(cmd.c_str(), "r");
        while (fgets(buffer, 1000, stream) != NULL)
            data.append(buffer);
        pclose(stream);
        return data;
    }

    void CHIMPPlannerInterface::writeProblemToFile() {
        std::ofstream dest;
        dest.open(problem_path.c_str());
        dest << problem_instance;
        dest.close();
    }

    std::string CHIMPPlannerInterface::prepareCommand() {
        std::string str = planner_command;
        std::size_t dit = str.find("DOMAIN");
        if (dit != std::string::npos) str.replace(dit, 6, domain_path);
        std::size_t pit = str.find("PROBLEM");
        if (pit != std::string::npos) str.replace(pit, 7, problem_path);
        std::size_t oit = str.find("OUTPUT");
        if (oit != std::string::npos) str.replace(oit, 6, data_path + "plan.txt");
        std::size_t eit = str.find("ESTEREL");
        if (eit != std::string::npos) str.replace(eit, 7, data_path + "esterel.yaml");
        return str;
    }

    static bool containsEsterel(const std::string& commandStr) {
        return commandStr.find("ESTEREL") != std::string::npos;
    }

    static std::string readEsterel(const std::string& esterelPath) {
        std::ifstream esterelFile;
        esterelFile.open(esterelPath.c_str());
        std::stringstream ss;
        ss << esterelFile.rdbuf();
        esterelFile.close();
        return ss.str();
    }

    bool CHIMPPlannerInterface::runPlanner() {

        if (use_problem_topic && problem_instance_received) {
            RCLCPP_INFO(get_logger(), "ROS2Plan: (%s) (%s) Writing problem to file.", get_name(), problem_name.c_str());
            writeProblemToFile();
        }

        std::string commandString = prepareCommand();
        RCLCPP_INFO(get_logger(), "ROS2Plan: (%s) (%s) Running: %s", get_name(), problem_name.c_str(), commandString.c_str());
        std::string plan = runCommand(commandString.c_str());
        RCLCPP_INFO(get_logger(), "ROS2Plan: (%s) (%s) Planning complete", get_name(), problem_name.c_str());

        std::ifstream planfile;
        std::string planFilePath = data_path + "plan.txt";
        planfile.open(planFilePath.c_str());
        std::string line;

        bool solved = false;
        std::string actions = "";
        while (std::getline(planfile, line)) {
            if (line.find("; Solution found", 0) != std::string::npos) {
                solved = true;
            } else if (line.find("; Actions:", 0) == std::string::npos) {
                // consume useless lines
            } else {
                std::stringstream ss;
                ss.str("");
                std::string innerLine;
                while (std::getline(planfile, innerLine)) {
                    if (innerLine.length() < 2)
                        break;
                    ss << innerLine << std::endl;
                }
                actions = ss.str();
            }
        }
        planfile.close();

        if (containsEsterel(planner_command)) {
            planner_output = readEsterel(data_path + "esterel.yaml");
        } else {
            planner_output = actions;
        }

        if (!solved) RCLCPP_INFO(get_logger(), "ROS2Plan: (%s) (%s) Plan was unsolvable.", get_name(), problem_name.c_str());
        else RCLCPP_INFO(get_logger(), "ROS2Plan: (%s) (%s) Plan was solved.", get_name(), problem_name.c_str());

        return solved;
    }

} // close namespace

int main(int argc, char **argv) {
    srand(static_cast<unsigned>(time(0)));
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_ros2plan::CHIMPPlannerInterface>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
