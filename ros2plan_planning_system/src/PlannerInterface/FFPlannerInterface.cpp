#include "ros2plan_planning_system/PlannerInterface/FFPlannerInterface.h"
#include <algorithm>

namespace KCL_ros2plan {

    FFPlannerInterface::FFPlannerInterface()
        : PlannerInterface("ros2plan_planner_interface"),
          use_ffha(false)
    {
        declare_parameter("use_ffha", false);
        use_ffha = get_parameter("use_ffha").as_bool();
    }

    std::string FFPlannerInterface::runCommand(std::string cmd) {
        std::string data;
        FILE *stream;
        char buffer[1000];
        stream = popen(cmd.c_str(), "r");
        while (fgets(buffer, 1000, stream) != NULL)
            data.append(buffer);
        pclose(stream);
        return data;
    }

    bool FFPlannerInterface::runPlanner() {
        planner_output.clear();

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

        RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Running: %s", get_name(), problem_name.c_str(), commandString.c_str());
        std::string plan = runCommand(commandString.c_str());
        RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Planning complete", get_name(), problem_name.c_str());

        std::ifstream planfile;
        planfile.open((data_path + "plan.pddl").c_str());
        std::string line;

        bool solved = false;
        while (!solved && std::getline(planfile, line)) {
            if (line.find("ff: found legal plan") != line.npos) {
                solved = true;
            }
        }

        if (solved) {
            if (use_ffha) {
                std::getline(planfile, line);
            } else {
                while (std::getline(planfile, line)) {
                    if (line.substr(0, 4) == "step") {
                        line = line.substr(4);
                        break;
                    }
                }
            }

            while (line.find("plan cost") == line.npos &&
                   line.find("Total cost") == line.npos &&
                   line.find("time spend") == line.npos &&
                   line.size() > 0) {
                std::stringstream ss(line);
                std::string aux;
                ss >> aux;
                std::string add = use_ffha ? " " : " (";
                planner_output += aux + add;
                while (ss >> aux) {
                    planner_output += aux;
                    if (ss.good()) planner_output += " ";
                }
                add = use_ffha ? "  [0.001]\n" : ")  [0.001]\n";
                planner_output += add;
                std::getline(planfile, line);
            }
            std::transform(planner_output.begin(), planner_output.end(),
                           planner_output.begin(), ::tolower);
        }
        planfile.close();

        if (!solved) RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Plan was unsolvable.", get_name(), problem_name.c_str());
        else RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Plan was solved.", get_name(), problem_name.c_str());

        return solved;
    }

} // close namespace

int main(int argc, char **argv) {
    srand(static_cast<unsigned>(time(0)));
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_ros2plan::FFPlannerInterface>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
