#include "rosplan_planning_system/PlannerInterface/PPRPPlannerInterface.h"
#include <regex>

namespace KCL_rosplan {

    PPRPPlannerInterface::PPRPPlannerInterface()
        : PlannerInterface("rosplan_planner_interface")
    {
    }

    std::string PPRPPlannerInterface::runCommand(std::string cmd) {
        std::string data;
        FILE *stream;
        char buffer[1000];
        stream = popen(cmd.c_str(), "r");
        while (fgets(buffer, 1000, stream) != NULL)
            data.append(buffer);
        pclose(stream);
        return data;
    }

    bool PPRPPlannerInterface::runPlanner() {

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
            if (line.find("No solution") != line.npos) break;
            if (line.find("Dead ends") != line.npos) solved = true;
        }

        if (solved) {
            int idx = 0;
            std::regex actionline_rgx("_DETDUP_.*_\\d+");
            std::regex tr_ws("^ +| +$|( ) +");
            while (std::getline(planfile, line) && line.find("Plan length") == line.npos) {
                planner_output += std::to_string(idx) + ": (";
                ++idx;
                line = line.substr(0, line.find(" ("));
                line = std::regex_replace(line, actionline_rgx, "");
                line = std::regex_replace(line, tr_ws, "$1");
                planner_output += line;
                planner_output += ")  [0.001]\n";
            }
        }

        planfile.close();

        if (!solved) RCLCPP_WARN(get_logger(), "KCL: (%s) (%s) Plan was unsolvable.", get_name(), problem_name.c_str());
        else RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Plan was solved.", get_name(), problem_name.c_str());

        return solved;
    }

} // close namespace

int main(int argc, char **argv) {
    srand(static_cast<unsigned>(time(0)));
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_rosplan::PPRPPlannerInterface>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
