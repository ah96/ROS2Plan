//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 28/09/18.
//
#include "rosplan_planning_system/PlannerInterface/RDDLSIMPlannerInterface.h"
#include <regex>
#include <sstream>

namespace KCL_rosplan {

    RDDLSIMPlannerInterface::RDDLSIMPlannerInterface()
        : PlannerInterface("rosplan_planner_interface")
    {
    }

    std::string RDDLSIMPlannerInterface::runCommand(std::string cmd) {
        std::string data;
        FILE *stream;
        char buffer[1000];
        stream = popen(cmd.c_str(), "r");
        while (fgets(buffer, 1000, stream) != NULL)
            data.append(buffer);
        pclose(stream);
        return data;
    }

    bool RDDLSIMPlannerInterface::runPlanner() {

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
        planner_output.clear();

        std::regex actionline_rgx("\\*\\* Actions received: \\[(.*)\\]");
        std::regex action_name_params_rgx(",?\\s?(.+)\\((.*)\\)|,?\\s?(.+)");
        std::smatch match;
        int action_idx = 0;
        while (std::getline(planfile, line)) {
            if (std::regex_search(line, match, actionline_rgx)) {
                std::istringstream action_list(match[1].str());
                std::string action;
                while (getline(action_list, action, ';')) {
                    if (std::regex_search(action, match, action_name_params_rgx)) {
                        std::string action_name = (match[1].str().empty()) ? match[3].str() : match[1].str();
                        planner_output += std::to_string(action_idx) + ": (" + action_name;
                        std::istringstream p(match[2].str());
                        std::string param;
                        while (getline(p, param, ',')) {
                            if (!param.empty() && param[0] != ' ') planner_output += " ";
                            planner_output += param;
                        }
                        planner_output += ")  [0.001]\n";
                    }
                }
                ++action_idx;
            }
        }
        planfile.close();
        bool solved = !planner_output.empty();
        if (!solved) RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Plan was unsolvable.", get_name(), problem_name.c_str());
        else RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Plan was solved.", get_name(), problem_name.c_str());

        return solved;
    }

} // close namespace

int main(int argc, char **argv) {
    srand(static_cast<unsigned>(time(0)));
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_rosplan::RDDLSIMPlannerInterface>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
