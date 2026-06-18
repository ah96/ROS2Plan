#include "rosplan_planning_system/PlannerInterface/LPGPlannerInterface.h"

namespace KCL_rosplan {

    LPGPlannerInterface::LPGPlannerInterface()
        : PlannerInterface("rosplan_planner_interface")
    {
    }

    std::string LPGPlannerInterface::runCommand(std::string cmd) {
        std::string data;
        FILE *stream;
        char buffer[1000];
        stream = popen(cmd.c_str(), "r");
        while (fgets(buffer, 1000, stream) != NULL)
            data.append(buffer);
        pclose(stream);
        return data;
    }

    bool LPGPlannerInterface::runPlanner() {

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
        std::size_t oit = str.find("-out ");

        RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Changing '-out' to 'data_path/lpgplan'. !!! '-out' must be the last parameter in the planner_command !!!",
                    get_name(), problem_name.c_str());
        if (oit != std::string::npos) str.replace(oit, 500, "-out " + data_path + "lpgplan");

        std::string updatePlan = "cp " + data_path + "lpgplan.SOL" + " " + data_path + "plan.pddl";

        RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Running: %s", get_name(), problem_name.c_str(), str.c_str());
        std::string plan = runCommand(str.c_str());
        RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Planning complete", get_name(), problem_name.c_str());

        runCommand(updatePlan.c_str());

        std::ifstream planfile;
        planfile.open((data_path + "plan.pddl").c_str());
        std::string line;
        std::stringstream ss;

        bool solved = false;
        double planDuration;

        while (std::getline(planfile, line)) {
            if (line.find("0.0", 0) != std::string::npos) {
                solved = true;
            } else if (line.find("; ", 0) == std::string::npos) {
                // consume useless lines
            }
            planDuration = 0;

            if (line.length() > 1) {
                ss.str("");
            }
            while (std::getline(planfile, line)) {
                if (line.length() < 2)
                    break;
                ss << line << std::endl;
            }
            planner_output = ss.str();
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
    auto node = std::make_shared<KCL_rosplan::LPGPlannerInterface>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
