#include "rosplan_planning_system/PlannerInterface/TFDPlannerInterface.h"

namespace KCL_rosplan {

    TFDPlannerInterface::TFDPlannerInterface()
        : PlannerInterface("rosplan_planner_interface")
    {
    }

    std::string TFDPlannerInterface::runCommand(std::string cmd) {
        std::string data;
        FILE *stream;
        char buffer[1000];
        stream = popen(cmd.c_str(), "r");
        while (fgets(buffer, 1000, stream) != NULL)
            data.append(buffer);
        pclose(stream);
        return data;
    }

    bool TFDPlannerInterface::runPlanner() {
        std::string plannerPlanPath = data_path + "../../rosplan_planning_system/common/bin/tfd-src-0.4/downward/";
        std::string tfdOutputName = "tfdplan";

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

        RCLCPP_INFO(get_logger(), "KCL: (%s) Removing old TFD plans with (%s) name.", get_name(), tfdOutputName.c_str());
        std::string removeOldPlans = "cd " + plannerPlanPath + " && rm " + tfdOutputName + ".*";
        runCommand(removeOldPlans);

        RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Running: %s", get_name(), problem_name.c_str(), str.c_str());
        std::string plan = runCommand(str.c_str());
        RCLCPP_INFO(get_logger(), "KCL: (%s) (%s) Planning complete", get_name(), problem_name.c_str());

        std::string bestPlanCommand = "cd " + plannerPlanPath + " && ls | grep " + tfdOutputName + " | tail -1";
        std::string bestPlan = runCommand(bestPlanCommand.c_str());
        if (!bestPlan.empty() && bestPlan.back() == '\n') {
            bestPlan.erase(bestPlan.size() - 1);
        }

        std::string updatePlanCommand = "cp " + plannerPlanPath + bestPlan + " " + data_path + "plan.pddl";
        runCommand(updatePlanCommand.c_str());

        std::ifstream planfile;
        planfile.open((data_path + "plan.pddl").c_str());
        std::string line;
        std::stringstream ss;

        bool solved = false;
        double planDuration;

        while (std::getline(planfile, line)) {
            if (line.find("0.0", 0) != std::string::npos) {
                solved = true;
            }
            planDuration = 0;
            ss.str("");
            do {
                if (line.length() < 2)
                    break;
                ss << line << std::endl;
            } while (std::getline(planfile, line));

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
    auto node = std::make_shared<KCL_rosplan::TFDPlannerInterface>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
