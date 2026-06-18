//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#ifndef ROSPLAN_PLANNING_SYSTEM_ONLINEPLANNERINTERFACE_H
#define ROSPLAN_PLANNING_SYSTEM_ONLINEPLANNERINTERFACE_H

#include "PlannerInterface.h"
#include "rosplan_dispatch_msgs/srv/get_planning_params.hpp"
#include <fstream>
#include <thread>

namespace KCL_rosplan {

    class OnlinePlannerInterface : public PlannerInterface {
    private:
        std::string runCommand(std::string cmd);
        std::thread planner_;
        bool planner_running_;

        rclcpp::Service<rosplan_dispatch_msgs::srv::GetPlanningParams>::SharedPtr get_planner_params_;

    protected:
        bool runPlanner() override;
        void getPlannerParams(
            const std::shared_ptr<rosplan_dispatch_msgs::srv::GetPlanningParams::Request> req,
            std::shared_ptr<rosplan_dispatch_msgs::srv::GetPlanningParams::Response> res);

    public:
        OnlinePlannerInterface();
        ~OnlinePlannerInterface();
    };
}

#endif //ROSPLAN_PLANNING_SYSTEM_ONLINEPLANNERINTERFACE_H
