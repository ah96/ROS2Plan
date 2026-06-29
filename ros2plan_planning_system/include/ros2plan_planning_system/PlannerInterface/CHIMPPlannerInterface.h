// Created by Sebastian Stock <sebastian.stock@dfki.de> on 2020-08-17

#ifndef KCL_CHIMP_planner_interface
#define KCL_CHIMP_planner_interface

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>

namespace KCL_ros2plan {

    class CHIMPPlannerInterface : public PlannerInterface
    {
    private:
        std::string runCommand(std::string cmd);
        std::string prepareCommand();
        void writeProblemToFile();

    protected:
        bool runPlanner() override;

    public:
        CHIMPPlannerInterface();
        virtual ~CHIMPPlannerInterface() = default;
    };

} // close namespace

#endif
