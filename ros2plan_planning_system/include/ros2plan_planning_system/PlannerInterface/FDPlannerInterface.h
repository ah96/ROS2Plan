#ifndef KCL_FD_planner_interface
#define KCL_FD_planner_interface

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>

namespace KCL_ros2plan {

    class FDPlannerInterface : public PlannerInterface
    {
    private:
        std::string runCommand(std::string cmd);

    protected:
        bool runPlanner() override;

    public:
        FDPlannerInterface();
        virtual ~FDPlannerInterface() = default;
    };

} // close namespace

#endif
