#ifndef KCL_LPG_planner_interface
#define KCL_LPG_planner_interface

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>

namespace KCL_ros2plan {

    class LPGPlannerInterface : public PlannerInterface
    {
    private:
        std::string runCommand(std::string cmd);

    protected:
        bool runPlanner() override;

    public:
        LPGPlannerInterface();
        virtual ~LPGPlannerInterface() = default;
    };

} // close namespace

#endif
