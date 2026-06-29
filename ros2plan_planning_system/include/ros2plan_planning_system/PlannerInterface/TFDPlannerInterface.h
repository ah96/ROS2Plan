#ifndef KCL_TFD_planner_interface
#define KCL_TFD_planner_interface

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>

namespace KCL_ros2plan {

    class TFDPlannerInterface : public PlannerInterface
    {
    private:
        std::string runCommand(std::string cmd);

    protected:
        bool runPlanner() override;

    public:
        TFDPlannerInterface();
        virtual ~TFDPlannerInterface() = default;
    };

} // close namespace

#endif
