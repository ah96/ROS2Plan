#ifndef KCL_FF_planner_interface
#define KCL_FF_planner_interface

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>

namespace KCL_ros2plan {

    class FFPlannerInterface : public PlannerInterface
    {
    private:
        bool use_ffha;
        std::string runCommand(std::string cmd);

    protected:
        bool runPlanner() override;

    public:
        FFPlannerInterface();
        virtual ~FFPlannerInterface() = default;
    };

} // close namespace

#endif
