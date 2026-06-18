#ifndef KCL_UPM_planner_interface
#define KCL_UPM_planner_interface

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>

namespace KCL_rosplan {

    class UPMPlannerInterface : public PlannerInterface
    {
    private:
        std::string runCommand(std::string cmd);

    protected:
        std::string input_problem_path;
        bool runPlanner() override;

    public:
        UPMPlannerInterface();
        virtual ~UPMPlannerInterface() = default;
    };

} // close namespace

#endif
