#ifndef KCL_POPF_planner_interface
#define KCL_POPF_planner_interface

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>

namespace KCL_rosplan {

    class POPFPlannerInterface : public PlannerInterface
    {
    private:
        std::string runCommand(std::string cmd);

    protected:
        bool runPlanner() override;

    public:
        POPFPlannerInterface();
        virtual ~POPFPlannerInterface() = default;
    };

} // close namespace

#endif
