#ifndef KCL_PPRP_planner_interface
#define KCL_PPRP_planner_interface

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>

namespace KCL_rosplan {

    class PPRPPlannerInterface : public PlannerInterface
    {
    private:
        std::string runCommand(std::string cmd);

    protected:
        bool runPlanner() override;

    public:
        PPRPPlannerInterface();
        virtual ~PPRPPlannerInterface() = default;
    };

} // close namespace

#endif
