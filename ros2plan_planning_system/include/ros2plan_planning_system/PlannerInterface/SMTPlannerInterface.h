#ifndef KCL_SMT_planner_interface
#define KCL_SMT_planner_interface

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>

namespace KCL_ros2plan {

    class SMTPlannerInterface : public PlannerInterface
    {
    private:
        std::string runCommand(std::string cmd);

    protected:
        bool runPlanner() override;

    public:
        SMTPlannerInterface();
        virtual ~SMTPlannerInterface() = default;
    };

} // close namespace

#endif
