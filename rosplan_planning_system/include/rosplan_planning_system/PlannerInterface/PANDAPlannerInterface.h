#ifndef KCL_PANDA_planner_interface
#define KCL_PANDA_planner_interface

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>

namespace KCL_rosplan {

    class PANDAPlannerInterface : public PlannerInterface
    {
    private:
        std::string runCommand(std::string cmd);

    protected:
        bool runPlanner() override;

    public:
        PANDAPlannerInterface();
        virtual ~PANDAPlannerInterface() = default;
    };

} // close namespace

#endif
