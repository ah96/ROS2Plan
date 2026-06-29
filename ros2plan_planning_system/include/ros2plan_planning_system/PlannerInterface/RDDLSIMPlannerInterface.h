//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 28/09/18.
//

#ifndef KCL_RDDLSIM_planner_interface
#define KCL_RDDLSIM_planner_interface

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>

namespace KCL_ros2plan {

    class RDDLSIMPlannerInterface : public PlannerInterface
    {
    private:
        std::string runCommand(std::string cmd);

    protected:
        bool runPlanner() override;

    public:
        RDDLSIMPlannerInterface();
        virtual ~RDDLSIMPlannerInterface() = default;
    };

} // close namespace

#endif
