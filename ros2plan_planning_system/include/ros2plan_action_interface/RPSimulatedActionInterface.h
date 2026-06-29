#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <random>

#include "ros2plan_action_interface/RPActionInterface.h"

#ifndef KCL_simulatedaction
#define KCL_simulatedaction

/**
 * This file defines the RPSimulatedActionInterface class.
 * RPSimulatedActionInterface is used to simulate synthetic actions (non physics based simulator)
 */
namespace KCL_ros2plan {

    class RPSimulatedActionInterface : public RPActionInterface
    {
    private:

        double action_duration;
        double action_duration_stddev;
        double action_probability;

    public:

        RPSimulatedActionInterface();

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const ros2plan_dispatch_msgs::msg::ActionDispatch::SharedPtr msg) override;
    };
}
#endif
