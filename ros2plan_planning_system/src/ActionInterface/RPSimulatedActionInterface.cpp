#include "ros2plan_action_interface/RPSimulatedActionInterface.h"

namespace KCL_ros2plan {

    RPSimulatedActionInterface::RPSimulatedActionInterface()
        : RPActionInterface("ros2plan_simulated_action"),
          action_duration(-1.0),
          action_duration_stddev(0.0),
          action_probability(1.0)
    {
        declare_parameter("action_duration", -1.0);
        declare_parameter("action_duration_stddev", 0.0);
        declare_parameter("action_probability", 1.0);

        action_duration       = get_parameter("action_duration").as_double();
        action_duration_stddev = get_parameter("action_duration_stddev").as_double();
        action_probability    = get_parameter("action_probability").as_double();
    }

    bool RPSimulatedActionInterface::concreteCallback(
        const ros2plan_dispatch_msgs::msg::ActionDispatch::SharedPtr msg)
    {
        double duration = msg->duration;
        if (action_duration >= 0) {
            duration = action_duration;
        }

        if (action_duration_stddev > 0) {
            std::default_random_engine generator(
                static_cast<unsigned>(rclcpp::Clock().now().seconds()));
            std::normal_distribution<double> distribution(duration, action_duration_stddev);
            duration = std::max(distribution(generator), 0.0);
        }

        RCLCPP_INFO(get_logger(), "KCL: (%s) Action completing with probability %g and duration %g",
                    params.name.c_str(), action_probability, duration);

        if (duration > 0) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(duration));
        }

        return (rand() % 100) <= static_cast<int>(100 * action_probability);
    }

} // close namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KCL_ros2plan::RPSimulatedActionInterface>();
    node->setup();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
