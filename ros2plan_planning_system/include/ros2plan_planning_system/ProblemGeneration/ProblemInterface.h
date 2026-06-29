#include <rclcpp/rclcpp.hpp>

#include "ProblemGeneratorFactory.h"

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

#include "ros2plan_dispatch_msgs/srv/problem_service.hpp"

#ifndef KCL_problem_interface
#define KCL_problem_interface

/**
 * This file contains an interface to the problem.
 */
namespace KCL_ros2plan {

    class ProblemInterface : public rclcpp::Node
    {
    private:

        rclcpp::CallbackGroup::SharedPtr srv_cbg_;

        /* params */
        std::string problem_path;
        std::string problem_name;
        std::string knowledge_base;

        rclcpp::Node::SharedPtr svc_node_;
        ProblemGeneratorPtr problem_generator_;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr problem_generation_service_;
        rclcpp::Service<ros2plan_dispatch_msgs::srv::ProblemService>::SharedPtr problem_generation_params_service_;

    public:

        ProblemInterface();
        virtual ~ProblemInterface() = default;

        bool runProblemServer(const std::string& problemPath);

        void runProblemServerDefault(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res);

        void runProblemServerParams(
            const std::shared_ptr<ros2plan_dispatch_msgs::srv::ProblemService::Request> req,
            std::shared_ptr<ros2plan_dispatch_msgs::srv::ProblemService::Response> res);

        /* ROS interface */
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr problem_publisher_;
    };

} // close namespace

#endif
