/**
 *
 * Copyright [2019] <KCL King's College London>
 *
 * Author: Martin Koling (martinh.koling@kcl.ac.uk)
 * Author: Oscar Lima (oscar.lima@dfki.de)
 *
 * Unit tests for ROS2Plan problem interface (ROS2 port)
 */

#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros2plan_dispatch_msgs/srv/problem_service.hpp"

using namespace std::chrono_literals;
using ProblemService = ros2plan_dispatch_msgs::srv::ProblemService;

// In ROS2 the problem interface advertises its services with relative names and the default
// (empty) namespace, so they resolve to the root namespace.
static const char * kProblemSrv = "/problem_generation_server_params";
static const char * kProblemTopic = "/problem_instance";

static std::string test_problem_path()
{
    return ament_index_cpp::get_package_share_directory("ros2plan_planning_system") +
           "/test/pddl/amazon/generated_test_problem.pddl";
}

// The PDDL problem generator emits a cosmetic blank line after every domain predicate's
// proposition/TIL block (this matches upstream ROSPlan exactly — the number of blank lines just
// depends on how many predicates the domain declares). Compare on the meaningful, non-blank
// content lines so the test verifies the generated objects/facts/goal rather than blank-line count.
static std::string strip_blank_lines(const std::string & s)
{
    std::string out;
    std::istringstream iss(s);
    std::string line;
    while (std::getline(iss, line)) {
        if (!line.empty()) {
            out += line;
            out += '\n';
        }
    }
    return out;
}

// Calls the problem-generation (params) service synchronously.
static bool call_problem_service(bool & problem_generated, std::string & problem_string)
{
    auto node = rclcpp::Node::make_shared("problem_interface_test_client");
    auto client = node->create_client<ProblemService>(kProblemSrv);
    if (!client->wait_for_service(10s)) {
        return false;
    }
    auto req = std::make_shared<ProblemService::Request>();
    req->problem_path = test_problem_path();
    req->problem_string_response = true;

    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, future, 10s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        return false;
    }
    auto res = future.get();
    problem_generated = res->problem_generated;
    problem_string = res->problem_string;
    return true;
}

static const std::string known_problem =
    "(define (problem task)\n(:domain driverlog-simple)\n(:objects\n    "
    "home amazon london myhouse - place\n    driver - driver\n    truck - truck\n    mydvd - item\n)\n"
    "(:init\n    (at driver home)\n    (at truck amazon)\n    (at mydvd amazon)\n\n\n    (link amazon "
    "london)\n    (link london amazon)\n    (link london myhouse)\n    (link myhouse london)\n\n    "
    "(path home amazon)\n    (path amazon home)\n\n)\n(:goal (and\n    (at mydvd myhouse)\n))\n)\n";

GTEST_TEST(ProblemInterfaceTests, Test1_problem_generated) {
    bool problem_generated = false;
    std::string problem_string;
    ASSERT_TRUE(call_problem_service(problem_generated, problem_string));
    EXPECT_TRUE(problem_generated);
}

GTEST_TEST(ProblemInterfaceTests, Test2_problem_string_against_known_problem) {
    bool problem_generated = false;
    std::string problem_string;
    ASSERT_TRUE(call_problem_service(problem_generated, problem_string));
    EXPECT_TRUE(problem_generated);
    ASSERT_EQ(strip_blank_lines(known_problem), strip_blank_lines(problem_string));
}

GTEST_TEST(ProblemInterfaceTests, Test3_problem_instance_against_known_problem) {
    auto node = rclcpp::Node::make_shared("problem_interface_test_sub");

    std::string received;
    bool problem_received = false;
    // The publisher is latched (transient_local), so a transient_local subscriber receives the
    // last published problem even if it connects after publication.
    auto sub = node->create_subscription<std_msgs::msg::String>(
        kProblemTopic, rclcpp::QoS(1).transient_local(),
        [&](std_msgs::msg::String::SharedPtr msg) {
            received = msg->data;
            problem_received = true;
        });

    // Trigger problem generation (which publishes the problem on the topic).
    auto client = node->create_client<ProblemService>(kProblemSrv);
    ASSERT_TRUE(client->wait_for_service(10s));
    auto req = std::make_shared<ProblemService::Request>();
    req->problem_path = test_problem_path();
    req->problem_string_response = true;
    auto future = client->async_send_request(req);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, 10s),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->problem_generated);

    // Wait for the latched topic message.
    auto start = node->now();
    while (!problem_received && (node->now() - start) < rclcpp::Duration(10s)) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(50ms);
    }

    ASSERT_TRUE(problem_received);
    ASSERT_EQ(strip_blank_lines(received), strip_blank_lines(known_problem));
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
