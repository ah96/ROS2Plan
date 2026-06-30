/**
 *
 * Copyright [2019] <KCL King's College London>
 *
 * Author: Martin Koling (martinh.koling@kcl.ac.uk)
 * Author: Oscar Lima (oscar.lima@dfki.de)
 *
 * Unit tests for ROS2Plan planner interface (ROS2 port)
 */

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros2plan_dispatch_msgs/srv/planning_service.hpp"

using namespace std::chrono_literals;
using PlanningService = ros2plan_dispatch_msgs::srv::PlanningService;

// In ROS2 the planner interface advertises its services and topics with relative names and the
// default (empty) namespace, so they resolve to the root namespace.
static const char * kPlanningSrv = "/planning_server_params";
static const char * kPlannerTopic = "/planner_output";
static const char * kProblemTopic = "/problem_instance";

static std::string pkg_share()
{
    return ament_index_cpp::get_package_share_directory("ros2plan_planning_system");
}

static std::string test_data_dir()
{
    return pkg_share() + "/test/pddl/test_domain/";
}

// Path to the POPF binary installed under the package share directory.
static std::string popf_command()
{
    return "timeout 10 " + pkg_share() + "/common/bin/popf -n DOMAIN PROBLEM";
}

// Fills the common request fields (data/domain paths + planner command). The caller sets the
// per-test problem_path.
static void fill_common_request(PlanningService::Request & req)
{
    req.use_problem_topic = false;
    req.data_path = test_data_dir();
    req.domain_path = test_data_dir() + "domain.pddl";
    req.planner_command = popf_command();
}

// Calls the planning (params) service synchronously and returns whether the call succeeded.
static bool call_planning_service(rclcpp::Node::SharedPtr node,
                                  const PlanningService::Request & request_in,
                                  bool & plan_found)
{
    auto client = node->create_client<PlanningService>(kPlanningSrv);
    if (!client->wait_for_service(10s)) {
        return false;
    }
    auto req = std::make_shared<PlanningService::Request>(request_in);
    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, future, 30s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        return false;
    }
    plan_found = future.get()->plan_found;
    return true;
}

GTEST_TEST(PlannerInterfaceTests, Test1_plan_found) {
    auto node = rclcpp::Node::make_shared("planner_interface_test_plan_found");

    PlanningService::Request req;
    fill_common_request(req);
    req.problem_path = test_data_dir() + "test_problem.pddl";

    bool plan_found = false;
    ASSERT_TRUE(call_planning_service(node, req, plan_found));
    EXPECT_TRUE(plan_found);
}

GTEST_TEST(PlannerInterfaceTests, Test2_format_published_on_planner_output) {
    auto node = rclcpp::Node::make_shared("planner_interface_test_output");

    std::string last_plan;
    bool plan_received = false;
    // The planner output is published with transient_local QoS, so a transient_local subscriber
    // receives the last published plan even if it connects after publication.
    auto sub = node->create_subscription<std_msgs::msg::String>(
        kPlannerTopic, rclcpp::QoS(1).transient_local(),
        [&](std_msgs::msg::String::SharedPtr msg) {
            last_plan = msg->data;
            plan_received = true;
        });

    PlanningService::Request req;
    fill_common_request(req);
    req.problem_path = test_data_dir() + "test_problem.pddl";

    bool plan_found = false;
    ASSERT_TRUE(call_planning_service(node, req, plan_found));
    EXPECT_TRUE(plan_found);

    // Wait for the latched plan message.
    auto start = node->now();
    while (!plan_received && (node->now() - start) < rclcpp::Duration(10s)) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(50ms);
    }

    ASSERT_TRUE(plan_received);

    std::string known_plan = "0.000: (movetob ball)  [0.001]\n";
    ASSERT_EQ(last_plan, known_plan);
}

GTEST_TEST(PlannerInterfaceTests, Test3_problem_without_solution) {
    auto node = rclcpp::Node::make_shared("planner_interface_test_no_solution");

    PlanningService::Request req;
    fill_common_request(req);
    req.problem_path = test_data_dir() + "test_problem_no_solution.pddl";

    bool plan_found = false;
    ASSERT_TRUE(call_planning_service(node, req, plan_found));
    EXPECT_FALSE(plan_found);
}

GTEST_TEST(PlannerInterfaceTests, Test4_invalid_pddl_syntax) {
    auto node = rclcpp::Node::make_shared("planner_interface_test_invalid_syntax");

    PlanningService::Request req;
    fill_common_request(req);
    req.problem_path = test_data_dir() + "test_problem_invalid_syntax.pddl";

    bool plan_found = false;
    ASSERT_TRUE(call_planning_service(node, req, plan_found));
    EXPECT_FALSE(plan_found);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
