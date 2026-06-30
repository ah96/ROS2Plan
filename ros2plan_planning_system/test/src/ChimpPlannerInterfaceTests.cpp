/**
 *
 * Copyright [2019] <KCL King's College London>
 *
 * Author: Martin Koling (martinh.koling@kcl.ac.uk)
 * Author: Oscar Lima (oscar.lima@dfki.de)
 * Author: Sebastian Stock (sebastian.stock@dfki.de)
 *
 * Unit tests for ROS2Plan CHIMP planner interface (ROS2 port)
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

// CHIMP hierarchical (HDDL) fixtures live under the installed test/ directory.
static std::string chimp_data_dir()
{
    return pkg_share() + "/test/hierarchical_domains/chimp/restaurant";
}

// The CHIMP planner is launched through its gradle wrapper. The interface substitutes the DOMAIN,
// PROBLEM and OUTPUT tokens before invoking the command.
static std::string chimp_command(int timeout_secs)
{
    std::string chimp_path =
        ament_index_cpp::get_package_share_directory("ros2plan_dependencies") + "/chimp";
    return "timeout " + std::to_string(timeout_secs) + " " + chimp_path +
           "/gradlew run -p " + chimp_path + " -Dexec.args=\"DOMAIN PROBLEM -o=OUTPUT\"";
}

// Fills the common request fields (data/domain paths + planner command). The caller sets the
// per-test problem_path.
static void fill_common_request(PlanningService::Request & req)
{
    req.use_problem_topic = false;
    req.data_path = chimp_data_dir();
    req.domain_path = chimp_data_dir() + "/race_waiter_domain.ddl";
    req.problem_path = chimp_data_dir() + "/serve_coffee_problem.pdl";
    req.planner_command = chimp_command(50);
}

// Removes blank lines so an exact-string comparison is tolerant of differing trailing/blank-line
// whitespace between the reference plan and the produced plan.
static std::string strip_blank_lines(const std::string & in)
{
    std::stringstream ss(in);
    std::string line;
    std::string out;
    while (std::getline(ss, line)) {
        // trim a trailing carriage return (in case of CRLF line endings)
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.find_first_not_of(" \t") == std::string::npos) {
            continue;  // skip empty / whitespace-only lines
        }
        out += line;
        out += '\n';
    }
    return out;
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
    if (rclcpp::spin_until_future_complete(node, future, 120s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        return false;
    }
    plan_found = future.get()->plan_found;
    return true;
}

GTEST_TEST(PlannerInterfaceTests, Test1_plan_found) {
    auto node = rclcpp::Node::make_shared("chimp_planner_interface_test_plan_found");

    PlanningService::Request req;
    fill_common_request(req);

    bool plan_found = false;
    ASSERT_TRUE(call_planning_service(node, req, plan_found));
    EXPECT_TRUE(plan_found);
}

GTEST_TEST(PlannerInterfaceTests, Test2_format_published_on_planner_output) {
    auto node = rclcpp::Node::make_shared("chimp_planner_interface_test_output");

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

    // Publish the input problem on the problem topic; the interface writes it out to the
    // configured problem_path before planning.
    auto problem_pub = node->create_publisher<std_msgs::msg::String>(kProblemTopic, 1);

    PlanningService::Request req;
    fill_common_request(req);
    req.use_problem_topic = true;
    // The planner interface **writes** the (received) problem to this file before planning.
    req.problem_path = chimp_data_dir() + "/generated_problem.pdl";

    // Input problem that is sent via the topic.
    std::ifstream t(chimp_data_dir() + "/test_m_drive_robot_1.pdl");
    std::stringstream ss;
    ss << t.rdbuf();
    std_msgs::msg::String problem_msg;
    problem_msg.data = ss.str();

    // Give the subscriber on the node side time to connect, then publish the problem.
    auto pub_start = node->now();
    while ((node->now() - pub_start) < rclcpp::Duration(1s)) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(50ms);
    }
    problem_pub->publish(problem_msg);
    // Let the problem propagate to the interface before requesting the plan.
    auto settle = node->now();
    while ((node->now() - settle) < rclcpp::Duration(1s)) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(50ms);
    }

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

    std::string known_plan =
        "0.001: (!move_torso TorsoDownPosture) [4.000]\n"
        "0.001: (!tuck_arms ArmTuckedPosture ArmTuckedPosture) [4.000]\n"
        "4.002: (!move_base preManipulationAreaNorthTable1) [0.002]\n";
    ASSERT_EQ(strip_blank_lines(last_plan), strip_blank_lines(known_plan));
}

GTEST_TEST(PlannerInterfaceTests, Test3_problem_without_solution) {
    auto node = rclcpp::Node::make_shared("chimp_planner_interface_test_no_solution");

    PlanningService::Request req;
    fill_common_request(req);
    // A shorter timeout: CHIMP will not find a solution for this problem.
    req.planner_command = chimp_command(3);
    req.problem_path = chimp_data_dir() + "/test_problem_no_solution.pdl";

    bool plan_found = false;
    ASSERT_TRUE(call_planning_service(node, req, plan_found));
    EXPECT_FALSE(plan_found);
}

GTEST_TEST(PlannerInterfaceTests, Test4_invalid_problem_syntax) {
    auto node = rclcpp::Node::make_shared("chimp_planner_interface_test_invalid_syntax");

    PlanningService::Request req;
    fill_common_request(req);
    req.problem_path = chimp_data_dir() + "/test_problem_invalid_syntax.pdl";

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
