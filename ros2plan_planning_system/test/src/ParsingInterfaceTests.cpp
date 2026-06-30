/**
 *
 * Copyright [2019] <KCL King's College London>
 *
 * Author: Martin Koling (martinh.koling@kcl.ac.uk)
 * Author: Oscar Lima (oscar.lima@dfki.de)
 *
 * Unit tests for ROS2Plan parsing interface (ROS2 port)
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "std_srvs/srv/empty.hpp"
#include "ros2plan_dispatch_msgs/srv/parsing_service.hpp"
#include "ros2plan_dispatch_msgs/msg/complete_plan.hpp"
#include "ros2plan_dispatch_msgs/msg/esterel_plan.hpp"
#include "ros2plan_dispatch_msgs/msg/action_dispatch.hpp"

using namespace std::chrono_literals;

using Empty = std_srvs::srv::Empty;
using ParsingService = ros2plan_dispatch_msgs::srv::ParsingService;
using CompletePlan = ros2plan_dispatch_msgs::msg::CompletePlan;
using EsterelPlan = ros2plan_dispatch_msgs::msg::EsterelPlan;
using ActionDispatch = ros2plan_dispatch_msgs::msg::ActionDispatch;

// In ROS2 each parser node advertises its services/topics with relative names. The launch harness
// places the simple parser in the "/simple" namespace and the esterel parser in the "/esterel"
// namespace, so the relative "parse_plan", "parse_plan_from_file" and "complete_plan" names resolve
// under those namespaces (rather than colliding in the root namespace as they would by default).
static const char * kSimpleParsePlanSrv = "/simple/parse_plan";
static const char * kSimpleParseFromFileSrv = "/simple/parse_plan_from_file";
static const char * kSimplePlanTopic = "/simple/complete_plan";

static const char * kEsterelParseFromFileSrv = "/esterel/parse_plan_from_file";
static const char * kEsterelPlanTopic = "/esterel/complete_plan";

// Shared parsing results, populated by the topic-collecting tests and read by the follow-up tests.
static std::vector<ActionDispatch> simple_plan_messages;
static EsterelPlan retrieved_esterel_plan;

static std::string test_plan_path()
{
    return ament_index_cpp::get_package_share_directory("ros2plan_planning_system") +
           "/test/pddl/amazon/test_plan.pddl";
}

// Calls a parse_plan_from_file service synchronously and returns whether the plan was parsed.
static bool call_parse_from_file(const std::string & srv_name, bool & plan_parsed)
{
    auto node = rclcpp::Node::make_shared("parsing_interface_test_client");
    auto client = node->create_client<ParsingService>(srv_name);
    if (!client->wait_for_service(10s)) {
        return false;
    }
    auto req = std::make_shared<ParsingService::Request>();
    req->plan_path = test_plan_path();

    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, future, 10s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        return false;
    }
    plan_parsed = future.get()->plan_parsed;
    return true;
}

GTEST_TEST(ParsingInterfaceTests, Test1_parse_plan) {
    auto node = rclcpp::Node::make_shared("parsing_interface_test_parse_plan");
    auto client = node->create_client<Empty>(kSimpleParsePlanSrv);
    ASSERT_TRUE(client->wait_for_service(10s));

    auto req = std::make_shared<Empty::Request>();
    auto future = client->async_send_request(req);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, 10s),
              rclcpp::FutureReturnCode::SUCCESS);
}

GTEST_TEST(ParsingInterfaceTests, Test2_parse_plan_from_file) {
    bool plan_parsed = false;
    ASSERT_TRUE(call_parse_from_file(kSimpleParseFromFileSrv, plan_parsed));
    ASSERT_TRUE(plan_parsed);
}

GTEST_TEST(ParsingInterfaceTests, Test3_simple_plan_messages_published_on_plan_topic) {
    auto node = rclcpp::Node::make_shared("parsing_interface_test_simple_sub");

    bool plan_received = false;
    // The simple parser publishes on a latched (transient_local) topic, so the subscriber must also
    // be transient_local to receive the last published plan.
    auto sub = node->create_subscription<CompletePlan>(
        kSimplePlanTopic, rclcpp::QoS(1).transient_local(),
        [&](CompletePlan::SharedPtr msg) {
            simple_plan_messages.clear();
            for (const auto & action : msg->plan) {
                simple_plan_messages.push_back(action);
            }
            plan_received = true;
        });

    // Trigger parsing from file (which publishes the parsed plan on the topic).
    auto client = node->create_client<ParsingService>(kSimpleParseFromFileSrv);
    ASSERT_TRUE(client->wait_for_service(10s));
    auto req = std::make_shared<ParsingService::Request>();
    req->plan_path = test_plan_path();
    auto future = client->async_send_request(req);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, 10s),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->plan_parsed);

    // Wait for the latched topic message.
    auto start = node->now();
    while (!plan_received && (node->now() - start) < rclcpp::Duration(10s)) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(50ms);
    }

    ASSERT_TRUE(plan_received);

    std::vector<std::string> actual_action_names;
    std::vector<std::string> expected_action_names = {"load-truck", "walk", "board-truck",
                        "drive-truck", "drive-truck", "unload-truck"};

    for (const auto & action : simple_plan_messages) {
        actual_action_names.push_back(action.name);
    }

    ASSERT_EQ(expected_action_names, actual_action_names);
}

GTEST_TEST(ParsingInterfaceTests, Test4_action_id_unique) {

    bool unique = true;
    int prevId = 1000;
    int currId;
    for (const auto & action : simple_plan_messages) {
        currId = action.action_id;
        if (currId == prevId) {
            unique = false;
            break;
        } else {
            prevId = action.action_id;
        }
    }

    EXPECT_TRUE(unique);
}

GTEST_TEST(ParsingInterfaceTests, Test5_full_complete_plan) {

    std::vector<int> actual_parameter_size;
    std::vector<float> actual_durations;
    std::vector<float> actual_dispatch_time;

    std::vector<int> expected_parameter_size = {3, 3, 3, 4, 4, 3};
    std::vector<float> expected_durations = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    std::vector<float> expected_dispatch_time = {0, 0, 0.001, 0.002, 0.003, 0.004};

    for (const auto & action : simple_plan_messages) {
        actual_parameter_size.push_back(action.parameters.size());
        actual_durations.push_back(action.duration);
        actual_dispatch_time.push_back(action.dispatch_time);
    }

    ASSERT_EQ(expected_parameter_size, actual_parameter_size);
    ASSERT_EQ(expected_durations, actual_durations);
    ASSERT_EQ(expected_dispatch_time, actual_dispatch_time);
}

GTEST_TEST(ParsingInterfaceTests, Test6_esterel_plan_messages_published_on_plan_topic) {
    auto node = rclcpp::Node::make_shared("parsing_interface_test_esterel_sub");

    bool plan_received = false;
    // The esterel parser publishes on a latched (transient_local) topic.
    auto sub = node->create_subscription<EsterelPlan>(
        kEsterelPlanTopic, rclcpp::QoS(1).transient_local(),
        [&](EsterelPlan::SharedPtr msg) {
            retrieved_esterel_plan = *msg;
            plan_received = true;
        });

    // Trigger esterel parsing from file (which publishes the esterel plan on the topic).
    auto client = node->create_client<ParsingService>(kEsterelParseFromFileSrv);
    ASSERT_TRUE(client->wait_for_service(10s));
    auto req = std::make_shared<ParsingService::Request>();
    req->plan_path = test_plan_path();
    auto future = client->async_send_request(req);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, 10s),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->plan_parsed);

    // Wait for the latched topic message.
    auto start = node->now();
    while (!plan_received && (node->now() - start) < rclcpp::Duration(10s)) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(50ms);
    }

    ASSERT_TRUE(plan_received);

    int actual_number_of_nodes = retrieved_esterel_plan.nodes.size();
    int actual_number_of_edges = retrieved_esterel_plan.edges.size();

    int expected_number_of_nodes = 13;
    int expected_number_of_edges = 21;

    ASSERT_EQ(expected_number_of_nodes, actual_number_of_nodes);
    ASSERT_EQ(expected_number_of_edges, actual_number_of_edges);
}

GTEST_TEST(ParsingInterfaceTests, Test7_esterel_plan_node_id_unique) {

    bool unique = true;
    int prevId = 1000;
    int currId;

    for (const auto & node : retrieved_esterel_plan.nodes) {
        currId = node.node_id;
        if (currId == prevId) {
            unique = false;
            break;
        } else {
            prevId = node.node_id;
        }
    }

    EXPECT_TRUE(unique);
}

GTEST_TEST(ParsingInterfaceTests, Test8_esterel_plan_edge_id_unique) {

    bool unique = true;
    int prevId = 1000;
    int currId;

    for (const auto & edge : retrieved_esterel_plan.edges) {
        std::cout << edge.edge_id << std::endl;
        currId = edge.edge_id;
        if (currId == prevId) {
            unique = false;
            break;
        } else {
            prevId = edge.edge_id;
        }
    }

    EXPECT_TRUE(unique);
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
