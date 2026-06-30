/**
 *
 * Copyright [2019] <KCL King's College London>
 *
 * Author: Martin Koling (martinh.koling@kcl.ac.uk)
 * Author: Oscar Lima (oscar.lima@dfki.de)
 *
 * Unit tests for ROS2Plan plan dispatch (ROS2 port)
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include "ros2plan_dispatch_msgs/srv/dispatch_service.hpp"
#include "ros2plan_dispatch_msgs/msg/action_dispatch.hpp"
#include "ros2plan_dispatch_msgs/msg/action_feedback.hpp"

using namespace std::chrono_literals;

using Empty = std_srvs::srv::Empty;
using DispatchService = ros2plan_dispatch_msgs::srv::DispatchService;
using ActionDispatch = ros2plan_dispatch_msgs::msg::ActionDispatch;
using ActionFeedback = ros2plan_dispatch_msgs::msg::ActionFeedback;

// In ROS2 every interface node advertises its services/topics with relative names and the default
// (empty) namespace, so they resolve to the root namespace (the node *name* is not a namespace).
// These are the actual fully-resolved names the running nodes register.
static const char * kProblemSrv  = "/problem_generation_server";   // std_srvs/Empty (ProblemInterface)
static const char * kPlanningSrv = "/planning_server";             // std_srvs/Empty (PlannerInterface)
static const char * kParseSrv    = "/parse_plan";                  // std_srvs/Empty (PDDLEsterelPlanParser)
static const char * kDispatchSrv = "/dispatch_plan";               // DispatchService (PlanDispatcher)

static const char * kActionDispatchTopic = "/action_dispatch";     // ActionDispatch (transient_local)
static const char * kActionFeedbackTopic = "/action_feedback";     // ActionFeedback (transient_local)
static const char * kPlanGraphTopic      = "/plan_graph";          // std_msgs/String (transient_local)

// Calls a std_srvs/Empty service synchronously, returning true once the call completes.
static bool call_empty_service(const rclcpp::Node::SharedPtr & node, const std::string & srv_name)
{
    auto client = node->create_client<Empty>(srv_name);
    if (!client->wait_for_service(10s)) {
        return false;
    }
    auto req = std::make_shared<Empty::Request>();
    auto future = client->async_send_request(req);
    return rclcpp::spin_until_future_complete(node, future, 10s) ==
           rclcpp::FutureReturnCode::SUCCESS;
}

// Runs the full planning pipeline up to (and including) the dispatch_plan service call:
//   problem_generation_server -> planning_server -> parse_plan -> dispatch_plan
// On success the DispatchService response is returned via `out_response`.
static bool run_pipeline_and_dispatch(
    const rclcpp::Node::SharedPtr & node,
    DispatchService::Response & out_response)
{
    if (!call_empty_service(node, kProblemSrv)) {
        return false;
    }
    rclcpp::sleep_for(800ms);

    if (!call_empty_service(node, kPlanningSrv)) {
        return false;
    }
    rclcpp::sleep_for(800ms);

    if (!call_empty_service(node, kParseSrv)) {
        return false;
    }
    rclcpp::sleep_for(800ms);

    auto dispatch_client = node->create_client<DispatchService>(kDispatchSrv);
    if (!dispatch_client->wait_for_service(10s)) {
        return false;
    }
    auto req = std::make_shared<DispatchService::Request>();
    auto future = dispatch_client->async_send_request(req);
    // /dispatch_plan blocks until the whole plan has been dispatched and executed, so allow
    // generous time for the full turtlebot plan to run through the simulated actions.
    if (rclcpp::spin_until_future_complete(node, future, 120s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        return false;
    }
    out_response = *future.get();
    return true;
}

// The whole pipeline (problem -> plan -> parse -> dispatch) is run ONCE and its results cached.
// Running it per-test would not work: the simulated actions update the knowledge base, so after the
// first dispatch the goal is already achieved and a re-run produces an empty plan. The driver below
// subscribes to action_dispatch / action_feedback / plan_graph BEFORE dispatching (volatile QoS,
// which is compatible with any publisher durability) and spins while the blocking dispatch runs, so
// it collects every message of the single dispatch live. Each test then reads the cached results.
struct DispatchOnce {
    bool ran = false;
    bool pipeline_ok = false;
    DispatchService::Response response;
    std::vector<ActionDispatch> actions;
    std::vector<ActionFeedback> feedback;
    std::vector<std::string> graphs;
};

static DispatchOnce & dispatched()
{
    static DispatchOnce s;
    if (!s.ran) {
        s.ran = true;
        auto node = rclcpp::Node::make_shared("plan_dispatch_test_driver");
        auto ad_sub = node->create_subscription<ActionDispatch>(
            kActionDispatchTopic, rclcpp::QoS(1000),
            [&](ActionDispatch::SharedPtr m) { s.actions.push_back(*m); });
        auto af_sub = node->create_subscription<ActionFeedback>(
            kActionFeedbackTopic, rclcpp::QoS(1000),
            [&](ActionFeedback::SharedPtr m) { s.feedback.push_back(*m); });
        auto pg_sub = node->create_subscription<std_msgs::msg::String>(
            kPlanGraphTopic, rclcpp::QoS(1000),
            [&](std_msgs::msg::String::SharedPtr m) { s.graphs.push_back(m->data); });

        // Let the subscriptions match the publishers before driving the pipeline.
        rclcpp::sleep_for(500ms);
        s.pipeline_ok = run_pipeline_and_dispatch(node, s.response);

        // Spin a little longer to collect any trailing feedback.
        auto start = node->now();
        while ((node->now() - start) < rclcpp::Duration(3s)) {
            rclcpp::spin_some(node);
            rclcpp::sleep_for(100ms);
        }
    }
    return s;
}

GTEST_TEST(PlanDispatchTests, Test1_plan_dispatch) {
    auto & state = dispatched();
    ASSERT_TRUE(state.pipeline_ok);
    EXPECT_TRUE(state.response.success);
    EXPECT_TRUE(state.response.goal_achieved);
}

GTEST_TEST(PlanDispatchTests, Test2_published_action_dispatch) {
    auto & state = dispatched();
    ASSERT_TRUE(state.pipeline_ok);

    // The dispatcher dispatched the plan's actions; verify they were published with sane fields.
    ASSERT_FALSE(state.actions.empty());
    const ActionDispatch & last = state.actions.back();
    EXPECT_GE(last.action_id, 0);
    EXPECT_GT(last.duration, 0.0f);
    EXPECT_FALSE(last.name.empty());
}

GTEST_TEST(PlanDispatchTests, Test3_published_action_feedback) {
    auto & state = dispatched();
    ASSERT_TRUE(state.pipeline_ok);

    // Every dispatched action produces feedback (enabled + completed); there must be some.
    ASSERT_FALSE(state.feedback.empty());
}

GTEST_TEST(PlanDispatchTests, Test4_published_plan_graph) {
    auto & state = dispatched();
    ASSERT_TRUE(state.pipeline_ok);

    ASSERT_FALSE(state.graphs.empty());
    // The esterel dispatcher publishes the plan graph (Graphviz digraph) for the dispatched plan.
    // The full temporal graph is plan-specific, so check it is a well-formed graph that contains
    // the plan-start node and the actions of the turtlebot plan rather than matching it verbatim.
    const std::string & graph = state.graphs.back();
    EXPECT_NE(graph.find("digraph plan {"), std::string::npos);
    EXPECT_NE(graph.find("plan_start"), std::string::npos);
    EXPECT_NE(graph.find("undock"), std::string::npos);
    EXPECT_NE(graph.find("localise"), std::string::npos);
    EXPECT_NE(graph.find("goto_waypoint"), std::string::npos);
    EXPECT_NE(graph.find("dock_end"), std::string::npos);
}

// Run all the tests that were declared with GTEST_TEST()
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
