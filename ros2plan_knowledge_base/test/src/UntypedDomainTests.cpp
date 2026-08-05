/**
 * Regression tests for untyped PDDL support in the ROS2Plan knowledge base.
 *
 * A domain without a (:types) declaration leaves the parser's type pointer null on every
 * constant, object and parameter. Reading those pointers unguarded took the node down with a
 * segmentation fault while it was still parsing the problem file, so the tests below are as
 * much about the node surviving the untyped fixture as about the values it reports: if any
 * guard is removed, the knowledge base dies during start-up and every case here fails to get
 * a response.
 */

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "ros2plan_knowledge_msgs/srv/get_domain_name_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_operator_details_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_operator_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_type_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_instance_service.hpp"

using namespace std::chrono_literals;

namespace kmsrv = ros2plan_knowledge_msgs::srv;

// PDDL's implicit supertype, which the knowledge base reports wherever a symbol carries no
// explicit type of its own.
static const char * const kImplicitType = "object";

template <typename ServiceT>
static typename ServiceT::Response::SharedPtr call_service(
    const std::string & srv_name,
    const typename ServiceT::Request::SharedPtr & req)
{
    auto node = rclcpp::Node::make_shared("untyped_domain_test_client");
    auto client = node->create_client<ServiceT>(srv_name);
    if (!client->wait_for_service(10s)) {
        return nullptr;
    }
    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, future, 10s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        return nullptr;
    }
    return future.get();
}

template <typename ServiceT>
static typename ServiceT::Response::SharedPtr call_service(const std::string & srv_name)
{
    return call_service<ServiceT>(srv_name, std::make_shared<typename ServiceT::Request>());
}

// The knowledge base does not promise an ordering for instance lists, so compare as sets.
static std::vector<std::string> sorted(std::vector<std::string> v)
{
    std::sort(v.begin(), v.end());
    return v;
}

// Reaching this at all means the node parsed the untyped domain and problem without dying.
GTEST_TEST(UntypedDomainTests, Test1_domain_name) {

    auto res = call_service<kmsrv::GetDomainNameService>("/ros2plan_knowledge_base/domain/name");

    ASSERT_TRUE(res != nullptr);
    ASSERT_EQ("untyped_test", res->domain_name);
}

// An untyped domain declares no types at all; the list is legitimately empty rather than
// implicitly containing "object".
GTEST_TEST(UntypedDomainTests, Test2_no_types_declared) {

    auto res = call_service<kmsrv::GetDomainTypeService>("/ros2plan_knowledge_base/domain/types");

    ASSERT_TRUE(res != nullptr);
    ASSERT_TRUE(res->types.empty());
}

// Objects from the problem's :objects block, which is where the crash was originally reported.
GTEST_TEST(UntypedDomainTests, Test3_untyped_objects_are_stored) {

    auto req = std::make_shared<kmsrv::GetInstanceService::Request>();
    req->type_name = "";  // no type filter: return every instance
    auto res = call_service<kmsrv::GetInstanceService>(
        "/ros2plan_knowledge_base/state/instances", req);

    ASSERT_TRUE(res != nullptr);
    ASSERT_EQ(std::vector<std::string>({"robot", "wpa", "wpb"}), sorted(res->instances));
}

// Untyped objects are filed under the implicit supertype, so querying "object" finds them.
GTEST_TEST(UntypedDomainTests, Test4_objects_filed_under_implicit_type) {

    auto req = std::make_shared<kmsrv::GetInstanceService::Request>();
    req->type_name = kImplicitType;
    auto res = call_service<kmsrv::GetInstanceService>(
        "/ros2plan_knowledge_base/state/instances", req);

    ASSERT_TRUE(res != nullptr);
    ASSERT_EQ(std::vector<std::string>({"robot", "wpa", "wpb"}), sorted(res->instances));
}

// The domain's :constants block takes a separate code path from the problem's :objects.
GTEST_TEST(UntypedDomainTests, Test5_untyped_constants_are_stored) {

    auto req = std::make_shared<kmsrv::GetInstanceService::Request>();
    req->type_name = "";
    req->include_constants = true;
    auto res = call_service<kmsrv::GetInstanceService>(
        "/ros2plan_knowledge_base/state/instances", req);

    ASSERT_TRUE(res != nullptr);
    ASSERT_EQ(std::vector<std::string>({"depot", "robot", "wpa", "wpb"}), sorted(res->instances));
}

GTEST_TEST(UntypedDomainTests, Test6_operator_parameters_use_implicit_type) {

    auto res = call_service<kmsrv::GetDomainOperatorService>(
        "/ros2plan_knowledge_base/domain/operators");

    ASSERT_TRUE(res != nullptr);
    ASSERT_EQ(1u, res->operators.size());
    ASSERT_EQ("move", res->operators[0].name);
    ASSERT_EQ(3u, res->operators[0].typed_parameters.size());
    for (const auto & param : res->operators[0].typed_parameters) {
        ASSERT_EQ(kImplicitType, param.value);
    }
}

// move's numeric effect increases (travelled) by (distance ?from ?to), and the arguments of
// that function term are read through a third code path again. Walking the operator's effects
// is what dereferences them.
GTEST_TEST(UntypedDomainTests, Test7_function_term_arguments_use_implicit_type) {

    auto req = std::make_shared<kmsrv::GetDomainOperatorDetailsService::Request>();
    req->name = "move";
    auto res = call_service<kmsrv::GetDomainOperatorDetailsService>(
        "/ros2plan_knowledge_base/domain/operator_details", req);

    ASSERT_TRUE(res != nullptr);
    ASSERT_EQ("move", res->op.formula.name);

    // A plain (non-durative) action carries no timing of its own, so accept the effect in
    // either slot rather than pinning the test to one.
    std::vector<ros2plan_knowledge_msgs::msg::DomainAssignment> assignments;
    assignments.insert(assignments.end(),
        res->op.at_start_assign_effects.begin(), res->op.at_start_assign_effects.end());
    assignments.insert(assignments.end(),
        res->op.at_end_assign_effects.begin(), res->op.at_end_assign_effects.end());
    ASSERT_FALSE(assignments.empty()) << "move should have a numeric effect on (travelled)";

    // Find "distance" on the right-hand side of the assignment.
    bool found_distance = false;
    for (const auto & assignment : assignments) {
        for (const auto & token : assignment.rhs.tokens) {
            if (token.function.name == "distance") {
                found_distance = true;
                ASSERT_EQ(2u, token.function.typed_parameters.size());
                for (const auto & param : token.function.typed_parameters) {
                    ASSERT_EQ(kImplicitType, param.value);
                }
            }
        }
    }
    ASSERT_TRUE(found_distance) << "move's effect should increase by (distance ?from ?to)";
}

GTEST_TEST(UntypedDomainTests, Test8_initial_state_and_goal) {

    auto fact_req = std::make_shared<kmsrv::GetAttributeService::Request>();
    fact_req->predicate_name = "at";
    auto facts = call_service<kmsrv::GetAttributeService>(
        "/ros2plan_knowledge_base/state/propositions", fact_req);

    ASSERT_TRUE(facts != nullptr);
    ASSERT_EQ(1u, facts->attributes.size());
    ASSERT_EQ("at", facts->attributes[0].attribute_name);

    auto goal_req = std::make_shared<kmsrv::GetAttributeService::Request>();
    goal_req->predicate_name = "";
    auto goals = call_service<kmsrv::GetAttributeService>(
        "/ros2plan_knowledge_base/state/goals", goal_req);

    ASSERT_TRUE(goals != nullptr);
    ASSERT_EQ(1u, goals->attributes.size());
    ASSERT_EQ("at", goals->attributes[0].attribute_name);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
