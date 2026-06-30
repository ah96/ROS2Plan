/**
 *
 * Copyright [2019] <KCL King's College London>
 *
 * Author: Martin Koling (martinh.koling@kcl.ac.uk)
 * Author: Oscar Lima (oscar.lima@dfki.de)
 *
 * Unit tests for ROS2Plan knowledge base (ROS2 port)
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "ros2plan_knowledge_msgs/srv/get_domain_name_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_operator_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_operator_details_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_predicate_details_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_domain_type_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_instance_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_attribute_service.hpp"
#include "ros2plan_knowledge_msgs/srv/get_metric_service.hpp"
#include "ros2plan_knowledge_msgs/srv/knowledge_update_service.hpp"
#include "ros2plan_knowledge_msgs/srv/knowledge_update_service_array.hpp"
#include "ros2plan_knowledge_msgs/srv/knowledge_query_service.hpp"
#include "ros2plan_knowledge_msgs/msg/knowledge_item.hpp"
#include "ros2plan_knowledge_msgs/msg/domain_formula.hpp"

using namespace std::chrono_literals;

namespace kmsrv = ros2plan_knowledge_msgs::srv;
namespace kmmsg = ros2plan_knowledge_msgs::msg;

// In ROS2 the knowledge base advertises its services with the "~/" prefix, so with the node named
// "ros2plan_knowledge_base" they resolve to node-namespaced paths such as
// "/ros2plan_knowledge_base/domain/name". These are the same paths the original ROS1 test used.

// Calls a typed service synchronously on a freshly created node. Returns the response shared pointer
// on success, or nullptr if the service is unavailable or the call did not complete in time.
template <typename ServiceT>
static typename ServiceT::Response::SharedPtr call_service(
    const std::string & srv_name,
    const typename ServiceT::Request::SharedPtr & req)
{
    auto node = rclcpp::Node::make_shared("knowledge_base_test_client");
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

// Convenience overload for services whose request carries no caller-set fields (a default-constructed
// request is sent).
template <typename ServiceT>
static typename ServiceT::Response::SharedPtr call_service(const std::string & srv_name)
{
    return call_service<ServiceT>(srv_name, std::make_shared<typename ServiceT::Request>());
}

GTEST_TEST(KnowledgeBaseTests, Test1_domain_name) {

    std::string srv_name = "/ros2plan_knowledge_base/domain/name";
    auto res = call_service<kmsrv::GetDomainNameService>(srv_name);

    ASSERT_TRUE(res != nullptr);
    ASSERT_EQ("driverlog-simple", res->domain_name);
}

GTEST_TEST(KnowledgeBaseTests, Test2_domain_functions) {

    std::string srv_name = "/ros2plan_knowledge_base/domain/functions";
    auto res = call_service<kmsrv::GetDomainAttributeService>(srv_name);

    ASSERT_TRUE(res != nullptr);
    // Amazon-domain does not contain any functions
    EXPECT_FALSE(res->items.size());
}

GTEST_TEST(KnowledgeBaseTests, Test3_domain_predicates) {

    std::string srv_name = "/ros2plan_knowledge_base/domain/predicates";

    std::vector<std::string> expected_predicate_names = {"at", "in", "link", "path"};

    auto res = call_service<kmsrv::GetDomainAttributeService>(srv_name);
    ASSERT_TRUE(res != nullptr);

    std::vector<std::string> actual_predicate_names;

    for (auto rit=res->items.begin(); rit!=res->items.end(); rit++) {
        actual_predicate_names.push_back(rit->name);
    }

    ASSERT_EQ(expected_predicate_names, actual_predicate_names);
}

GTEST_TEST(KnowledgeBaseTests, Test4_domain_operators) {

    std::string srv_name = "/ros2plan_knowledge_base/domain/operators";

    std::vector<std::string> expected_operator_names = {"load-truck", "unload-truck", "board-truck", "get-out", "drive-truck", "walk"};

    auto res = call_service<kmsrv::GetDomainOperatorService>(srv_name);
    ASSERT_TRUE(res != nullptr);

    std::vector<std::string> actual_operator_names;

    for (auto rit=res->operators.begin(); rit!=res->operators.end(); rit++) {
        actual_operator_names.push_back(rit->name);
    }

    ASSERT_EQ(expected_operator_names, actual_operator_names);
}

GTEST_TEST(KnowledgeBaseTests, Test5_domain_types){

    std::string srv_name = "/ros2plan_knowledge_base/domain/types";

    std::vector<std::string> expected_type_names = {"place", "locatable", "driver", "truck", "item"};
    std::vector<std::string> expected_super_type_names = {"object", "object", "locatable", "locatable", "locatable"};

    auto res = call_service<kmsrv::GetDomainTypeService>(srv_name);
    ASSERT_TRUE(res != nullptr);

    std::vector<std::string> actual_type_names = res->types;
    std::vector<std::string> actual_super_type_names = res->super_types;

    ASSERT_EQ(expected_type_names, actual_type_names);
    ASSERT_EQ(expected_super_type_names, actual_super_type_names);
}

GTEST_TEST(KnowledgeBaseTests, Test6_domain_operator_details) {

    std::string srv_name = "/ros2plan_knowledge_base/domain/operator_details";

    auto req = std::make_shared<kmsrv::GetDomainOperatorDetailsService::Request>();
    req->name = "load-truck";

    auto res = call_service<kmsrv::GetDomainOperatorDetailsService>(srv_name, req);
    ASSERT_TRUE(res != nullptr);

    std::string detailed_operator_name = res->op.formula.name;

    ASSERT_EQ(req->name, detailed_operator_name);
}

GTEST_TEST(KnowledgeBaseTests, Test7_domain_predicate_details) {

    std::string srv_name = "/ros2plan_knowledge_base/domain/predicate_details";

    auto req = std::make_shared<kmsrv::GetDomainPredicateDetailsService::Request>();
    req->name = "path";

    auto res = call_service<kmsrv::GetDomainPredicateDetailsService>(srv_name, req);
    ASSERT_TRUE(res != nullptr);

    std::string detailed_predicate_name = res->predicate.name;

    ASSERT_EQ(req->name, detailed_predicate_name);
}

GTEST_TEST(KnowledgeBaseTests, Test8_state_instances){

    std::string srv_name = "/ros2plan_knowledge_base/state/instances";

    std::vector<std::string> expected_instance_names = {"home", "amazon", "london", "myhouse"};

    auto req = std::make_shared<kmsrv::GetInstanceService::Request>();
    req->type_name = "place";

    auto res = call_service<kmsrv::GetInstanceService>(srv_name, req);
    ASSERT_TRUE(res != nullptr);

    std::vector<std::string> actual_instance_names = res->instances;

    ASSERT_EQ(expected_instance_names, actual_instance_names);
}

GTEST_TEST(KnowledgeBaseTests, Test9_state_functions){

    std::string srv_name = "/ros2plan_knowledge_base/state/functions";

    auto req = std::make_shared<kmsrv::GetAttributeService::Request>();
    req->predicate_name = "link";

    auto res = call_service<kmsrv::GetAttributeService>(srv_name, req);
    ASSERT_TRUE(res != nullptr);

    // Amazon-domain does not contain any functions
    EXPECT_FALSE(res->attributes.size());
}

GTEST_TEST(KnowledgeBaseTests, Test10_state_propositions){

    std::string srv_name = "/ros2plan_knowledge_base/state/propositions";

    // Expected to be 4 propositions with name matching "link"
    std::vector<std::string> expected_propositions_names = {"link", "link", "link", "link"};

    auto req = std::make_shared<kmsrv::GetAttributeService::Request>();
    req->predicate_name = "link";

    auto res = call_service<kmsrv::GetAttributeService>(srv_name, req);
    ASSERT_TRUE(res != nullptr);

    std::vector<std::string> actual_propositions_names;

    for (auto rit=res->attributes.begin(); rit!=res->attributes.end(); rit++) {
        actual_propositions_names.push_back(rit->attribute_name);
    }

    ASSERT_EQ(expected_propositions_names, actual_propositions_names);
}

GTEST_TEST(KnowledgeBaseTests, Test11_state_goals){

    std::string srv_name = "/ros2plan_knowledge_base/state/goals";

    //Regardless of request data the response should be the same.
    {
        auto req = std::make_shared<kmsrv::GetAttributeService::Request>();
        req->predicate_name = "at";
        auto res = call_service<kmsrv::GetAttributeService>(srv_name, req);
        ASSERT_TRUE(res != nullptr);
        ASSERT_EQ("at", res->attributes[0].attribute_name);
    }

    {
        auto req = std::make_shared<kmsrv::GetAttributeService::Request>();
        req->predicate_name = "";
        auto res = call_service<kmsrv::GetAttributeService>(srv_name, req);
        ASSERT_TRUE(res != nullptr);
        ASSERT_EQ("at", res->attributes[0].attribute_name);
        // check for size of arguments to be equal to a known value (there is 1 goal in the problem file)
        EXPECT_TRUE(res->attributes.size());
    }

    {
        auto req = std::make_shared<kmsrv::GetAttributeService::Request>();
        req->predicate_name = "dgfhfhjfgh";
        auto res = call_service<kmsrv::GetAttributeService>(srv_name, req);
        ASSERT_TRUE(res != nullptr);
        ASSERT_EQ("at", res->attributes[0].attribute_name);
    }
}

GTEST_TEST(KnowledgeBaseTests, Test12_state_metric){

    std::string srv_name = "/ros2plan_knowledge_base/state/metric";

    auto res = call_service<kmsrv::GetMetricService>(srv_name);
    ASSERT_TRUE(res != nullptr);

    // Amazon-problem does not have any metric

    ASSERT_EQ("", res->metric.attribute_name);
}

GTEST_TEST(KnowledgeBaseTests, Test13_state_til){

    std::string srv_name = "/ros2plan_knowledge_base/state/timed_knowledge";

    auto req = std::make_shared<kmsrv::GetAttributeService::Request>();
    req->predicate_name = "at";

    auto res = call_service<kmsrv::GetAttributeService>(srv_name, req);
    ASSERT_TRUE(res != nullptr);

    // Amazon-problem does not have timed-initial-literals(TILs).

    EXPECT_FALSE(res->attributes.size());
}

GTEST_TEST(KnowledgeBaseTests, Test14_clear){

    std::string srv_name1 = "/ros2plan_knowledge_base/state/goals";

    {
        auto req = std::make_shared<kmsrv::GetAttributeService::Request>();
        req->predicate_name = "anystring";
        auto res = call_service<kmsrv::GetAttributeService>(srv_name1, req);
        ASSERT_TRUE(res != nullptr);
        // Check that goal attribute_name is "at"
        ASSERT_EQ("at", res->attributes[0].attribute_name);
    }

    std::string srv_name2 = "/ros2plan_knowledge_base/clear";
    auto clear_res = call_service<std_srvs::srv::Empty>(srv_name2);
    ASSERT_TRUE(clear_res != nullptr);

    {
        auto req = std::make_shared<kmsrv::GetAttributeService::Request>();
        req->predicate_name = "anystring";
        auto res = call_service<kmsrv::GetAttributeService>(srv_name1, req);
        ASSERT_TRUE(res != nullptr);
        // Check that no goal exists after knowledge base is cleared
        EXPECT_FALSE(res->attributes.size());
    }
}

GTEST_TEST(KnowledgeBaseTests, Test15_update){

    std::string srv_name1 = "/ros2plan_knowledge_base/clear";
    auto clear_res = call_service<std_srvs::srv::Empty>(srv_name1);
    ASSERT_TRUE(clear_res != nullptr);

    std::string srv_name2 = "/ros2plan_knowledge_base/update";

    kmmsg::KnowledgeItem knowledgeItem;

    knowledgeItem.knowledge_type = 0;
    knowledgeItem.instance_type = "driver";
    // knowledgeItem.instance_name = "Martin"; // TODO: uncomment when #199 is solved
    knowledgeItem.instance_name = "martin"; // TODO: delete once #199 is solved

    auto req = std::make_shared<kmsrv::KnowledgeUpdateService::Request>();
    req->update_type = kmsrv::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
    req->knowledge = knowledgeItem;

    auto res = call_service<kmsrv::KnowledgeUpdateService>(srv_name2, req);
    ASSERT_TRUE(res != nullptr);

    EXPECT_TRUE(res->success);

    std::string srv_name3 = "/ros2plan_knowledge_base/state/instances";

    std::vector<std::string> expected_added_instance = {"martin"};

    auto instanceReq = std::make_shared<kmsrv::GetInstanceService::Request>();
    instanceReq->type_name = "driver";

    auto instanceRes = call_service<kmsrv::GetInstanceService>(srv_name3, instanceReq);
    ASSERT_TRUE(instanceRes != nullptr);

    ASSERT_EQ(expected_added_instance, instanceRes->instances);
}

GTEST_TEST(KnowledgeBaseTests, Test16_update_array){

    std::string srv_name1 = "/ros2plan_knowledge_base/clear";
    auto clear_res = call_service<std_srvs::srv::Empty>(srv_name1);
    ASSERT_TRUE(clear_res != nullptr);

    std::string srv_name2 = "/ros2plan_knowledge_base/update_array";

    kmmsg::KnowledgeItem knowledgeItem1;
    kmmsg::KnowledgeItem knowledgeItem2;
    kmmsg::KnowledgeItem knowledgeItem3;

    knowledgeItem1.knowledge_type = 0;
    knowledgeItem1.instance_type = "driver";
    knowledgeItem1.instance_name = "Martin";

    knowledgeItem2.knowledge_type = 0;
    knowledgeItem2.instance_type = "place";
    knowledgeItem2.instance_name = "home";

    knowledgeItem3.knowledge_type = 0;
    knowledgeItem3.instance_type = "place";
    knowledgeItem3.instance_name = "london";

    auto req = std::make_shared<kmsrv::KnowledgeUpdateServiceArray::Request>();
    req->update_type = {0, 0, 0};
    req->knowledge = {knowledgeItem1, knowledgeItem2, knowledgeItem3};

    auto res = call_service<kmsrv::KnowledgeUpdateServiceArray>(srv_name2, req);
    ASSERT_TRUE(res != nullptr);

    EXPECT_TRUE(res->success);
}

GTEST_TEST(KnowledgeBaseTests, Test17_query){

    std::string srv_name = "/ros2plan_knowledge_base/query_state";

    kmmsg::KnowledgeItem knowledgeItem1;
    kmmsg::KnowledgeItem knowledgeItem2;
    kmmsg::KnowledgeItem knowledgeItem3;

    knowledgeItem1.knowledge_type = 0;
    knowledgeItem1.instance_type = "driver";
    knowledgeItem1.instance_name = "Martin";

    knowledgeItem2.knowledge_type = 0;
    knowledgeItem2.instance_type = "place";
    knowledgeItem2.instance_name = "home";

    knowledgeItem3.knowledge_type = 0;
    knowledgeItem3.instance_type = "driver";
    knowledgeItem3.instance_name = "Jonas";

    auto req = std::make_shared<kmsrv::KnowledgeQueryService::Request>();
    req->knowledge = {knowledgeItem1, knowledgeItem2, knowledgeItem3};

    auto res = call_service<kmsrv::KnowledgeQueryService>(srv_name, req);
    ASSERT_TRUE(res != nullptr);

    EXPECT_FALSE(res->all_true);

    std::vector<bool> expected_results = {true, true, false};

    std::vector<bool> actual_results;

    for (auto rit=res->results.begin(); rit!=res->results.end(); rit++) {
        actual_results.push_back(*rit);
    }

    ASSERT_EQ(expected_results, actual_results);

    ASSERT_EQ("Jonas", res->false_knowledge[0].instance_name);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
