/**
 * This file parses the output of a planner and generates a list of ActionDispatch messages.
 */
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <fstream>

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rosplan_dispatch_msgs/srv/parsing_service.hpp"
#include "rosplan_dispatch_msgs/msg/action_dispatch.hpp"

#ifndef KCL_plan_parser
#define KCL_plan_parser

namespace KCL_rosplan {

    class PlanParser : public rclcpp::Node
    {
    protected:

        rclcpp::CallbackGroup::SharedPtr srv_cbg_;
        rclcpp::CallbackGroup::SharedPtr sub_cbg_;

        template<typename ClientT, typename ReqT, typename ResT>
        bool callServiceSync(std::shared_ptr<ClientT>& client,
                             std::shared_ptr<ReqT> req,
                             std::shared_ptr<ResT>& res)
        {
            if (!client->wait_for_service(std::chrono::seconds(5))) return false;
            auto future = client->async_send_request(req);
            auto status = future.wait_for(std::chrono::seconds(5));
            if (status != std::future_status::ready) return false;
            res = future.get();
            return true;
        }

        rclcpp::Service<rosplan_dispatch_msgs::srv::ParsingService>::SharedPtr parse_plan_from_file_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr parse_plan_srv_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_output_sub_;

        /* plan subscription */
        std::string planner_output;
        bool planner_output_received;
        double planner_output_time;

        std::vector<rosplan_dispatch_msgs::msg::ActionDispatch> action_list;

        virtual void reset() = 0;
        virtual void preparePlan() = 0;
        virtual void publishPlan() = 0;

    public:

        explicit PlanParser(const std::string& node_name);

        void plannerCallback(const std_msgs::msg::String::SharedPtr plannerOutput);

        void parsePlanFromFile(
            const std::shared_ptr<rosplan_dispatch_msgs::srv::ParsingService::Request> req,
            std::shared_ptr<rosplan_dispatch_msgs::srv::ParsingService::Response> res);

        void parsePlan(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res);
    };

} // close namespace

#endif
