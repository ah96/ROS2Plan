#include "ros2plan_knowledge_base/KnowledgeBaseFactory.h"
#include <rclcpp/rclcpp.hpp>

namespace KCL_ros2plan {

    KnowledgeBasePtr KnowledgeBaseFactory::createKB(KnowledgeBaseFactory::KB kb_type) {
        if (kb_type == KB::PDDL)
            return std::make_shared<PDDLKnowledgeBase>();
#ifdef ROS2PLAN_BUILD_PPDDL
        else if (kb_type == KB::PPDDL)
            return std::make_shared<PPDDLKnowledgeBase>();
#endif
#ifdef ROS2PLAN_BUILD_RDDL
        else if (kb_type == KB::RDDL)
            return std::make_shared<RDDLKnowledgeBase>();
#endif
        else if (kb_type == KB::HDDL || kb_type == KB::CHIMP)
            return std::make_shared<HDDLKnowledgeBase>();

        RCLCPP_ERROR(rclcpp::get_logger("ros2plan_knowledge_base"),
                     "KCL: Unknown Knowledge Base type.");
        rclcpp::shutdown();
        return nullptr;
    }

} // namespace KCL_ros2plan
