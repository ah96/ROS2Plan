#ifndef ROS2PLAN_KNOWLEDGE_BASE_KNOWLEDGEBASEFACTORY_H
#define ROS2PLAN_KNOWLEDGE_BASE_KNOWLEDGEBASEFACTORY_H

#include "ros2plan_knowledge_base/PDDLKnowledgeBase.h"
#include "ros2plan_knowledge_base/HDDLKnowledgeBase.h"

#ifdef ROS2PLAN_BUILD_RDDL
#include "ros2plan_knowledge_base/RDDLKnowledgeBase.h"
#endif
#ifdef ROS2PLAN_BUILD_PPDDL
#include "ros2plan_knowledge_base/PPDDLKnowledgeBase.h"
#endif

namespace KCL_ros2plan {

    typedef std::shared_ptr<KnowledgeBase> KnowledgeBasePtr;

    class KnowledgeBaseFactory {
    public:
        enum KB {
            PDDL,
            PPDDL,
            RDDL,
            HDDL,
            CHIMP
        };

        static KnowledgeBasePtr createKB(KB kb_type);
    };
}

#endif // ROS2PLAN_KNOWLEDGE_BASE_KNOWLEDGEBASEFACTORY_H
