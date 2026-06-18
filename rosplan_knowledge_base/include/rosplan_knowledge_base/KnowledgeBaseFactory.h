#ifndef ROSPLAN_KNOWLEDGE_BASE_KNOWLEDGEBASEFACTORY_H
#define ROSPLAN_KNOWLEDGE_BASE_KNOWLEDGEBASEFACTORY_H

#include "rosplan_knowledge_base/PDDLKnowledgeBase.h"
#include "rosplan_knowledge_base/HDDLKnowledgeBase.h"

#ifdef ROSPLAN_BUILD_RDDL
#include "rosplan_knowledge_base/RDDLKnowledgeBase.h"
#endif
#ifdef ROSPLAN_BUILD_PPDDL
#include "rosplan_knowledge_base/PPDDLKnowledgeBase.h"
#endif

namespace KCL_rosplan {

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

#endif // ROSPLAN_KNOWLEDGE_BASE_KNOWLEDGEBASEFACTORY_H
