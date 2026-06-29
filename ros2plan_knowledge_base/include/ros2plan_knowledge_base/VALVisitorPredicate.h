#ifndef KCL_valvisitor_predicate
#define KCL_valvisitor_predicate

#include <sstream>
#include <string>
#include <vector>

#include "VALfiles/ptree.h"
#include "VALfiles/VisitController.h"

#include "ros2plan_knowledge_msgs/msg/domain_formula.hpp"

namespace KCL_ros2plan {

    class VALVisitorPredicate : public VAL1_2::VisitController
    {
    public:
        ros2plan_knowledge_msgs::msg::DomainFormula msg;

        virtual void visit_pred_decl(VAL1_2::pred_decl *);
    };

} // namespace KCL_ros2plan
#endif
