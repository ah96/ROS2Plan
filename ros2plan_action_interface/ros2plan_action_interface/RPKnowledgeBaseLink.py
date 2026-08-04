from collections import defaultdict

import rclpy
from rclpy.node import Node

from ros2plan_knowledge_msgs.srv import (
    KnowledgeUpdateServiceArray,
    GetDomainPredicateDetailsService,
    GetDomainOperatorDetailsService,
    GetDomainAttributeService,
    GetAttributeService,
    GetInstanceService,
)
from ros2plan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue


class RPKnowledgeBaseLink:

    AT_START = 0
    AT_END = 1

    def __init__(self, node: Node):
        self._node = node
        # Satellite node not added to any executor — used for blocking service calls
        # from non-executor threads without risk of deadlock.
        # See ServiceActionInterface: immune to the process-wide `-r __node:=` remap.
        self._svc_node = Node('rp_kb_link_svc_node', use_global_arguments=False)

        kb = '/ros2plan_knowledge_base'
        self._update_kb_client = self._svc_node.create_client(
            KnowledgeUpdateServiceArray, kb + '/update_array')
        self._get_predicates_client = self._svc_node.create_client(
            GetDomainPredicateDetailsService, kb + '/domain/predicate_details')
        self._get_operator_details_client = self._svc_node.create_client(
            GetDomainOperatorDetailsService, kb + '/domain/operator_details')
        self._get_functions_client = self._svc_node.create_client(
            GetDomainAttributeService, kb + '/domain/functions')
        self._get_instances_client = self._svc_node.create_client(
            GetInstanceService, kb + '/state/instances')
        self._get_state_propositions_client = self._svc_node.create_client(
            GetAttributeService, kb + '/state/propositions')
        self._get_state_functions_client = self._svc_node.create_client(
            GetAttributeService, kb + '/state/functions')

        self.sensed_predicates = defaultdict(bool)

    def _call_service(self, client, request, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._svc_node, future, timeout_sec=timeout_sec)
        return future.result() if future.done() else None

    # ===================== #
    # action effect methods #
    # ===================== #

    def pack_simple_effect(self, effect, bound_parameters, kus, add_effect):
        if self.sensed_predicates[effect.name]:
            return
        pred_details = self.get_kb_predicate_details(effect.name)
        if pred_details is not None:
            self.sensed_predicates[effect.name] = pred_details.predicate.is_sensed
            if self.sensed_predicates[effect.name]:
                return
        item = self.ground_simple_effect(effect, bound_parameters)
        kus.knowledge.append(item)
        kus.update_type.append(
            KnowledgeUpdateServiceArray.Request.ADD_KNOWLEDGE if add_effect
            else KnowledgeUpdateServiceArray.Request.REMOVE_KNOWLEDGE)

    def kb_apply_action_effects(self, pddl_action_msg, effect_time):
        kus = KnowledgeUpdateServiceArray.Request()
        operator_name = pddl_action_msg.name.split(' ')[0]
        operator_details = self.get_kb_operator_details(operator_name)
        if not operator_details:
            self._node.get_logger().error(
                'KCL: ({}) error fetching operator details from KB: {}'.format(
                    self._node.get_name(), operator_name))
            return
        bound_parameters = {}
        for op_param in operator_details.formula.typed_parameters:
            for p in pddl_action_msg.parameters:
                if op_param.key == p.key:
                    bound_parameters[p.key] = p.value
                    break
            else:
                self._node.get_logger().error(
                    'KCL: ({}) aborting applying action effect — missing parameter {}'.format(
                        self._node.get_name(), op_param.key))
                return
        if effect_time == self.AT_START:
            for eff in operator_details.at_start_add_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=True)
            for eff in operator_details.at_start_del_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=False)
        if effect_time == self.AT_END:
            for eff in operator_details.at_end_add_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=True)
            for eff in operator_details.at_end_del_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=False)
        self.update_kb(kus)

    def kb_undo_action_effects(self, pddl_action_msg, effect_time):
        kus = KnowledgeUpdateServiceArray.Request()
        operator_name = pddl_action_msg.name.split(' ')[0]
        operator_details = self.get_kb_operator_details(operator_name)
        if not operator_details:
            return
        bound_parameters = {}
        for op_param in operator_details.formula.typed_parameters:
            for p in pddl_action_msg.parameters:
                if op_param.key == p.key:
                    bound_parameters[p.key] = p.value
                    break
        if effect_time == self.AT_START:
            for eff in operator_details.at_start_add_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=False)
            for eff in operator_details.at_start_del_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=True)
        if effect_time == self.AT_END:
            for eff in operator_details.at_end_add_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=False)
            for eff in operator_details.at_end_del_effects:
                self.pack_simple_effect(eff, bound_parameters, kus, add_effect=True)
        self.update_kb(kus)

    def ground_simple_effect(self, effect, bound_parameters):
        item = KnowledgeItem()
        item.knowledge_type = KnowledgeItem.FACT
        item.attribute_name = effect.name
        for tp in effect.typed_parameters:
            pair = KeyValue()
            pair.key = tp.key
            pair.value = bound_parameters.get(tp.key, tp.key)
            item.values.append(pair)
        return item

    # ================================== #
    # update service(s) to knowledge base #
    # ================================== #

    def update_kb(self, kus):
        if len(kus.update_type) == 0:
            return
        result = self._call_service(self._update_kb_client, kus)
        if result is None:
            self._node.get_logger().error(
                'KCL: ({}) Failed to update knowledge base'.format(self._node.get_name()))

    # =============================== #
    # request data from knowledge base #
    # =============================== #

    def get_kb_attribute(self, attribute_name):
        req = GetAttributeService.Request()
        req.predicate_name = attribute_name
        ret = self._call_service(self._get_state_propositions_client, req)
        if ret and len(ret.attributes) > 0:
            return ret.attributes
        ret2 = self._call_service(self._get_state_functions_client, req)
        if ret2:
            return ret2.attributes
        return None

    def get_kb_operator_details(self, operator_name):
        req = GetDomainOperatorDetailsService.Request()
        req.name = operator_name
        ret = self._call_service(self._get_operator_details_client, req)
        return ret.op if ret else None

    def get_kb_predicate_details(self, predicate_name):
        req = GetDomainPredicateDetailsService.Request()
        req.name = predicate_name
        return self._call_service(self._get_predicates_client, req)

    def get_kb_function_details(self, func_name):
        req = GetDomainPredicateDetailsService.Request()
        req.name = func_name
        return self._call_service(self._get_functions_client, req)

    def get_kb_instances(self, type_name):
        req = GetInstanceService.Request()
        req.type_name = type_name
        ret = self._call_service(self._get_instances_client, req)
        return ret.instances if ret else None
