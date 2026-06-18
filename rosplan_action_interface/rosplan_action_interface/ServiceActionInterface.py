import importlib
import threading

import rclpy
from rclpy.node import Node
from rosplan_dispatch_msgs.msg import ActionFeedback

from .BaseActionInterface import BaseActionInterface


class ServiceActionInterface(BaseActionInterface):

    def __init__(self, action_config, node: Node):
        super().__init__(action_config, node)
        self._has_default_topic = 'default_service' in action_config
        self._has_default_msg_type = 'default_service_type' in action_config
        self._has_default_request = 'default_service_request' in action_config
        self._has_default_response = 'default_service_response' in action_config
        # Satellite node for blocking service calls from a non-executor thread
        self._svc_node = Node('rosplan_service_action_svc_node')

    def run(self, dispatch_msg):
        t = threading.Thread(target=self.run_thread, args=(dispatch_msg,))
        t.start()

    def run_thread(self, dispatch_msg):
        self._node.get_logger().info(
            'KCL: ({}) Interfacing service {}'.format(
                self._node.get_name(), self._action_name))

        plan_action_id = (dispatch_msg.plan_id, dispatch_msg.action_id)
        self._action_dispatch_msg[plan_action_id] = dispatch_msg
        self._action_status[plan_action_id] = ActionFeedback.ACTION_ENABLED

        action_server = None
        goal_msg_type = None

        if self._has_default_topic:
            action_server = self.parse_config_string(
                self._action_config['default_service'], dispatch_msg)[0]
        if self._has_default_msg_type:
            goal_msg_type = self.parse_config_string(
                self._action_config['default_service_type'], dispatch_msg)[0]

        override_request = None
        override_result = None
        if 'pddl_parameters' in self._action_config:
            pddl_params = self._action_config['pddl_parameters']
            for config in self._action_config['parameter_values']:
                if self.params_match(config, pddl_params, dispatch_msg):
                    if 'service' in config:
                        action_server = self.parse_config_string(config['service'], dispatch_msg)[0]
                    if 'service_type' in config:
                        goal_msg_type = self.parse_config_string(config['service_type'], dispatch_msg)[0]
                    if 'service_request' in config:
                        override_request = config['service_request']
                    if 'service_response' in config:
                        override_result = config['service_response']

        i = goal_msg_type.find('/')
        pkg_name = goal_msg_type[:i]
        msg_name = goal_msg_type[i+1:]

        pkg = importlib.import_module(pkg_name + '.srv')
        srv_type = getattr(pkg, msg_name)

        client = self._svc_node.create_client(srv_type, action_server)
        if not client.wait_for_service(timeout_sec=10.0):
            self._node.get_logger().warn(
                'KCL: ({}) Plan {} Action {}: Service {} not available'.format(
                    self._node.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id,
                    action_server))
            self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED
            return

        request = srv_type.Request()

        if self._has_default_request:
            for param, value in self._action_config['default_service_request'].items():
                self.populate_goal_msg(request, param, value, dispatch_msg)
        if override_request:
            for param, value in override_request.items():
                self.populate_goal_msg(request, param, value, dispatch_msg)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._svc_node, future, timeout_sec=10.0)

        if not future.done():
            self._node.get_logger().warn(
                'KCL: ({}) Plan {} Action {}: Service {} timed out'.format(
                    self._node.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id,
                    self.get_action_name()))
            self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED
            return

        result = future.result()
        if result is None:
            self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED
            return

        results_correct = True
        if self._has_default_response:
            for param, value in self._action_config['default_service_response'].items():
                if not override_result or param not in override_result:
                    results_correct = self.check_result_msg(result, param, value, dispatch_msg)
                    if not results_correct:
                        break
        if results_correct and override_result:
            for param, value in override_result.items():
                results_correct = self.check_result_msg(result, param, value, dispatch_msg)
                if not results_correct:
                    break

        if results_correct:
            self._node.get_logger().info(
                'KCL: ({}) Plan {} Action {}: Service {} finished to goal state'.format(
                    self._node.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id,
                    self.get_action_name()))
            self._action_status[plan_action_id] = ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE
        else:
            self._node.get_logger().info(
                'KCL: ({}) Plan {} Action {}: Service {} failed'.format(
                    self._node.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id,
                    self.get_action_name()))
            self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED
