import importlib

from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from ros2plan_dispatch_msgs.msg import ActionFeedback

from .BaseActionInterface import BaseActionInterface


class ActionlibActionInterface(BaseActionInterface):

    def __init__(self, action_config, node: Node):
        super().__init__(action_config, node)
        self._has_default_topic = 'default_actionlib_topic' in action_config
        self._has_default_msg_type = 'default_actionlib_msg_type' in action_config
        self._has_default_goal = 'default_actionlib_goal' in action_config
        self._has_default_result = 'default_actionlib_result' in action_config
        self._action_client = {}

    def clean_action(self, plan_action_id):
        super().clean_action(plan_action_id)
        if plan_action_id in self._action_client:
            del self._action_client[plan_action_id]

    def create_action_client(self, action_server, msg_action):
        client = ActionClient(self._node, msg_action, action_server)
        while not client.wait_for_server(timeout_sec=10.0):
            if not __import__('rclpy').ok():
                return None
            self._node.get_logger().warn(
                'KCL: ({}) Waiting for action server {}'.format(
                    self._node.get_name(), action_server))
        return client

    def run(self, dispatch_msg):
        self._node.get_logger().info(
            'KCL: ({}) Interfacing action {}'.format(
                self._node.get_name(), self._action_name))

        plan_action_id = (dispatch_msg.plan_id, dispatch_msg.action_id)
        self._action_dispatch_msg[plan_action_id] = dispatch_msg
        self._action_status[plan_action_id] = ActionFeedback.ACTION_ENABLED

        action_server = None
        goal_msg_type = None

        if self._has_default_topic:
            action_server = self.parse_config_string(
                self._action_config['default_actionlib_topic'], dispatch_msg)[0]
        if self._has_default_msg_type:
            goal_msg_type = self.parse_config_string(
                self._action_config['default_actionlib_msg_type'], dispatch_msg)[0]

        override_goal = None
        override_result = None
        if 'pddl_parameters' in self._action_config:
            pddl_params = self._action_config['pddl_parameters']
            for config in self._action_config['parameter_values']:
                if self.params_match(config, pddl_params, dispatch_msg):
                    if 'actionlib_topic' in config:
                        action_server = self.parse_config_string(config['actionlib_topic'], dispatch_msg)[0]
                    if 'actionlib_msg_type' in config:
                        goal_msg_type = self.parse_config_string(config['actionlib_msg_type'], dispatch_msg)[0]
                    if 'actionlib_goal' in config:
                        override_goal = config['actionlib_goal']
                    if 'actionlib_result' in config:
                        override_result = config['actionlib_result']

        i = goal_msg_type.find('/')
        pkg_name = goal_msg_type[:i]
        msg_name = goal_msg_type[i+1:]

        pkg = importlib.import_module(pkg_name + '.action')
        msg_action = getattr(pkg, msg_name)
        goal_class = getattr(pkg, msg_name).Goal

        self._action_client[plan_action_id] = self.create_action_client(action_server, msg_action)
        if self._action_client[plan_action_id] is None:
            self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED
            return

        goal_msg = goal_class()
        if self._has_default_goal:
            for param, value in self._action_config['default_actionlib_goal'].items():
                self.populate_goal_msg(goal_msg, param, value, dispatch_msg)
        if override_goal is not None:
            for param, value in override_goal.items():
                self.populate_goal_msg(goal_msg, param, value, dispatch_msg)

        self._node.get_logger().info(
            'KCL: ({}) Plan {} Action {}: Calling action server {}'.format(
                self._node.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, action_server))

        send_goal_future = self._action_client[plan_action_id].send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda f: self._goal_response_callback(f, plan_action_id, override_result, dispatch_msg))

    def _goal_response_callback(self, future, plan_action_id, override_result, dispatch_msg):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn(
                'KCL: ({}) Plan {} Action {}: Goal rejected by server'.format(
                    self._node.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id))
            self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self.action_finished_cb(
                override_result, plan_action_id, dispatch_msg,
                f.result().status, f.result().result))

    def action_finished_cb(self, override_result, plan_action_id, dispatch_msg, status, result):
        results_correct = True
        if self._has_default_result:
            for param, value in self._action_config['default_actionlib_result'].items():
                if not override_result or param not in override_result:
                    results_correct = self.check_result_msg(result, param, value, dispatch_msg)
                    if not results_correct:
                        break

        if results_correct and override_result is not None:
            for param, value in override_result.items():
                results_correct = self.check_result_msg(result, param, value, dispatch_msg)
                if not results_correct:
                    break

        if status == GoalStatus.STATUS_SUCCEEDED and results_correct:
            self._node.get_logger().info(
                'KCL: ({}) Plan {} Action {}: ActionLib {} finished to goal state'.format(
                    self._node.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id,
                    self.get_action_name()))
            self._action_status[plan_action_id] = ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE
        else:
            self._node.get_logger().info(
                'KCL: ({}) Plan {} Action {}: ActionLib {} failed'.format(
                    self._node.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id,
                    self.get_action_name()))
            self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED
