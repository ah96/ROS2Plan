import yaml

import rclpy
from rclpy.node import Node

from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback

from .RPKnowledgeBaseLink import RPKnowledgeBaseLink
from .ActionlibActionInterface import ActionlibActionInterface
from .ServiceActionInterface import ServiceActionInterface
from .FSMActionInterface import FSMActionInterface


class ActionInterfaceManager(Node):

    def __init__(self):
        super().__init__('rosplan_action_interface_manager')

        self._action_interfaces = {}
        self._kb_link = RPKnowledgeBaseLink(self)

        self.declare_parameter('actions_config', '')
        self.declare_parameter('action_feedback_topic', 'action_feedback')
        self.declare_parameter('action_dispatch_topic', 'action_dispatch')

        config_path = self.get_parameter('actions_config').value
        if not config_path:
            self.get_logger().error(
                'KCL: ({}) Error: ~actions_config parameter not set. '
                'Provide path to YAML config file.'.format(self.get_name()))
            rclpy.shutdown()
            return

        try:
            with open(config_path) as f:
                config = yaml.safe_load(f)
            self.cfg_actions = config.get('actions', [])
        except Exception as e:
            self.get_logger().error(
                'KCL: ({}) Error loading config file {}: {}'.format(
                    self.get_name(), config_path, e))
            rclpy.shutdown()
            return

        if not self.cfg_actions:
            self.get_logger().error(
                'KCL: ({}) Error: no actions found in config file {}.'.format(
                    self.get_name(), config_path))
            rclpy.shutdown()
            return

        aft = self.get_parameter('action_feedback_topic').value
        adt = self.get_parameter('action_dispatch_topic').value

        self._action_feedback_pub = self.create_publisher(ActionFeedback, aft, 10)
        self.create_subscription(ActionDispatch, adt, self.dispatch_callback, 10)

        self._parse_config()

        self.get_logger().info('KCL: ({}) Ready to receive'.format(self.get_name()))

        self._run_timer = self.create_timer(0.1, self._run_tick)

    # ====================== #
    # PDDL action messages   #
    # ====================== #

    def dispatch_callback(self, pddl_action_msg):
        if pddl_action_msg.name not in self._action_interfaces:
            return
        self.publish_feedback(
            pddl_action_msg.plan_id, pddl_action_msg.action_id,
            ActionFeedback.ACTION_DISPATCHED_TO_GOAL_STATE)
        self._kb_link.kb_apply_action_effects(pddl_action_msg, RPKnowledgeBaseLink.AT_START)
        self._action_interfaces[pddl_action_msg.name].run(pddl_action_msg)

    def publish_feedback(self, plan_id, action_id, status):
        fb = ActionFeedback()
        fb.action_id = action_id
        fb.plan_id = plan_id
        fb.status = status
        self._action_feedback_pub.publish(fb)

    # ================== #
    # interface status   #
    # ================== #

    def _run_tick(self):
        for interface in list(self._action_interfaces.values()):
            for act in list(interface._action_status.keys()):
                status = interface._action_status.get(act)
                if status is None:
                    continue
                if status == ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE:
                    self._kb_link.kb_apply_action_effects(
                        interface._action_dispatch_msg[act], RPKnowledgeBaseLink.AT_END)
                if status in (ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE,
                               ActionFeedback.ACTION_FAILED):
                    self.get_logger().info(
                        'KCL: ({}) Reporting action complete: {} {}. Action {}.'.format(
                            self.get_name(), act, interface._action_name,
                            'succeeded' if status == ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE
                            else 'failed'))
                    self.publish_feedback(act[0], act[1], status)
                    interface.clean_action(act)

    # ============== #
    # YAML parsing   #
    # ============== #

    def _parse_config(self):
        for action in self.cfg_actions:
            if action['interface_type'] == 'actionlib':
                self._action_interfaces[action['name']] = ActionlibActionInterface(action, self)
            elif action['interface_type'] == 'service':
                self._action_interfaces[action['name']] = ServiceActionInterface(action, self)
            elif action['interface_type'] == 'fsm':
                self._action_interfaces[action['name']] = FSMActionInterface(action, self)


def main(args=None):
    rclpy.init(args=args)
    node = ActionInterfaceManager()
    if not rclpy.ok():
        return
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
