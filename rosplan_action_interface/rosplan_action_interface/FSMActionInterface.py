import threading

from rclpy.node import Node
from rosplan_dispatch_msgs.msg import ActionFeedback

from .BaseActionInterface import BaseActionInterface
from .ActionlibActionInterface import ActionlibActionInterface
from .ServiceActionInterface import ServiceActionInterface


class Transition:
    def __init__(self, state_name, effects):
        self._to_state = state_name
        self._effects = ''

    def get_to_state(self):
        return self._to_state

    def get_effects(self):
        return self._effects


class State:
    def __init__(self, state_name, action_interface, transitions):
        self._state_name = state_name
        self._action_interface = action_interface
        self._transitions = transitions

    def get_state_name(self):
        return self._state_name

    def get_action_interface(self):
        return self._action_interface

    def get_transition(self, transition_name, counter):
        return self._transitions[transition_name][counter]


class FSMActionInterface(BaseActionInterface):

    def __init__(self, action_config, node: Node):
        super().__init__(action_config, node)
        self._states = {}
        self._running_state = {}
        self._transition_value = {}
        self._to_start_state = {}
        self._transitions_counters = {}
        self._parent_ai = None

        for state_config in action_config.get('states', []):
            if state_config['interface_type'] == 'actionlib':
                ai = ActionlibActionInterface(state_config, node)
            elif state_config['interface_type'] == 'service':
                ai = ServiceActionInterface(state_config, node)
            elif state_config['interface_type'] == 'fsm':
                ai = FSMActionInterface(state_config, node)
            else:
                continue
            transitions = self._parse_transitions(state_config)
            self._states[state_config['name']] = State(state_config['name'], ai, transitions)

    def run(self, dispatch_msg):
        t = threading.Thread(target=self.run_thread, args=(dispatch_msg,))
        t.start()

    def run_thread(self, dispatch_msg):
        self._node.get_logger().info(
            'KCL: ({}) Plan {} Action {}: State machine {} running'.format(
                self._node.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id,
                self.get_action_name()))

        pid = dispatch_msg.plan_id
        aid = dispatch_msg.action_id
        plan_action_id = (pid, aid)

        if pid not in self._running_state:
            self._running_state[pid] = {}
            self._transition_value[pid] = {}
            self._to_start_state[pid] = {}
            self._transitions_counters[pid] = {}
        if aid not in self._running_state[pid]:
            self._running_state[pid][aid] = None
            self._transition_value[pid][aid] = None
            self._to_start_state[pid][aid] = False
            self._transitions_counters[pid][aid] = {'succeeded': 0, 'failed': 0}

        self._running_state[pid][aid] = self._states['ba1']
        self._running_state[pid][aid].get_action_interface().run(dispatch_msg)

        fsm_done = False
        while not fsm_done:
            transition_value = self._transition_value[pid][aid]
            if transition_value is None:
                continue

            self._transition_value[pid][aid] = None
            transition = None

            if transition_value == 'succeeded':
                cnt = self._transitions_counters[pid][aid]['succeeded']
                transition = self._running_state[pid][aid].get_transition('succeeded', cnt)
                self._transitions_counters[pid][aid]['succeeded'] += 1
                self._transitions_counters[pid][aid]['failed'] = 0
            elif transition_value == 'failed':
                cnt = self._transitions_counters[pid][aid]['failed']
                transition = self._running_state[pid][aid].get_transition('failed', cnt)
                self._transitions_counters[pid][aid]['failed'] += 1
                self._transitions_counters[pid][aid]['succeeded'] = 0

            if transition is None:
                continue

            to = transition.get_to_state()
            if to.find('ba') != -1:
                old_state = self._running_state[pid][aid].get_state_name()
                self._running_state[pid][aid] = self._states[to]
                new_state = self._running_state[pid][aid].get_state_name()
                if old_state != new_state:
                    self._transitions_counters[pid][aid] = {'succeeded': 0, 'failed': 0}
                self._running_state[pid][aid].get_action_interface().run(dispatch_msg)
            elif to == 'goal_state':
                self._action_status[plan_action_id] = ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE
                fsm_done = True
            elif to == 'start_state':
                self._action_status[plan_action_id] = ActionFeedback.ACTION_SUCCEEDED_TO_START_STATE
                fsm_done = True
            elif to == 'error_state':
                self._node.get_logger().warn(
                    'KCL: ({}) Plan {} Action {}: State machine {} error'.format(
                        self._node.get_name(), pid, aid, self.get_action_name()))
                fsm_done = True

    def _parse_transitions(self, state):
        transitions = {'succeeded': [], 'failed': []}
        for t in state.get('transitions', {}).get('succeeded', []):
            transitions['succeeded'].append(Transition(t['to_state'], t.get('effects', '')))
        for t in state.get('transitions', {}).get('failed', []):
            transitions['failed'].append(Transition(t['to_state'], t.get('effects', '')))
        return transitions
