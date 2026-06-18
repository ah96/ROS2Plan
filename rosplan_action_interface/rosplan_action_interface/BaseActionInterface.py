from abc import abstractmethod

from rclpy.node import Node
from rosplan_dispatch_msgs.msg import ActionFeedback


class BaseActionInterface:

    def __init__(self, action_config, node: Node):
        self._node = node
        self._action_name = action_config['name']
        self._interface_type = action_config['interface_type']
        self._action_config = action_config

        self._action_dispatch_msg = {}
        self._action_status = {}

    def get_action_name(self):
        return self._action_name

    @staticmethod
    def params_match(config, pddl_params, dispatch_msg):
        for p in dispatch_msg.parameters:
            for i in range(len(pddl_params)):
                if p.key == pddl_params[i] and p.value != config['values'][i]:
                    return False
        return True

    @abstractmethod
    def run(self, dispatch_msg):
        pass

    def clean_action(self, plan_action_id):
        del self._action_status[plan_action_id]
        del self._action_dispatch_msg[plan_action_id]

    # ===================== #
    # msg parsing methods   #
    # ===================== #

    def populate_goal_msg(self, goal_msg, param, value, dispatch_msg):
        param_string = self.parse_config_string(param, dispatch_msg)[0]
        value_string = self.parse_config_string(value, dispatch_msg)[0]
        p = eval('goal_msg.' + param_string)
        if isinstance(p, str):
            exec("goal_msg." + param_string + " = '" + value_string + "'",
                 {}, {'goal_msg': goal_msg})
        else:
            exec('goal_msg.' + param_string + ' = ' + value_string,
                 {}, {'goal_msg': goal_msg})

    def check_result_msg(self, result_msg, param, value, dispatch_msg):
        param_string = self.parse_config_string(param, dispatch_msg)[0]
        value_string = self.parse_config_string(value, dispatch_msg)[0]
        p = eval('result_msg.' + param_string)
        if isinstance(p, str):
            return eval("result_msg." + param_string + " == '" + value_string + "'",
                        {}, {'result_msg': result_msg})
        return eval('result_msg.' + param_string + ' == ' + value_string,
                    {}, {'result_msg': result_msg})

    def parse_config_string(self, config_string, action_msg):
        if not isinstance(config_string, str):
            return str(config_string), len(str(config_string))

        return_string = ''
        i = 0
        while i < len(config_string):
            if config_string[i:i+12] == '($pddlparam ':
                param_key, length = self.parse_config_string(config_string[i+12:], action_msg)
                i += length + 12
                for p in action_msg.parameters:
                    if p.key == param_key:
                        return_string += p.value
            elif config_string[i:i+11] == '($rosparam ':
                param_key, length = self.parse_config_string(config_string[i+11:], action_msg)
                try:
                    self._node.declare_parameter(param_key, '')
                except Exception:
                    pass
                param_value = str(self._node.get_parameter(param_key).value)
                return_string += param_value
                i += length + 11
            elif config_string[i] == ')':
                return return_string, i
            else:
                return_string += config_string[i]
            i += 1
        return return_string, len(config_string)
