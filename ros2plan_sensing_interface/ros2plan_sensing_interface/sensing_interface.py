import re
import threading
import importlib
import importlib.util
from functools import reduce
from threading import Lock

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

from ros2plan_knowledge_msgs.srv import (
    KnowledgeUpdateServiceArray,
    GetDomainPredicateDetailsService,
    GetDomainAttributeService,
    GetAttributeService,
    GetInstanceService,
    SetNamedBool,
)
from ros2plan_knowledge_msgs.msg import KnowledgeItem, StatusUpdate
from diagnostic_msgs.msg import KeyValue


class Ros2planSensing(Node):

    def __init__(self):
        super().__init__('ros2plan_sensing_interface')

        self.mutex = Lock()
        self.srv_mutex = Lock()
        self.dump_cache_mutex = Lock()

        self.declare_parameter('knowledge_base', '/ros2plan_knowledge_base')
        self.declare_parameter('main_rate', 10.0)
        self.declare_parameter('topics', [''])
        self.declare_parameter('services', [''])
        self.declare_parameter('functions', [''])

        self.knowledge_base = self.get_parameter('knowledge_base').value
        kb = self.knowledge_base

        # Satellite node for blocking service calls from background threads.
        # use_global_arguments=False keeps the name below: launch_ros passes the node name
        # as a process-wide `-r __node:=` remap, which would otherwise rename this node too
        # and leave two nodes sharing one name in the graph.
        self._svc_node = Node('ros2plan_sensing_svc_node', use_global_arguments=False)

        self._update_kb_client = self._svc_node.create_client(
            KnowledgeUpdateServiceArray, kb + '/update_array')
        self._get_predicates_client = self._svc_node.create_client(
            GetDomainPredicateDetailsService, kb + '/domain/predicate_details')
        self._get_functions_client = self._svc_node.create_client(
            GetDomainAttributeService, kb + '/domain/functions')
        self._get_instances_client = self._svc_node.create_client(
            GetInstanceService, kb + '/state/instances')
        self._get_state_props_client = self._svc_node.create_client(
            GetAttributeService, kb + '/state/propositions')
        self._get_state_funcs_client = self._svc_node.create_client(
            GetAttributeService, kb + '/state/functions')
        self._set_sensed_client = self._svc_node.create_client(
            SetNamedBool, kb + '/update_sensed_predicates')

        self.create_subscription(
            StatusUpdate, kb + '/status/update', self.kb_update_status, 10)

        # Load config — topics and services come from a YAML file (same as action_interface)
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value

        self.cfg_topics = {}
        self.cfg_service = {}
        self.functions_path = None

        if config_path:
            import yaml
            try:
                with open(config_path) as f:
                    cfg = yaml.safe_load(f)
                self.cfg_topics = cfg.get('topics', {}) or {}
                self.cfg_service = cfg.get('services', {}) or {}
                funcs = cfg.get('functions', None)
                if funcs:
                    self.functions_path = funcs[0] if isinstance(funcs, list) else funcs
            except Exception as e:
                self.get_logger().error(
                    'KCL: (Ros2planSensing) Failed to load config {}: {}'.format(config_path, e))
                rclpy.shutdown()
                return
        else:
            self.get_logger().error(
                'KCL: (Ros2planSensing) Error: config_path parameter not set.')
            rclpy.shutdown()
            return

        if self.functions_path:
            regexp = re.compile(r'\$\(find (.*)\)')
            m = regexp.match(self.functions_path)
            if m:
                try:
                    pkg_path = get_package_share_directory(m.group(1))
                    self.functions_path = regexp.sub(pkg_path, self.functions_path)
                except Exception as e:
                    self.get_logger().error(
                        'KCL: (Ros2planSensing) Package not found: {}'.format(e))
                    rclpy.shutdown()
                    return

        # Load user scripts module
        self.scripts = None
        if self.functions_path:
            spec = importlib.util.spec_from_file_location('sensing_scripts', self.functions_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self.scripts = module
            self.scripts.get_kb_attribute = self.get_kb_attribute
            self.scripts.rclpy = rclpy
            # ROS 2 has no global parameter server, so scripts that need parameters
            # read them off this node instead of the ROS 1 rospy.get_param.
            self.scripts.node = self

        self.sensed_topics = {}
        self.sensed_services = {}
        self.params = {}

        # Subscribe to all configured topics
        for predicate_name, predicate_info in self.cfg_topics.items():
            if isinstance(predicate_info, list):
                for i, pi in enumerate(predicate_info):
                    pi['sub_idx'] = i
                    if not self.subscribe_topic(predicate_name, pi):
                        self.get_logger().info(
                            'Could not subscribe for predicate ' + predicate_name)
            else:
                predicate_info['sub_idx'] = 0
                if not self.subscribe_topic(predicate_name, predicate_info):
                    self.get_logger().info(
                        'Could not subscribe for predicate ' + predicate_name)

        # Create service clients
        self.service_clients = []
        self.service_names = []
        self.service_type_names = []
        self.service_predicate_names = []
        self.last_called_time = []
        self.time_between_calls = []
        self.request_src = []
        self.response_process_src = []
        self.clients_sub_idx = []

        for predicate_name, predicate_info in self.cfg_service.items():
            if isinstance(predicate_info, list):
                for i, pi in enumerate(predicate_info):
                    pi['sub_idx'] = i
                    if not self.create_service_client(predicate_name, pi):
                        self.get_logger().info(
                            'Could not create client for predicate ' + predicate_name)
            else:
                predicate_info['sub_idx'] = 0
                if not self.create_service_client(predicate_name, predicate_info):
                    self.get_logger().info(
                        'Could not create client for predicate ' + predicate_name)

        hz = self.get_parameter('main_rate').value
        self.create_timer(1.0 / hz, self.update_kb)

        self.get_logger().info('KCL: (Ros2planSensing) Ready.')

    # ======================== #
    # service helper           #
    # ======================== #

    def _call_service(self, client, request, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._svc_node, future, timeout_sec=timeout_sec)
        return future.result() if future.done() else None

    # ======================== #
    # parameter loading        #
    # ======================== #

    def load_params(self, predicate_name, predicate_info):
        kb_info = None
        req = GetDomainPredicateDetailsService.Request()
        req.name = predicate_name
        ret = self._call_service(self._get_predicates_client, req)
        if ret:
            kb_info = ret.predicate
        else:
            kb_info = self.get_function_params(predicate_name)
            if not kb_info:
                self.get_logger().error(
                    'KCL: (Ros2planSensing) Could not find predicate or function %s' % predicate_name)
                return (False, False)

        if predicate_name not in self.params:
            self.params[predicate_name] = {}

        kb_params = []
        for p in kb_info.typed_parameters:
            req2 = GetInstanceService.Request()
            req2.type_name = p.value
            ret2 = self._call_service(self._get_instances_client, req2)
            kb_params.append(ret2.instances if ret2 else [])

        if 'params' in predicate_info:
            params = predicate_info['params']
            if len(kb_params) != len(params):
                self.get_logger().error(
                    'KCL: (Ros2planSensing) Parameters for %s do not match the knowledge base' % predicate_name)
                # No rclpy.shutdown() here: unlike ROS 1's signal_shutdown, it invalidates the
                # context immediately, so every later service call in this loop would raise.
                # The (False, True) return already makes the caller skip this predicate.
                return (False, True)
            wildcard = False
            for i, p in enumerate(params):
                if p not in ('*', '_'):
                    if p in kb_params[i]:
                        kb_params[i] = [p]
                    else:
                        self.get_logger().error(
                            'KCL: (Ros2planSensing) Unknown instance "%s" for predicate "%s"' % (p, predicate_name))
                        # See above: skip the predicate instead of tearing down the context.
                        return (False, True)
                else:
                    wildcard = True
            self.params[predicate_name][predicate_info['sub_idx']] = kb_params if wildcard else params
            return (True, True)
        else:
            self.params[predicate_name][predicate_info['sub_idx']] = kb_params
            return (True, False)

    # ======================== #
    # topic subscription       #
    # ======================== #

    def subscribe_topic(self, predicate_name, predicate_info):
        params_loaded = self.load_params(predicate_name, predicate_info)
        if not params_loaded[0]:
            return False
        if len(predicate_info) < 2 + int(params_loaded[1]):
            self.get_logger().error('Error: wrong config for predicate %s' % predicate_name)
            return False
        try:
            msg_type_str = predicate_info['msg_type']
        except KeyError:
            self.get_logger().error('Error: msg_type not specified for %s' % predicate_name)
            return False

        # A config may name a message package that is not installed (the shipped
        # example.yaml uses kobuki_msgs, which has no ROS 2 release). Skip just that
        # predicate rather than taking the whole sensing interface down with it.
        try:
            msg_class = self.import_msg(msg_type_str)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(
                'KCL: (Ros2planSensing) Predicate %s: message type "%s" is unavailable (%s); '
                'skipping this predicate.' % (predicate_name, msg_type_str, e))
            return False

        try:
            topics = predicate_info['topic']
            if isinstance(topics, list):
                for topic in topics:
                    pi = predicate_info.copy()
                    pi['publisher'] = topic
                    self.create_subscription(
                        msg_class, str(topic),
                        lambda msg, pn=predicate_name, pi=pi: self.subs_callback(msg, (pn, pi)),
                        10)
            else:
                self.create_subscription(
                    msg_class, topics,
                    lambda msg, pn=predicate_name, pi=predicate_info: self.subs_callback(msg, (pn, pi)),
                    10)
        except KeyError:
            self.get_logger().error('Error: topic not specified for %s' % predicate_name)
            return False

        try:
            req = SetNamedBool.Request()
            req.name = predicate_name
            req.value = True
            self._call_service(self._set_sensed_client, req)
        except Exception as e:
            self.get_logger().error(
                'KCL: (Ros2planSensing) Could not update sensing info for %s: %s' % (predicate_name, e))

        self.get_logger().info(
            'KCL: (Ros2planSensing) %s: subscribed to %s (%s)' % (
                predicate_name, predicate_info.get('topic', '?'), msg_type_str))
        return True

    # ======================== #
    # service client creation  #
    # ======================== #

    def create_service_client(self, predicate_name, predicate_info):
        params_loaded = self.load_params(predicate_name, predicate_info)
        if not params_loaded[0]:
            return False
        if len(predicate_info) < 2 + int(params_loaded[1]):
            self.get_logger().error('Error: wrong config for predicate %s' % predicate_name)
            return False
        try:
            srv_type_str = predicate_info['srv_type']
        except KeyError:
            self.get_logger().error('Error: srv_type not specified for %s' % predicate_name)
            return False

        # Same as subscribe_topic: an unavailable srv package skips this predicate
        # rather than aborting the node.
        try:
            srv_class = self.import_srv(srv_type_str)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(
                'KCL: (Ros2planSensing) Predicate %s: service type "%s" is unavailable (%s); '
                'skipping this predicate.' % (predicate_name, srv_type_str, e))
            return False

        self.service_type_names.append(srv_class)
        self.clients_sub_idx.append(predicate_info['sub_idx'])

        try:
            client = self._svc_node.create_client(srv_class, predicate_info['service'])
            self.service_clients.append(client)
            self.service_names.append(predicate_info['service'])
        except KeyError:
            self.get_logger().error('Error: service not specified for %s' % predicate_name)
            return False

        self.service_predicate_names.append(predicate_name)
        self.last_called_time.append(Time())

        try:
            self.time_between_calls.append(predicate_info['time_between_calls'])
        except KeyError:
            self.get_logger().error('Error: time_between_calls not specified for %s' % predicate_name)
            return False

        self.request_src.append(predicate_info.get('request', None))
        self.response_process_src.append(predicate_info.get('operation', None))

        try:
            req = SetNamedBool.Request()
            req.name = predicate_name
            req.value = True
            self._call_service(self._set_sensed_client, req)
        except Exception as e:
            self.get_logger().error(
                'KCL: (Ros2planSensing) Could not update sensing info for %s: %s' % (predicate_name, e))

        self.get_logger().info(
            'KCL: (Ros2planSensing) %s: client for %s (%s) ready' % (
                predicate_name, predicate_info['service'], srv_type_str))
        return True

    # ======================== #
    # helper methods           #
    # ======================== #

    def get_function_params(self, func_name):
        req = GetDomainAttributeService.Request()
        ret = self._call_service(self._get_functions_client, req)
        if not ret:
            return None
        for item in ret.items:
            if item.name == func_name:
                return item
        return None

    def import_msg(self, ros_msg_string):
        i = ros_msg_string.find('/')
        pkg_name = ros_msg_string[:i]
        msg_name = ros_msg_string[i+1:]
        pkg = importlib.import_module(pkg_name + '.msg')
        cls = getattr(pkg, msg_name)
        globals()[msg_name] = cls
        if self.scripts:
            setattr(self.scripts, msg_name, cls)
        return cls

    def import_srv(self, ros_srv_string):
        i = ros_srv_string.find('/')
        pkg_name = ros_srv_string[:i]
        srv_name = ros_srv_string[i+1:]
        pkg = importlib.import_module(pkg_name + '.srv')
        srv_cls = getattr(pkg, srv_name)
        globals()[srv_name] = srv_cls
        if self.scripts:
            setattr(self.scripts, srv_name, srv_cls)
            setattr(self.scripts, srv_name + 'Request', srv_cls.Request)
        return srv_cls

    def get_kb_attribute(self, attribute_name):
        req = GetAttributeService.Request()
        req.predicate_name = attribute_name
        try:
            ret = self._call_service(self._get_state_props_client, req)
            if ret and len(ret.attributes) > 0:
                return ret.attributes
            ret2 = self._call_service(self._get_state_funcs_client, req)
            return ret2.attributes if ret2 else []
        except Exception as e:
            self.get_logger().warn(
                'KCL: (Ros2planSensing) Failed to get KB attribute: %s' % str(e))
            return []

    # ======================== #
    # subscription callback    #
    # ======================== #

    def subs_callback(self, msg, predicate):
        pred_name, pred_info = predicate
        if 'operation' in pred_info:
            python_string = pred_info['operation']
        elif self.scripts and pred_name in dir(self.scripts):
            if 'publisher' in pred_info:
                python_string = ("self.scripts.{}(msg, self.params[pred_name][pred_info['sub_idx']]"
                                 ", pred_info['publisher'])").format(pred_name)
            else:
                python_string = ("self.scripts.{}(msg, self.params[pred_name][pred_info['sub_idx']])"
                                 ).format(pred_name)
        else:
            self.get_logger().error(
                'KCL: (Ros2planSensing) Predicate "%s" has no function or processing info' % pred_name)
            return

        with self.mutex:
            result = eval(python_string, globals(), locals())

        if isinstance(result, list):
            for params, val in result:
                with self.mutex:
                    key = pred_name + ':' + params
                    old = self.sensed_topics.get(key)
                    changed = (old is None) or (old[0] ^ val if isinstance(val, bool) else old[0] != val)
                    if changed:
                        self.sensed_topics[key] = (val, True, False)
        else:
            flat = self.params[pred_name][pred_info['sub_idx']]
            if isinstance(flat[0], list):
                self.get_logger().error(
                    'KCL: (Ros2planSensing) Predicate "%s" must have fully instantiated params' % pred_name)
                return
            params = reduce(lambda r, x: r + ':' + x, flat)
            with self.mutex:
                key = pred_name + ':' + params
                old = self.sensed_topics.get(key)
                changed = (old is None) or (old[0] ^ result if isinstance(result, bool) else old[0] != result)
                if changed:
                    self.sensed_topics[key] = (result, True, False)

    # ======================== #
    # service polling thread   #
    # ======================== #

    def call_services(self):
        self.get_logger().info('KCL: (Ros2planSensing) Waiting for services...')
        for name in self.service_names:
            idx = self.service_names.index(name)
            self.service_clients[idx].wait_for_service(timeout_sec=20.0)
        self.get_logger().info('KCL: (Ros2planSensing) All services ready.')

        while rclpy.ok():
            for i in range(len(self.service_clients)):
                now = self.get_clock().now()
                elapsed_sec = (now - self.last_called_time[i]).nanoseconds / 1e9
                if elapsed_sec < self.time_between_calls[i]:
                    continue

                pred_name = self.service_predicate_names[i]

                if self.request_src[i]:
                    python_string = self.request_src[i]
                elif self.scripts and ('req_' + pred_name) in dir(self.scripts):
                    python_string = 'self.scripts.req_{}()'.format(pred_name)
                else:
                    self.get_logger().error(
                        'KCL: (Ros2planSensing) Predicate "%s" has no request method' % pred_name)
                    continue

                req = eval(python_string, globals(), locals())

                future = self.service_clients[i].call_async(req)
                rclpy.spin_until_future_complete(self._svc_node, future, timeout_sec=10.0)
                if not future.done() or future.result() is None:
                    self.get_logger().error(
                        'KCL: (Ros2planSensing) Service call failed for %s' % pred_name)
                    continue

                res = future.result()
                self.last_called_time[i] = now

                if self.response_process_src[i]:
                    python_string = self.response_process_src[i]
                elif self.scripts and pred_name in dir(self.scripts):
                    python_string = ("self.scripts.{}(res, self.params[pred_name][self.clients_sub_idx[i]])"
                                     ).format(pred_name)
                else:
                    self.get_logger().error(
                        'KCL: (Ros2planSensing) Predicate "%s" has no response handler' % pred_name)
                    continue

                result = eval(python_string, globals(), locals())

                if isinstance(result, list):
                    for params, val in result:
                        with self.srv_mutex:
                            key = pred_name + ':' + params
                            old = self.sensed_services.get(key)
                            changed = (old is None) or (old[0] ^ val if isinstance(val, bool) else old[0] != val)
                            if changed:
                                self.sensed_services[key] = (val, True, False)
                else:
                    raw = self.params[pred_name][self.clients_sub_idx[i]]
                    params = reduce(lambda r, x: r + ':' + x, raw) if raw else ''
                    if isinstance(params, list):
                        self.get_logger().error(
                            'KCL: (Ros2planSensing) Predicate "%s" must have fully instantiated params' % pred_name)
                        continue
                    with self.srv_mutex:
                        key = pred_name + ':' + params
                        old = self.sensed_services.get(key)
                        changed = (old is None) or (old[0] ^ result if isinstance(result, bool) else old[0] != result)
                        if changed:
                            self.sensed_services[key] = (result, True, False)

    # ======================== #
    # KB update (timer cb)     #
    # ======================== #

    def update_kb(self):
        kus = KnowledgeUpdateServiceArray.Request()

        with self.dump_cache_mutex:
            with self.mutex:
                sensed = dict(self.sensed_topics)
            with self.srv_mutex:
                sensed.update(self.sensed_services)

            for predicate, (val, changed, updated) in sensed.items():
                if updated:
                    continue

                if predicate in self.sensed_topics:
                    with self.mutex:
                        self.sensed_topics[predicate] = (self.sensed_topics[predicate][0], False, True)
                else:
                    with self.srv_mutex:
                        self.sensed_services[predicate] = (self.sensed_services[predicate][0], False, True)

                predicate_info = predicate.split(':')
                ki = KnowledgeItem()
                ki.attribute_name = predicate_info.pop(0)
                if isinstance(val, bool):
                    ki.knowledge_type = KnowledgeItem.FACT
                    ki.is_negative = not val
                else:
                    ki.knowledge_type = KnowledgeItem.FUNCTION
                    ki.function_value = float(val)

                for i, param in enumerate(predicate_info):
                    kv = KeyValue()
                    kv.key = 'p' + str(i)
                    kv.value = param
                    ki.values.append(kv)

                kus.update_type.append(KnowledgeUpdateServiceArray.Request.ADD_KNOWLEDGE)
                kus.knowledge.append(ki)

        if len(kus.update_type) > 0:
            result = self._call_service(self._update_kb_client, kus)
            if result is None:
                self.get_logger().error(
                    'KCL: (Ros2planSensing) Failed to update knowledge base')

    # ======================== #
    # KB status subscriber     #
    # ======================== #

    def kb_update_status(self, msg):
        if msg.last_update_client == self.get_name():
            return
        with self.dump_cache_mutex:
            with self.mutex:
                for key, (val, changed, _) in self.sensed_topics.items():
                    self.sensed_topics[key] = (val, changed, False)
            with self.srv_mutex:
                self.sensed_services.clear()


def main(args=None):
    rclpy.init(args=args)
    node = Ros2planSensing()
    if not rclpy.ok():
        return

    svc_thread = threading.Thread(target=node.call_services, daemon=True)
    svc_thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
