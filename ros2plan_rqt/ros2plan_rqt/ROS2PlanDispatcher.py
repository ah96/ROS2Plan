import os
from itertools import product

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from ros2plan_dispatch_msgs.msg import CompletePlan, ActionFeedback, ActionDispatch
from ros2plan_knowledge_msgs.srv import (
    GetDomainAttributeService, GetDomainTypeService,
    GetAttributeService, GetInstanceService, KnowledgeUpdateService,
)
from ros2plan_knowledge_msgs.msg import KnowledgeItem

from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import QHeaderView, QTreeWidgetItem, QListWidgetItem, QWidget
from python_qt_binding.QtGui import QIcon


class PlanViewWidget(QWidget):

    _column_names = ['action_id', 'dispatch_time', 'action_name', 'duration', 'status']
    _action_list = []
    _status_list = {}
    _predicate_param_type_list = {}
    _predicate_param_label_list = {}

    _type_list = []
    _goal_list = {}
    _fact_list = {}

    def __init__(self, node, plugin=None):
        super(PlanViewWidget, self).__init__()

        self._node = node
        self._svc_node = Node('ros2plan_rqt_dispatcher_svc')

        ui_file = os.path.join(get_package_share_directory('ros2plan_rqt'), 'resource', 'dispatcher.ui')
        loadUi(ui_file, self)
        self.setObjectName('ROS2PlanDispatcherUI')

        self._node.get_logger().info('Waiting for ros2plan_knowledge_base services (5 sec timeout)')

        # Clients
        self._predicates_client = self._svc_node.create_client(
            GetDomainAttributeService, 'ros2plan_knowledge_base/domain/predicates')
        self._types_client = self._svc_node.create_client(
            GetDomainTypeService, 'ros2plan_knowledge_base/domain/types')
        self._goals_client = self._svc_node.create_client(
            GetAttributeService, 'ros2plan_knowledge_base/state/goals')
        self._props_client = self._svc_node.create_client(
            GetAttributeService, 'ros2plan_knowledge_base/state/propositions')
        self._instances_client = self._svc_node.create_client(
            GetInstanceService, 'ros2plan_knowledge_base/state/instances')
        self._update_client = self._svc_node.create_client(
            KnowledgeUpdateService, 'ros2plan_knowledge_base/update')

        # Populate goal/fact combo boxes
        try:
            resp = self._call_service(self._predicates_client, GetDomainAttributeService.Request())
            if resp:
                for pred in resp.items:
                    self.goalNameComboBox.addItem(pred.name)
                    self.factNameComboBox.addItem(pred.name)
                    self._predicate_param_type_list[pred.name] = [p.value for p in pred.typed_parameters]
                    self._predicate_param_label_list[pred.name] = [p.key for p in pred.typed_parameters]
        except Exception as e:
            print('Service call failed: %s' % e)

        self._handle_goal_name_changed(0)
        self._handle_fact_name_changed(0)

        # Populate type combo box
        try:
            resp = self._call_service(self._types_client, GetDomainTypeService.Request())
            if resp:
                for typename in resp.types:
                    self.typeComboBox.addItem(typename)
                    self._type_list.append(typename)
        except Exception as e:
            print('Service call failed: %s' % e)

        # Publishers / subscribers
        self._plan_pub = self._node.create_publisher(String, '/kcl_ros2plan/planning_commands', 10)
        self._node.create_subscription(CompletePlan, '/kcl_ros2plan/plan', self.plan_callback, 10)
        self._node.create_subscription(ActionFeedback, '/kcl_ros2plan/action_feedback', self.action_feedback_callback, 10)
        self._node.create_subscription(String, '/kcl_ros2plan/system_state', self.system_status_callback, 10)

        # Signals
        self.planButton.clicked[bool].connect(self._handle_plan_clicked)
        self.pauseButton.clicked[bool].connect(self._handle_pause_clicked)
        self.cancelButton.clicked[bool].connect(self._handle_cancel_clicked)
        self.addInstanceButton.clicked[bool].connect(self._handle_add_instance_clicked)
        self.removeGoalButton.clicked[bool].connect(self._handle_remove_goal_clicked)
        self.removeFactButton.clicked[bool].connect(self._handle_remove_fact_clicked)
        self.addGoalButton.clicked[bool].connect(self._handle_add_goal_clicked)
        self.addFactButton.clicked[bool].connect(self._handle_add_fact_clicked)
        self.goalNameComboBox.currentIndexChanged[int].connect(self._handle_goal_name_changed)
        self.factNameComboBox.currentIndexChanged[int].connect(self._handle_fact_name_changed)

        self._plugin = plugin
        self.planView.sortByColumn(0, Qt.AscendingOrder)
        header = self.planView.header()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)

        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)

        self._timer_refresh_plan = QTimer(self)
        self._timer_refresh_plan.timeout.connect(self.refresh_plan)
        self._timer_refresh_goals = QTimer(self)
        self._timer_refresh_goals.timeout.connect(self.refresh_model)

        self.refresh_model()

    def _call_service(self, client, request, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._svc_node, future, timeout_sec=timeout_sec)
        return future.result() if future.done() else None

    def start(self):
        self._timer_refresh_plan.start(1000)
        self._timer_refresh_goals.start(10000)

    def refresh_model(self):
        # Goals
        selected_list = [item.text() for item in self.goalView.selectedItems()]
        try:
            req = GetAttributeService.Request()
            req.predicate_name = ''
            resp = self._call_service(self._goals_client, req)
            if resp:
                self.goalView.clear()
                self._goal_list.clear()
                for goal in resp.attributes:
                    goalText = '(' + goal.attribute_name
                    for kv in goal.values:
                        goalText += ' ' + kv.value
                    goalText += ')'
                    item = QListWidgetItem(self.goalView)
                    item.setText(goalText)
                    self._goal_list[goalText] = goal
                    if goalText in selected_list:
                        item.setSelected(True)
        except Exception as e:
            print('Service call failed: %s' % e)

        # Facts
        selected_list = [item.text() for item in self.modelView.selectedItems()]
        try:
            req = GetAttributeService.Request()
            req.predicate_name = ''
            resp = self._call_service(self._props_client, req)
            if resp:
                self.modelView.clear()
                self._fact_list.clear()
                for attribute in resp.attributes:
                    attributeText = ''
                    if attribute.is_negative:
                        attributeText = '(not '
                    attributeText += '(' + attribute.attribute_name
                    for kv in attribute.values:
                        attributeText += ' ' + kv.value
                    attributeText += ')'
                    if attribute.is_negative:
                        attributeText += ')'
                    item = QListWidgetItem(self.modelView)
                    item.setText(attributeText)
                    self._fact_list[attributeText] = attribute
                    if attributeText in selected_list:
                        item.setSelected(True)
        except Exception as e:
            print('Service call failed: %s' % e)

        # Instances
        expanded_list = []
        root = self.instanceView.invisibleRootItem()
        for i in range(root.childCount()):
            item = root.child(i)
            if item.isExpanded():
                expanded_list.append(item.text(0))
        self.instanceView.clear()
        for typename in self._type_list:
            req = GetInstanceService.Request()
            req.type_name = typename
            resp = self._call_service(self._instances_client, req)
            item = QTreeWidgetItem(self.instanceView)
            item.setText(0, typename)
            if resp:
                for instancename in resp.instances:
                    inst = QTreeWidgetItem(item)
                    inst.setText(0, instancename)
            if typename in expanded_list:
                item.setExpanded(True)

    def refresh_plan(self):
        expanded_list = []
        root = self.planView.invisibleRootItem()
        for i in range(root.childCount()):
            item = root.child(i)
            if item.isExpanded():
                expanded_list.append(item.text(self._column_index['action_id']))
        self.planView.clear()
        for action in self._action_list:
            item = QTreeWidgetItem(self.planView)
            item.setText(self._column_index['action_id'], str(action.action_id))
            item.setText(self._column_index['dispatch_time'], str(action.dispatch_time))
            item.setText(self._column_index['duration'], str(action.duration))
            item.setText(self._column_index['status'], self._status_list.get(str(action.action_id), '-'))
            action_name = '(' + action.name
            for keyval in action.parameters:
                param = QTreeWidgetItem(item)
                param.setText(self._column_index['action_id'], '')
                param.setText(self._column_index['action_name'], '- ' + keyval.key + ': ' + keyval.value)
                param.setText(self._column_index['dispatch_time'], '')
                param.setText(self._column_index['duration'], '')
                param.setText(self._column_index['status'], '')
                action_name += ' ' + keyval.value
            item.setText(self._column_index['action_name'], action_name + ')')
            if str(action.action_id) in expanded_list:
                item.setExpanded(True)

    def _handle_plan_clicked(self, checked):
        self._status_list.clear()
        self._plan_pub.publish(String(data='plan'))

    def _handle_pause_clicked(self, checked):
        self._status_list.clear()
        self._plan_pub.publish(String(data='pause'))

    def _handle_cancel_clicked(self, checked):
        self._status_list.clear()
        self._plan_pub.publish(String(data='cancel'))

    def plan_callback(self, data):
        self._action_list = data.plan

    def action_feedback_callback(self, data):
        self._status_list[str(data.action_id)] = data.status
        self.refresh_model()

    def system_status_callback(self, data):
        self.statusLabel.setText(data.data)

    def _handle_predicate_name_change(self, predName, combo):
        if not predName or predName not in self._predicate_param_type_list:
            return
        combo.clear()
        parameters = []
        for param_type in self._predicate_param_type_list[predName]:
            try:
                req = GetInstanceService.Request()
                req.type_name = param_type
                resp = self._call_service(self._instances_client, req)
                parameters.append(resp.instances if resp else [])
            except Exception as e:
                print('Service call failed: %s' % e)
        for element in product(*parameters):
            combo.addItem(' '.join(element))

    def _handle_goal_name_changed(self, index):
        self._handle_predicate_name_change(self.goalNameComboBox.itemText(index), self.goalComboBox)

    def _handle_fact_name_changed(self, index):
        self._handle_predicate_name_change(self.factNameComboBox.itemText(index), self.factComboBox)

    def _handle_add_button_clicked(self, update_type, predName, combo):
        try:
            req = KnowledgeUpdateService.Request()
            req.update_type = update_type
            knowledge = KnowledgeItem()
            knowledge.knowledge_type = KnowledgeItem.FACT
            knowledge.attribute_name = predName
            for index, param in enumerate(combo.currentText().split()):
                pair = KeyValue()
                pair.key = (self._predicate_param_label_list[knowledge.attribute_name])[index]
                pair.value = param
                knowledge.values.append(pair)
            req.knowledge = knowledge
            self._call_service(self._update_client, req)
        except Exception as e:
            print('Service call failed: %s' % e)

    def _handle_add_goal_clicked(self, data):
        self._handle_add_button_clicked(
            KnowledgeUpdateService.Request.ADD_GOAL,
            self.goalNameComboBox.currentText(), self.goalComboBox)
        self.refresh_model()

    def _handle_add_fact_clicked(self, data):
        self._handle_add_button_clicked(
            KnowledgeUpdateService.Request.ADD_KNOWLEDGE,
            self.factNameComboBox.currentText(), self.factComboBox)
        self.refresh_model()

    def _handle_remove_button_clicked(self, update_type, removeNameList, removeMsgList):
        for item in removeNameList:
            try:
                req = KnowledgeUpdateService.Request()
                req.update_type = update_type
                req.knowledge = removeMsgList[item.text()]
                self._call_service(self._update_client, req)
            except Exception as e:
                print('Service call failed: %s' % e)
        self.refresh_model()

    def _handle_remove_goal_clicked(self, checked):
        self._handle_remove_button_clicked(
            KnowledgeUpdateService.Request.REMOVE_GOAL,
            self.goalView.selectedItems(), self._goal_list)

    def _handle_remove_fact_clicked(self, checked):
        self._handle_remove_button_clicked(
            KnowledgeUpdateService.Request.REMOVE_KNOWLEDGE,
            self.modelView.selectedItems(), self._fact_list)

    def _handle_add_instance_clicked(self, checked):
        if self.instanceNameEdit.text() == '':
            return
        try:
            req = KnowledgeUpdateService.Request()
            req.update_type = KnowledgeUpdateService.Request.ADD_KNOWLEDGE
            knowledge = KnowledgeItem()
            knowledge.knowledge_type = KnowledgeItem.INSTANCE
            knowledge.instance_type = self.typeComboBox.currentText()
            knowledge.instance_name = self.instanceNameEdit.text()
            req.knowledge = knowledge
            self._call_service(self._update_client, req)
        except Exception as e:
            print('Service call failed: %s' % e)
        self.refresh_model()

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
