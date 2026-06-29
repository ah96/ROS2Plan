import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from diagnostic_msgs.msg import KeyValue
from ros2plan_dispatch_msgs.msg import ActionFeedback, ActionDispatch
from ros2plan_knowledge_msgs.srv import GetDomainOperatorService, GetInstanceService

from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import QHeaderView, QTreeWidgetItem, QListWidgetItem, QComboBox, QWidget
from python_qt_binding.QtGui import QIcon


class ActionDispatchWidget(QWidget):

    _operator_list = []
    _parameter_type_list = {}
    _parameter_label_list = {}

    def __init__(self, node, plugin=None):
        super(ActionDispatchWidget, self).__init__()

        self._node = node
        self._svc_node = Node('ros2plan_rqt_action_dispatch_svc')

        ui_file = os.path.join(get_package_share_directory('ros2plan_rqt'), 'resource', 'action_dispatcher.ui')
        loadUi(ui_file, self)
        self.setObjectName('ROS2PlanActionDispatcherUI')

        self._operators_client = self._svc_node.create_client(
            GetDomainOperatorService, '/kcl_ros2plan/get_domain_operators')
        self._instances_client = self._svc_node.create_client(
            GetInstanceService, '/kcl_ros2plan/get_current_instances')

        # Populate operator combo box
        try:
            resp = self._call_service(self._operators_client, GetDomainOperatorService.Request())
            if resp:
                for op in resp.operators:
                    self.operatorNameComboBox.addItem(op.name)
                    self._parameter_type_list[op.name] = [p.value for p in op.typed_parameters]
                    self._parameter_label_list[op.name] = [p.key for p in op.typed_parameters]
        except Exception as e:
            print('Service call failed: %s' % e)

        self._handle_operator_name_changed(0)

        self.dispatchButton.clicked[bool].connect(self._handle_dispatch_clicked)
        self.operatorNameComboBox.currentIndexChanged[int].connect(self._handle_operator_name_changed)

        self._plugin = plugin
        self._dispatch_pub = self._node.create_publisher(ActionDispatch, '/kcl_ros2plan/action_dispatch', 10)
        self._node.create_subscription(
            ActionFeedback, '/kcl_ros2plan/action_feedback', self.action_feedback_callback, 10)

    def _call_service(self, client, request, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._svc_node, future, timeout_sec=timeout_sec)
        return future.result() if future.done() else None

    def _handle_dispatch_clicked(self, checked):
        self.statusLabel.setText('')
        msg = ActionDispatch()
        msg.name = self.operatorNameComboBox.currentText()
        root = self.operatorView.invisibleRootItem()
        for i in range(root.childCount()):
            item = root.child(i)
            cmb = self.operatorView.itemWidget(item, 2)
            kv = KeyValue()
            kv.key = item.text(0)
            kv.value = cmb.currentText()
            msg.parameters.append(kv)
        self._dispatch_pub.publish(msg)

    def action_feedback_callback(self, data):
        self.statusLabel.setText(data.status)

    def _handle_operator_name_change(self, operatorName, combo):
        if not operatorName or operatorName not in self._parameter_label_list:
            return
        self.operatorView.clear()
        for i in range(len(self._parameter_label_list[operatorName])):
            cmb = QComboBox()
            try:
                req = GetInstanceService.Request()
                req.type_name = self._parameter_type_list[operatorName][i]
                resp = self._call_service(self._instances_client, req)
                if resp:
                    for instancename in resp.instances:
                        cmb.addItem(instancename, instancename)
            except Exception as e:
                print('Service call failed: %s' % e)
            item = QTreeWidgetItem(self.operatorView)
            item.setText(0, self._parameter_label_list[operatorName][i])
            item.setText(1, self._parameter_type_list[operatorName][i])
            self.operatorView.setItemWidget(item, 2, cmb)

    def _handle_operator_name_changed(self, index):
        self._handle_operator_name_change(
            self.operatorNameComboBox.itemText(index), self.operatorNameComboBox)

    def start(self):
        pass

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
