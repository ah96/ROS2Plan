import os

import rclpy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String

from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QWidget


class ProblemViewerWidget(QWidget):

    _problem_text = ''
    _update_signal = pyqtSignal()

    def __init__(self, node, plugin=None):
        super(ProblemViewerWidget, self).__init__()

        self._node = node

        ui_file = os.path.join(get_package_share_directory('rosplan_rqt'), 'resource', 'problem_viewer.ui')
        loadUi(ui_file, self)
        self.setObjectName('ROSPlanProblemViewer')

        self._plugin = plugin
        self._node.create_subscription(String, '/kcl_rosplan/problem', self.problem_received, 10)
        self._update_signal.connect(self.update_problem)

    def problem_received(self, msg):
        self._problem_text = msg.data
        self._update_signal.emit()

    def update_problem(self):
        self.textEdit.setPlainText(self._problem_text)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
