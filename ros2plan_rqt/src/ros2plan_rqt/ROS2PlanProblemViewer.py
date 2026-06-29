#!/usr/bin/env python

import os
import rospy
import rospkg
import sys

from itertools import product
from string import join, split

from std_msgs.msg import *
from diagnostic_msgs.msg import KeyValue
from ros2plan_dispatch_msgs.msg import *
from ros2plan_knowledge_msgs.srv import *
from ros2plan_knowledge_msgs.msg import *

from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtCore import Qt, QTimer, QUrl, Signal, Slot, pyqtSignal
if QT_BINDING_VERSION.startswith('4'):
    from python_qt_binding.QtGui import  QWidget
else:
    from python_qt_binding.QtWidgets import QWidget

class ProblemViewerWidget(QWidget):

    _problem_text = ""
    _update_signal = pyqtSignal()

    def __init__(self, plugin=None):
        super(ProblemViewerWidget, self).__init__()

        # Create QWidget
        ui_file = os.path.join(rospkg.RosPack().get_path('ros2plan_rqt'), 'resource', 'problem_viewer.ui')
        loadUi(ui_file, self)
        self.setObjectName('ROS2PlanProblemViewer')

        self._plugin = plugin
        rospy.Subscriber("/kcl_ros2plan/problem", String, self.problem_received)

        self._update_signal.connect(self.update_problem)

    """
    updating problem view
    """
    def problem_received(self, msg):
        self._problem_text = msg.data
        self._update_signal.emit()

    def update_problem(self):
	    self.textEdit.setPlainText(self._problem_text)

    """
    Qt methods
    """ 
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
