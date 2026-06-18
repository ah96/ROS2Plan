import os

from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

from rosplan_rqt.xdot_qt import DotWidget

from python_qt_binding import loadUi, QT_BINDING_VERSION
from PyQt5.QtWidgets import QWidget, QFileDialog


class EsterelPlanViewerWidget(QWidget):

    def __init__(self, node, plugin=None):
        super(EsterelPlanViewerWidget, self).__init__()

        self._node = node

        ui_file = os.path.join(
            get_package_share_directory('rosplan_rqt'), 'resource', 'esterel_plan_viewer.ui')
        loadUi(ui_file, self, {'DotWidget': DotWidget})
        self.setObjectName('ROSPlanEsterelPlanViewer')

        self.refreshButton.clicked[bool].connect(self._handle_refresh_clicked)
        self.saveButton.clicked[bool].connect(self._handle_save_button_clicked)

        self._sub = self._node.create_subscription(
            String, '/rosplan_plan_dispatcher/plan_graph', self.planReceivedCallback, 10)

        self.first_time_graph_received = True
        self.graph = None
        self.gen_single_node('no plan received')

    def gen_single_node(self, node_text):
        graph = 'digraph plan {0[ label="' + node_text + '",style=filled,fillcolor=white,fontcolor=black];}'
        self.xdot_widget.set_dotcode(graph)
        self.xdot_widget.zoom_image(5.0, center=True)

    def planReceivedCallback(self, msg):
        self.graph = msg.data
        self._node.get_logger().info('Rendering graph started...')
        if self.first_time_graph_received:
            self.gen_single_node('Plan received! rendering...')
        self.xdot_widget.set_dotcode(msg.data)
        if self.first_time_graph_received:
            self.xdot_widget.zoom_to_fit()
            self.first_time_graph_received = False
        self.xdot_widget.update()
        self._node.get_logger().info('Rendering graph ended!')

    def _handle_refresh_clicked(self, checked):
        self._sub.destroy()
        self._sub = self._node.create_subscription(
            String, self.topicText.text(), self.planReceivedCallback, 10)

    def save_graph(self, full_path):
        if self.graph:
            with open(full_path, 'w') as dot_file:
                dot_file.write(self.graph)
            self._node.get_logger().info('Graph saved in %s', full_path)
        else:
            self._node.get_logger().error(
                'Could not save graph: empty. Check topic %s', self.topicText.text())

    def _handle_save_button_clicked(self, checked):
        self._node.get_logger().info('Saving esterel plan to dot file')
        fileName = QFileDialog.getSaveFileName(self, 'Save Esterel plan to dot file', '', 'Graph xdot Files (*.dot)')
        if fileName[0] == '':
            self._node.get_logger().info('User cancelled save')
        else:
            full_dot_path = fileName[0]
            if '.dot' not in full_dot_path:
                full_dot_path += '.dot'
            self.save_graph(full_dot_path)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
