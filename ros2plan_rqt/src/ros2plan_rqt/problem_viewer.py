from rqt_gui_py.plugin import Plugin

from .ROS2PlanProblemViewer import ProblemViewerWidget

class ROS2PlanProblemViewer(Plugin):

    def __init__(self, context):
        super(ROS2PlanProblemViewer, self).__init__(context)
        self.setObjectName('ProblemViewer')

        self._widget = ProblemViewerWidget(self)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
