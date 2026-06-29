from rqt_gui_py.plugin import Plugin

from .ROS2PlanDispatcher import PlanViewWidget

class ROS2PlanDispatcher(Plugin):

    def __init__(self, context):
        super(ROS2PlanDispatcher, self).__init__(context)
        self.setObjectName('Dispatcher')

        self._widget = PlanViewWidget(context.node, self)

        self._widget.start()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
