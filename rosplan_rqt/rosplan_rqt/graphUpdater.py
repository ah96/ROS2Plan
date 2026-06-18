from rosplan_dispatch_msgs.msg import CompletePlan, ActionFeedback
from python_qt_binding.QtCore import QTimer


class PlanViewWidget:

    _column_names = ['action_id', 'dispatch_time', 'action_name', 'duration', 'status']
    _action_list = []
    _status_list = {}

    def __init__(self, node):
        self._node = node
        self.setObjectName('ROSPlanGraphUpdater')

        self._timer_refresh_plan = QTimer(self)
        self._timer_refresh_plan.timeout.connect(self.refresh_plan)

        self._node.create_subscription(CompletePlan, '/kcl_rosplan/plan', self.plan_callback, 10)
        self._node.create_subscription(ActionFeedback, '/kcl_rosplan/action_feedback', self.action_feedback_callback, 10)

    def start(self):
        self._timer_refresh_plan.start(1000)

    def refresh_plan(self):
        expanded_list = []
        root = self.planView.invisibleRootItem()
        for i in range(root.childCount()):
            item = root.child(i)
            if item.isExpanded():
                expanded_list.append(item.text(self._column_index['action_id']))
        self.planView.clear()
        from python_qt_binding.QtWidgets import QTreeWidgetItem
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

    def plan_callback(self, data):
        self._action_list = data.plan

    def action_feedback_callback(self, data):
        self._status_list[str(data.action_id)] = data.status
        self.refresh_model()
