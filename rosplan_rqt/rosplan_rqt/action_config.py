from qt_gui.plugin import Plugin
from .action_config_widget import ActionConfigWidget


class ActionConfig(Plugin):

    def __init__(self, context):
        super(ActionConfig, self).__init__(context)
        self.setObjectName('ActionConfig')

        self._widget = ActionConfigWidget()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument('-q', '--quiet', action='store_true', dest='quiet',
                            help='Put plugin in silent mode')
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments:', args)
            print('unknowns:', unknowns)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
