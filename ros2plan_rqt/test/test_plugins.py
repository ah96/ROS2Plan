#!/usr/bin/env python3
"""Every class advertised in plugin.xml must actually be importable.

rqt resolves these lazily at GUI load time, so a renamed module or class stays invisible
until someone opens the plugin. These tests fail the build instead.
"""

import os
import xml.etree.ElementTree as ET

import importlib

from ament_index_python.packages import get_package_share_directory

import pytest

MODULES = [
    'ros2plan_rqt.dispatcher',
    'ros2plan_rqt.action_dispatcher',
    'ros2plan_rqt.problem_viewer',
    'ros2plan_rqt.esterel_plan_viewer',
    'ros2plan_rqt.action_config',
    'ros2plan_rqt.action_config_widget',
    'ros2plan_rqt.graphUpdater',
    'ros2plan_rqt.xdot_qt',
]


def _plugin_classes():
    plugin_xml = os.path.join(get_package_share_directory('ros2plan_rqt'), 'plugin.xml')
    return [c.get('type') for c in ET.parse(plugin_xml).getroot().iter('class')]


@pytest.mark.parametrize('module', MODULES)
def test_module_imports(module):
    importlib.import_module(module)


def test_plugin_xml_lists_all_five_plugins():
    assert len(_plugin_classes()) == 5


@pytest.mark.parametrize('type_str', _plugin_classes())
def test_plugin_class_resolves(type_str):
    module_name, _, class_name = type_str.rpartition('.')
    module = importlib.import_module(module_name)
    assert hasattr(module, class_name), \
        '{} declares {} but the class is missing'.format(module_name, class_name)
