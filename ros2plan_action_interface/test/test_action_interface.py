#!/usr/bin/env python3
"""Import and wiring checks for the action interface package.

These modules have no other coverage, so a syntax error or a stale ROS 1 import would
otherwise only surface when someone launches the manager.
"""

import importlib
import os

from ament_index_python.packages import get_package_share_directory

import pytest

import yaml

MODULES = [
    'ros2plan_action_interface.ActionInterfaceManager',
    'ros2plan_action_interface.BaseActionInterface',
    'ros2plan_action_interface.ServiceActionInterface',
    'ros2plan_action_interface.ActionlibActionInterface',
    'ros2plan_action_interface.FSMActionInterface',
    'ros2plan_action_interface.RPKnowledgeBaseLink',
]


@pytest.mark.parametrize('module', MODULES)
def test_module_imports(module):
    importlib.import_module(module)


@pytest.mark.parametrize('module', MODULES)
def test_no_ros1_imports(module):
    """rospy/actionlib are ROS 1 only; importing them would fail at runtime."""
    source = importlib.import_module(module).__file__
    with open(source) as f:
        text = f.read()
    assert 'import rospy' not in text, '{} still imports rospy'.format(module)


def test_example_config_is_installed_and_parses():
    """The manager needs this file to exist in the install space to be usable at all."""
    path = os.path.join(
        get_package_share_directory('ros2plan_action_interface'),
        'config', 'example_config.yaml')
    assert os.path.isfile(path), 'example_config.yaml is not installed'
    with open(path) as f:
        cfg = yaml.safe_load(f)
    assert 'actions' in cfg and cfg['actions'], 'example config declares no actions'
    # Every action must name an interface type the manager can dispatch to.
    for action in cfg['actions']:
        assert 'name' in action
        assert action.get('interface_type') in ('actionlib', 'service', 'fsm'), \
            'action {} has unknown interface_type {}'.format(
                action.get('name'), action.get('interface_type'))
