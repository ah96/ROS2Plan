#!/usr/bin/env python3
"""Import and packaging checks for the sensing interface.

The sensing scripts referenced by the YAML `functions:` entry are loaded at runtime from
the install space, so both "is it installed" and "is it valid Python 3" have to be checked
here -- neither shows up until the node starts.
"""

import importlib
import os
import py_compile

from ament_index_python.packages import get_package_share_directory

import pytest

import yaml

SHARE = get_package_share_directory('ros2plan_sensing_interface')
CONFIGS = ['example.yaml', 'icaps2019_tutorial.yaml']
SCRIPTS = ['example.py', 'icaps2019_tutorial.py']


def test_module_imports():
    importlib.import_module('ros2plan_sensing_interface.sensing_interface')


def test_no_ros1_imports():
    source = importlib.import_module('ros2plan_sensing_interface.sensing_interface').__file__
    with open(source) as f:
        assert 'import rospy' not in f.read(), 'sensing_interface still imports rospy'


@pytest.mark.parametrize('name', CONFIGS + SCRIPTS)
def test_installed(name):
    assert os.path.isfile(os.path.join(SHARE, name)), \
        '{} is not installed into the share directory'.format(name)


@pytest.mark.parametrize('name', SCRIPTS)
def test_sensing_script_is_valid_python3(name):
    """These shipped as unported Python 2 (a bare `print` statement) at one point."""
    py_compile.compile(os.path.join(SHARE, name), doraise=True)


@pytest.mark.parametrize('name', SCRIPTS)
def test_sensing_script_has_no_ros1_api(name):
    with open(os.path.join(SHARE, name)) as f:
        text = f.read()
    assert 'rospy.' not in text, '{} still uses the ROS 1 rospy API'.format(name)


@pytest.mark.parametrize('name', CONFIGS)
def test_config_functions_path_resolves(name):
    """Each config points at a sensing script via $(find <pkg>)/<file>."""
    with open(os.path.join(SHARE, name)) as f:
        cfg = yaml.safe_load(f)
    funcs = cfg.get('functions')
    if not funcs:
        pytest.skip('{} declares no functions'.format(name))
    entry = funcs[0] if isinstance(funcs, list) else funcs
    assert entry.startswith('$(find ')
    pkg = entry[len('$(find '):entry.index(')')]
    resolved = os.path.join(get_package_share_directory(pkg), entry.split(')/', 1)[1])
    assert os.path.isfile(resolved), '{} points at missing {}'.format(name, resolved)
