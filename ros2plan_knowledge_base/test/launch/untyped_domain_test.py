#!/usr/bin/env python3
# Launch-test harness for untyped PDDL support in the knowledge base.
# Brings up the knowledge base with the untyped fixture, then runs the gtest executable that
# checks it parsed the domain and problem instead of dying on their null type pointers.

import os
import unittest

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # The untyped PDDL fixture lives in the planning_system package, as the other fixtures do.
    pkg = get_package_share_directory('ros2plan_planning_system')
    common = os.path.join(pkg, 'test', 'pddl', 'untyped')
    domain_path = os.path.join(common, 'domain.pddl')
    problem_path = os.path.join(common, 'problem.pddl')

    knowledge_base = launch_ros.actions.Node(
        package='ros2plan_knowledge_base',
        executable='knowledgeBase',
        name='ros2plan_knowledge_base',
        output='screen',
        parameters=[{
            'domain_path': domain_path,
            'problem_path': problem_path,
            'use_unknowns': False,
        }],
    )

    gtest = ExecuteProcess(
        cmd=[LaunchConfiguration('test_exe')],
        name='untyped_domain_gtest',
        output='screen',
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('test_exe', description='Path to the gtest executable'),
        knowledge_base,
        # Give the node a moment to come up and the KB to parse the domain/problem.
        TimerAction(period=3.0, actions=[gtest]),
        launch_testing.actions.ReadyToTest(),
    ]), {'gtest': gtest, 'knowledge_base': knowledge_base}


class TestUntypedDomain(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, gtest):
        # Wait for the gtest process to finish (it has its own internal timeouts).
        proc_info.assertWaitForShutdown(process=gtest, timeout=90)


@launch_testing.post_shutdown_test()
class TestUntypedDomainExitCodes(unittest.TestCase):

    def test_gtest_exit_code(self, proc_info, gtest):
        # gtest returns 0 only if all assertions passed.
        launch_testing.asserts.assertExitCodes(proc_info, process=gtest)

    def test_knowledge_base_did_not_crash(self, proc_info, knowledge_base):
        # The bug this covers was a segmentation fault, which shows up as a negative exit code
        # (-11) rather than a failed assertion. Assert on the node's own exit explicitly:
        # without this, a knowledge base that died during start-up would be indistinguishable
        # from one shut down normally at the end of the test.
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -15],  # clean exit, or the SIGINT/SIGTERM teardown
            process=knowledge_base)
