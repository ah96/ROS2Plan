#!/usr/bin/env python3
# Launch-test harness for the ROS2Plan knowledge base (ROS2 port of knowledge_base.test).
# Brings up the knowledge base node with the amazon fixture, then runs the gtest executable that
# exercises the knowledge base services.

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
    # The amazon PDDL fixture lives in the planning_system package.
    pkg = get_package_share_directory('ros2plan_planning_system')
    common = os.path.join(pkg, 'test', 'pddl', 'amazon')
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
        name='knowledge_base_gtest',
        output='screen',
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('test_exe', description='Path to the gtest executable'),
        knowledge_base,
        # Give the node a moment to come up and the KB to parse the domain/problem.
        TimerAction(period=3.0, actions=[gtest]),
        launch_testing.actions.ReadyToTest(),
    ]), {'gtest': gtest}


class TestKnowledgeBase(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, gtest):
        # Wait for the gtest process to finish (it has its own internal timeouts).
        proc_info.assertWaitForShutdown(process=gtest, timeout=90)


@launch_testing.post_shutdown_test()
class TestKnowledgeBaseExitCode(unittest.TestCase):

    def test_gtest_exit_code(self, proc_info, gtest):
        # gtest returns 0 only if all assertions passed.
        launch_testing.asserts.assertExitCodes(proc_info, process=gtest)
