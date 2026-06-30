#!/usr/bin/env python3
# Launch-test harness for the ROS2Plan problem interface (ROS2 port of problem_interface.test).
# Brings up the knowledge base and the problem interface, then runs the gtest executable that
# exercises the problem-generation service and topic.

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

    problem_interface = launch_ros.actions.Node(
        package='ros2plan_planning_system',
        executable='problemInterface',
        name='ros2plan_problem_interface',
        output='screen',
        parameters=[{
            'knowledge_base': 'ros2plan_knowledge_base',
            'domain_path': domain_path,
            'problem_path': problem_path,
            'problem_topic': 'problem_instance',
            'planning_language': 'PDDL',
        }],
    )

    gtest = ExecuteProcess(
        cmd=[LaunchConfiguration('test_exe')],
        name='problem_interface_gtest',
        output='screen',
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('test_exe', description='Path to the gtest executable'),
        knowledge_base,
        problem_interface,
        # Give the nodes a moment to come up and the KB to parse the domain.
        TimerAction(period=3.0, actions=[gtest]),
        launch_testing.actions.ReadyToTest(),
    ]), {'gtest': gtest}


class TestProblemInterface(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, gtest):
        # Wait for the gtest process to finish (it has its own internal timeouts).
        proc_info.assertWaitForShutdown(process=gtest, timeout=90)


@launch_testing.post_shutdown_test()
class TestProblemInterfaceExitCode(unittest.TestCase):

    def test_gtest_exit_code(self, proc_info, gtest):
        # gtest returns 0 only if all assertions passed.
        launch_testing.asserts.assertExitCodes(proc_info, process=gtest)
