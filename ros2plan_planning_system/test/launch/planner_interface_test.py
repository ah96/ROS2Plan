#!/usr/bin/env python3
# Launch-test harness for the ROS2Plan planner interface (ROS2 port of planner_interface.test).
# Brings up the knowledge base, the problem interface and the POPF planner interface, then runs the
# gtest executable that exercises the planning service and the planner_output topic.

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
    test_domain = os.path.join(pkg, 'test', 'pddl', 'test_domain')
    domain_path = os.path.join(test_domain, 'domain.pddl')
    problem_path = os.path.join(test_domain, 'test_problem.pddl')
    data_path = test_domain + os.sep
    popf = os.path.join(pkg, 'common', 'bin', 'popf')
    planner_command = 'timeout 10 ' + popf + ' -n DOMAIN PROBLEM'

    # Knowledge base loaded with the test domain/problem so the problem interface can generate a
    # problem if needed. The planner tests themselves pass explicit paths via the service request.
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

    planner_interface = launch_ros.actions.Node(
        package='ros2plan_planning_system',
        executable='popf_planner_interface',
        name='ros2plan_planner_interface',
        output='screen',
        parameters=[{
            'use_problem_topic': False,
            'problem_topic': 'problem_instance',
            'planner_topic': 'planner_output',
            'domain_path': domain_path,
            'problem_path': problem_path,
            'data_path': data_path,
            'planner_command': planner_command,
        }],
    )

    gtest = ExecuteProcess(
        cmd=[LaunchConfiguration('test_exe')],
        name='planner_interface_gtest',
        output='screen',
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('test_exe', description='Path to the gtest executable'),
        knowledge_base,
        problem_interface,
        planner_interface,
        # Give the nodes a moment to come up and the KB to parse the domain.
        TimerAction(period=3.0, actions=[gtest]),
        launch_testing.actions.ReadyToTest(),
    ]), {'gtest': gtest}


class TestPlannerInterface(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, gtest):
        # Wait for the gtest process to finish (it has its own internal timeouts).
        proc_info.assertWaitForShutdown(process=gtest, timeout=150)


@launch_testing.post_shutdown_test()
class TestPlannerInterfaceExitCode(unittest.TestCase):

    def test_gtest_exit_code(self, proc_info, gtest):
        # gtest returns 0 only if all assertions passed.
        launch_testing.asserts.assertExitCodes(proc_info, process=gtest)
