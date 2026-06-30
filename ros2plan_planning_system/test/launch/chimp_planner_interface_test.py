#!/usr/bin/env python3
# Launch-test harness for the ROS2Plan CHIMP planner interface (ROS2 port of
# chimp_planner_interface.test).
#
# Brings up the knowledge base (loaded with the hierarchical CHIMP/HDDL restaurant domain) and the
# CHIMP planner interface node, then runs the gtest executable that exercises the planning service
# and the planner_output topic.
#
# NOTE: This test only runs end-to-end when ROS2Plan is built with -DROS2PLAN_COMPILE_CHIMP=ON
# (which requires the Java CHIMP planner under ros2plan_dependencies/chimp). When CHIMP is disabled
# the chimp_planner_interface executable is not built and this launch test is not registered.

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
    chimp_dir = os.path.join(pkg, 'test', 'hierarchical_domains', 'chimp', 'restaurant')
    domain_path = os.path.join(chimp_dir, 'race_waiter_domain.ddl')
    problem_path = os.path.join(chimp_dir, 'serve_coffee_problem.pdl')
    data_path = chimp_dir + os.sep

    # CHIMP is launched through its gradle wrapper; the interface substitutes the DOMAIN, PROBLEM
    # and OUTPUT tokens at run time. Requires the Java CHIMP planner under ros2plan_dependencies.
    try:
        chimp_pkg = get_package_share_directory('ros2plan_dependencies')
        chimp_path = os.path.join(chimp_pkg, 'chimp')
    except Exception:
        chimp_path = 'chimp'
    planner_command = (
        'timeout 50 ' + os.path.join(chimp_path, 'gradlew') + ' run -p ' + chimp_path +
        ' -Dexec.args="DOMAIN PROBLEM -o=OUTPUT"'
    )

    # Knowledge base loaded with the hierarchical (HDDL) CHIMP domain/problem. The planner tests
    # themselves pass explicit paths via the service request, so the KB is supplementary.
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

    chimp_planner_interface = launch_ros.actions.Node(
        package='ros2plan_planning_system',
        executable='chimp_planner_interface',
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
        name='chimp_planner_interface_gtest',
        output='screen',
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('test_exe', description='Path to the gtest executable'),
        knowledge_base,
        chimp_planner_interface,
        # Give the nodes a moment to come up and the KB to parse the domain.
        TimerAction(period=3.0, actions=[gtest]),
        launch_testing.actions.ReadyToTest(),
    ]), {'gtest': gtest}


class TestChimpPlannerInterface(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, gtest):
        # Wait for the gtest process to finish (it has its own internal timeouts).
        proc_info.assertWaitForShutdown(process=gtest, timeout=180)


@launch_testing.post_shutdown_test()
class TestChimpPlannerInterfaceExitCode(unittest.TestCase):

    def test_gtest_exit_code(self, proc_info, gtest):
        # gtest returns 0 only if all assertions passed.
        launch_testing.asserts.assertExitCodes(proc_info, process=gtest)
