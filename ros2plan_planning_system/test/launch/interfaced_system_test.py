#!/usr/bin/env python3
# Integration test for the *shipped* launch files.
#
# The other launch tests build their own launch descriptions, so they pass even when the
# launch files under launch/ are broken. This one includes interfaced_planning_system.launch.py
# and simulated_action.launch.py as users actually run them, and drives the full pipeline:
#   problem_generation_server -> planning_server -> parse_plan -> dispatch_plan
# It fails if any node dies on startup, if the topic wiring between stages is not connected,
# or if the plan does not reach the simulated action interfaces.

import os
import unittest

from ament_index_python.packages import get_package_share_directory

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

import pytest

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_srvs.srv import Empty
from ros2plan_dispatch_msgs.srv import DispatchService
from ros2plan_dispatch_msgs.msg import ActionDispatch, EsterelPlan

# PDDL actions in the turtlebot domain; each needs an interface for dispatch to complete.
ACTIONS = ['goto_waypoint', 'localise', 'dock', 'undock']


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    pkg = get_package_share_directory('ros2plan_planning_system')
    turtlebot = os.path.join(pkg, 'test', 'pddl', 'turtlebot')
    launch_dir = os.path.join(pkg, 'launch')

    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'interfaced_planning_system.launch.py')),
        launch_arguments={
            'problem_path': os.path.join(turtlebot, 'initial_state_problem.pddl'),
        }.items(),
    )

    simulated = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'includes', 'simulated_action.launch.py')),
            launch_arguments={
                'pddl_action_name': action,
                'action_duration': '0.1',
            }.items(),
        )
        for action in ACTIONS
    ]

    return launch.LaunchDescription([
        system,
        # Give the knowledge base time to parse the domain before the action interfaces
        # query it for operator details.
        TimerAction(period=5.0, actions=simulated),
        TimerAction(period=10.0, actions=[launch_testing.actions.ReadyToTest()]),
    ])


class TestInterfacedSystem(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('interfaced_system_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, srv_type, name, request=None, timeout=60.0):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout),
            'service {} never appeared -- the node advertising it probably died'.format(name))
        future = client.call_async(request if request is not None else srv_type.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), 'service {} did not respond'.format(name))
        return future.result()

    def _collect(self, msg_type, topic, timeout=60.0):
        """Subscribe and return the first message, or None on timeout.

        These publishers are latched (transient_local), and the plan is published by the
        parse_plan call before this subscription exists, so the QoS has to match to receive
        the already-published message.
        """
        received = []
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.node.create_subscription(
            msg_type, topic, lambda m: received.append(m), qos)
        end = self.node.get_clock().now().nanoseconds + int(timeout * 1e9)
        while not received and self.node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return received[0] if received else None

    def _wait_for_subscribers(self, topic, count, timeout=60.0):
        """Block until `count` subscribers are on `topic`.

        The action interfaces each query the knowledge base for operator details before
        subscribing, so how long they take to appear depends on machine load. Waiting for
        them is what makes this test deterministic -- and it matters beyond the assertion:
        dispatching before they are all up leaves the dispatcher blocked when the test tears
        down, which turns into a shutdown-time abort.
        """
        end = self.node.get_clock().now().nanoseconds + int(timeout * 1e9)
        found = 0
        while self.node.get_clock().now().nanoseconds < end:
            found = self.node.count_subscribers(topic)
            if found >= count:
                return found
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return found

    def test_1_pipeline_runs_to_a_dispatched_plan(self):
        subscribers = self._wait_for_subscribers('/action_dispatch', len(ACTIONS))
        self.assertGreaterEqual(
            subscribers, len(ACTIONS),
            'only {}/{} simulated action interfaces subscribed to /action_dispatch'.format(
                subscribers, len(ACTIONS)))

        # The services live at the root namespace, not under the node name.
        self._call(Empty, '/problem_generation_server')
        self._call(Empty, '/planning_server')
        self._call(Empty, '/parse_plan')

        # The parser publishes here only if it received the planner output over /planner_output,
        # which is the wiring that used to be broken.
        plan = self._collect(EsterelPlan, '/complete_plan', timeout=30.0)
        self.assertIsNotNone(plan, 'no plan on /complete_plan: parser -> dispatcher wiring')
        self.assertGreater(len(plan.nodes), 0, 'parsed plan has no nodes')

        result = self._call(DispatchService, '/dispatch_plan', timeout=120.0)
        self.assertTrue(result.success, 'dispatch reported failure')
        self.assertTrue(
            result.goal_achieved,
            'plan dispatched but goal not achieved -- actions did not execute')

    def test_2_dispatcher_and_interfaces_share_topics(self):
        # A publisher with no subscriber (or vice versa) is the signature of the launch-file
        # topic mismatch this suite exists to catch.
        self.assertGreater(
            self.node.count_publishers('/action_dispatch'), 0,
            'nothing publishes /action_dispatch: the dispatcher is on a different topic')
        self.assertGreater(
            self.node.count_publishers('/action_feedback'), 0,
            'nothing publishes /action_feedback: the action interfaces are on a different topic')


@launch_testing.post_shutdown_test()
class TestInterfacedSystemShutdown(unittest.TestCase):

    def test_no_node_crashed(self, proc_info):
        # Catches the startup crashes this test exists to prevent (e.g. a launch file
        # passing an int where a node declared a double).
        for info in proc_info:
            self.assertIn(
                info.returncode, (0, -2, -15),
                'process {} exited with {}'.format(info.process_name, info.returncode))
