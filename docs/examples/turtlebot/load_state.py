#!/usr/bin/env python3
"""
Load the turtlebot initial state into the ROS2Plan Knowledge Base.

Usage:
    source ~/ros2_ws/install/setup.bash
    python3 load_state.py [--waypoints N] [--robot ROBOT_NAME]
"""

import argparse
import sys
import rclpy
from rclpy.node import Node
from ros2plan_knowledge_msgs.msg import KnowledgeItem
from ros2plan_knowledge_msgs.srv import KnowledgeUpdateServiceArray
from diagnostic_msgs.msg import KeyValue


class StateLoader(Node):
    def __init__(self, waypoints, robot):
        super().__init__('turtlebot_state_loader')
        self._waypoints = waypoints
        self._robot = robot
        self._kb = 'ros2plan_knowledge_base'

        # Satellite node to make blocking service calls
        self._svc_node = rclpy.create_node('state_loader_svc')
        self._client = self._svc_node.create_client(
            KnowledgeUpdateServiceArray,
            f'/{self._kb}/update_array')

    def call_update(self, items):
        req = KnowledgeUpdateServiceArray.Request()
        req.knowledge = items
        req.update_type = [KnowledgeUpdateServiceArray.Request.ADD_KNOWLEDGE] * len(items)
        if not self._client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('KB update service not available')
            return False
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._svc_node, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().error('KB update timed out')
            return False
        return future.result().success

    def load(self):
        items = []

        # --- Waypoint instances ---
        for i in range(self._waypoints):
            ki = KnowledgeItem()
            ki.knowledge_type = KnowledgeItem.INSTANCE
            ki.instance_type = 'waypoint'
            ki.instance_name = f'wp{i}'
            items.append(ki)

        # --- Robot instance ---
        ki = KnowledgeItem()
        ki.knowledge_type = KnowledgeItem.INSTANCE
        ki.instance_type = 'robot'
        ki.instance_name = self._robot
        items.append(ki)

        # --- Facts: robot_at, docked, dock_at ---
        for pred, params in [
            ('robot_at',  [self._robot, 'wp0']),
            ('docked',    [self._robot]),
            ('dock_at',   ['wp0']),
        ]:
            ki = KnowledgeItem()
            ki.knowledge_type = KnowledgeItem.FACT
            ki.attribute_name = pred
            ki.values = [KeyValue(key=f'?p{j}', value=v) for j, v in enumerate(params)]
            items.append(ki)

        # --- Initial fact: the robot's start waypoint counts as visited.
        #     A robot cannot goto_waypoint its own current waypoint, so wp0 has
        #     to be visited up front for the 'visited wp0' goal to be reachable.
        #     The remaining waypoints are NOT added here — they are the goal,
        #     set separately in set_goals(). (Adding them as facts here would
        #     leave the goal already satisfied and the planner would return an
        #     empty plan.) ---
        ki = KnowledgeItem()
        ki.knowledge_type = KnowledgeItem.FACT
        ki.attribute_name = 'visited'
        ki.values = [KeyValue(key='?wp', value='wp0')]
        items.append(ki)

        ok = self.call_update(items)
        if ok:
            self.get_logger().info(
                f'Loaded {len(items)} knowledge items ({self._waypoints} waypoints, '
                f'robot: {self._robot})')
        else:
            self.get_logger().error('Failed to load state')
        return ok

    def set_goals(self):
        goal_items = []
        for i in range(self._waypoints):
            ki = KnowledgeItem()
            ki.knowledge_type = KnowledgeItem.FACT
            ki.attribute_name = 'visited'
            ki.values = [KeyValue(key='?wp', value=f'wp{i}')]
            goal_items.append(ki)

        ki = KnowledgeItem()
        ki.knowledge_type = KnowledgeItem.FACT
        ki.attribute_name = 'docked'
        ki.values = [KeyValue(key='?v', value=self._robot)]
        goal_items.append(ki)

        # Use the goal update service
        goal_svc = self._svc_node.create_client(
            KnowledgeUpdateServiceArray,
            f'/{self._kb}/update_array')
        req = KnowledgeUpdateServiceArray.Request()
        req.knowledge = goal_items
        req.update_type = [KnowledgeUpdateServiceArray.Request.ADD_GOAL] * len(goal_items)
        if not goal_svc.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('KB goal service not available')
            return False
        future = goal_svc.call_async(req)
        rclpy.spin_until_future_complete(self._svc_node, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().error('KB goal update timed out')
            return False
        self.get_logger().info(f'Set {len(goal_items)} goals')
        return future.result().success


def main():
    parser = argparse.ArgumentParser(description='Load turtlebot state into ROS2Plan KB')
    parser.add_argument('--waypoints', type=int, default=5,
                        help='Number of waypoints (default: 5)')
    parser.add_argument('--robot', type=str, default='kenny',
                        help='Robot name (default: kenny)')
    args = parser.parse_args()

    rclpy.init()
    loader = StateLoader(args.waypoints, args.robot)
    ok = loader.load()
    loader.set_goals()
    loader.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
