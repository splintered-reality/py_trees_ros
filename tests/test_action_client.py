#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees.console as console
import py_trees_ros
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy
import rclpy.action
import rclpy.executors
import unittest

##############################################################################
# Helpers
##############################################################################


def assert_banner():
    print(console.green + "----- Asserts -----" + console.reset)


def assert_details(text, expected, result):
    print(console.green + text +
          "." * (40 - len(text)) +
          console.cyan + "{}".format(expected) +
          console.yellow + " [{}]".format(result) +
          console.reset)


def create_action_client():
    behaviour = py_trees_ros.actions.ActionClient(
        name="dock",
        action_type=py_trees_actions.Dock,
        action_name="dock",
        action_goal=py_trees_actions.Dock.Goal(dock=True),  # noqa
        generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)
    )
    return behaviour


def print_unicode_tree(tree):
    print(
        "\n" +
        py_trees.display.unicode_tree(
            tree.root,
            visited=tree.snapshot_visitor.visited,
            previously_visited=tree.snapshot_visitor.previously_visited
        )
    )


class RejectGoalServer(object):
    def __init__(self, node_name, action_name, action_type):
        self.node = rclpy.create_node(node_name)

        self.action_server = rclpy.action.ActionServer(
            node=self.node,
            action_type=action_type,
            action_name=action_name,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),  # needed?
            execute_callback=lambda goal_handle: None,
            goal_callback=self.goal_callback
        )

    def goal_callback(self, unused_goal_request):
        return rclpy.action.server.GoalResponse.REJECT

    def shutdown(self):
        self.action_server.destroy()
        self.node.destroy_node()


class DockFailedServer(py_trees_ros.mock.dock.Dock):
    def __init__(self):
        super().__init__()

    def execute_goal_callback(self, goal_handle):
        result = self.action_type.Result()
        goal_handle.abort()
        return result

##############################################################################
# Tests
##############################################################################


class TestActionServers(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        console.banner("ROS Init")
        rclpy.init()

        cls.timeout = 3.0
        cls.number_of_iterations = 100

    @classmethod
    def tearDownClass(cls):
        console.banner("ROS Shutdown")
        rclpy.shutdown()

    def setUp(self):
        pass

    ########################################
    # Success
    ########################################

    def test_success(self):
        console.banner("Client Success")

        server = py_trees_ros.mock.dock.Dock(duration=1.5)

        root = create_action_client()
        tree = py_trees_ros.trees.BehaviourTree(
            root=root,
            unicode_tree_debug=False
        )
        tree.setup()

        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(server.node)
        executor.add_node(tree.node)

        assert_banner()

        assert_details("root.status", "INVALID", root.status)
        self.assertEqual(root.status, py_trees.common.Status.INVALID)

        tree.tick()

        assert_details("root.status", "RUNNING", root.status)
        self.assertEqual(root.status, py_trees.common.Status.RUNNING)

        tree.tick_tock(
            period_ms=100,
            number_of_iterations=self.number_of_iterations
        )

        while tree.count < self.number_of_iterations and root.status == py_trees.common.Status.RUNNING:
            executor.spin_once(timeout_sec=0.05)

        assert_details("root.status", "SUCCESS", root.status)
        self.assertEqual(root.status, py_trees.common.Status.SUCCESS)

        tree.shutdown()
        server.shutdown()
        executor.shutdown()

    ########################################
    # Priority Interrupt
    ########################################

    def test_priority_interrupt(self):
        console.banner("Priority Interrupt")

        server = py_trees_ros.mock.dock.Dock(duration=1.5)

        action_client = create_action_client()
        success_eventually = py_trees.behaviours.Count(
            name="Success Eventually",
            fail_until=4,
            success_until=1000
        )
        root = py_trees.composites.Selector()
        root.add_children([success_eventually, action_client])
        tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
        tree.setup()

        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(server.node)
        executor.add_node(tree.node)

        assert_banner()

        tree.tick()
        print_unicode_tree(tree)

        assert_details("action_client.status", "RUNNING", root.status)
        self.assertEqual(action_client.status, py_trees.common.Status.RUNNING)

        tree.tick_tock(
            period_ms=100,
            number_of_iterations=self.number_of_iterations
        )

        while tree.count < self.number_of_iterations and "cancelled" not in action_client.feedback_message:
            executor.spin_once(timeout_sec=0.1)
        print_unicode_tree(tree)
        assert_details("action_client.status", "INVALID", action_client.status)
        self.assertEqual(action_client.status, py_trees.common.Status.INVALID)

        # hack to make sure action client is in a state that lets it
        # shut down without segfaulting
        if action_client.get_result_future is not None:
            while not action_client.get_result_future.done():
                executor.spin_once(timeout_sec=0.1)

        tree.shutdown()
        server.shutdown()
        executor.shutdown()

    ########################################
    # Rejection
    ########################################

    def test_rejection(self):
        console.banner("Client Rejection")

        server = RejectGoalServer(
            node_name="reject",
            action_name="dock",
            action_type=py_trees_actions.Dock,
        )

        root = create_action_client()
        tree = py_trees_ros.trees.BehaviourTree(root=root)

        # ROS Setup
        tree.setup()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(server.node)
        executor.add_node(tree.node)
        tree.tick()
        tree.tick_tock(period_ms=100, number_of_iterations=self.number_of_iterations)

        # ROS Spin
        while tree.count < self.number_of_iterations and root.status == py_trees.common.Status.RUNNING:
            executor.spin_once(timeout_sec=0.05)

        print("")
        assert_banner()
        assert_details("root.status", "FAILURE", root.status)
        self.assertEqual(root.status, py_trees.common.Status.FAILURE)

        tree.shutdown()
        server.shutdown()
        executor.shutdown()

    ########################################
    # Aborted
    ########################################

    def test_aborted(self):
        console.banner("Server Aborted")

        server = DockFailedServer()

        root = create_action_client()
        tree = py_trees_ros.trees.BehaviourTree(root=root)

        # ROS Setup
        tree.setup()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(server.node)
        executor.add_node(tree.node)
        tree.tick()
        tree.tick_tock(period_ms=100, number_of_iterations=self.number_of_iterations)

        # ROS Spin
        while tree.count < self.number_of_iterations and root.status == py_trees.common.Status.RUNNING:
            executor.spin_once(timeout_sec=0.05)

        print("")
        assert_banner()
        assert_details("root.status", "FAILURE", root.status)
        self.assertEqual(root.status, py_trees.common.Status.FAILURE)

        tree.shutdown()
        server.shutdown()
        executor.shutdown()

##############################################################################
# Main
##############################################################################


if __name__ == '__main__':
    unittest.main()
