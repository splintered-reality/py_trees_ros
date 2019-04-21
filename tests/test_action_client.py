#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import action_msgs.msg as action_msgs  # GoalStatus
import py_trees
import py_trees.console as console
import py_trees_ros
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy
import rclpy.executors
import time
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
        action_goal=py_trees_actions.Dock.Goal(dock=True),
        generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.percentage_completed)
    )
    return behaviour


def print_ascii_tree(tree):
    print(
        "\n" +
        py_trees.display.ascii_tree(
            tree.root,
            visited=tree.snapshot_visitor.visited,
            previously_visited=tree.snapshot_visitor.previously_visited
        )
    )

##############################################################################
# Tests
##############################################################################


class TestActionServers(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        console.banner("ROS Init")
        rclpy.init()
        cls.server = py_trees_ros.mock.dock.Dock(duration=1.5)

        cls.timeout = 3.0
        cls.blackboard = py_trees.blackboard.Blackboard()

    @classmethod
    def tearDownClass(cls):
        console.banner("ROS Shutdown")
        cls.server.shutdown()
        rclpy.shutdown()

    def setUp(self):
        pass

##############################################################################
# Success
##############################################################################

    def foo_test_success(self):
        console.banner("Client Success")

        root = create_action_client()
        tree = py_trees_ros.trees.BehaviourTree(root=root, ascii_tree_debug=False)
        tree.setup()

        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(self.server.node)
        executor.add_node(tree.node)

        assert_banner()

        assert_details("root.status", "INVALID", root.status)
        self.assertEqual(root.status, py_trees.common.Status.INVALID)

        tree.tick()

        assert_details("root.status", "RUNNING", root.status)
        self.assertEqual(root.status, py_trees.common.Status.RUNNING)

        number_of_iterations = 100
        tree.tick_tock(
            period_ms=100,
            number_of_iterations=number_of_iterations
        )

        while tree.count < number_of_iterations and root.status == py_trees.common.Status.RUNNING:
            executor.spin_once(timeout_sec=0.05)

        assert_details("root.status", "SUCCESS", root.status)
        self.assertEqual(root.status, py_trees.common.Status.SUCCESS)

        tree.shutdown()
        executor.shutdown()

    def test_priority_interrupt(self):
        console.banner("Priority Interrupt")

        number_of_iterations = 5

        action_client = create_action_client()
        success_eventually = py_trees.behaviours.Count(
            name="Success Eventually",
            fail_until=number_of_iterations-1,
            success_until=100
        )
        root = py_trees.composites.Selector()
        root.add_children([success_eventually, action_client])
        tree = py_trees_ros.trees.BehaviourTree(root=root, ascii_tree_debug=False)
        tree.setup()

        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(self.server.node)
        executor.add_node(tree.node)

        assert_banner()

        tree.tick()
        print_ascii_tree(tree)

        assert_details("action_client.status", "RUNNING", root.status)
        self.assertEqual(action_client.status, py_trees.common.Status.RUNNING)

        tree.tick_tock(
            period_ms=100,
            number_of_iterations=number_of_iterations
        )

        while tree.count < number_of_iterations:
            executor.spin_once(timeout_sec=0.1)
        print_ascii_tree(tree)
        assert_details("action_client.status", "INVALID", action_client.status)
        self.assertEqual(action_client.status, py_trees.common.Status.INVALID)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        tree.shutdown()
        executor.shutdown()

##############################################################################
# Preemption
##############################################################################

##############################################################################
# Cancel
##############################################################################


if __name__ == '__main__':
    unittest.main()
