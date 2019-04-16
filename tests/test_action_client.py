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
import py_trees_ros
import py_trees.console as console
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

##############################################################################
# Tests
##############################################################################


class TestActionServers(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        console.banner("ROS Init")
        rclpy.init()
        cls.server = py_trees_ros.mock.dock.Dock(duration=0.5)

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

    def generic_success_test(
            self,
            title
         ):
        console.banner(title)

        root = py_trees_ros.actions.ActionClient(name="foo")
        root.setup()

        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(self.server.node)
        # executor.add_node(client.node)

        root.tick_once()

        assert_banner()
        assert_details("root.status", "RUNNING", root.status)

        executor.shutdown()
        # client.shutdown()

    def test_client_success(self):
        self.generic_success_test(title="Client Success")

##############################################################################
# Preemption
##############################################################################

##############################################################################
# Cancel
##############################################################################


if __name__ == '__main__':
    unittest.main()
