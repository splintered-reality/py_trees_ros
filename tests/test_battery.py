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
import rclpy
import rclpy.executors
import sensor_msgs.msg as sensor_msgs
import time
import unittest

import py_trees_ros

##############################################################################
# Helpers
##############################################################################


class Battery(object):

    def __init__(self):
        self.node = rclpy.create_node("battery_publisher")
        self.publisher = self.node.create_publisher(
            msg_type=sensor_msgs.BatteryState,
            topic="/battery/state"
            )
        self.timer = self.node.create_timer(
            timer_period_sec=0.1,
            callback=self.publish_battery_state
        )

    def publish_battery_state(self):
        msg = sensor_msgs.BatteryState()
        msg.percentage = 80.0
        self.publisher.publish(msg)

    def shutdown(self):
        self.timer.cancel()
        self.node.destroy_timer(self.timer)
        self.node.destroy_node()


class create_and_setup_tree(object):
    def __init__(self):
        root = py_trees_ros.battery.ToBlackboard("BatteryToBB")

        self.tree = py_trees_ros.trees.BehaviourTree(
            root=root,
            ascii_tree_debug=True
        )

    def __enter__(self):
        self.tree.setup()
        return self.tree

    def __exit__(self, unused_type, unused_value, unused_traceback):
        self.tree.shutdown()

##############################################################################
# Tests
##############################################################################


class TestBattery(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        console.banner("ROS Init")
        rclpy.init()
        cls.timeout = 0.4

    @classmethod
    def tearDownClass(cls):
        console.banner("ROS Shutdown")
        rclpy.shutdown()

    def setUp(self):
        pass

    def test_battery(self):
        console.banner("Test Battery Message")
        executor = rclpy.executors.SingleThreadedExecutor()
        battery = Battery()
        blackboard = py_trees.blackboard.Blackboard()
        with create_and_setup_tree() as tree:
            executor.add_node(tree.node)
            executor.add_node(battery.node)
            start_time = time.monotonic()
            while (time.monotonic() - start_time) < self.timeout:
                executor.spin_once(timeout_sec=0.1)
                tree.tick()
                if tree.root.status == py_trees.common.Status.SUCCESS:
                    break
        print("----- Asserts -----")
        print("tree.root.status..................{} [{}]".format(
            py_trees.common.Status.SUCCESS, tree.root.status))
        self.assertTrue(tree.root.status == py_trees.common.Status.SUCCESS)
        print("battery/percentage................80.0 [{}]".format(
            blackboard.battery.percentage)
        )
        self.assertTrue(blackboard.battery.percentage == 80.0)
        battery.shutdown()


if __name__ == '__main__':
    unittest.main()
