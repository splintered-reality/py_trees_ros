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
import rclpy
import rclpy.executors
import std_msgs.msg as std_msgs

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


def setup_module(module):
    console.banner("ROS Init")
    rclpy.init()


def teardown_module(module):
    console.banner("ROS Shutdown")
    rclpy.shutdown()


def timeout():
    return 0.3


def number_of_iterations():
    return 100


def qos_profile():
    return py_trees_ros.utilities.qos_profile_unlatched()


class EmptyPublisher(object):
    def __init__(self, node_name, qos_profile):

        self.node = rclpy.create_node(node_name)
        self._publisher = self.node.create_publisher(
            msg_type=std_msgs.Empty,
            topic="~/empty",
            qos_profile=qos_profile
        )
        self.timer = self.node.create_timer(
            timer_period_sec=0.1,
            callback=self.publish
        )

    def publish(self):
        self._publisher.publish(std_msgs.Empty())

    def shutdown(self):
        self._publisher.destroy()
        self.timer.cancel()
        self.node.destroy_timer(self.timer)
        self.node.destroy_node()

##############################################################################
# Tests
##############################################################################


def test_wait_for_data():
    console.banner("Wait for Data")

    publisher = EmptyPublisher(node_name="wait_for_data", qos_profile=qos_profile())
    root = py_trees_ros.subscribers.WaitForData(
        topic_name="/wait_for_data/empty",
        topic_type=std_msgs.Empty,
        qos_profile=qos_profile()
    )
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(publisher.node)
    executor.add_node(tree.node)

    assert_banner()

    tree.tick_tock(
        period_ms=100,
        number_of_iterations=number_of_iterations()
    )

    while tree.count < number_of_iterations() and root.status != py_trees.common.Status.SUCCESS:
        executor.spin_once(timeout_sec=0.05)

    assert_details("root.status", "SUCCESS", root.status)
    assert(root.status == py_trees.common.Status.SUCCESS)

    tree.shutdown()
    publisher.shutdown()
    executor.shutdown()
