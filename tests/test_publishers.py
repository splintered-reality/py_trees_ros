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
import pytest
import rclpy
import rclpy.executors
import std_msgs.msg as std_msgs
import time

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


def topic_name():
    return "/empty"


def blackboard_key():
    return "/empty"


def timeout():
    return 0.3


def number_of_iterations():
    return 3


def qos_profile():
    return py_trees_ros.utilities.qos_profile_unlatched()


class Subscriber(object):
    def __init__(self, node_name, topic_name, topic_type, qos_profile):

        self.node = rclpy.create_node(node_name)
        self.subscriber = self.node.create_subscription(
            msg_type=topic_type,
            topic=topic_name,
            callback=self.callback,
            qos_profile=qos_profile
        )
        self.count = 0

    def callback(self, msg):
        self.count += 1

    def shutdown(self):
        self.subscriber.destroy()
        self.node.destroy_node()


def create_all_the_things():
    subscriber = Subscriber(
        node_name="catcher_of_nothing",
        topic_name=topic_name(),
        topic_type=std_msgs.Empty,
        qos_profile=qos_profile()
    )
    root = py_trees_ros.publishers.FromBlackboard(
        topic_name=topic_name(),
        topic_type=std_msgs.Empty,
        qos_profile=qos_profile(),
        blackboard_variable=blackboard_key()
    )
    return subscriber, root

##############################################################################
# Tests
##############################################################################


def test_publish_with_existing_data():
    console.banner("Publish Existing Data")

    py_trees.blackboard.Blackboard.set(
        variable_name=blackboard_key(),
        value=std_msgs.Empty()
    )

    subscriber, root = create_all_the_things()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(subscriber.node)
    executor.add_node(tree.node)

    assert_banner()

    start_time = time.monotonic()
    while ((time.monotonic() - start_time) < timeout()) and subscriber.count == 0:
        tree.tick()
        executor.spin_once(timeout_sec=0.05)

    assert_details("root.status", "SUCCESS", root.status)
    assert(root.status == py_trees.common.Status.SUCCESS)

    py_trees.blackboard.Blackboard.clear()
    tree.shutdown()
    subscriber.shutdown()
    executor.shutdown()


def test_fail_with_no_data():
    console.banner("Fail with No Data")

    subscriber, root = create_all_the_things()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(subscriber.node)
    executor.add_node(tree.node)
    assert_banner()

    tree.tick()
    executor.spin_once(timeout_sec=0.05)

    assert_details("root.status", "FAILURE", root.status)
    assert(root.status == py_trees.common.Status.FAILURE)

    tree.shutdown()
    subscriber.shutdown()
    executor.shutdown()


def test_exception_with_wrong_data():
    console.banner("Exception with Wrong Data")

    py_trees.blackboard.Blackboard.set(
        variable_name=blackboard_key(),
        value=5.0
    )

    subscriber, root = create_all_the_things()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(subscriber.node)
    executor.add_node(tree.node)
    assert_banner()

    with pytest.raises(TypeError) as unused_e_info:  # e_info survives outside this block
        tree.tick()

    tree.shutdown()
    subscriber.shutdown()
    executor.shutdown()
