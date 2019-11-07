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
import tf2_ros

import geometry_msgs.msg as geometry_msgs

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


def number_of_iterations():
    return 1000


def source_frame():
    return "noggin"


def target_frame():
    return "noodle"


def qos_profile():
    return py_trees_ros.utilities.qos_profile_unlatched()


class Broadcaster(object):
    def __init__(self):
        self.node = rclpy.create_node("broadcast")
        self.broadcaster = tf2_ros.TransformBroadcaster(
            node=self.node,
            qos=qos_profile())
        self.x = 0.3
        self.timer = self.node.create_timer(
            timer_period_sec=0.1,
            callback=self.send_transform
        )

    def send_transform(self):
        t = geometry_msgs.TransformStamped()
        t.header.stamp = rclpy.clock.Clock().now().to_msg()
        t.header.frame_id = source_frame()
        t.child_frame_id = target_frame()
        t.transform.translation.x = self.x
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        print(console.green + "Broadcast: {}".format(self.x) + console.reset)
        self.broadcaster.sendTransform(t)
        self.x += 0.1

    def shutdown(self):
        self.timer.cancel()
        self.node.destroy_timer(self.timer)
        self.node.destroy_node()


class create_behaviours(object):

    def __init__(self, static):
        self.static = static

    def __enter__(self):
        transform = geometry_msgs.Transform()
        transform.translation.x = 3.0
        transform.translation.y = 0.0
        transform.translation.z = 0.0
        transform.rotation.x = 0.0
        transform.rotation.y = 0.0
        transform.rotation.z = 0.0
        transform.rotation.w = 1.0
        py_trees.blackboard.Blackboard.set("transform", transform)
        self.from_blackboard = py_trees_ros.transforms.FromBlackboard(
            variable_name="transform",
            target_frame=target_frame(),
            source_frame=source_frame(),
            qos_profile=qos_profile(),
            static=self.static,
            name="From Blackboard"
        )
        self.to_blackboard = py_trees_ros.transforms.ToBlackboard(
            variable_name="noggin_to_noodle",
            target_frame=target_frame(),
            source_frame=source_frame(),
            qos_profile=qos_profile(),
            name="To Blackboard"
        )
        self.broadcaster_node = rclpy.create_node("broadcaster")
        self.listener_node = rclpy.create_node("listener")
        self.from_blackboard.setup(node=self.broadcaster_node)
        self.to_blackboard.setup(node=self.listener_node)
        self.executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.broadcaster_node)
        self.executor.add_node(self.listener_node)
        return (self.from_blackboard, self.to_blackboard, self.executor)

    def __exit__(self, unused_type, unused_value, unused_traceback):
        self.broadcaster_node.destroy_node()
        self.listener_node.destroy_node()
        self.executor.shutdown()


##############################################################################
# Tests
##############################################################################


def test_to_blackboard_success():
    console.banner("To Blackboard Success")
    broadcaster = Broadcaster()
    to_blackboard = py_trees_ros.transforms.ToBlackboard(
        variable_name="noggin_to_noodle",
        target_frame=target_frame(),
        source_frame=source_frame(),
        qos_profile=qos_profile(),
        name="To Blackboard"
    )
    node = rclpy.create_node("listener")
    to_blackboard.setup(node=node)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(broadcaster.node)

    count = 0
    while count < number_of_iterations() and to_blackboard.status != py_trees.common.Status.SUCCESS:
        print(console.green + "Listener::update().....{}".format(count) + console.reset)
        to_blackboard.tick_once()
        executor.spin_once(timeout_sec=0.05)
        count += 1

    assert_details("to_blackboard.status", "SUCCESS", to_blackboard.status)
    assert(to_blackboard.status == py_trees.common.Status.SUCCESS)

    broadcaster.shutdown()
    node.destroy_node()
    executor.shutdown()


def test_to_blackboard_blocking():
    console.banner("To Blackboard Blocking")
    to_blackboard = py_trees_ros.transforms.ToBlackboard(
        variable_name="noggin_to_noodle",
        target_frame=target_frame(),
        source_frame=source_frame(),
        qos_profile=qos_profile(),
        name="To Blackboard"
    )
    node = rclpy.create_node("listener")
    to_blackboard.setup(node=node)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    to_blackboard.tick_once()
    executor.spin_once(timeout_sec=0.05)

    assert_details("to_blackboard.status", "RUNNING", to_blackboard.status)
    assert(to_blackboard.status == py_trees.common.Status.RUNNING)
    assert_details(
        "blackboard.noggin_to_noodle",
        None,
        to_blackboard.blackboard.noggin_to_noodle
    )
    assert(to_blackboard.blackboard.noggin_to_noodle is None)

    node.destroy_node()
    executor.shutdown()


def test_from_blackboard():
    console.banner("From Blackboard")

    for static in {True, False}:
        with create_behaviours(static=True) as (from_blackboard, to_blackboard, executor):
            print(console.green + "---------------------------" + console.reset)
            print(console.green + "Static: {}".format(static) + console.reset)
            print(console.green + "---------------------------" + console.reset)
            count = 0
            while count < number_of_iterations() and to_blackboard.status != py_trees.common.Status.SUCCESS:
                print(console.green + "Tick.....{}".format(count) + console.reset)
                print(console.green + "Listener........{}".format(to_blackboard.feedback_message) + console.reset)
                print(console.green + "Broadcaster.....{}".format(from_blackboard.feedback_message) + console.reset)
                from_blackboard.tick_once()
                to_blackboard.tick_once()
                executor.spin_once(timeout_sec=0.05)
                count += 1

    assert_details("to_blackboard.status", "SUCCESS", to_blackboard.status)
    assert(to_blackboard.status == py_trees.common.Status.SUCCESS)
