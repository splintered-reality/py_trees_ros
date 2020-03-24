#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import threading

import py_trees
import py_trees.console as console
import py_trees_ros
import rclpy

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

##############################################################################
# Tests
##############################################################################


def test_blackboard_pickle():
    console.banner("Test Pickle Failure")
    node = rclpy.create_node("pickle")
    print("Set 'foo' to a thread lock object")
    py_trees.blackboard.Blackboard.set("foo", threading.Lock())
    print("Create Sub Blackboard")
    sub_blackboard = py_trees_ros.blackboard.SubBlackboard(node=node)
    print("Update with warning - will raise exceptions if pickle errors are not caught")
    sub_blackboard.update({"foo"})
    assert(True)
    print("Update Quietly")
    sub_blackboard.update({"foo"})
    assert(True)
