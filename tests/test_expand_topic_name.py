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

##############################################################################
# Tests
##############################################################################


def test_snapshot_stream_topic_name():
    console.banner("Snapshot Stream Topic Names")
    pairs = {
        None: '/tree/snapshot_streams/_snapshots_0',
        "foo": "/tree/snapshot_streams/foo",
        "~/foo": "/tree/foo",
        "/bar/foo": "/bar/foo"
    }
    default_node_name = "tree"
    node = rclpy.create_node(node_name=default_node_name)
    for name, expected in pairs.items():
        expanded = py_trees_ros.trees.SnapshotStream.expand_topic_name(node, name)
        assert_details(str(name), expected, expanded)
        assert(expanded == expected)
    node.destroy_node()
