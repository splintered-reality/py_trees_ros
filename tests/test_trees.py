#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees.console as console
import rclpy
import rclpy.executors
import time
import unittest

import py_trees_ros

##############################################################################
# Helpers
##############################################################################


class create_and_setup_tree(object):
    def __init__(self):
        root = py_trees.composites.Parallel("Tutorial")
        topics2bb = py_trees.composites.Sequence("Topics2BB")
        priorities = py_trees.composites.Selector("Priorities")
        idle = py_trees.behaviours.Running(name="Idle")
        flipper = py_trees.behaviours.Periodic(name="Flipper", n=2)

        root.add_child(topics2bb)
        root.add_child(priorities)
        priorities.add_child(flipper)
        priorities.add_child(idle)

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


class TestTrees(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        console.banner("ROS Init")
        print(" - init")
        rclpy.init()
        print(" - node")
        cls.node = rclpy.create_node(
            'test_trees',
            start_parameter_services=False
            )

    @classmethod
    def tearDownClass(cls):
        console.banner("ROS Shutdown")
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        pass

    def test_tick_tock(self):
        console.banner("Test Tick Tock")
        executor = rclpy.executors.SingleThreadedExecutor()
        with create_and_setup_tree() as tree:
            executor.add_node(tree.node)
            tree.tick_tock(period_ms=100, number_of_iterations=2)
            start_time = time.monotonic()
            while (time.monotonic() - start_time) < 0.4:
                executor.spin_once(timeout_sec=0.1)
                if tree.count > 2:
                    break
        print("----- Asserts -----")
        print("tree.count == 2................{}".format(tree.count <= 3))
        self.assertTrue(tree.count == 2)


if __name__ == '__main__':
    unittest.main()
