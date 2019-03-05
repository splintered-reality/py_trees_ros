#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import multiprocessing
import py_trees.console as console
import rclpy
import rclpy.executors
import unittest

import py_trees_ros

##############################################################################
# Tests
##############################################################################


class TestExchange(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        console.banner("ROS Init")
        print(" - init")
        rclpy.init()
        print(" - node")
        cls.node = rclpy.create_node(
            'test_exchange',
            start_parameter_services=False
            )
        print(" - exchange")
        cls.exchange = py_trees_ros.blackboard.Exchange()

    @classmethod
    def tearDownClass(cls):
        console.banner("ROS Shutdown")
        cls.node.destroy_node()
        cls.exchange.shutdown()
        rclpy.shutdown()

    def setUp(self):
        pass

    def test_exchange_setup(self):
        console.banner("Test Setup")
        try:
            self.exchange.setup()
        except Exception as e:
            self.fail("exchange.setup() raised an exception [{}]".format(str(e)))

    def test_write(self):
        console.banner("Test Write")
        self.exchange.blackboard.dude = "Bob"
        self.exchange.blackboard.dudette = "Sarah"
        self.exchange.blackboard.count = 1
        print("exchange.blackboard.dude: {} [{}]".format(self.exchange.blackboard.dude, "Bob"))
        self.assertEqual("Bob", self.exchange.blackboard.dude)
        print("exchange.blackboard.dudette: {} [{}]".format(self.exchange.blackboard.dudette, "Sarah"))
        self.assertEqual("Sarah", self.exchange.blackboard.dudette)
        print("exchange.blackboard.count: {} [{}]".format(self.exchange.blackboard.count, 1))
        self.assertEqual(1, self.exchange.blackboard.count)

    def test_list_variables(self):
        py_trees_ros.programs.blackboard_watcher.main("--list-variables")
        watcher = multiprocessing.Process(
            target=py_trees_ros.programs.blackboard_watcher.main("--list-variables"),
            args=("--list-variables")
        )
        watcher.start()
        watcher.join()


if __name__ == '__main__':
    unittest.main()
