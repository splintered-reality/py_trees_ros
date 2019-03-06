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
# Helpers
##############################################################################


class create_and_setup_exchange(object):
    def __init__(self):
        self.exchange = py_trees_ros.blackboard.Exchange()

    def __enter__(self):
        self.exchange.setup()
        return self.exchange

    def __exit__(self, unused_type, unused_value, unused_traceback):
        self.exchange.shutdown()

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

    @classmethod
    def tearDownClass(cls):
        console.banner("ROS Shutdown")
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        pass

    def test_exchange_setup(self):
        console.banner("Test Setup")
        print("----- Asserts -----")
        try:
            exchange = py_trees_ros.blackboard.Exchange()
            exchange.setup()
            exchange.shutdown()
            print("no exceptions [True]")
        except Exception as e:
            print("no exceptions [False]")
            self.fail("exchange.setup() raised an exception [{}]".format(str(e)))

    def test_write(self):
        console.banner("Test Write")
        with create_and_setup_exchange() as exchange:
            exchange.blackboard.dude = "Bob"
            exchange.blackboard.dudette = "Sarah"
            exchange.blackboard.count = 1
            print("----- Asserts -----")
            print("exchange.blackboard.dude: {} [{}]".format(exchange.blackboard.dude, "Bob"))
            self.assertEqual("Bob", exchange.blackboard.dude)
            print("exchange.blackboard.dudette: {} [{}]".format(exchange.blackboard.dudette, "Sarah"))
            self.assertEqual("Sarah", exchange.blackboard.dudette)
            print("exchange.blackboard.count: {} [{}]".format(exchange.blackboard.count, 1))
            self.assertEqual(1, exchange.blackboard.count)

    def test_list_variables(self):
        console.banner("Test List Variables")
        executor = rclpy.executors.SingleThreadedExecutor()
        with create_and_setup_exchange() as exchange:
            exchange.blackboard.dude = "Bob"
            blackboard_watcher = py_trees_ros.blackboard.BlackboardWatcher()
            blackboard_watcher.setup(timeout_sec=1.0)
            future = blackboard_watcher.request_list_variables()
            executor.add_node(blackboard_watcher.node)
            executor.add_node(exchange.node)
            executor.spin_until_future_complete(future)
            print("----- Asserts -----")
            print("'dude' in future.result().variables [{}]".format(
                "dude" in future.result().variables)
            )
            self.assertTrue("dude" in future.result().variables)


if __name__ == '__main__':
    unittest.main()
