#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

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
        self.node = rclpy.create_node('test_exchange', start_parameter_services=False)

    def __enter__(self):
        self.exchange.setup(self.node)
        return self.exchange

    def __exit__(self, unused_type, unused_value, unused_traceback):
        self.node.destroy_node()


class BlackboardListener(object):
    def __init__(self):
        self.data = None  # str object

    def callback(self, msg):
        self.data = msg.data

##############################################################################
# Tests
##############################################################################


class TestExchange(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        console.banner("ROS Init")
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        console.banner("ROS Shutdown")
        rclpy.shutdown()

    def setUp(self):
        pass

    def test_exchange_setup(self):
        console.banner("Test Setup")
        print("----- Asserts -----")
        try:
            exchange = py_trees_ros.blackboard.Exchange()
            node = rclpy.create_node('test_exchange', start_parameter_services=False)
            exchange.setup(node)
            node.destroy_node()
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

    def test_watcher(self):
        console.banner("Test Watcher")
        executor = rclpy.executors.SingleThreadedExecutor()
        with create_and_setup_exchange() as exchange:
            exchange.blackboard.count = 1
            blackboard_watcher = py_trees_ros.blackboard.BlackboardWatcher()
            blackboard_watcher.setup(timeout_sec=1.0)
            blackboard_listener = BlackboardListener()
            executor.add_node(blackboard_watcher.node)
            executor.add_node(exchange.node)
            blackboard_watcher.open_connection(
                variables=[],
                callback=blackboard_listener.callback,
                executor=executor
            )
            while not blackboard_listener.data:
                exchange.blackboard.count += 1
                exchange.publish_blackboard()
                executor.spin_once(timeout_sec=0.5)
            print("----- Asserts -----")
            print("'count' in blackboard contents [{}]".format(
                "count" in blackboard_listener.data)
            )
            self.assertTrue("count" in blackboard_listener.data)

    def test_view(self):
        console.banner("Test View")
        executor = rclpy.executors.SingleThreadedExecutor()
        with create_and_setup_exchange() as exchange:
            exchange.blackboard.dude = "Bob"
            exchange.blackboard.count = 1
            blackboard_watcher = py_trees_ros.blackboard.BlackboardWatcher()
            blackboard_watcher.setup(timeout_sec=1.0)
            blackboard_listener = BlackboardListener()
            executor.add_node(blackboard_watcher.node)
            executor.add_node(exchange.node)
            blackboard_watcher.open_connection(
                variables=["count"],
                callback=blackboard_listener.callback,
                executor=executor
            )
            while not blackboard_listener.data:
                exchange.blackboard.count += 1
                exchange.publish_blackboard()
                executor.spin_once(timeout_sec=0.5)
            print("----- Asserts -----")
            print("'count' in blackboard contents [{}]".format(
                "count" in blackboard_listener.data)
            )
            self.assertTrue("count" in blackboard_listener.data)


if __name__ == '__main__':
    unittest.main()
