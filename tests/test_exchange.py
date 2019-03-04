#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
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
# Tests
##############################################################################


class TestExchange(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        print("setupClass")
        rclpy.init()
        cls.node = rclpy.create_node(
            'test_exchange',
            start_parameter_services=False
            )
        cls.exchange = py_trees_ros.blackboard.Exchange()

    @classmethod
    def tearDownClass(cls):
        print("tearDownClass")
        cls.node.destroy_node()
        cls.exchange.shutdown()
        rclpy.shutdown()

    def setUp(self):
        print("setUp")

    def test_exchange_setup(self):
        print("test_exchange_setup")
        self.assertGreater(5.0, 2.0)
        self.assertGreater(5.0, 4.0)
        try:
            self.exchange.setup()
        except Exception as e:
            self.fail("exchange.setup() raised an exception [{}]".format(str(e)))


if __name__ == '__main__':
    unittest.main()
