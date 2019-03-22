#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import operator
import py_trees
import py_trees.console as console
import rclpy
import rclpy.executors
import sensor_msgs.msg as sensor_msgs
import time
import unittest

import py_trees_ros

##############################################################################
# Helpers
##############################################################################


class Battery(object):

    def __init__(self):
        self.node = rclpy.create_node("battery_publisher")
        self.publisher = self.node.create_publisher(
            msg_type=sensor_msgs.BatteryState,
            topic="/battery/state"
            )
        self.timer = self.node.create_timer(
            timer_period_sec=0.1,
            callback=self.publish_battery_state
        )

    def publish_battery_state(self):
        msg = sensor_msgs.BatteryState()
        msg.percentage = 80.0
        self.publisher.publish(msg)

    def shutdown(self):
        self.timer.cancel()
        self.node.destroy_timer(self.timer)
        self.node.destroy_node()


##############################################################################
# Tests
##############################################################################


class TestBattery(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        console.banner("ROS Init")
        rclpy.init()
        cls.timeout = 0.4
        cls.blackboard = py_trees.blackboard.Blackboard()

    @classmethod
    def tearDownClass(cls):
        console.banner("ROS Shutdown")
        rclpy.shutdown()

    def setUp(self):
        pass

    def test_to_blackboard_initialised(self):
        console.banner("ToBlackboard - Initialised")
        self.blackboard.clear()
        unused_root = py_trees_ros.subscribers.ToBlackboard(
            name="ToBlackboard",
            topic_name="/battery/state",
            topic_type=sensor_msgs.BatteryState,
            blackboard_variables={'percentage': 'percentage'},
            initialise_variables={'percentage': 0.10},
            clearing_policy=py_trees.common.ClearingPolicy.NEVER)
        print("----- Asserts -----")
        print("blackboard.percentage.............{} [{}]".format(
            0.10, self.blackboard.percentage))
        self.assertTrue(self.blackboard.percentage == 0.10)

    def test_to_blackboard_not_initialised(self):
        console.banner("ToBlackboard - Uninitialised")
        self.blackboard.clear()
        unused_root = py_trees_ros.subscribers.ToBlackboard(
            name="ToBlackboard",
            topic_name="/battery/state",
            topic_type=sensor_msgs.BatteryState,
            blackboard_variables={'percentage': 'percentage'},
            initialise_variables={},
            clearing_policy=py_trees.common.ClearingPolicy.NEVER)
        print("----- Asserts -----")
        print("blackboard.percentage.............raises AttributeError")
        with self.assertRaises(AttributeError):
            print(self.blackboard.percentage)

    def impl_to_blackboard_write(self, policy):
        console.banner("ToBlackboard - Write [Policy: {}]".format(policy.name))
        self.blackboard.clear()
        battery = Battery()
        root = py_trees_ros.subscribers.ToBlackboard(
            name="ToBlackboard",
            topic_name="/battery/state",
            topic_type=sensor_msgs.BatteryState,
            blackboard_variables={'percentage': 'percentage'},
            initialise_variables={'percentage': 10.0},
            clearing_policy=policy)
        tree = py_trees_ros.trees.BehaviourTree(
            root=root,
            ascii_tree_debug=True
        )
        tree.setup()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(tree.node)
        executor.add_node(battery.node)
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < self.timeout:
            executor.spin_once(timeout_sec=0.1)
            tree.tick()
            if tree.root.status == py_trees.common.Status.SUCCESS:
                break
        print("----- Asserts -----")
        print("tree.root.status................{} [{}]".format(
            py_trees.common.Status.SUCCESS, tree.root.status))
        self.assertTrue(tree.root.status == py_trees.common.Status.SUCCESS)
        print("percentage........................80.0 [{}]".format(
            self.blackboard.percentage)
        )
        self.assertTrue(self.blackboard.percentage == 80.0)
        if policy == py_trees.common.ClearingPolicy.NEVER:
            print("message...........................!None [{}]".format(root.msg))
            self.assertTrue(root.msg is not None)
        elif policy == py_trees.common.ClearingPolicy.ON_SUCCESS:
            print("message...........................None [{}]".format(root.msg))
            self.assertTrue(root.msg is None)
        battery.shutdown()
        tree.shutdown()

    def test_to_blackboard_write_clearing_never(self):
        self.impl_to_blackboard_write(policy=py_trees.common.ClearingPolicy.NEVER)

    def test_to_blackboard_write_clearing_on_success(self):
        self.impl_to_blackboard_write(policy=py_trees.common.ClearingPolicy.ON_SUCCESS)

    def test_to_blackboard_bad_topic(self):
        console.banner("ToBlackboard - Write")
        self.blackboard.clear()
        battery = Battery()
        root = py_trees_ros.subscribers.ToBlackboard(
            name="ToBlackboard",
            topic_name="/wrong/topic",
            topic_type=sensor_msgs.BatteryState,
            blackboard_variables={'percentage': 'percentage'},
            initialise_variables={'percentage': 0.10},
            clearing_policy=py_trees.common.ClearingPolicy.NEVER)
        tree = py_trees_ros.trees.BehaviourTree(
            root=root,
            ascii_tree_debug=True
        )
        tree.setup()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(tree.node)
        executor.add_node(battery.node)
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < self.timeout:
            executor.spin_once(timeout_sec=0.1)
            tree.tick()
        print("----- Asserts -----")
        print("tree.root.status................{} [{}]".format(
            py_trees.common.Status.RUNNING, tree.root.status))
        self.assertTrue(tree.root.status == py_trees.common.Status.RUNNING)
        battery.shutdown()
        tree.shutdown()

    def test_wait_for_data_and_clear_on_initialise(self):
        console.banner("Wait For Data [Policy: ON_INITIALISE]")
        self.blackboard.clear()
        battery = Battery()
        root = py_trees_ros.subscribers.WaitForData(
            name="WaitForData",
            topic_name="/battery/state",
            topic_type=sensor_msgs.BatteryState,
            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE)
        tree = py_trees_ros.trees.BehaviourTree(
            root=root,
            ascii_tree_debug=True
        )
        tree.setup()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(tree.node)
        executor.add_node(battery.node)
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < self.timeout:
            executor.spin_once(timeout_sec=0.1)
            tree.tick()
            if tree.root.status != py_trees.common.Status.RUNNING:
                break
        print("----- Asserts -----")
        print("tree.root.status..................{} [{}]".format(
            py_trees.common.Status.SUCCESS, tree.root.status))
        self.assertTrue(tree.root.status == py_trees.common.Status.SUCCESS)
        tree.root.initialise()
        print("message...........................None [{}]".format(root.msg))
        self.assertTrue(root.msg is None)
        battery.shutdown()
        tree.shutdown()

    def impl_check_data(self, succeed, fail_on_bad_comparison=False):
        console.banner("CheckData - {}".format("Succeed" if succeed else "Fail"))
        self.blackboard.clear()
        battery = Battery()
        root = py_trees_ros.subscribers.CheckData(
            name="CheckData",
            topic_name="/battery/state",
            topic_type=sensor_msgs.BatteryState,
            variable_name="percentage",
            expected_value=80.0 if succeed else 70.0,
            fail_if_no_data=False,
            fail_if_bad_comparison=fail_on_bad_comparison,
            comparison_operator=operator.eq,
            clearing_policy=py_trees.common.ClearingPolicy.NEVER
        )
        tree = py_trees_ros.trees.BehaviourTree(
            root=root,
            ascii_tree_debug=True
        )
        tree.setup()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(tree.node)
        executor.add_node(battery.node)
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < self.timeout:
            executor.spin_once(timeout_sec=0.1)
            tree.tick()
            if tree.root.status != py_trees.common.Status.RUNNING:
                break
        print("----- Asserts -----")
        if succeed:
            expected_result = py_trees.common.Status.SUCCESS
        elif not fail_on_bad_comparison:
            expected_result = py_trees.common.Status.RUNNING
        else:
            expected_result = py_trees.common.Status.FAILURE
        print("tree.root.status................{} [{}]".format(
            expected_result,
            tree.root.status))
        self.assertTrue(tree.root.status == expected_result)
        battery.shutdown()
        tree.shutdown()

    def test_xcheck_data_succeed(self):
        self.impl_check_data(succeed=True)

    def test_xcheck_data_running_on_fail(self):
        self.impl_check_data(succeed=False)

    def test_xcheck_data_fail_on_fail(self):
        self.impl_check_data(succeed=False, fail_on_bad_comparison=True)


if __name__ == '__main__':
    unittest.main()
