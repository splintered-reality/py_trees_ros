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
from std_srvs.srv import SetBool, Trigger
import rclpy
import rclpy.action
import rclpy.executors
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


def create_service_client(from_blackboard=False, key_response=None, wait_for_server_timeout_sec=2.0):
    if from_blackboard:
        behaviour = py_trees_ros.service_clients.FromBlackboard(
            name="set_bool_client",
            service_type=SetBool,
            service_name="set_bool",
            key_request="request",
            key_response=key_response,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec
        )
    else:
        behaviour = py_trees_ros.service_clients.FromConstant(
            name="set_bool_client",
            service_type=SetBool,
            service_name="set_bool",
            service_request=SetBool.Request(data=True),
            key_response=key_response,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec
        )
    return behaviour


def print_unicode_tree(tree):
    print(
        "\n" +
        py_trees.display.unicode_tree(
            tree.root,
            visited=tree.snapshot_visitor.visited,
            previously_visited=tree.snapshot_visitor.previously_visited
        )
    )


def setup_module(module):
    console.banner("ROS Init")
    rclpy.init()


def teardown_module(module):
    console.banner("ROS Shutdown")
    rclpy.shutdown()


def timeout():
    return 3.0


def number_of_iterations():
    return 100

##############################################################################
# Tests
##############################################################################


########################################
# Success, From Constant
########################################

def test_success():
    console.banner("Client Success")

    server = py_trees_ros.mock.set_bool.SetBoolServer(sleep_time=1.0)

    root = create_service_client()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=False
    )
    tree.setup()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(server.node)
    executor.add_node(tree.node)

    assert_banner()

    assert_details("root.status", "INVALID", root.status)
    assert(root.status == py_trees.common.Status.INVALID)

    tree.tick()

    assert_details("root.status", "RUNNING", root.status)
    assert(root.status == py_trees.common.Status.RUNNING)

    tree.tick_tock(
        period_ms=100,
        number_of_iterations=number_of_iterations()
    )

    while tree.count < number_of_iterations() and root.status == py_trees.common.Status.RUNNING:
        executor.spin_once(timeout_sec=0.05)

    assert_details("root.status", "SUCCESS", root.status)
    assert(root.status == py_trees.common.Status.SUCCESS)

    tree.shutdown()
    server.shutdown()
    executor.shutdown()

########################################
# Success, From Constant, Write Response to Blackboard
########################################

def test_success_write_to_blackboard():
    console.banner("Client Success, Write to Blackboard")

    server = py_trees_ros.mock.set_bool.SetBoolServer(sleep_time=1.0)

    root = create_service_client(key_response="response")
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=False
    )
    tree.setup()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(server.node)
    executor.add_node(tree.node)

    assert_banner()

    tree.tick()

    tree.tick_tock(
        period_ms=100,
        number_of_iterations=number_of_iterations()
    )

    while tree.count < number_of_iterations() and root.status == py_trees.common.Status.RUNNING:
        executor.spin_once(timeout_sec=0.05)

    assert_details("root.status", "SUCCESS", root.status)
    assert(root.status == py_trees.common.Status.SUCCESS)

    blackboard = py_trees.blackboard.Client()
    blackboard.register_key(key="response", access=py_trees.common.Access.READ)
    assert_details("blackboard.response.success", True, blackboard.response.success)
    assert(blackboard.response.success == True)
    assert_details("blackboard.response.message", py_trees_ros.mock.set_bool.SetBoolServer.SUCCESS_MESSAGE, blackboard.response.message)
    assert(blackboard.response.message == py_trees_ros.mock.set_bool.SetBoolServer.SUCCESS_MESSAGE)

    tree.shutdown()
    server.shutdown()
    executor.shutdown()

########################################
# Success, From Blackboard
########################################

def test_success_from_blackboard():
    console.banner("Client Success, From Blackboard")

    server = py_trees_ros.mock.set_bool.SetBoolServer(sleep_time=1.0)

    root = create_service_client(from_blackboard=True)
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=False
    )
    tree.setup()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(server.node)
    executor.add_node(tree.node)

    assert_banner()

    tree.tick()

    # Nothing on blackboard yet
    assert_details("No request on blackboard - root.status", "FAILURE", root.status)
    assert(root.status == py_trees.common.Status.FAILURE)

    py_trees.blackboard.Blackboard.set(
        variable_name="/request",
        value=SetBool.Request(data=True)  # noqa
    )

    tree.tick()

    # Request exists on blackboard
    assert_details("Request exists - root.status", "RUNNING", root.status)
    assert(root.status == py_trees.common.Status.RUNNING)

    tree.shutdown()
    server.shutdown()
    executor.shutdown()

########################################
# Success, Long Service Call
########################################

def test_success_long_service_call():
    console.banner("Client Success, Long Service Call")

    service_time = 5.0 # secs
    server = py_trees_ros.mock.set_bool.SetBoolServer(sleep_time=service_time)

    root = create_service_client()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=False
    )
    tree.setup()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(server.node)
    executor.add_node(tree.node)

    assert_banner()

    assert_details("root.status", "INVALID", root.status)
    assert(root.status == py_trees.common.Status.INVALID)

    start_time = time.monotonic()
    while time.monotonic() - start_time < service_time:
        tree.tick()
        assert_details("Remaining time %f secs - root.status" % (service_time - (time.monotonic() - start_time)), "RUNNING", root.status)
        assert(root.status == py_trees.common.Status.RUNNING)
        time.sleep(0.2)
        executor.spin_once(timeout_sec=0.05)

    time.sleep(0.2)
    tree.tick()

    assert_details("root.status", "SUCCESS", root.status)
    assert(root.status == py_trees.common.Status.SUCCESS)

    tree.shutdown()
    server.shutdown()
    executor.shutdown()

########################################
# Failure, Server does not exist
########################################


def test_failure_server_does_not_exist():
    console.banner("Client Failure, Server does not exist")

    timeout = 0.1
    root = create_service_client(from_blackboard=True, wait_for_server_timeout_sec=timeout)
    tree = py_trees_ros.trees.BehaviourTree(root=root)

    start_time = time.monotonic()
    try:
        tree.setup()
    except py_trees_ros.exceptions.TimedOutError:
        duration = time.monotonic() - start_time
    assert_details("Tree Setup Timeout", "delta+{}".format(timeout), duration)
    assert(duration < 10*timeout)

    tree.shutdown()

########################################
# Failure, Request of wrong type
########################################

def test_failure_from_blackboard():
    console.banner("Client Failure, Request of wrong type")

    server = py_trees_ros.mock.set_bool.SetBoolServer(sleep_time=1.0)

    root = create_service_client(from_blackboard=True)
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=False
    )
    tree.setup()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(server.node)
    executor.add_node(tree.node)

    assert_banner()

    py_trees.blackboard.Blackboard.set(
        variable_name="/request",
        value=Trigger.Request()  # noqa
    )

    tree.tick()

    # Request of wrong type
    assert_details("Request of wrong type - root.status", "FAILURE", root.status)
    assert(root.status == py_trees.common.Status.FAILURE)

    tree.shutdown()
    server.shutdown()
    executor.shutdown()

########################################
# Priority Interrupt
########################################

def test_priority_interrupt():
    console.banner("Priority Interrupt")

    sleep_time = 3.0
    server = py_trees_ros.mock.set_bool.SetBoolServer(sleep_time=sleep_time)

    service_client = create_service_client()
    success_eventually = py_trees.behaviours.StatusQueue(
        name="Success Eventually",
        queue=[
            py_trees.common.Status.FAILURE,
        ],
        eventually=py_trees.common.Status.SUCCESS,
    )
    root = py_trees.composites.Selector(name="Selector", memory=False)
    root.add_children([success_eventually, service_client])
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(server.node)
    executor.add_node(tree.node)

    assert_banner()

    tree.tick()

    assert_details("Pre-failure: service_client.status", "RUNNING", service_client.status)
    assert(service_client.status == py_trees.common.Status.RUNNING)

    tree.tick()

    assert_details("Post-failure: service_client.status", "INVALID", service_client.status)
    assert(service_client.status == py_trees.common.Status.INVALID)

    # Sleep longer than sleep time to ensure nothing happens after the server callback ends
    time.sleep(sleep_time*1.5)

    tree.shutdown()
    server.shutdown()
    executor.shutdown()