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
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
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


def create_action_client(from_blackboard=False, wait_for_server_timeout_sec=2.0):
    if from_blackboard:
        behaviour = py_trees_ros.action_clients.FromBlackboard(
            name="dock",
            action_type=py_trees_actions.Dock,
            action_name="dock",
            key="goal",
            generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed),
            wait_for_server_timeout_sec=wait_for_server_timeout_sec
        )
    else:
        behaviour = py_trees_ros.action_clients.FromConstant(
            name="dock",
            action_type=py_trees_actions.Dock,
            action_name="dock",
            action_goal=py_trees_actions.Dock.Goal(dock=True),  # noqa
            generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed),
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


class RejectGoalServer(object):
    def __init__(self, node_name, action_name, action_type):
        self.node = rclpy.create_node(node_name)

        self.action_server = rclpy.action.ActionServer(
            node=self.node,
            action_type=action_type,
            action_name=action_name,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),  # needed?
            execute_callback=lambda goal_handle: None,
            goal_callback=self.goal_callback
        )

    def goal_callback(self, unused_goal_request):
        return rclpy.action.server.GoalResponse.REJECT

    def shutdown(self):
        self.action_server.destroy()
        self.node.destroy_node()


class DockFailedServer(py_trees_ros.mock.dock.Dock):
    def __init__(self):
        super().__init__()

    def execute_goal_callback(self, goal_handle):
        result = self.action_type.Result()
        goal_handle.abort()
        return result


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
# Success
########################################

def test_success():
    console.banner("Client Success")

    server = py_trees_ros.mock.dock.Dock(duration=1.5)

    root = create_action_client()
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
# Priority Interrupt
########################################


def test_priority_interrupt():
    console.banner("Priority Interrupt")

    server = py_trees_ros.mock.dock.Dock(duration=1.5)

    action_client = create_action_client()
    success_eventually = py_trees.behaviours.Count(
        name="Success Eventually",
        fail_until=4,
        success_until=1000
    )
    root = py_trees.composites.Selector()
    root.add_children([success_eventually, action_client])
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(server.node)
    executor.add_node(tree.node)

    assert_banner()

    tree.tick()
    print_unicode_tree(tree)

    assert_details("action_client.status", "RUNNING", root.status)
    assert(action_client.status == py_trees.common.Status.RUNNING)

    tree.tick_tock(
        period_ms=100,
        number_of_iterations=number_of_iterations()
    )

    while tree.count < number_of_iterations() and "cancelled" not in action_client.feedback_message:
        executor.spin_once(timeout_sec=0.1)
    print_unicode_tree(tree)
    assert_details("action_client.status", "INVALID", action_client.status)
    assert(action_client.status == py_trees.common.Status.INVALID)

    # hack to make sure action client is in a state that lets it
    # shut down without segfaulting
    if action_client.get_result_future is not None:
        while not action_client.get_result_future.done():
            executor.spin_once(timeout_sec=0.1)

    tree.shutdown()
    server.shutdown()
    executor.shutdown()

########################################
# Rejection
########################################


def test_rejection():
    console.banner("Client Rejection")

    server = RejectGoalServer(
        node_name="reject",
        action_name="dock",
        action_type=py_trees_actions.Dock,
    )

    root = create_action_client()
    tree = py_trees_ros.trees.BehaviourTree(root=root)

    # ROS Setup
    tree.setup()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(server.node)
    executor.add_node(tree.node)
    tree.tick()
    tree.tick_tock(period_ms=100, number_of_iterations=number_of_iterations())

    # ROS Spin
    while tree.count < number_of_iterations() and root.status == py_trees.common.Status.RUNNING:
        executor.spin_once(timeout_sec=0.05)

    print("")
    assert_banner()
    assert_details("root.status", "FAILURE", root.status)
    assert(root.status == py_trees.common.Status.FAILURE)

    tree.shutdown()
    server.shutdown()
    executor.shutdown()

########################################
# Aborted
########################################


def test_aborted():
    console.banner("Server Aborted")

    server = DockFailedServer()

    root = create_action_client()
    tree = py_trees_ros.trees.BehaviourTree(root=root)

    # ROS Setup
    tree.setup()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(server.node)
    executor.add_node(tree.node)
    tree.tick()
    tree.tick_tock(period_ms=100, number_of_iterations=number_of_iterations())

    # ROS Spin
    while tree.count < number_of_iterations() and root.status == py_trees.common.Status.RUNNING:
        executor.spin_once(timeout_sec=0.05)

    print("")
    assert_banner()
    assert_details("root.status", "FAILURE", root.status)
    assert(root.status == py_trees.common.Status.FAILURE)

    tree.shutdown()
    server.shutdown()
    executor.shutdown()

########################################
# From Blackboard
########################################


def test_from_blackboard():
    console.banner("From Blackboard")

    server = py_trees_ros.mock.dock.Dock(duration=1.5)

    root = create_action_client(from_blackboard=True)
    tree = py_trees_ros.trees.BehaviourTree(root=root)

    # ROS Setup
    tree.setup()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(server.node)
    executor.add_node(tree.node)

    print("")
    assert_banner()

    tree.tick()

    # Nothing on blackboard yet
    assert_details("No goal on blackboard - root.status", "FAILURE", root.status)
    assert(root.status == py_trees.common.Status.FAILURE)

    py_trees.blackboard.Blackboard.set(
        variable_name="/goal",
        value=py_trees_actions.Dock.Goal(dock=True)  # noqa
    )

    tree.tick()

    # Nothing on blackboard yet
    assert_details("Goal exists - root.status", "RUNNING", root.status)
    assert(root.status == py_trees.common.Status.RUNNING)

    tree.shutdown()
    server.shutdown()
    executor.shutdown()

########################################
# Timeouts
########################################


def test_timeouts():
    console.banner("Timeouts")

    timeout = 0.1
    root = create_action_client(from_blackboard=True, wait_for_server_timeout_sec=timeout)
    tree = py_trees_ros.trees.BehaviourTree(root=root)

    start_time = time.monotonic()
    try:
        tree.setup()
    except py_trees_ros.exceptions.TimedOutError:
        duration = time.monotonic() - start_time
    assert_details("Tree Setup Timeout", "delta+{}".format(timeout), duration)
    assert(duration < 10*timeout)

    tree.shutdown()
