#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
About
^^^^^

Round out the scan job subtree with:

* docking/undocking
* moving out and back to home on either side of the scan
* cancel request handlers
* failure notifications (flashing red)

Nothing new here technically, this serves as a reasonable template from which
to start your own job subtrees.

Tree
^^^^

.. graphviz:: dot/tutorial-seven.dot

.. literalinclude:: ../py_trees_ros/tutorials/seven.py
   :language: python
   :linenos:
   :lines: 80-184
   :caption: py_trees_ros/tutorials/seven.py#create_root

Tree Status Reports
^^^^^^^^^^^^^^^^^^^

Of interest also here is the use of a post tick handler to do
tree introspection and publish a status report. In this example
the status report is very simple, it merely publishes whether
it is "scanning", "cancelling" or is "idle'.

.. literalinclude:: ../py_trees_ros/tutorials/seven.py
   :language: python
   :linenos:
   :lines: 187-211
   :caption: py_trees_ros/tutorials/seven.py#reality_report

Running
^^^^^^^

.. code-block:: bash

    $ roslaunch py_trees_ros tutorial_seven.launch --screen

**Playing with the Spaghetti**

* Press the scan button to start a scan
* Pre-empting no longer works, press the cancel button to interrupt a scan
"""

##############################################################################
# Imports
##############################################################################

import move_base_msgs.msg as move_base_msgs
import py_trees
import py_trees_ros
import py_trees.console as console
import py_trees_msgs.msg as py_trees_msgs
import rospy
import std_msgs.msg as std_msgs
import sys

##############################################################################
# Behaviours
##############################################################################


def create_root():
    # behaviours
    root = py_trees.composites.Parallel("Tutorial")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    scan2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Scan2BB",
        topic_name="/dashboard/scan",
        variable_name="event_scan_button"
    )
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        variable_name="event_cancel_button"
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(name="Battery2BB",
                                                   topic_name="/battery/state",
                                                   threshold=30.0
                                                   )
    priorities = py_trees.composites.Selector("Priorities")
    battery_check = py_trees.meta.success_is_failure(py_trees.composites.Selector)(name="Battery Emergency")
    is_battery_ok = py_trees.blackboard.CheckBlackboardVariable(
        name="Battery Ok?",
        variable_name='battery_low_warning',
        expected_value=False
    )
    flash_led_strip = py_trees_ros.tutorials.behaviours.FlashLedStrip(
        name="Flash Red",
        colour="red")
    scan = py_trees.composites.Sequence(name="Scan")
    is_scan_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Scan?",
        variable_name='event_scan_button',
        expected_value=True
    )
    scan_or_die = py_trees.composites.Selector(name="Scan or Die")
    die = py_trees_ros.tutorials.behaviours.FlashLedStrip(
        name="Uh Oh",
        colour="red")
    ere_we_go = py_trees.composites.Sequence(name="Ere we Go")
    undock = py_trees_ros.actions.ActionClient(
        name="UnDock",
        action_namespace="/dock",
        action_spec=py_trees_msgs.DockAction,
        action_goal=py_trees_msgs.DockGoal(False),
        override_feedback_message_on_running="undocking"
    )
    scan_or_be_cancelled = py_trees.composites.Selector("Scan or Be Cancelled")
    cancelling = py_trees.composites.Sequence("Cancelling?")
    is_cancel_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Cancel?",
        variable_name='event_cancel_button',
        expected_value=True
    )
    move_home_after_cancel = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_namespace="/move_base",
        action_spec=move_base_msgs.MoveBaseAction,
        action_goal=move_base_msgs.MoveBaseGoal()
    )
    move_out_and_scan = py_trees.composites.Sequence("Move Out and Scan")
    move_base = py_trees_ros.actions.ActionClient(
        name="Move Out",
        action_namespace="/move_base",
        action_spec=move_base_msgs.MoveBaseAction,
        action_goal=move_base_msgs.MoveBaseGoal()
    )
    scanning = py_trees.composites.Parallel(name="Scanning", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    scan_context_switch = py_trees_ros.tutorials.behaviours.ScanContext("Context Switch")
    scan_rotate = py_trees_ros.actions.ActionClient(
        name="Rotate",
        action_namespace="/rotate",
        action_spec=py_trees_msgs.RotateAction,
        action_goal=py_trees_msgs.RotateGoal(),
        override_feedback_message_on_running="rotating"
    )
    scan_flash_blue = py_trees_ros.tutorials.behaviours.FlashLedStrip(name="Flash Blue", colour="blue")
    move_home_after_scan = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_namespace="/move_base",
        action_spec=move_base_msgs.MoveBaseAction,
        action_goal=move_base_msgs.MoveBaseGoal()
    )
    celebrate = py_trees.composites.Parallel(name="Celebrate", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    celebrate_flash_green = py_trees_ros.tutorials.behaviours.FlashLedStrip(name="Flash Green", colour="green")
    celebrate_pause = py_trees.timers.Timer("Pause", duration=3.0)
    dock = py_trees_ros.actions.ActionClient(
        name="Dock",
        action_namespace="/dock",
        action_spec=py_trees_msgs.DockAction,
        action_goal=py_trees_msgs.DockGoal(True),
        override_feedback_message_on_running="docking"
    )
    idle = py_trees.behaviours.Running(name="Idle")

    # tree
    root.add_children([topics2bb, priorities])
    topics2bb.add_children([scan2bb, cancel2bb, battery2bb])
    priorities.add_children([battery_check, scan, idle])
    battery_check.add_children([is_battery_ok, flash_led_strip])
    scan.add_children([is_scan_requested, scan_or_die])
    scan_or_die.add_children([ere_we_go, die])
    ere_we_go.add_children([undock, scan_or_be_cancelled, dock, celebrate])
    scan_or_be_cancelled.add_children([cancelling, move_out_and_scan])
    cancelling.add_children([is_cancel_requested, move_home_after_cancel])
    move_out_and_scan.add_children([move_base, scanning, move_home_after_scan])
    scanning.add_children([scan_context_switch, scan_rotate, scan_flash_blue])
    celebrate.add_children([celebrate_flash_green, celebrate_pause])
    return root


class SplinteredReality(object):

    def __init__(self):
        self.tree = py_trees_ros.trees.BehaviourTree(create_root())
        self.tree.add_post_tick_handler(self.publish_reality_report)
        self.report_publisher = rospy.Publisher("~report", std_msgs.String, queue_size=5)

    def setup(self):
        return self.tree.setup(timeout=15)

    def publish_reality_report(self, tree):
        if tree.tip().has_parent_with_name("Battery Emergency"):
            self.report_publisher.publish("battery")
        elif tree.tip().has_parent_with_name("Cancelling?"):
            self.report_publisher.publish("cancelling")
        elif tree.tip().has_parent_with_name("Scan"):
            self.report_publisher.publish("scanning")
        else:
            self.report_publisher.publish("idle")

    def tick_tock(self):
        self.tree.tick_tock(500)

    def shutdown(self):
        self.tree.interrupt()


##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the demo script.
    """
    rospy.init_node("tree")
    splintered_reality = SplinteredReality()
    rospy.on_shutdown(splintered_reality.shutdown)
    if not splintered_reality.setup():
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    splintered_reality.tick_tock()
