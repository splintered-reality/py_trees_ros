#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
A few behaviours to support the tutorials.
"""

##############################################################################
# Imports
##############################################################################

import copy
import move_base_msgs.msg as move_base_msgs
import py_trees
import py_trees_msgs.msg as py_trees_msgs
import py_trees_ros
import rospy
import std_msgs.msg as std_msgs
import threading

##############################################################################
# Behaviours
##############################################################################


class Scan(object):
    """
    A job handler that instantiates a subtree for scanning to be executed by
    a behaviour tree.
    """

    def __init__(self):
        """
        Tune into a channel for incoming goal requests. This is a simple
        subscriber here but more typically would be a service or action interface.
        """
        self._subscriber = rospy.Subscriber("/dashboard/scan", std_msgs.Empty, self.incoming)
        self._goal = None
        self._lock = threading.Lock()

    @property
    def goal(self):
        """
        Getter for the variable indicating whether or not a goal has recently been received
        but not yet handled. It simply makes sure it is wrapped with the appropriate locking.
        """
        with self._lock:
            g = copy.copy(self._goal) or self._goal
        return g

    @goal.setter
    def goal(self, value):
        """
        Setter for the variable indicating whether or not a goal has recently been received
        but not yet handled. It simply makes sure it is wrapped with the appropriate locking.
        """
        with self._lock:
            self._goal = value

    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        if self.goal:
            rospy.logerr("Scan: rejecting new goal, previous still in the pipeline")
        else:
            self.goal = msg

    def create_report_string(self, subtree_root):
        """
        Introspect the subtree root to determine an appropriate human readable status report string.

        Args:
            subtree_root (:class:`~py_trees.behaviour.Behaviour`): introspect the subtree

        Returns:
            :obj:`str`: human readable substring
        """
        if subtree_root.tip().has_parent_with_name("Cancelling?"):
            return "cancelling"
        else:
            return "scanning"

    @staticmethod
    def create_root(goal=std_msgs.Empty()):
        """
        Create the job subtree based on the incoming goal specification.

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        # beahviors
        root = py_trees.composites.Selector(name="Scan or Die")
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
        rospy.logwarn("Setting up rotating")
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
        # Subtree
        root.add_children([ere_we_go, die])
        ere_we_go.add_children([undock, scan_or_be_cancelled, dock, celebrate])
        scan_or_be_cancelled.add_children([cancelling, move_out_and_scan])
        cancelling.add_children([is_cancel_requested, move_home_after_cancel])
        move_out_and_scan.add_children([move_base, scanning, move_home_after_scan])
        scanning.add_children([scan_context_switch, scan_rotate, scan_flash_blue])
        celebrate.add_children([celebrate_flash_green, celebrate_pause])
        return root
