#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
ROS Visitors are entities that can be passed to a ROS tree implementation
(e.g. :class:`~py_trees_ros.trees.BehaviourTree`) and used to either visit
each and every behaviour in the tree, or visit behaviours as the tree is
traversed in an executing tick. At each behaviour, the visitor
runs its own method on the behaviour to do as it wishes - logging, introspecting).

.. warning:: Visitors should not modify the behaviours they visit.

.. seealso:: The base interface and core visitors in :mod:`py_trees.visitors`
"""

##############################################################################
# Imports
##############################################################################


import py_trees.visitors
import py_trees_ros_interfaces.msg as py_trees_msgs
import rclpy
import time

from . import conversions

##############################################################################
# Visitors
##############################################################################


class SetupLogger(py_trees.visitors.VisitorBase):
    """
    Use as a visitor to :meth:`py_trees_ros.trees.TreeManager.setup`
    to log the name and timings of each behaviours' setup
    to the ROS debug channel.

    Args:
        node: an rclpy node that will provide debug logger
    """
    def __init__(self, node: rclpy.node.Node):
        super().__init__(full=True)
        self.node = node

    def initialise(self):
        """
        Initialise the timestamping chain.
        """
        self.start_time = time.monotonic()
        self.last_time = self.start_time

    def run(self, behaviour):
        current_time = time.monotonic()
        self.node.get_logger().debug(
            "'{}'.setup: {:.4f}s".format(behaviour.name, current_time - self.last_time)
        )
        self.last_time = current_time

    def finalise(self):
        current_time = time.monotonic()
        self.node.get_logger().debug(
            "Total tree setup time: {:.4f}s".format(current_time - self.start_time)
        )


class TreeToMsgVisitor(py_trees.visitors.VisitorBase):
    """
    Visits the entire tree and gathers all behaviours as
    messages for the tree logging publishers.

    Attributes:
        tree (:class:`py_trees_msgs.msg.BehaviourTree`): tree representation in message form
    """
    def __init__(self):
        """
        Well
        """
        super(TreeToMsgVisitor, self).__init__()
        self.full = True  # examine all nodes

    def initialise(self):
        """
        Initialise and stamp a :class:`py_trees_msgs.msg.BehaviourTree`
        instance.
        """
        self.tree = py_trees_msgs.BehaviourTree()
        # TODO: crystal api
        # self.tree.stamp = rclpy.clock.Clock.now().to_msg()

    def run(self, behaviour):
        """
        Convert the behaviour into a message and append to the tree.

        Args:
            behaviour (:class:`~py_trees.behaviour.Behaviour`): behaviour to convert
        """
        self.tree.behaviours.append(conversions.behaviour_to_msg(behaviour))
