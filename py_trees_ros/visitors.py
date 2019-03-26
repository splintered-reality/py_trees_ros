#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
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

import py_trees
import py_trees_ros_interfaces.msg as py_trees_msgs
import rclpy

from . import conversions

##############################################################################
# Visitors
##############################################################################


class TreeToMsgVisitor(py_trees.visitors.VisitorBase):
    """
    Visits the entire tree and gathers all behaviours as
    messages for the tree logging publishers.

    Attributes:
        tree (:class:`py_trees_msgs.msg.BehaviourTree`): tree representation in message form
    """
    def __init__(self):
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
