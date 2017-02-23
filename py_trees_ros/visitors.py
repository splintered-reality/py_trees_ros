#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
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
import py_trees_msgs.msg as py_trees_msgs
import rospy

from . import conversions

##############################################################################
# Visitors
##############################################################################


class SnapshotVisitor(py_trees.visitors.VisitorBase):
    """
    Visits the tree in tick-tock, recording runtime information for publishing
    the information as a snapshot view of the tree after the iteration has
    finished.

    Attributes:
        nodes (dict): dictionary of behaviour id (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs
        running_nodes([uuid.UUID]): list of id's for behaviours which were traversed in the current tick
        previously_running_nodes([uuid.UUID]): list of id's for behaviours which were traversed in the last tick

    .. seealso::

        This visitor should be used with the :class:`~py_trees_ros.trees.BehaviourTree` class to collect
        information to publish for both bagging and the rqt monitoring plugin.
    """
    def __init__(self):
        super(SnapshotVisitor, self).__init__()
        self.nodes = {}
        self.running_nodes = []
        self.previously_running_nodes = []

    def initialise(self):
        """
        Switch running to previously running and then reset all other variables. This will
        get called before a tree ticks.
        """
        self.nodes = {}
        self.previously_running_nodes = self.running_nodes
        self.running_nodes = []

    def run(self, behaviour):
        """
        This method gets run as each behaviour is ticked. Catch the id and status and store it.
        Additionally add it to the running list if it is :data:`~py_trees.common.Status.RUNNING`.

        Args:
            behaviour (:class:`~py_trees.behaviour.Behaviour`): behaviour that is ticking
        """
        self.nodes[behaviour.id] = behaviour.status
        if behaviour.status == py_trees.common.Status.RUNNING:
            self.running_nodes.append(behaviour.id)


class LoggingVisitor(py_trees.visitors.VisitorBase):
    """
    Visits the entire tree and gathers all behaviours as
    messages for the tree logging publishers.

    Attributes:
        tree (:class:`py_trees_msgs.msg.BehaviourTree`): tree representation in message form
    """
    def __init__(self):
        super(LoggingVisitor, self).__init__()
        self.full = True  # examine all nodes

    def initialise(self):
        """
        Initialise and stamp a :class:`py_trees_msgs.msg.BehaviourTree`
        instance.
        """
        self.tree = py_trees_msgs.BehaviourTree()
        self.tree.header.stamp = rospy.Time.now()

    def run(self, behaviour):
        """
        Convert the behaviour into a message and appendd it to the tree.

        Args:
            behaviour (:class:`~py_trees.behaviour.Behaviour`): behaviour to convert
        """
        self.tree.behaviours.append(conversions.behaviour_to_msg(behaviour))
