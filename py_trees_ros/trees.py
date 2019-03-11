#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.github.com/splintered-reality/py_trees_ros/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
The :class:`ROS Behaviour Tree <py_trees_ros.trees.BehaviourTree>`
extends the core :class:`Behaviour Tree <py_trees.trees.BehaviourTree>` class
with a few ROS style adornments. The major features currently include:

* Publishers for ascii/dot tree visualisations and the blackboard
* A publisher which dumps the entire tree at every change for the rqt plugin
* Bagging of the tree for offline visualisation and debugging
"""

##############################################################################
# Imports
##############################################################################

import datetime
import os
import py_trees
import py_trees.console as console
import py_trees_ros_interfaces.msg as py_trees_msgs
# import rosbag
# TODO: import rospkg
import rclpy
import std_msgs.msg as std_msgs
import threading
import time

from . import blackboard
from . import conversions
from . import exceptions
from . import utilities
from . import visitors

##############################################################################
# ROS Trees
##############################################################################


class BehaviourTree(py_trees.trees.BehaviourTree):
    """
    Extend the :class:`py_trees.trees.BehaviourTree` class with
    a few bells and whistles for ROS:

    * ros publishers with snapshot ascii/dot graph views of the tree
    * ros publisher with data representation of the entire tree for monitoring/bagging
    * ros publisher showing what the current tip is
    * a blackboard exchange with introspection and watcher services


    ROS Publishers:
        * **~ascii/tree** (:class:`std_msgs.msg.String`)

          * static view of the entire tree (debugging)
        * **~ascii/snapshot** (:class:`std_msgs.msg.String`)

          * runtime ascii snapshot view of the ticking tree (debugging)
        * **~dot/tree** (:class:`std_msgs.msg.String`)

          * static dot graph of the entire tree (debugging)
        * **~log/tree** (:class:`py_trees_msgs.msg.BehaviourTree`)

          * representation of the entire tree in message form for rqt/bagging
        * **~tip** (:class:`py_trees_msgs.msg.Behaviour`)

          * the tip of the tree after the last tick

    .. seealso::
        It also exposes publishers and services from the blackboard exchange
        in it's private namespace. Refer to :class:`~py_trees_ros.blackboard.Exchange` for details.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): root node of the tree

    Raises:
        AssertionError: if incoming root variable is not the correct type
    """
    def __init__(self, root):
        super(BehaviourTree, self).__init__(root)
        self.snapshot_visitor = visitors.SnapshotVisitor()
        self.winds_of_change_visitor = py_trees.visitors.WindsOfChangeVisitor()
        # self.logging_visitor = visitors.LoggingVisitor()
        self.visitors.append(self.snapshot_visitor)
        # self.visitors.append(self.logging_visitor)
        # self._bag_closed = False

        # now = datetime.datetime.now()
        # topdir = utilities.get_py_trees_home()
        # subdir = os.path.join(topdir, now.strftime('%Y-%m-%d'))
        # if not os.path.exists(topdir):
        #     os.makedirs(topdir)
        # if not os.path.exists(subdir):
        #     os.makedirs(subdir)

        # opens in ros home directory for the user
        # TODO: self.bag = rosbag.Bag(subdir + '/behaviour_tree_' + now.strftime("%H-%M-%S") + '.bag', 'w')

        # self.last_tree = py_trees_msgs.BehaviourTree()
        # self.lock = threading.Lock()

        # delay ROS specific artifacts so we can create/introsepct on this class
        # without having to go live.
        self.node = None
        self.publishers = None

        # _cleanup must come last as it assumes the existence of the bag
        # TODO: rospy.on_shutdown(self._cleanup)

    def setup(self, timeout: float=py_trees.common.Duration.INFINITE):
        """
        Setup the publishers, exechange and add ros-relevant pre/post tick handlers to the tree.
        Ultimately relays this call down to all the behaviours in the tree.

        Args:
            timeout (:obj:`float`): time (s) to wait (use common.Duration.INFINITE to block indefinitely)

        Raises:
            Exception: be ready to catch if any of the behaviours raise an exception
        """
        default_node_name = "tree"
        try:
            self.node = rclpy.create_node(default_node_name)
        except rclpy.exceptions.NotInitializedException:
            return RuntimeError("rlcpy not yet initialised [{}]".format(default_node_name))
        self._setup_publishers()
        self.blackboard_exchange = blackboard.Exchange()
        if not self.blackboard_exchange.setup():
            return False
        self.post_tick_handlers.append(self._publish_tree_snapshots)
        self.post_tick_handlers.append(self.blackboard_exchange.publish_blackboard)
        super().setup(timeout)

    def _setup_publishers(self):
        latched = True
        self.publishers = utilities.Publishers(
            self.node,
            [
                ("ascii_tree", "~/ascii/tree", std_msgs.String, latched),
                ("ascii_snapshot", "~/ascii/snapshot", std_msgs.String, latched),
                ("dot_tree", "~/dot/tree", std_msgs.String, latched),
                ("log_tree", "~/log/tree", py_trees_msgs.BehaviourTree, latched),
                ("tip", "~/tip", py_trees_msgs.Behaviour, latched)
            ]
        )

        # publish current state
        self._publish_tree_modifications(self.root)
        # set a handler to publish future modifications whenever the tree is modified
        # (e.g. pruned). The tree_update_handler method is in the base class, set this
        # to the callback function here.
        self.tree_update_handler = self._publish_tree_modifications

    def _publish_tree_modifications(self, root):
        """
        Publishes updates when the whole tree has been modified.

        This function is passed in as a visitor to the underlying behaviour tree and triggered
        when there has been a change.
        """
        if self.publishers is None:
            self.node.get_logger().error("call setup() on this tree to initialise the ros components")
            return
        self.publishers.ascii_tree.publish(
            std_msgs.String(
                data=py_trees.display.ascii_tree(root)
            )
        )
        # TODO: remove once ROS2 supports unicode in message strings
        dot_tree = py_trees.console.forceably_replace_unicode_chars(
            py_trees.display.stringify_dot_tree(root)
        )
        self.publishers.dot_tree.publish(std_msgs.String(data=dot_tree))

    def _publish_tree_snapshots(self, tree):
        """
        Callback that runs on a :class:`BehaviourTree <py_trees.trees.BehaviourTree>` after
        it has ticked.

        :param tree: the behaviour tree
        :type tree: :py:class:`BehaviourTree <py_trees.trees.BehaviourTree>`
        """
        # checks
        if self.publishers is None:
            self.node.get_logger().error("call setup() on this tree to initialise the ros components")
            return
        if self.root.tip() is None:
            self.node.get_logger().error("the root behaviour failed to return a tip [cause: tree is in an INVALID state]")
            return

        # snapshot
        snapshot = "\n\n{}".format(py_trees.display.ascii_tree(self.root, snapshot_information=self.snapshot_visitor))
        snapshot = py_trees.console.forceably_replace_unicode_chars(snapshot)
        self.publishers.ascii_snapshot.publish(std_msgs.String(data=snapshot))

        # if there's been a change, serialise, publish and log
        if self.winds_of_change_visitor.changed:
            # serialisation
            tree_message = py_trees_msgs.BehaviourTree()
            tree_message.tick = self.count
            tree_message.stamp = rclpy.clock.Clock().now().to_msg()
            for behaviour in tree.root.iterate():
                msg = conversions.behaviour_to_msg(behaviour)
                msg.is_active = True if behaviour.id in self.snapshot_visitor.nodes else False
                tree_message.behaviours.append(msg)
            # publish
            self.publishers.log_tree.publish(tree_message)
            self.publishers.tip.publish(conversions.behaviour_to_msg(self.root.tip()))
            # with self.lock:
            #     if not self._bag_closed:
            #         # self.bag.write(self.publishers.log_tree.name, self.logging_visitor.tree)
            #         pass

    def _cleanup(self):
        with self.lock:
            # self.bag.close()
            self.interrupt_tick_tocking = True
            self._bag_closed = True

##############################################################################
# Tree Watcher
##############################################################################


class Watcher(object):
    """
    The tree watcher sits on the other side of a running
    :class:`~py_trees_ros.trees.BehaviourTree` and is a useful mechanism for
    quick introspection of it's current state.

    .. seealso:: :ref:`py-trees-tree-watcher`, :ref:`py-trees-blackboard-watcher`
    """
    def __init__(self, namespace_hint):
        """
        Args:
            namespace_hint (:obj:`str`): (optionally) used to locate the blackboard
                                         if there exists more than one
        """
        self.namespace_hint = namespace_hint
        self.topic_names = {
            'ascii/snapshot': None,
            'ascii/tree': None,
            'dot/tree': None,
            'log/tree': None,
            'tip': None,
        }
        self.topic_type_strings = {
            'ascii/snapshot': 'std_msgs/String',
            'ascii/tree': 'std_msgs/String',
            'dot/tree': 'std_msgs/String',
            'log/tree': 'py_trees_ros_interfaces/BehaviourTree',
            'tip': 'py_trees_ros_interfaces/Behaviour',
        }
        self.topic_types = {
            'ascii/snapshot': std_msgs.String,
            'ascii/tree': std_msgs.String,
            'dot/tree': std_msgs.String,
            'log/tree': py_trees_msgs.BehaviourTree,
            'tip': py_trees_msgs.Behaviour,
        }
        self.subscribers = {}

    def setup(self, timeout):
        """
        Args:
            timeout (:obj:`float`): time to wait (0.0 is blocking forever)
        Raises:
            :class:`~py_trees_ros.exceptions.NotFoundError`: if no services were found
            :class:`~py_trees_ros.exceptions.MultipleFoundError`: if multiple services were found
        """
        default_node_name = "watcher_" + str(os.getpid())
        try:
            self.node = rclpy.create_node(default_node_name)
            time.sleep(0.1)  # ach, the magic foo before discovery works
        except rclpy.exceptions.NotInitializedException:
            print(console.red + "ERROR: rlcpy not yet initialised [{}]".format(default_node_name) + console.reset)
            return False
        # Note: this assumes that the services are not dynamically available (i.e.
        # go up and down frequently)
        self.topic_names['log/tree'] = utilities.find_topic(
            self.node,
            self.topic_type_strings['log/tree'],
            self.namespace_hint
        )

        # fineprint: assumption that the others are set relative to that and not remapped!
        root = "/" + "".join(self.topic_names['log/tree'].split('/')[:-2])
        self.topic_names['ascii/snapshot'] = root + "/ascii/snapshot"  # noqa
        self.topic_names['ascii/tree']     = root + "/ascii/tree"      # noqa
        self.topic_names['dot/tree']       = root + "/dot/tree"        # noqa
        self.topic_names['tip']            = root + "/tip"             # noqa

    def connect_to_ascii_tree(self):
        key = 'ascii/tree'
        print("[DJS] Connection to: {}".format(self.topic_names[key]))
        self.subscribers[key] = self.node.create_subscription(
            msg_type=self.topic_types[key],
            topic=self.topic_names[key],
            callback=self.callback_string_to_stdout,
            qos_profile=utilities.qos_profile_latched_topic()
        )

    def connect_to_dot_tree(self):
        key = 'dot/tree'
        self.subscribers[key] = self.node.create_subscription(
            msg_type=self.topic_types[key],
            topic=self.topic_names[key],
            callback=self.callback_string_to_stdout,
            qos_profile=utilities.qos_profile_latched_topic()
        )

    def connect_to_ascii_snapshot(self):
        key = 'ascii/snapshot'
        self.subscribers[key] = self.node.create_subscription(
            msg_type=self.topic_types[key],
            topic=self.topic_names[key],
            callback=self.callback_string_to_stdout,
            qos_profile=utilities.qos_profile_latched_topic()
        )

    def callback_string_to_stdout(self, msg):
        """
        Formats the string message coming in
        Args
            timeout (:class:`std_msgs.msg.String`): string message
        """
        print("{}".format(msg.data))
