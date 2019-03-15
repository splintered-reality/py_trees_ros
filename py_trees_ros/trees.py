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
import enum
import os
import py_trees
import py_trees.console as console
import py_trees_ros_interfaces.msg as py_trees_msgs
# import rosbag
# TODO: import rospkg
import rclpy
import subprocess
import tempfile
import time
import unique_identifier_msgs.msg as unique_identifier_msgs

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

    * ros publishers that serialises a snapshot of the tree for viewing/logging
    * a blackboard exchange with introspection and watcher services


    ROS Publishers:
        * **~/snapshots** (:class:`py_trees_msgs.msg.BehaviourTree`)

    .. seealso::
        It also exposes publishers and services from the blackboard exchange
        in it's private namespace. Refer to :class:`~py_trees_ros.blackboard.Exchange` for details.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): root node of the tree
        ascii_tree_debug (:obj:`bool`, optional): print to console the visited ascii tree after every tick

    Raises:
        AssertionError: if incoming root variable is not the correct type
    """
    def __init__(self,
                 root,
                 ascii_tree_debug=False):
        super(BehaviourTree, self).__init__(root)
        self.snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        if ascii_tree_debug:
            self.add_post_tick_handler(
                lambda tree: self._ascii_tree_post_tick_handler(self.snapshot_visitor, tree)
            )
        self.winds_of_change_visitor = py_trees.visitors.WindsOfChangeVisitor()
        self.visitors.append(self.snapshot_visitor)
        self.visitors.append(self.winds_of_change_visitor)

        # self.logging_visitor = visitors.LoggingVisitor()
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
        self.post_tick_handlers.append(self._publish_snapshots)
        self.post_tick_handlers.append(self.blackboard_exchange.publish_blackboard)
        super().setup(timeout)

    def _setup_publishers(self):
        latched = True
        self.publishers = utilities.Publishers(
            self.node,
            [
                ("snapshots", "~/snapshots", py_trees_msgs.BehaviourTree, latched),
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
        # TODO: publish a snapshot

    def _publish_snapshots(self, tree: py_trees.trees.BehaviourTree):
        """
        Post-tick handler that checks for changes in the tree as a result
        of it's last tick and publishes an update on a ROS topic.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): the behaviour tree that has just been ticked
        """
        # checks
        if self.publishers is None:
            self.node.get_logger().error("call setup() on this tree to initialise the ros components")
            return
        if self.root.tip() is None:
            self.node.get_logger().error("the root behaviour failed to return a tip [cause: tree is in an INVALID state]")
            return

        # if there's been a change, serialise, publish and log
        if self.winds_of_change_visitor.changed:
            # serialisation
            tree_message = py_trees_msgs.BehaviourTree()
            tree_message.statistics.count = self.count
            tree_message.statistics.stamp = rclpy.clock.Clock().now().to_msg()
            for behaviour in tree.root.iterate():
                msg = conversions.behaviour_to_msg(behaviour)
                msg.is_active = True if behaviour.id in self.snapshot_visitor.visited else False
                tree_message.behaviours.append(msg)
            # publish
            self.publishers.snapshots.publish(tree_message)
            # with self.lock:
            #     if not self._bag_closed:
            #         # self.bag.write(self.publishers.log_tree.name, self.logging_visitor.tree)
            #         pass

    def _ascii_tree_post_tick_handler(self, snapshot_visitor, tree):
        print(
            py_trees.display.ascii_tree(
                tree.root,
                visited=snapshot_visitor.visited,
                previously_visited=tree.snapshot_visitor.previously_visited
            )
        )

    def _cleanup(self):
        with self.lock:
            # self.bag.close()
            self.interrupt_tick_tocking = True
            self._bag_closed = True

##############################################################################
# Tree Watcher
##############################################################################


class WatcherMode(enum.Enum):
    """An enumerator specifying the view mode for the watcher"""

    ASCII_SNAPSHOT = "ASCII_SNAPSHOT"
    """Print an ascii art view of the behaviour tree's current state after the last tick"""
    ASCII_TREE = "ASCII_TREE"
    """Print an ascii art representation of the static tree (sans visited path/status/feedback messages)."""
    DOT_TREE = "DOT_TREE"
    """Render with the dot graph representation of the static tree (using an application or text to console)."""


class Watcher(object):
    """
    The tree watcher sits on the other side of a running
    :class:`~py_trees_ros.trees.BehaviourTree` and is a useful mechanism for
    quick introspection of it's current state.

    Args:
        namespace_hint (:obj:`str`): (optionally) used to locate the blackboard
                                     if there exists more than one
        mode (:class:`~py_trees_ros.trees.WatcherMode`): viewing mode for the watcher

    .. seealso:: :ref:`py-trees-tree-watcher`, :ref:`py-trees-blackboard-watcher`
    """
    def __init__(
            self,
            namespace_hint: str,
            mode: WatcherMode=WatcherMode.ASCII_SNAPSHOT
         ):
        self.namespace_hint = namespace_hint
        self.subscribers = None
        self.viewing_mode = mode
        self.snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.done = False
        self.xdot_process = None
        self.rendered = None

    def setup(self):
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
        # taking advantage of there being only one publisher per message
        # type in the namespace to do auto-discovery of names
        topic_names = {}
        for key, topic_type_string in [
            ('snapshots', 'py_trees_ros_interfaces/BehaviourTree')
        ]:
            topic_names[key] = utilities.find_topic(
                self.node,
                topic_type_string,
                self.namespace_hint
            )
        self.subscribers = utilities.Subscribers(
            node=self.node,
            subscriber_details=[
                ("snapshots", topic_names["snapshots"], py_trees_msgs.BehaviourTree, True, self.callback_snapshot),
            ]
        )

    def callback_snapshot(self, msg):
        """
        Formats the string message coming in.

        Args
            msg (:class:`py_trees_ros_interfaces.msg.BehaviourTree`):serialised snapshot
        """
        ####################
        # Processing
        ####################
        self.snapshot_visitor.previously_visited = self.snapshot_visitor.visited
        self.snapshot_visitor.visited = {}
        serialised_behaviours = {}
        root_id = None
        for serialised_behaviour in msg.behaviours:
            if serialised_behaviour.parent_id == unique_identifier_msgs.UUID():
                root_id = conversions.msg_to_uuid4(serialised_behaviour.own_id)
            serialised_behaviours[
                conversions.msg_to_uuid4(serialised_behaviour.own_id)
            ] = serialised_behaviour

        def deserialise_tree_recursively(msg):
            behaviour = conversions.msg_to_behaviour(msg)
            for serialised_child_id in msg.child_ids:
                child_id = conversions.msg_to_uuid4(serialised_child_id)
                child = deserialise_tree_recursively(
                    serialised_behaviours[child_id]
                )
                child.parent = behaviour
                behaviour.children.append(child)
            if msg.is_active:
                self.snapshot_visitor.visited[behaviour.id] = behaviour.status
            return behaviour

        # we didn't set the tip in any behaviour, but nothing depends
        # on that right now
        root = deserialise_tree_recursively(serialised_behaviours[root_id])

        ####################
        # Streaming
        ####################
        if self.viewing_mode == WatcherMode.ASCII_SNAPSHOT:
            print(py_trees.display.ascii_tree(
                root=root,
                visited=self.snapshot_visitor.visited,
                previously_visited=self.snapshot_visitor.previously_visited
                )
            )
        ####################
        # Printing
        ####################
        elif self.viewing_mode == WatcherMode.ASCII_TREE:
            print("")
            print(py_trees.display.ascii_tree(
                root=root,
                show_status=True,
                visited=self.snapshot_visitor.visited,
                previously_visited=self.snapshot_visitor.previously_visited
                )
            )
            self.done = True
        ####################
        # Dot Graph
        ####################
        elif self.viewing_mode == WatcherMode.DOT_TREE and not self.rendered:
            self.rendered = True
            directory_name = tempfile.mkdtemp()
            py_trees.display.render_dot_tree(
                root=root,
                target_directory=directory_name
            )
            xdot_program = py_trees.utilities.which('xdot')

            if not xdot_program:
                console.logerror("No xdot viewer found, skipping display [hint: sudo apt install xdot]")
                print(py_trees.display.dot_graph(root=root).to_string())
                self.done = True
                self.xdot_process = 1
                return

            filename = py_trees.utilities.get_valid_filename(root.name) + '.dot'
            if xdot_program:
                try:
                    self.xdot_process = subprocess.Popen(
                        [
                            xdot_program,
                            os.path.join(directory_name, filename)
                        ]
                    )
                except KeyboardInterrupt:
                    pass
            self.done = True
