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
The :class:`py_trees_ros.trees.BehaviourTree` class
extends the core :class:`py_trees.trees.BehaviourTree` class
with a ROS publisher that publishes the initial and updated snapshots of
the tree whenever the tree changes.

'Change' is defined by deletion or insertion of behaviours into the tree
or the status of any behaviour in the tree changing from tick to tick.
"""

##############################################################################
# Imports
##############################################################################

import collections
# import datetime
import enum
import functools
import os
import math
import py_trees
import py_trees.console as console
import py_trees_ros_interfaces.msg as py_trees_msgs
# import rosbag
import rclpy
import statistics
import subprocess
import tempfile
import time
import unique_identifier_msgs.msg as unique_identifier_msgs

from . import blackboard
from . import conversions
from . import utilities

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

        self.statistics = None
        self.tick_start_time = None
        self.time_series = collections.deque([])
        self.tick_interval_series = collections.deque([])
        self.tick_duration_series = collections.deque([])

        self.pre_tick_handlers.append(self._statistics_pre_tick_handler)
        self.post_tick_handlers.append(self._statistics_post_tick_handler)

        self.timer = None

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
        self.blackboard_exchange.setup(self.node)
        self.post_tick_handlers.append(self._on_change_post_tick_handler)
        self.post_tick_handlers.append(self.blackboard_exchange.publish_blackboard)

        # share the tree's node with it's behaviours
        super().setup(timeout, node=self.node)

    def _setup_publishers(self):
        latched = True
        self.publishers = utilities.Publishers(
            self.node,
            [
                ("snapshots", "~/snapshots", py_trees_msgs.BehaviourTree, latched),
            ]
        )

        # publish current state
        self._publish_serialised_tree()

        # set a handler to publish future modifications whenever the tree is modified
        # (e.g. pruned). The tree_update_handler method is in the base class, set this
        # to the callback function here.
        self.tree_update_handler = self._publish_serialised_tree

    def tick_tock(
            self,
            period_ms,
            number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
            pre_tick_handler=None,
            post_tick_handler=None
         ):
        """
        Tick continuously at the period specified.

        This is a re-implementation of the
        :meth:`~py_trees.trees.BehaviourTree.tick_tock`
        tick_tock that takes advantage of the rclpy timers so callbacks are interleaved inbetween
        rclpy callbacks (keeps everything synchronous so no need for locks).

        Args:
            period_ms (:obj:`float`): sleep this much between ticks (milliseconds)
            number_of_iterations (:obj:`int`): number of iterations to tick-tock
            pre_tick_handler (:obj:`func`): function to execute before ticking
            post_tick_handler (:obj:`func`): function to execute after ticking
        """
        period_s = period_ms / 1000.0
        self.timer = self.node.create_timer(
            period_s,
            functools.partial(
                self._tick_tock_timer_callback,
                period_ms=period_ms,
                number_of_iterations=number_of_iterations,
                pre_tick_handler=pre_tick_handler,
                post_tick_handler=post_tick_handler
            )
        )
        self.tick_tock_count = 0

    def shutdown(self):
        """
        Cleanly shut down rclpy timers and nodes.
        """
        if self.timer is not None:
            self.timer.cancel()
            self.node.destroy_timer(self.timer)
        self.node.destroy_node()

    def _tick_tock_timer_callback(
            self,
            period_ms,
            number_of_iterations,
            pre_tick_handler,
            post_tick_handler
         ):
        """
        Tick tock callback passed to the timer to be periodically triggered.

        Args:
            period_ms (:obj:`float`): sleep this much between ticks (milliseconds)
            number_of_iterations (:obj:`int`): number of iterations to tick-tock
            pre_tick_handler (:obj:`func`): function to execute before ticking
            post_tick_handler (:obj:`func`): function to execute after ticking
        """
        if (number_of_iterations == py_trees.trees.CONTINUOUS_TICK_TOCK or
                self.tick_tock_count < number_of_iterations):
            self.tick(pre_tick_handler, post_tick_handler)
            self.tick_tock_count += 1
        else:
            self.timer.cancel()

    def _statistics_pre_tick_handler(self, tree: py_trees.trees.BehaviourTree):
        """
        Pre-tick handler that resets the statistics and starts the clock.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): the behaviour tree that has just been ticked
        """
        if len(self.time_series) == 10:
            self.time_series.popleft()
            self.tick_interval_series.popleft()

        rclpy_start_time = rclpy.clock.Clock().now()
        self.time_series.append(conversions.rclpy_time_to_float(rclpy_start_time))
        if len(self.time_series) == 1:
            self.tick_interval_series.append(0.0)
        else:
            self.tick_interval_series.append(self.time_series[-1] - self.time_series[-2])

        self.statistics = py_trees_msgs.Statistics()
        self.statistics.count = self.count
        self.statistics.stamp = rclpy_start_time.to_msg()
        self.statistics.tick_interval = self.tick_interval_series[-1]
        self.statistics.tick_interval_average = sum(self.tick_interval_series) / len(self.tick_interval_series)
        if len(self.tick_interval_series) > 1:
            self.statistics.tick_interval_variance = statistics.variance(
                self.tick_interval_series,
                self.statistics.tick_interval_average
            )
        else:
            self.statistics.tick_interval_variance = 0.0

    def _statistics_post_tick_handler(self, tree: py_trees.trees.BehaviourTree):
        """
        Post-tick handler that completes the statistics generation.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): the behaviour tree that has just been ticked
        """
        duration = conversions.rclpy_time_to_float(rclpy.clock.Clock().now()) - self.time_series[-1]

        if len(self.tick_duration_series) == 10:
            self.tick_duration_series.popleft()
        self.tick_duration_series.append(duration)

        self.statistics.tick_duration = duration
        self.statistics.tick_duration_average = sum(self.tick_duration_series) / len(self.tick_duration_series)
        if len(self.tick_duration_series) > 1:
            self.statistics.tick_duration_variance = statistics.variance(
                self.tick_duration_series,
                self.statistics.tick_duration_average
            )
        else:
            self.statistics.tick_duration_variance = 0.0

    def _on_change_post_tick_handler(self, tree: py_trees.trees.BehaviourTree):
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
            self._publish_serialised_tree()
            # with self.lock:
            #     if not self._bag_closed:
            #         # self.bag.write(self.publishers.log_tree.name, self.logging_visitor.tree)
            #         pass

    def _publish_serialised_tree(self):
        """"
        Args:
            tree (:class:`~py_trees.trees_ros.BehaviourTree`): the behaviour tree that has just been ticked
        """
        # could just use 'self' here...
        tree_message = py_trees_msgs.BehaviourTree()
        if self.statistics is not None:
            tree_message.statistics = self.statistics
        for behaviour in self.root.iterate():
            msg = conversions.behaviour_to_msg(behaviour)
            msg.is_active = True if behaviour.id in self.snapshot_visitor.visited else False
            tree_message.behaviours.append(msg)
        self.publishers.snapshots.publish(tree_message)

    def _ascii_tree_post_tick_handler(self, snapshot_visitor, tree):
        print(
            "\n" +
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

    .. seealso:: :mod:`py_trees_ros.programs.tree_watcher`
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
            console.banner("Tick {}".format(msg.statistics.count))
            print(py_trees.display.ascii_tree(
                root=root,
                visited=self.snapshot_visitor.visited,
                previously_visited=self.snapshot_visitor.previously_visited
                )
            )
            print(console.green + "-"*80 + "\n" + console.reset)
            print("Timestamp: {}".format(
                conversions.rclpy_time_to_float(
                    rclpy.time.Time.from_msg(
                        msg.statistics.stamp
                        )
                    )
                )
            )
            print("Duration : {:.3f}/{:.3f}/{:.3f} (ms) [time/avg/stddev]".format(
                msg.statistics.tick_duration*1000,
                msg.statistics.tick_duration_average*1000,
                math.sqrt(msg.statistics.tick_duration_variance)*1000
                )
            )
            print("Interval : {:.3f}/{:.3f}/{:.3f} (s) [time/avg/stddev]".format(
                msg.statistics.tick_interval,
                msg.statistics.tick_interval_average,
                math.sqrt(msg.statistics.tick_interval_variance)
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
