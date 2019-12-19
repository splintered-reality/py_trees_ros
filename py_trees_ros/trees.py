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
import statistics
import subprocess
import tempfile
import time
import typing
import uuid

import diagnostic_msgs.msg as diagnostic_msgs  # noqa
import py_trees
import py_trees.console as console
import py_trees_ros_interfaces.msg as py_trees_msgs  # noqa
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa
import rcl_interfaces.msg as rcl_interfaces_msgs
import rclpy
import unique_identifier_msgs.msg as unique_identifier_msgs

from . import blackboard
from . import conversions
from . import exceptions
from . import utilities
from . import visitors

##############################################################################
# Tree Management
##############################################################################


class SnapshotStream(object):
    """
    SnapshotStream instances are responsible for creating / destroying
    a snapshot stream as well as the configurable curation of snapshots
    that are published on it.
    """
    _counter = 0
    """Incremental counter guaranteeing unique watcher names"""

    class Parameters(object):
        """
        Reconfigurable parameters for the snapshot stream.

        Args:
            blackboard_data: publish blackboard variables on the visited path
            blackboard_activity: enable and publish blackboard activity in the last tick
            snapshot_period: period between snapshots (use /inf to only publish on tree status changes)
        """
        def __init__(
            self,
            blackboard_data: bool=False,
            blackboard_activity: bool=False,
            snapshot_period: float=py_trees.common.Duration.INFINITE
        ):
            self.blackboard_data = blackboard_data
            self.blackboard_activity = blackboard_activity
            self.snapshot_period = py_trees.common.Duration.INFINITE.value if snapshot_period == py_trees.common.Duration.INFINITE else snapshot_period

    def __init__(
        self,
        node: rclpy.node.Node,
        name: str=None,
        topic_name: str=None,
        parameters: 'SnapshotStream.Parameters'=None,
    ):
        """
        Create the publisher, ready for streaming.

        Args:
            node: node to hook ros communications on
            name: human friendly key for this snapshot stream object
            topic_name: snapshot stream name, uniquely generated if None
        """
        self.topic_name = topic_name
        if self.topic_name is None:
            self.topic_name = "~/snapshot_streams/_snapshots_" + str(SnapshotStream._counter)
            SnapshotStream._counter += 1
        self.name = name if name is not None else self.topic_name.split('/')[-1]
        self.node = node
        self.publisher = self.node.create_publisher(
            msg_type=py_trees_msgs.BehaviourTree,
            topic=topic_name,
            qos_profile=utilities.qos_profile_latched()
        )
        self.parameters = parameters if parameters is not None else SnapshotStream.Parameters()
        self.last_snapshot_timestamp = None
        self.statistics = None

    def publish(
            self,
            root: py_trees.behaviour.Behaviour,
            changed: bool,
            statistics: py_trees_msgs.Statistics,
            visited_behaviour_ids: typing.Set[uuid.UUID],
            visited_blackboard_client_ids: typing.Set[uuid.UUID]
    ):
        """"
        Publish a snapshot, including only what has been parameterised.

        Args:
            changed: whether the tree status / graph changed or not
            root: the tree
            visited_behaviour_ids: behaviours on the visited path
            visited_blackboard_client_ids: blackboard clients belonging to behaviours on the visited path
        """
        if self.last_snapshot_timestamp is None:
            changed = True
            self.last_snapshot_timestamp = time.monotonic()
        current_timestamp = time.monotonic()
        elapsed_time = current_timestamp - self.last_snapshot_timestamp
        if (not changed and elapsed_time < self.parameters.snapshot_period):
            return

        tree_message = py_trees_msgs.BehaviourTree()
        tree_message.changed = changed

        # tree
        for behaviour in root.iterate():
            msg = conversions.behaviour_to_msg(behaviour)
            msg.is_active = True if behaviour.id in visited_behaviour_ids else False
            tree_message.behaviours.append(msg)

        # blackboard
        if self.parameters.blackboard_data:
            visited_keys = py_trees.blackboard.Blackboard.keys_filtered_by_clients(
                client_ids=visited_blackboard_client_ids
            )
            for key in visited_keys:
                try:
                    value = str(py_trees.blackboard.Blackboard.get(key))
                except KeyError:
                    value = "-"
                tree_message.blackboard_on_visited_path.append(
                    diagnostic_msgs.KeyValue(
                        key=key,
                        value=value
                    )
                )

        # activity stream
        #   TODO: checking the stream is not None is redundant, perhaps use it as an exception check?
        if self.parameters.blackboard_activity and py_trees.blackboard.Blackboard.activity_stream is not None:
            tree_message.blackboard_activity_stream = py_trees.display.unicode_blackboard_activity_stream(
                show_title=False
            )
        # other
        if statistics is not None:
            tree_message.statistics = statistics
        self.publisher.publish(tree_message)
        self.last_snapshot_timestamp = current_timestamp

    def shutdown(self):
        """
        Shutdown the (temporarily) created publisher.
        """
        self.node.destroy_publisher(self.publisher)


class BehaviourTree(py_trees.trees.BehaviourTree):
    """
    Extend the :class:`py_trees.trees.BehaviourTree` class with
    a few bells and whistles for ROS:

    ROS Parameters:
        * **default_snapshot_stream**: enable/disable the default snapshots stream in ~/snapshots` (default: False)
        * **default_snapshot_period**: periodically publish (default: :data:`math.inf`)
        * **default_snapshot_blackboard_data**: include tracking and visited variables (default: True)
        * **default_snapshot_blackboard_activity**: include the blackboard activity (default: False)
        * **setup_timeout**: time (s) to wait (default: :data:`math.inf`)

          * if :data:`math.inf`, it will block indefinitely

    ROS Publishers:
        * **~/snapshots** (:class:`py_trees_interfaces.msg.BehaviourTree`)

    ROS Services:
        * **~/blackboard_streams/close** (:class:`py_trees_ros_interfaces.srv.CloselackboardWatcher`)
        * **~/blackboard_streams/get_variables** (:class:`py_trees_ros_interfaces.srv.GetBlackboardVariables`)
        * **~/blackboard_streams/open** (:class:`py_trees_ros_interfaces.srv.OpenBlackboardWatcher`)
        * **~/snapshot_streams/close** (:class:`py_trees_ros_interfaces.srv.CloseSnapshotsStream`)
        * **~/snapshot_streams/open** (:class:`py_trees_ros_interfaces.srv.OpenSnapshotsStream`)
        * **~/snapshot_streams/reconfigure** (:class:`py_trees_ros_interfaces.srv.ReconfigureSnapshotsStream`)

    Topics and services are not intended for direct use, but facilitate the operation of the
    utilities :ref:`py-trees-tree-watcher` and :ref:`py-trees-blackboard-watcher`.

    Args:
        root: root node of the tree
        unicode_tree_debug: print to console the visited ascii tree after every tick

    Raises:
        AssertionError: if incoming root variable is not the correct type
    """
    def __init__(self,
                 root: py_trees.behaviour.Behaviour,
                 unicode_tree_debug: bool=False):
        super(BehaviourTree, self).__init__(root)
        if unicode_tree_debug:
            self.snapshot_visitor = py_trees.visitors.DisplaySnapshotVisitor()
        else:
            self.snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.visitors.append(self.snapshot_visitor)

        self.statistics = None
        self.tick_start_time = None
        self.time_series = collections.deque([])
        self.tick_interval_series = collections.deque([])
        self.tick_duration_series = collections.deque([])

        self.pre_tick_handlers.append(self._statistics_pre_tick_handler)
        self.post_tick_handlers.append(self._statistics_post_tick_handler)

        self.timer = None

        # delay ROS artifacts so we can construct the tree without a ROS connection
        self.node = None
        self.snapshot_streams = {}

    def setup(
            self,
            timeout: float=py_trees.common.Duration.INFINITE,
            visitor: py_trees.visitors.VisitorBase=None
    ):
        """
        Setup the publishers, exechange and add ros-relevant pre/post tick handlers to the tree.
        Ultimately relays this call down to all the behaviours in the tree.

        Args:
            timeout: time (s) to wait (use common.Duration.INFINITE to block indefinitely)
            visitor: runnable entities on each node after it's setup

        .. note:

           This method declares parameters for the snapshot_period and setup_timeout.
           These parameters take precedence over the period and timeout args provided here.
           If parameters are not configured at runtime, then the period and timeout args
           provided here will inialise the declared parameters.

        Raises:
            rclpy.exceptions.NotInitializedException: rclpy not yet initialised
            Exception: be ready to catch if any of the behaviours raise an exception
        """
        # node creation - can raise rclpy.exceptions.NotInitializedException
        default_node_name = "tree"
        self.node = rclpy.create_node(node_name=default_node_name)
        if visitor is None:
            visitor = visitors.SetupLogger(node=self.node)

        ########################################
        # ROS Comms
        ########################################
        self.snapshot_stream_services = utilities.Services(
            node=self.node,
            service_details=[
                ("close", "~/snapshot_streams/close", py_trees_srvs.CloseSnapshotStream, self._close_snapshot_stream),
                ("open", "~/snapshot_streams/open", py_trees_srvs.OpenSnapshotStream, self._open_snapshot_stream),
                ("reconfigure", "~/snapshot_streams/reconfigure", py_trees_srvs.ReconfigureSnapshotStream, self._reconfigure_snapshot_stream),
            ],
            introspection_topic_name="snapshot_streams/services"
        )
        self.blackboard_exchange = blackboard.Exchange()
        self.blackboard_exchange.setup(self.node)

        ################################################################################
        # Parameters
        ################################################################################
        self.node.set_parameters_callback(
            callback=self._set_parameters_callback
        )
        ########################################
        # default_snapshot_period
        ########################################
        self.node.declare_parameter(
            name='default_snapshot_period',
            value=2.0,  # DJS: py_trees.common.Duration.INFINITE.value,
            descriptor=rcl_interfaces_msgs.ParameterDescriptor(
                name="default_snapshot_period",
                type=rcl_interfaces_msgs.ParameterType.PARAMETER_DOUBLE,  # noqa
                description="time between snapshots, set to math.inf to only publish tree state changes",
                additional_constraints="",
                read_only=False,
                floating_point_range=[rcl_interfaces_msgs.FloatingPointRange(
                    from_value=0.0,
                    to_value=py_trees.common.Duration.INFINITE.value)]
            )
        )

        ########################################
        # default_snapshot_blackboard_data
        ########################################
        self.node.declare_parameter(
            name='default_snapshot_blackboard_data',
            value=True,
            descriptor=rcl_interfaces_msgs.ParameterDescriptor(
                name="default_snapshot_blackboard_data",
                type=rcl_interfaces_msgs.ParameterType.PARAMETER_BOOL,  # noqa
                description="append blackboard data (tracking status, visited variables) to the default snapshot stream",
                additional_constraints="",
                read_only=False,
            )
        )

        ########################################
        # default_snapshot_blackboard_activity
        ########################################
        self.node.declare_parameter(
            name='default_snapshot_blackboard_activity',
            value=True,
            descriptor=rcl_interfaces_msgs.ParameterDescriptor(
                name="default_snapshot_blackboard_activity",
                type=rcl_interfaces_msgs.ParameterType.PARAMETER_BOOL,  # noqa
                description="append the blackboard activity stream to the default snapshot stream",
                additional_constraints="",
                read_only=False,
            )
        )

        ########################################
        # default_snapshot_stream
        ########################################
        # make sure this is declared after other default snapshot parameters so that it can
        # utilise them in the set_parameter_callback
        self.node.declare_parameter(
            name='default_snapshot_stream',
            value=True,
            descriptor=rcl_interfaces_msgs.ParameterDescriptor(
                name="default_snapshot_stream",
                type=rcl_interfaces_msgs.ParameterType.PARAMETER_BOOL,  # noqa
                description="enable/disable the default snapshot stream in ~/snapshots",
                additional_constraints="",
                read_only=False,
            )
        )

        ########################################
        # setup_timeout
        ########################################
        self.node.declare_parameter(
            name='setup_timeout',
            value=timeout if timeout != py_trees.common.Duration.INFINITE else py_trees.common.Duration.INFINITE.value,
            descriptor=rcl_interfaces_msgs.ParameterDescriptor(
                name="setup_timeout",
                type=rcl_interfaces_msgs.ParameterType.PARAMETER_DOUBLE,  # noqa
                description="timeout for ROS tree setup (node, pubs, subs, ...)",
                additional_constraints="",
                read_only=True,
                floating_point_range=[rcl_interfaces_msgs.FloatingPointRange(
                    from_value=0.0,
                    to_value=py_trees.common.Duration.INFINITE.value)]
            )
        )
        # Get the resulting timeout
        setup_timeout = self.node.get_parameter("setup_timeout").value
        # Ugly workaround to accomodate use of the enum (TODO: rewind this)
        #   Need to pass the enum for now (instead of just a float) in case
        #   there are behaviours out in the wild that apply logic around the
        #   use of the enum
        if setup_timeout == py_trees.common.Duration.INFINITE.value:
            setup_timeout = py_trees.common.Duration.INFINITE

        ########################################
        # Behaviours
        ########################################
        try:
            super().setup(
                timeout=setup_timeout,
                visitor=visitor,
                node=self.node
            )
        except RuntimeError as e:
            if str(e) == "tree setup interrupted or timed out":
                raise exceptions.TimedOutError(str(e))
            else:
                raise

        ########################################
        # Setup Handlers
        ########################################
        # set a handler to publish future modifications whenever the tree is modified
        # (e.g. pruned). The tree_update_handler method is in the base class, set this
        # to the callback function here.
        self.tree_update_handler = self._on_tree_update_handler
        self.post_tick_handlers.append(self._snapshots_post_tick_handler)

    def _set_parameters_callback(
        self,
        parameters: typing.List[rclpy.parameter.Parameter]
    ) -> rcl_interfaces_msgs.SetParametersResult:
        """
        Callback that dynamically handles changes in parameters.

        Args:
            parameters: list of one or more declared parameters with updated values

        Returns:
            result of the set parameters requests
        """
        for parameter in parameters:
            if parameter.name == "default_snapshot_stream":
                if "default" in self.snapshot_streams:
                    self.snapshot_streams["default"].shutdown()
                    if self.snapshot_streams["default"].parameters.blackboard_activity:
                        self.blackboard_exchange.unregister_activity_stream_client()
                    del self.snapshot_streams["default"]
                if parameter.value:
                    self.snapshot_streams["default"] = SnapshotStream(
                        node=self.node,
                        name="default",
                        topic_name="~/snapshots",
                        parameters=SnapshotStream.Parameters(
                            snapshot_period=self.node.get_parameter("default_snapshot_period").value,
                            blackboard_data=self.node.get_parameter("default_snapshot_blackboard_data").value,
                            blackboard_activity=self.node.get_parameter("default_snapshot_blackboard_activity").value
                        )
                    )
                    if self.snapshot_streams["default"].parameters.blackboard_activity:
                        self.blackboard_exchange.register_activity_stream_client()
            elif parameter.name == "default_snapshot_blackboard_data":
                if "default" in self.snapshot_streams:
                    self.snapshot_streams["default"].parameters.blackboard_data = parameter.value
            elif parameter.name == "default_snapshot_blackboard_activity":
                if "default" in self.snapshot_streams:
                    if self.snapshot_streams["default"].parameters.blackboard_activity != parameter.value:
                        if parameter.value:
                            self.blackboard_exchange.register_activity_stream_client()
                        else:
                            self.blackboard_exchange.unregister_activity_stream_client()
                    self.snapshot_streams["default"].parameters.blackboard_activity = parameter.value
            elif parameter.name == "default_snapshot_period":
                if "default" in self.snapshot_streams:
                    self.snapshot_streams["default"].parameters.snapshot_period = parameter.value
        return rcl_interfaces_msgs.SetParametersResult(successful=True)

    def tick_tock(
            self,
            period_ms,
            number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
            pre_tick_handler=None,
            post_tick_handler=None):
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
        self.timer = self.node.create_timer(
            period_ms / 1000.0,  # unit 'seconds'
            functools.partial(
                self._tick_tock_timer_callback,
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
        # stop ticking if we're ticking
        if self.node is not None:
            if self.timer is not None:
                self.timer.cancel()
                self.node.destroy_timer(self.timer)
        # call shutdown on each behaviour first, in case it has
        # some esoteric shutdown steps
        super().shutdown()
        if self.node is not None:
            # shutdown the node - this *should* automagically clean
            # up any non-estoeric shutdown of ros communications
            # inside behaviours
            self.node.destroy_node()

    def _tick_tock_timer_callback(
            self,
            number_of_iterations,
            pre_tick_handler,
            post_tick_handler):
        """
        Tick tock callback passed to the timer to be periodically triggered.

        Args:
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

    def _on_tree_update_handler(self):
        """
        Whenever there has been a modification to the tree (insertion/pruning), publish
        the snapshot.
        """
        # only worth notifying once we've actually commenced
        if "default" in self.snapshot_streams:
            if self.statistics is not None:
                rclpy_start_time = rclpy.clock.Clock().now()
                self.statistics.stamp = rclpy_start_time.to_msg()
                self.snapshot_streams["default"].publish(
                    root=self.root,
                    changed=True,
                    statistics=self.statistics,
                    visited_behaviour_ids=self.snapshot_visitor.visited.keys(),
                    visited_blackboard_client_ids=self.snapshot_visitor.visited_blackboard_client_ids
                )

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

    def _snapshots_post_tick_handler(self, tree: py_trees.trees.BehaviourTree):
        """
        Post-tick handler that checks for changes in the tree/blackboard as a result
        of it's last tick and publish updates on ROS topics.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): the behaviour tree that has just been ticked
        """
        # checks
        if self.node is None:
            self.node.get_logger().error("call setup() on this tree to initialise the ros components")
            return
        if self.root.tip() is None:
            self.node.get_logger().error("the root behaviour failed to return a tip [hint: tree is in an INVALID state, usually as a result of incorrectly coded behaviours]")
            return

        # publish the default snapshot stream (useful for logging)
        if "default" in self.snapshot_streams:
            self.snapshot_streams["default"].publish(
                root=self.root,
                changed=self.snapshot_visitor.changed,
                statistics=self.statistics,
                visited_behaviour_ids=self.snapshot_visitor.visited.keys(),
                visited_blackboard_client_ids=self.snapshot_visitor.visited_blackboard_client_ids
            )

        # every tick publish on watchers, clear activity stream (note: not expensive as watchers by default aren't connected)
        self.blackboard_exchange.post_tick_handler(
            visited_client_ids=self.snapshot_visitor.visited_blackboard_client_ids  # .keys()
        )

    def _close_snapshot_stream(
        self,
        request: py_trees_srvs.CloseSnapshotStream.Request,  # noqa
        response: py_trees_srvs.CloseSnapshotStream.Response  # noqa
    ):
        return response

    def _open_snapshot_stream(
        self,
        request: py_trees_srvs.OpenSnapshotStream.Request,  # noqa
        response: py_trees_srvs.OpenSnapshotStream.Response  # noqa
    ):
        return response

    def _reconfigure_snapshot_stream(
        self,
        request: py_trees_srvs.ReconfigureSnapshotStream.Request,  # noqa
        response: py_trees_srvs.ReconfigureSnapshotStream.Response  # noqa
    ):
        return response

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

    DEFAULT_STREAM = "DEFAULT_STREAM"
    """Render ascii/unicode snapshots from the default stream (~/snapshots)"""
    CUSTOM_STREAM = "CUSTOM_STREAM"
    """Render ascii/unicode snapshots from a stream configured only for this client"""
    DOT_GRAPH = "DOT_GRAPH"
    """Render with the dot graph representation of the static tree (using an application or text to console)."""


class Watcher(object):
    """
    The tree watcher sits on the other side of a running
    :class:`~py_trees_ros.trees.BehaviourTree` and is a useful mechanism for
    quick introspection of it's current state.

    Args:
        namespace_hint: used to locate the blackboard if there exists more than one
        mode: viewing mode for the watcher
        display_blackboard_variables: display key-value pairs (on the visited path)
        display_blackboard_activity: display logged activity for the last tick
        display_statistics: display timing statistics

    .. seealso:: :mod:`py_trees_ros.programs.tree_watcher`
    """
    def __init__(
            self,
            namespace_hint: str,
            mode: WatcherMode=WatcherMode.CUSTOM_STREAM,
            display_blackboard_variables: bool=False,
            display_blackboard_activity: bool=False,
            display_statistics: bool=False):
        self.namespace_hint = namespace_hint
        self.subscribers = None
        self.viewing_mode = mode
        self.snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.done = False
        self.xdot_process = None
        self.rendered = None
        self.display_blackboard_variables = display_blackboard_variables
        self.display_blackboard_activity = display_blackboard_activity
        self.display_statistics = display_statistics

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
            self.node = rclpy.create_node(node_name=default_node_name)
            time.sleep(0.1)  # ach, the magic foo before discovery works
        except rclpy.exceptions.NotInitializedException:
            print(console.red + "ERROR: rlcpy not yet initialised [{}]".format(default_node_name) + console.reset)
            return False
        # taking advantage of there being only one publisher per message
        # type in the namespace to do auto-discovery of names
        topic_type_string = 'py_trees_ros_interfaces/msg/BehaviourTree'
        topic_names = utilities.find_topics(
            self.node,
            topic_type_string,
            self.namespace_hint
        )
        if not topic_names:
            raise exceptions.NotFoundError("topic not found [type: {}]".format(topic_type_string))
        elif len(topic_names) > 1:
            raise exceptions.MultipleFoundError("multiple topics found, use a namespace hint [type: {}]".format(topic_type_string))
        else:
            topic_name = topic_names[0]
        self.subscribers = utilities.Subscribers(
            node=self.node,
            subscriber_details=[
                ("snapshots", topic_name, py_trees_msgs.BehaviourTree, True, self.callback_snapshot),
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
                # invasive hack to revert the dummy child we added in msg_to_behaviour
                if isinstance(behaviour, py_trees.decorators.Decorator):
                    behaviour.children = [child]
                    behaviour.decorated = behaviour.children[0]
                else:
                    behaviour.children.append(child)
                child.parent = behaviour
            # set the current child so tip() works properly everywhere
            if behaviour.children:
                if msg.current_child_id != unique_identifier_msgs.UUID():
                    current_child_id = conversions.msg_to_uuid4(msg.current_child_id)
                    for index, child in enumerate(behaviour.children):
                        if child.id == current_child_id:
                            # somewhat ugly not having a consistent api here
                            if isinstance(behaviour, py_trees.composites.Selector):
                                behaviour.current_child = child
                            elif isinstance(behaviour, py_trees.composites.Chooser):
                                behaviour.current_child = child
                            elif isinstance(behaviour, py_trees.composites.Sequence):
                                behaviour.current_index = index
                            # else Parallel, nothing to do since it infers
                            # the current child from children's status on the fly
                            break
            if msg.is_active:
                self.snapshot_visitor.visited[behaviour.id] = behaviour.status
            return behaviour

        # we didn't set the tip in any behaviour, but nothing depends
        # on that right now
        root = deserialise_tree_recursively(serialised_behaviours[root_id])

        ####################
        # Streaming
        ####################
        if self.viewing_mode == WatcherMode.DEFAULT_STREAM:
            if msg.changed:
                colour = console.green
            else:
                colour = console.bold
            ####################
            # Banner
            ####################
            title = "Tick {}".format(msg.statistics.count)
            print(colour + "\n" + 80 * "*" + console.reset)
            print(colour + "* " + console.bold + title.center(80) + console.reset)
            print(colour + 80 * "*" + "\n" + console.reset)
            ####################
            # Tree Snapshot
            ####################
            print(
                py_trees.display.unicode_tree(
                    root=root,
                    visited=self.snapshot_visitor.visited,
                    previously_visited=self.snapshot_visitor.previously_visited
                )
            )
            print(colour + "-" * 80 + console.reset)
            ####################
            # Stream Variables
            ####################
            if self.display_blackboard_variables:
                print("")
                print(colour + "Blackboard Data" + console.reset)
                indent = " " * 4
                if not msg.blackboard_on_visited_path:
                    print(
                        console.cyan + indent + "-" + console.reset + " : " +
                        console.yellow + "-" + console.reset
                    )
                else:
                    # could probably re-use the unicode_blackboard by passing a dict to it
                    # like we've done for the activity stream
                    max_length = 0
                    for variable in msg.blackboard_on_visited_path:
                        max_length = len(variable.key) if len(variable.key) > max_length else max_length
                    for variable in msg.blackboard_on_visited_path:
                        print(
                            console.cyan + indent +
                            '{0: <{1}}'.format(variable.key, max_length + 1) + console.reset + ": " +
                            console.yellow + '{0}'.format(variable.value) + console.reset
                        )
            ####################
            # Stream Activity
            ####################
            if self.display_blackboard_activity:
                print("")
                print(colour + "Blackboard Activity Stream" + console.reset)
                if msg.blackboard_activity_stream:
                    print(msg.blackboard_activity_stream)
                else:
                    indent = " " * 4
                    print(
                        console.cyan + indent + "-" + console.reset + " : " +
                        console.yellow + "-" + console.reset
                    )
            ####################
            # Stream Statistics
            ####################
            if self.display_statistics:
                print("")
                print(colour + "Statistics" + console.reset)
                print(
                    console.cyan + "    Timestamp: " + console.yellow +
                    "{}".format(
                        conversions.rclpy_time_to_float(
                            rclpy.time.Time.from_msg(
                                msg.statistics.stamp
                            )
                        )
                    )
                )
                print(
                    console.cyan + "    Duration : " + console.yellow +
                    "{:.3f}/{:.3f}/{:.3f} (ms) [time/avg/stddev]".format(
                        msg.statistics.tick_duration * 1000,
                        msg.statistics.tick_duration_average * 1000,
                        math.sqrt(msg.statistics.tick_duration_variance) * 1000
                    )
                )
                print(
                    console.cyan + "    Interval : " + console.yellow +
                    "{:.3f}/{:.3f}/{:.3f} (s) [time/avg/stddev]".format(
                        msg.statistics.tick_interval,
                        msg.statistics.tick_interval_average,
                        math.sqrt(msg.statistics.tick_interval_variance)
                    )
                )
                print(console.reset)

        ####################
        # Dot Graph
        ####################
        elif self.viewing_mode == WatcherMode.DOT_GRAPH and not self.rendered:
            self.rendered = True
            directory_name = tempfile.mkdtemp()
            py_trees.display.render_dot_tree(
                root=root,
                target_directory=directory_name
            )
            xdot_program = py_trees.utilities.which('xdot')

            if not xdot_program:
                print("")
                console.logerror("No xdot viewer found [hint: sudo apt install xdot]")
                print("")
                print(py_trees.display.dot_tree(root=root).to_string())
                self.done = True
                self.xdot_process = None
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
