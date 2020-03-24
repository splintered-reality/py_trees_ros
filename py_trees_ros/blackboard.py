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
This module provides the :class:`py_trees_ros.blackboard.Exchange` class,
a ROS wrapper around a
:ref:`Blackboard <py_trees:blackboards-section>` that permits
introspection of either the entire board or a window onto a smaller part
of the board over a ROS API via the :ref:`py-trees-blackboard-watcher`
command line utility.
"""

##############################################################################
# Imports
##############################################################################

import copy
import pickle
import py_trees
import py_trees.console as console
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa
import rclpy.expand_topic_name
import rclpy.node
import std_msgs.msg as std_msgs
import typing
import uuid

from . import exceptions
from . import utilities

##############################################################################
# Blackboard Helpers
##############################################################################


class SubBlackboard(object):
    """
    Dynamically track the entire blackboard or part thereof and
    flag when there have been changes. This is a nice to have that only
    works if the blackboard contains fundamental variables (i.e. objects
    that can be pickled).

    Args:
        node: the node handle for ros logging of warnings if needed
    """
    def __init__(self, node: rclpy.node.Node):
        self.is_changed = False
        self.variable_names = set()
        self.pickled_storage = None
        self.node = node
        self.warned = False

    def update(self, variable_names: typing.Set[str]):
        """
        Check for changes to the blackboard scoped to the provided set of
        variable names (may be nested, e.g. battery.percentage). Checks
        the entire blackboard when variable_names is None.

        Args:
            variable_names: constrain the scope to track for changes
        """
        def handle_pickling_error():
            if not self.warned:
                self.node.get_logger().warning("You have objects on the blackboard that can't be pickled.")
                self.node.get_logger().warning("Any blackboard watchers will always receive updates,")
                self.node.get_logger().warning("regardless of whether the data changed or not")
                self.warned = True
            self.pickled_storage = None
            self.is_changed = True

        if variable_names is None:
            try:
                storage = copy.deepcopy(py_trees.blackboard.Blackboard.storage)
            except (TypeError, pickle.PicklingError):
                handle_pickling_error()
                return
            self.variable_names = py_trees.blackboard.Blackboard.keys()
        else:
            storage = {}
            for variable_name in variable_names:
                try:
                    storage[variable_name] = copy.deepcopy(
                        py_trees.blackboard.Blackboard.get(variable_name)
                    )
                except KeyError:
                    pass  # not an error, just not on the blackboard yet
                except (TypeError, pickle.PicklingError):
                    handle_pickling_error()
                    return
            self.variable_names = variable_names
        try:
            pickled_storage = pickle.dumps(storage, -1)
            self.is_changed = pickled_storage != self.pickled_storage
        except (TypeError, pickle.PicklingError):
            handle_pickling_error()
            return
        self.pickled_storage = pickled_storage

    def __str__(self):
        """
        Convenient printed representation of the sub-blackboard that this
        instance is currently tracking.
        """
        max_length = 0
        indent = " " * 4
        s = ""
        for name in self.variable_names:
            max_length = len(name) if len(name) > max_length else max_length
        for name in sorted(self.variable_names):
            try:
                value = py_trees.blackboard.Blackboard.get(name)
                lines = ("%s" % value).split('\n')
                if len(lines) > 1:
                    s += console.cyan + indent + '{0: <{1}}'.format(name, max_length + 1) + console.reset + ":\n"
                    for line in lines:
                        s += console.yellow + "    %s" % line + console.reset + "\n"
                else:
                    s += console.cyan + indent + '{0: <{1}}'.format(name, max_length + 1) + console.reset + ": " + console.yellow + "%s" % (value) + console.reset + "\n"
            except KeyError:
                value_string = "-"
                s += console.cyan + indent + '{0: <{1}}'.format(name, max_length + 1) + console.reset + ": " + console.yellow + "%s" % (value_string) + console.reset + "\n"
        return s.rstrip()  # get rid of the trailing newline...print will take care of adding a new line


class BlackboardView(object):
    """
    Utility class that enables tracking and publishing of relevant
    parts of the blackboard for a user. This is used by the
    :class:`~py_trees_ros.blackboard.Exchange` operator.

    Args:
        node: an rclpy node for communications handling
        topic_name: name of the topic for the publisher
        variable_names: requested variables to view
        filter_on_visited_path: constrain dynamically to the visited region of the blackboard
    """
    def __init__(
            self,
            node: rclpy.node.Node,
            topic_name: str,
            variable_names: typing.Set[str],
            filter_on_visited_path: bool,
            with_activity_stream: bool
    ):
        self.topic_name = topic_name
        self.variable_names = variable_names
        self.sub_blackboard = SubBlackboard(node=node)
        self.sub_activity_stream = py_trees.blackboard.ActivityStream()
        self.node = node
        self.publisher = self.node.create_publisher(
            msg_type=std_msgs.String,
            topic=topic_name,
            qos_profile=utilities.qos_profile_latched()
        )
        self.filter_on_visited_path = filter_on_visited_path
        self.with_activity_stream = with_activity_stream
        self.tracked_variable_names = set()
        self.tracked_keys = set()

    def shutdown(self):
        """
        Shutdown the temporarily created publisher.
        """
        self.node.destroy_publisher(self.publisher)

    def is_changed(self, visited_clients: typing.Set[uuid.UUID]) -> bool:
        """
        Dynamically adjusts the tracking parameters for the sub-blackboard
        against the clients that were visited (if desired) and proceeds
        to determine if there was a change since the last check.

        .. warning:: Since this caches the current blackboard, it can't be used
            multiple times in succession.

        Returns:
            :class:`bool`
        """
        if self.filter_on_visited_path:
            visited_keys = py_trees.blackboard.Blackboard.keys_filtered_by_clients(
                client_ids=visited_clients
            )
        view_variable_names = self.variable_names
        view_keys = [name.split('.')[0] for name in view_variable_names]

        self.tracked_variable_names = set()
        self.tracked_keys = set()
        if not self.variable_names:
            if self.filter_on_visited_path:
                self.tracked_variable_names = visited_keys
                self.tracked_keys = visited_keys
            else:
                self.tracked_variable_names = None
                self.tracked_keys = py_trees.blackboard.Blackboard.keys()
        else:
            for key, name in zip(view_keys, view_variable_names):
                if self.filter_on_visited_path:
                    if key in visited_keys:
                        self.tracked_variable_names.add(name)
                        self.tracked_keys.add(key)
                else:
                    self.tracked_variable_names.add(name)
                    self.tracked_keys.add(key)
        # update the sub blackboard
        self.sub_blackboard.update(self.tracked_variable_names)
        # update the sub activity stream
        if self.with_activity_stream and py_trees.blackboard.Blackboard.activity_stream is not None:
            self.sub_activity_stream.clear()
            for activity_item in py_trees.blackboard.Blackboard.activity_stream.data:
                if activity_item.key in self.tracked_keys:
                    self.sub_activity_stream.push(activity_item)
        # flag if / if not changes occurred
        return self.sub_blackboard.is_changed


##############################################################################
# Exchange
##############################################################################


class Exchange(object):
    """
    Establishes ros communications around a :class:`~py_trees.blackboard.Blackboard`
    that enable users to introspect or watch relevant parts of the blackboard.

    ROS Publishers:
        * **~/_watcher_<N>** (:class:`std_msgs.msg.String`)

          * streams the (sub)blackboard over a blackboard watcher connection

    ROS Services:
        * **~/get_variables** (:class:`py_trees_msgs.srv.GetBlackboardVariables`)

          * list all the blackboard variable names (not values)
        * **~/open** (:class:`py_trees_msgs.srv.OpenBlackboardStream`)

          * request a publisher to stream a part of the blackboard contents
        * **~/close** (:class:`py_trees_msgs.srv.CloseBlackboardStream`)

          * close a previously opened watcher

    Watching could be more simply enabled by just providing a *get* style service
    on a particular variable(s), however that would introduce an asynchronous
    interrupt on the library's usage and subsequently, locking. Instead, a
    watcher service requests for a stream, i.e. a publisher to be created for private
    use that streams only a subsection of the blackboard to the user. Additional
    services are provided for closing the stream and listing the variables on the
    blackboard.

    .. seealso::

        :class:`py_trees_ros.trees.BehaviourTree` (in which it is used) and
        :ref:`py-trees-blackboard-watcher` (working with the watchers).
    """
    _counter = 0
    """Incremental counter guaranteeing unique watcher names"""

    def __init__(self):
        self.node = None
        self.views = []
        self.services = {}
        self.activity_stream_clients = 0

    def setup(self, node: rclpy.node.Node):
        """
        This is where the ros initialisation of publishers and services happens. It is kept
        outside of the constructor for the same reasons that the familiar py_trees
        :meth:`~py_trees.trees.BehaviourTree.setup` method has - to enable construction
        of behaviours and trees offline (away from their execution environment) so that
        dot graphs and other visualisations of the tree can be created.

        Args:
            node (:class:`~rclpy.node.Node`): node to hook ros communications on

        Examples:

            It is expected that users will use this in their own customised tree custodian:

            .. code-block:: python

                class MyTreeManager(py_trees.trees.BehaviourTree):

                    def __init__(self):
                        pass

                    def setup(self, timeout):
                        super(MyTreeManager, self).setup(timeout)
                        self.exchange = py_trees_ros.blackboard.Exchange()
                        self.exchange.setup(timeout)

        .. seealso:: This class is used as illustrated above in :class:`~py_trees_ros.trees.BehaviourTree`.
        """
        self.node = node
        for service_name, service_type in [
            ("get_variables", py_trees_srvs.GetBlackboardVariables),
            ("open", py_trees_srvs.OpenBlackboardStream),
            ("close", py_trees_srvs.CloseBlackboardStream)
        ]:
            self.services[service_name] = self.node.create_service(
                srv_type=service_type,
                srv_name='~/blackboard_streams/' + service_name,
                callback=getattr(self, "_{}_service".format(service_name)),
                qos_profile=rclpy.qos.qos_profile_services_default
            )

    def _get_nested_keys(self):
        variables = []

        def inner(v, k):
            for attr in dir(type(v)):
                if not isinstance(v, (bool, list, str, int, float)):
                        if not attr.startswith("_"):
                            value = getattr(v, attr)
                            if not callable(value):
                                if not attr.isupper():
                                    variables.append(k + "." + attr)
                                    inner(value, k + "." + attr)

        for key in sorted(py_trees.blackboard.Blackboard.storage):
            variables.append(key)
            inner(py_trees.blackboard.Blackboard.storage[key], key)

        return variables

    def post_tick_handler(self, visited_client_ids: typing.List[uuid.UUID]=None):
        """
        Update blackboard watcher views, publish changes and
        clear the activity stream. Publishing is lazy, depending
        on blackboard watcher connections.

        Typically you would call this from a tree custodian (e.g.
        :class:`py_trees_ros.trees.BehaviourTree`) after each and every tick.

        Args:
            visited_client_ids: blackboard client unique identifiers
        """
        # update watcher views and publish
        if len(self.views) > 0:
            for view in self.views:
                if self.node.count_subscribers(view.topic_name) > 0:
                    if view.is_changed(visited_client_ids):  # update in here
                        msg = std_msgs.String()
                        if view.with_activity_stream:
                            msg.data = console.green + "Blackboard Data\n" + console.reset
                            msg.data += "{}\n".format(view.sub_blackboard)
                            msg.data += py_trees.display.unicode_blackboard_activity_stream(
                                activity_stream=view.sub_activity_stream.data
                            )
                        else:
                            msg.data = "{}".format(view.sub_blackboard)
                        view.publisher.publish(msg)

        # manage the activity stream
        if py_trees.blackboard.Blackboard.activity_stream is not None:
            if self.activity_stream_clients == 0:
                py_trees.blackboard.Blackboard.disable_activity_stream()
            else:
                py_trees.blackboard.Blackboard.activity_stream.clear()
        elif self.activity_stream_clients > 0:
            py_trees.blackboard.Blackboard.enable_activity_stream()

    def register_activity_stream_client(self):
        self.activity_stream_clients += 1

    def unregister_activity_stream_client(self):
        self.activity_stream_clients -= 1

    def _close_service(self, request, response):
        response.result = False
        for view in self.views:
            if view.topic_name == request.topic_name:
                if view.with_activity_stream:
                    self.unregister_activity_stream_client()
                view.shutdown()  # that node.destroy_publisher call makes havoc
                response.result = True
                break
        self.views[:] = [view for view in self.views if view.topic_name != request.topic_name]
        return response

    def _get_variables_service(self, unused_request, response):
        response.variables = self._get_nested_keys()
        return response

    def _open_service(self, request, response):
        response.topic = rclpy.expand_topic_name.expand_topic_name(
            topic_name="~/blackboard_streams/_watcher_" + str(Exchange._counter),
            node_name=self.node.get_name(),
            node_namespace=self.node.get_namespace())
        Exchange._counter += 1
        if request.with_activity_stream:
            self.register_activity_stream_client()
        view = BlackboardView(
            node=self.node,
            topic_name=response.topic,
            variable_names=set(request.variables),
            filter_on_visited_path=request.filter_on_visited_path,
            with_activity_stream=request.with_activity_stream
        )
        self.views.append(view)
        return response

##############################################################################
# Blackboard Watcher
##############################################################################


class BlackboardWatcher(object):
    """
    The blackboard watcher sits on the other side of the exchange and is a
    useful mechanism from which to pull all or part of the blackboard contents.
    This is useful for live introspection, or logging of the blackboard contents.

    Args:
        namespace_hint: (optionally) used to locate the blackboard if there exists more than one

    .. seealso:: :ref:`py-trees-blackboard-watcher`
    """
    def __init__(self, namespace_hint: str=None):
        self.namespace_hint = namespace_hint
        self.service_names = {
            'list': None,
            'open': None,
            'close': None
        }
        self.service_type_strings = {
            'list': 'py_trees_ros_interfaces/srv/GetBlackboardVariables',
            'open': 'py_trees_ros_interfaces/srv/OpenBlackboardStream',
            'close': 'py_trees_ros_interfaces/srv/CloseBlackboardStream'
        }
        self.service_types = {
            'list': py_trees_srvs.GetBlackboardVariables,
            'open': py_trees_srvs.OpenBlackboardStream,
            'close': py_trees_srvs.CloseBlackboardStream
        }

    def setup(self, timeout_sec: float):
        """
        Creates the node and checks that all of the server-side services are available
        for calling on to open a connection.

        Args:
            timeout_sec: time (s) to wait (use common.Duration.INFINITE to block indefinitely)

        Raises:
            :class:`~py_trees_ros.exceptions.NotFoundError`: if no services were found
            :class:`~py_trees_ros.exceptions.MultipleFoundError`: if multiple services were found
        """
        self.node = rclpy.create_node(
            node_name=utilities.create_anonymous_node_name(node_name='watcher'),
            start_parameter_services=False
        )
        for service_name in self.service_names.keys():
            # can raise NotFoundError and MultipleFoundError
            self.service_names[service_name] = utilities.find_service(
                node=self.node,
                service_type=self.service_type_strings[service_name],
                namespace=self.namespace_hint,
                timeout=timeout_sec
            )

    def create_service_client(self, key: str):
        """
        Convenience api for opening a service client and waiting for the service to appear.

        Args:
            key: one of 'open', 'close'.

        Raises:
            :class:`~py_trees_ros.exceptions.NotReadyError`: if setup() wasn't called to identify the relevant services to connect to.
            :class:`~py_trees_ros.exceptions.TimedOutError`: if it times out waiting for the server
        """
        if self.service_names[key] is None:
            raise exceptions.NotReadyError(
                "no known '{}' service known [did you call setup()?]".format(self.service_types[key])
            )
        client = self.node.create_client(
            srv_type=self.service_types[key],
            srv_name=self.service_names[key],
            qos_profile=rclpy.qos.qos_profile_services_default
        )
        # hardcoding timeouts will get us into trouble
        if not client.wait_for_service(timeout_sec=3.0):
            raise exceptions.TimedOutError(
                "timed out waiting for {}".format(self.service_names['close'])
            )
        return (self.service_types[key].Request(), client)

    def echo_blackboard_contents(self, msg: std_msgs.String):
        """
        Very simple formatter of incoming messages.

        Args:
            msg (:class:`std_msgs.String`): incoming blackboard message as a string.
        """
        print("{}".format(msg.data))

    def shutdown(self):
        """
        Perform any ros-specific shutdown.
        """
        if self.node:
            self.node.destroy_node()
