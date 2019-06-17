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
import operator
import pickle
import py_trees
import py_trees.console as console
import py_trees_ros_interfaces.srv as py_trees_srvs
import rclpy.expand_topic_name
import rclpy.node
import std_msgs.msg as std_msgs

from typing import Any

from . import exceptions
from . import utilities

##############################################################################
# SubView of a Blackboard
##############################################################################


class BlackboardView(object):
    """
    Utility class that enables tracking and publishing of relevant
    parts of the blackboard for a user. This is used by the
    :class:`~py_trees_ros.blackboard.Exchange` operator.

    Args:
        node: an rclpy node for communications handling
        topic_name: name of the topic for the publisher
        attrs (:obj:`???`):
    """
    def __init__(
            self,
            node: rclpy.node.Node,
            topic_name: str,
            attrs
         ):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.topic_name = topic_name
        self.attrs = attrs
        self.dict = {}
        self.cached_dict = {}
        self.node = node
        self.publisher = self.node.create_publisher(std_msgs.String, topic_name)

    def shutdown(self):
        """
        Shutdown the temporarily created publisher.
        """
        self.node.destroy_publisher(self.publisher)

    def _update_sub_blackboard(self):
        if not self.attrs:
            self.dict = copy.copy(self.blackboard.__dict__)
        else:
            for attr in self.attrs:
                if '/' in attr:
                    check_attr = operator.attrgetter(".".join(attr.split('/')))
                else:
                    check_attr = operator.attrgetter(attr)
                try:
                    value = check_attr(self.blackboard)
                    self.dict[attr] = value
                except AttributeError:
                    pass

    def is_changed(self):
        """
        Checks the current blackboard against a cached version. This
        then caches the current blackboard.

        .. warning:: Since this caches the current blackboard, it can't be used
            multiple times in succession.

        Returns:
            :class:`bool`
        """
        self._update_sub_blackboard()
        current_pickle = pickle.dumps(self.dict, -1)
        blackboard_changed = current_pickle != self.cached_dict
        self.cached_dict = current_pickle

        return blackboard_changed

    def __str__(self):
        """
        A formatted string representation of the view, taking care to trim the
        blackboard contents according to the view that was requested.

        Returns:
            :class:`str`
        """
        s = ""
        max_length = 0
        for k in self.dict.keys():
            max_length = len(k) if len(k) > max_length else max_length
        keys = sorted(self.dict)
        for key in keys:
            value = self.dict[key]
            if value is None:
                value_string = "-"
                s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "%s" % (value_string) + console.reset + "\n"
            else:
                lines = ("%s" % value).split('\n')
                if len(lines) > 1:
                    s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ":\n"
                    for line in lines:
                        s += console.yellow + "    %s" % line + console.reset + "\n"
                else:
                    s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "%s" % (value) + console.reset + "\n"
        return s.rstrip()  # get rid of the trailing newline...print will take care of adding a new line

##############################################################################
# Exchange
##############################################################################


class Exchange(object):
    """
    Establishes ros communications around a :class:`~py_trees.blackboard.Blackboard`
    that enable users to introspect or watch relevant parts of the blackboard.

    ROS Publishers:
        * **~/blackboard** (:class:`std_msgs.msg.String`)

          * streams (string form) the contents of the entire blackboard as it updates

    ROS Services:
        * **~/get_blackboard_variables** (:class:`py_trees_msgs.srv.GetBlackboardVariables`)

          * list all the blackboard variable names (not values)
        * **~/open_blackboard_watcher** (:class:`py_trees_msgs.srv.OpenBlackboardWatcher`)

          * request a publisher to stream a part of the blackboard contents
        * **~/close_blackboard_watcher** (:class:`py_trees_msgs.srv.CloseBlackboardWatcher`)

          * close a previously opened watcher

    Watching could be more simply enabled by just providing a *get* style service
    on a particular variable(s), however that would introduce an asynchronous
    interrupt on the library's usage and subsequently, locking. Instead, a
    watcher service requests for a stream, i.e. a publisher to be created for private
    use that streams only a subsection of the blackboard to the user. Additional
    services are provided for closing the stream and listing the variables on the
    blackboard.

    .. seealso::

        :class:`~py_trees_ros.trees.BehaviourTree` (in which it is used) and
        :ref:`py-trees-blackboard-watcher` (working with the watchers).
    """
    _counter = 0
    """Incremental counter guaranteeing unique watcher names"""

    def __init__(self):
        self.node = None
        self.blackboard = py_trees.blackboard.Blackboard()
        self.cached_blackboard_dict = {}
        self.views = []
        self.publisher = None
        self.services = {}

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
        self.publisher = self.node.create_publisher(std_msgs.String, '~/exchange/blackboard')
        for name in ["get_blackboard_variables",
                     "open_blackboard_watcher",
                     "close_blackboard_watcher"]:
            camel_case_name = ''.join(x.capitalize() for x in name.split('_'))
            self.services[name] = self.node.create_service(
                srv_type=getattr(py_trees_srvs, camel_case_name),
                srv_name='~/exchange/' + name,
                callback=getattr(self, "_{}_service".format(name))
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
                                    variables.append(k + "/" + attr)
                                    inner(value, k + "/" + attr)

        for key in sorted(self.blackboard.__dict__):
            variables.append(key)
            inner(self.blackboard.__dict__[key], key)

        return variables

    def _is_changed(self):
        current_pickle = pickle.dumps(self.blackboard.__dict__, -1)
        blackboard_changed = current_pickle != self.cached_blackboard_dict
        self.cached_blackboard_dict = current_pickle
        return blackboard_changed

    def publish_blackboard(self, unused_tree=None):
        """
        Lazy string publishing of the blackboard (the contents of
        the blackboard can be of many and varied types, so string form is the only
        way to transmit this across a ros message) on the **~blackboard** topic.

        Typically you would call this from a tree custodian (e.g.
        :class:`py_trees_ros.trees.BehaviourTree`) after each and every tick.

        .. note:: Lazy: it will only do the relevant string processing if there are subscribers present.

        Args:
            unused_tree (:obj:`any`): if used as a post_tick_handler, needs the argument, but nonetheless, gets unused
        """
        if self.publisher is None:
            self.node.get_logger().error("Blackboard Exchange: no publishers [hint: call setup() on the exchange]")
            return

        # publish blackboard
        #   the blackboard watcher doesn't actually use the lazy publisher, but
        #   if latching and string formatting in ROS gets up to speed, it will be a
        #   more useful thing to have around - easy to detect and rostopic echo the contents
        if self.node.count_subscribers(self.publisher.topic) > 0:
            if self._is_changed():
                msg = std_msgs.String()
                msg.data = "{0}".format(self.blackboard)
                self.publisher.publish(msg)

        # publish watchers
        # print("DJS: publish_blackboard()")
        # print("   DJS: # views = %s" % len(self.views))
        if len(self.views) > 0:
            for view in self.views:
                # print("   DJS: view publisher topic: %s" % (view.publisher.topic))
                # print("   DJS: # view publishers = %s" % self.node.count_publishers(view.topic_name))
                # print("   DJS: # view subscribers = %s" % self.node.count_subscribers(view.topic_name))
                if self.node.count_subscribers(view.topic_name) > 0:
                    if view.is_changed():
                        # print("     DJS: # printing on: %s" % view.topic_name)
                        msg = std_msgs.String()
                        msg.data = "{0}".format(view)
                        view.publisher.publish(msg)

    def _close_blackboard_watcher_service(self, request, response):
        response.result = False
        for view in self.views:
            # print("   DJS: close watcher? [%s][%s]" % (view.topic_name, request.topic_name))
            if view.topic_name == request.topic_name:
                view.shutdown()  # that node.destroy_publisher call makes havoc
                response.result = True
                break
        self.views[:] = [view for view in self.views if view.topic_name != request.topic_name]
        # print("DJS: close result: %s" % response.result)
        return response

    def _get_blackboard_variables_service(self, unused_request, response):
        response.variables = self._get_nested_keys()
        return response

    def _open_blackboard_watcher_service(self, request, response):
        response.topic = rclpy.expand_topic_name.expand_topic_name(
            topic_name="~/exchange/_watcher_" + str(Exchange._counter),
            node_name=self.node.get_name(),
            node_namespace=self.node.get_namespace())
        Exchange._counter += 1
        view = BlackboardView(self.node, response.topic, request.variables)
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

    .. seealso:: :ref:`py-trees-blackboard-watcher`
    """
    def __init__(
            self,
            namespace_hint: str=None
         ):
        """
        Args:
            namespace_hint: (optionally) used to locate the blackboard if there exists more than one
        """
        self.namespace_hint = namespace_hint
        self.service_names = {
            'list': None,
            'open': None,
            'close': None
        }
        self.service_type_strings = {
            'list': 'py_trees_ros_interfaces/srv/GetBlackboardVariables',
            'open': 'py_trees_ros_interfaces/srv/OpenBlackboardWatcher',
            'close': 'py_trees_ros_interfaces/srv/CloseBlackboardWatcher'
        }
        self.service_types = {
            'list': py_trees_srvs.GetBlackboardVariables,
            'open': py_trees_srvs.OpenBlackboardWatcher,
            'close': py_trees_srvs.CloseBlackboardWatcher
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
        # print("DJS: echo_blackboard_contents")
        print("{}".format(msg.data))

    def shutdown(self):
        """
        Perform any ros-specific shutdown.
        """
        if self.node:
            self.node.destroy_node()
