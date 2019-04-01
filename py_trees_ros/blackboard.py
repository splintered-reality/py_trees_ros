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
The :class:`py_trees_ros.blackboard.Exchange` class wraps a
:ref:`PyTrees Blackboard <pt:blackboards-section>` with a ROS API to provide easy
introspection of either the entire board or a window onto a smaller part
of the board from oustide the tree via the :ref:`py-trees-blackboard-watcher`
command line utility.

.. note::

   You get an exchange for free with the
   :class:`py_trees_ros.trees.BehaviourTree` manager.

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
import rclpy.executors
import rclpy.expand_topic_name
import rclpy.node
import std_msgs.msg as std_msgs
import time

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
        topic_name (:obj:`str`): name of the topic for the publisher
        attrs (:obj:`???`):
    """
    def __init__(self, node, topic_name, attrs):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.topic_name = topic_name
        self.attrs = attrs
        self.dict = {}
        self.cached_dict = {}
        self.publisher = node.create_publisher(std_msgs.String, topic_name)

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

    def _is_changed(self):
        self._update_sub_blackboard()
        current_pickle = pickle.dumps(self.dict, -1)
        blackboard_changed = current_pickle != self.cached_dict
        self.cached_dict = current_pickle

        return blackboard_changed

    def __str__(self):
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

        .. seealso:: This method is called in the way illustrated above in :class:`~py_trees_ros.trees.BehaviourTree`.
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

    def _close_watcher(self, req):
        for i, view in enumerate(self.views):
            if view.topic_name == req.topic_name:
                self.node.destroy_publisher(view.publisher)
                del self.views[i]
                return True
        return False

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
        if self.node.count_subscribers("~/blackboard") > 0:
            if self._is_changed():
                msg = std_msgs.String()
                msg.data = "{0}".format(self.blackboard)
                self.publisher.publish(msg)

        # publish watchers
        if len(self.views) > 0:
            for view in self.views:
                if self.node.count_publishers(view.topic_name) > 0:
                    if view._is_changed():
                        msg = std_msgs.String()
                        msg.data = "{0}".format(view)
                        view.publisher.publish(msg)

    def _close_blackboard_watcher_service(self, request, response):
        response.result = self._close_watcher(request)
        return response

    def _get_blackboard_variables_service(self, unused_request, response):
        response.variables = self._get_nested_keys()
        return response

    def _open_blackboard_watcher_service(self, request, response):
        response.topic = rclpy.expand_topic_name.expand_topic_name(
            topic_name="~/blackboard/watcher_" + str(Exchange._counter),
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
    This is extremely useful for logging, but also for introspecting in the
    runtime for various uses (e.g. reporting out on the current state to
    a fleet server).

    .. seealso:: :ref:`py-trees-blackboard-watcher`
    """
    def __init__(self,
                 callback=lambda *args, **kwargs: None,  # a noop
                 namespace_hint=None):
        """
        Args:
            callback: any method that accepts a string as an input argument
            namespace_hint (:obj:`str`): (optionally) used to locate the blackboard
                                         if there exists more than one
        """
        self.namespace_hint = namespace_hint
        self.service_names = {
            'list': None,
            'open': None,
            'close': None
        }
        self.service_type_strings = {
            'list': 'py_trees_ros_interfaces/GetBlackboardVariables',
            'open': 'py_trees_ros_interfaces/OpenBlackboardWatcher',
            'close': 'py_trees_ros_interfaces/CloseBlackboardWatcher'
        }
        self.service_types = {
            'list': py_trees_srvs.GetBlackboardVariables,
            'open': py_trees_srvs.OpenBlackboardWatcher,
            'close': py_trees_srvs.CloseBlackboardWatcher
        }
        self.watcher_topic_name = None
        self.watcher_subscriber = None
        self.callback = callback

    def setup(self, timeout_sec):
        """
        Args:
            timeout_sec (:obj:`float`): time (s) to wait (use common.Duration.INFINITE to block indefinitely)

        Raises:
            :class:`~py_trees_ros.exceptions.NotFoundError`: if no services were found
            :class:`~py_trees_ros.exceptions.MultipleFoundError`: if multiple services were found
        """
        self.node = rclpy.create_node(
            node_name=utilities.create_anonymous_node_name(node_name='watcher'),
            start_parameter_services=False
        )
        # Note: this assumes that the services are not dynamically available (i.e.
        # go up and down frequently)
        scanned_time = 0.0
        scanning_period = 0.1
        for service_name in self.service_names.keys():
            while True:
                try:
                    self.service_names[service_name] = utilities.find_service(
                        self.node,
                        self.service_type_strings[service_name],
                        self.namespace_hint
                    )
                    break
                except exceptions.NotFoundError:
                    time.sleep(scanning_period)
                    scanned_time += scanning_period
                    if scanned_time > timeout_sec:
                        raise exceptions.NotFoundError("Timed out scanning for blackboard services")
                    else:
                        continue
                except exceptions.MultipleFoundError as e:
                    raise e

    def request_list_variables(self):
        """
        Request of the blackboard a list of it's variables.

        Returns:
            :class:`~rclpy.task.Future`
        """
        request, client = self._create_service_client('list')
        future = client.call_async(request)
        return future

    def open_connection(self,
                        variables,
                        callback=None,
                        executor=None):
        """
        First calls the exchange to get details on the connect the watcher is required
        to open and then initiates that tunnel. Note that this does some spinning waiting
        for the service call, so if the executor must be shared with other nodes (e.g. for
        testing) make sure you pass in your own executor.

        Args:
            variables (:obj:`[str]`): list of variables to listen for
            callback (func, optional): method with a std_msgs/String argument, defaults to
                :meth:`blackboard_contents_callback()` in this class
            executor (:class:`~rclpy.executors.Executor`) used to spin the node looking for service callbacks

        Raises:
            :class:`~py_trees_ros.exceptions.NotReadyError`: if setup not executed or it hitherto failed
            :class:`~py_trees_ros.exceptions.ServiceError`: if the service failed to respond
            :class:`~py_trees_ros.exceptions.TimedOutError`: if the services could not be reached
        """
        if callback is None:
            callback = self.blackboard_contents_callback

        request, client = self._create_service_client('open')
        # convenience, just in case someone wrote the list of variables for argparse like a python list instead
        # of a space separated list of variables, i.e.
        #    py_trees-blackboard-watcher [count, dude] → ['[count,', 'dude]']  → ['count', 'dude']
        request.variables = [variable.strip(',[]') for variable in variables]
        future = client.call_async(request)
        if executor is None:
            executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self.node)
        executor.spin_until_future_complete(future)
        response = future.result()
        if response is None:
            raise exceptions.ServiceError(
                "service call failed [{}]".format(future.exception())
            )
        self.watcher_topic_name = response.topic
        self.watcher_subscriber = self.node.create_subscription(
            msg_type=std_msgs.String,
            topic=self.watcher_topic_name,
            callback=callback
        )

    def close_connection(self):
        """
        Raises:
            :class:`~py_trees_ros.exceptions.NotReadyError`: if setup not executed or it hitherto failed
            :class:`~py_trees_ros.exceptions.TimedOutError`: if the services could not be reached
        """
        if not self.watcher_topic_name:
            return  # Nothing to do
        request, client = self._create_service_client('close')
        request.topic_name = self.watcher_topic_name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        unused_response = future.result()
        self.node.destroy_subscription(self.watcher_subscriber)
        self.watcher_topic_name = None
        self.watcher_subscriber = None

    def _create_service_client(self, key):
        if self.service_names[key] is None:
            raise exceptions.NotReadyError(
                "no known '{}' service known [did you call setup()?]".format(self.service_types[key])
            )
        client = self.node.create_client(
            srv_type=self.service_types[key],
            srv_name=self.service_names[key]
            )
        # hardcoding timeouts will get us into trouble
        if not client.wait_for_service(timeout_sec=3.0):
            raise exceptions.TimedOutError(
                "timed out waiting for {}".format(self.service_names['close'])
            )
        return (self.service_types[key].Request(), client)

    def blackboard_contents_callback(self, msg):
        self.callback(msg.data)

    def shutdown(self):
        """
        Perform any ros-specific shutdown.
        """
        if self.node:
            self.node.destroy_node()
