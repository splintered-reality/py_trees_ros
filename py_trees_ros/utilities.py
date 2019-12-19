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
Assorted utility functions.
"""

##############################################################################
# Imports
##############################################################################

import os
import pathlib

import py_trees_ros_interfaces.msg as py_trees_msgs  # noqa
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa
import rclpy
import rclpy.node
import rclpy.qos
import time
import typing

from . import exceptions

##############################################################################
# Methods
##############################################################################


def find_service(node: rclpy.node.Node,
                 service_type: str,
                 namespace: str=None,
                 timeout: float=0.5):
    """
    Discover a service of the specified type and if necessary, under the specified
    namespace.

    Args:
        node (:class:`~rclpy.node.Node`): nodes have the discovery methods
        service_type (:obj:`str`): primary lookup hint
        namespace (:obj:`str`): secondary lookup hint
        timeout: immediately post node creation, can take time to discover the graph (sec)

    Returns:
        :obj:`str`: fully expanded service name

    Raises:
        :class:`~py_trees_ros.exceptions.NotFoundError`: if no services were found
        :class:`~py_trees_ros.exceptions.MultipleFoundError`: if multiple services were found
    """
    # TODO: follow the pattern of ros2cli to create a node without the need to init
    # rcl (might get rid of the magic sleep this way). See:
    #    https://github.com/ros2/ros2cli/blob/master/ros2service/ros2service/verb/list.py
    #    https://github.com/ros2/ros2cli/blob/master/ros2cli/ros2cli/node/strategy.py

    loop_period = 0.1  # seconds
    clock = rclpy.clock.Clock()
    start_time = clock.now()
    service_names = []
    while clock.now() - start_time < rclpy.time.Duration(seconds=timeout):
        # Returns a list of the form: [('exchange/blackboard', ['std_msgs/String'])
        service_names_and_types = node.get_service_names_and_types()
        service_names = [name for name, types in service_names_and_types if service_type in types]
        if namespace is not None:
            service_names = [name for name in service_names if namespace in name]
        if service_names:
            break
        time.sleep(loop_period)

    if not service_names:
        raise exceptions.NotFoundError("service not found [type: {}]".format(service_type))
    elif len(service_names) == 1:
        return service_names[0]
    else:
        raise exceptions.MultipleFoundError("multiple services found [type: {}]".format(service_type))


def find_topics(
        node: rclpy.node.Node,
        topic_type: str,
        namespace: str=None,
        timeout: float=0.5) -> typing.List[str]:
    """
    Discover a topic of the specified type and if necessary, under the specified
    namespace.

    Args:
        node: nodes have the discovery methods
        topic_type: primary lookup hint
        namespace: secondary lookup hint
        timeout: check every 0.1s until this timeout is reached (can be None -> checks once)

    .. note: Immediately post node creation, it can take some time to discover the graph.

    Returns:
        list of fully expanded topic names (can be empty)
    """
    # TODO: follow the pattern of ros2cli to create a node without the need to init
    # rcl (might get rid of the magic sleep this way). See:
    #    https://github.com/ros2/ros2cli/blob/master/ros2service/ros2service/verb/list.py
    #    https://github.com/ros2/ros2cli/blob/master/ros2cli/ros2cli/node/strategy.py
    loop_period = 0.1  # seconds
    clock = rclpy.clock.Clock()
    start_time = clock.now()
    topic_names = []
    while True:
        # Returns a list of the form: [('exchange/blackboard', ['std_msgs/String'])
        topic_names_and_types = node.get_topic_names_and_types()
        topic_names = [name for name, types in topic_names_and_types if topic_type in types]
        if namespace is not None:
            topic_names = [name for name in topic_names if namespace in name]
        if topic_names:
            break
        if timeout is None or (clock.now() - start_time) > rclpy.time.Duration(seconds=timeout):
            break
        else:
            time.sleep(loop_period)
    return topic_names


def basename(name):
    """
    Generate the basename from a ros name.

    Args:
        name (:obj:`str`): ros name

    Returns:
        :obj:`str`: name stripped up until the last slash or tilde character.
    Examples:

        .. code-block:: python

           basename("~dude")
           # 'dude'
           basename("/gang/dude")
           # 'dude'
    """
    return name.rsplit('/', 1)[-1].rsplit('~', 1)[-1]


def get_py_trees_home():
    """
    Find the default home directory used for logging, bagging and other
    esoterica.
    """
    # TODO: update with replacement for rospkg.get_ros_home() when it arrives
    home = os.path.join(str(pathlib.Path.home()), ".ros2", "py_trees")
    return home


def qos_profile_latched():
    """
    Convenience retrieval for a latched topic (publisher / subscriber)
    """
    return rclpy.qos.QoSProfile(
        history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1,
        durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
    )


def qos_profile_unlatched():
    """
    Default profile for an unlatched topic (in py_trees_ros).
    """
    return rclpy.qos.QoSProfile(
        history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1,
        durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
        reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
    )


def resolve_name(node, name):
    """
    Convenience function for getting the resolved name (similar to 'publisher.resolved_name' in ROS1).

    Args:
        node (:class:`rclpy.node.Node`): the node, namespace it *should* be relevant to
        name (obj:`str`): topic or service name

    .. note::

       This entirely depends on the user providing the relevant node, name pair.
    """
    return rclpy.expand_topic_name.expand_topic_name(
        name,
        node.get_name(),
        node.get_namespace()
    )


def create_anonymous_node_name(node_name="node") -> str:
    """
    Creates an anonoymous node name by adding a suffix created from
    a monotonic timestamp, sans the decimal.

    Returns:
        :obj:`str`: the unique, anonymous node name
    """
    return node_name + "_" + str(time.monotonic()).replace('.', '')


##############################################################################
# Convenience Classes
##############################################################################


class Publishers(object):
    """
    Utility class that groups the publishers together in one convenient structure.

    Args:
        publisher_details (obj:`tuple`): list of (str, str, msgType, bool, int) tuples representing
                                  (unique_name, topic_name, publisher_type, latched)
                                  specifications for creating publishers

    Examples:
        Convert the incoming list of specifications into proper variables of this class.

        .. code-block:: python

           publishers = py_trees.utilities.Publishers(
               [
                   ('foo', '~/foo', std_msgs.String, True, 5),
                   ('bar', '/foo/bar', std_msgs.String, False, 5),
                   ('foobar', '/foo/bar', std_msgs.String, False, 5),
               ]
           )
    """
    def __init__(self, node, publisher_details, introspection_topic_name="publishers"):
        # TODO: check for the correct setting of publisher_details
        self.publisher_details_msg = []
        for (name, topic_name, publisher_type, latched) in publisher_details:
            if latched:
                self.__dict__[name] = node.create_publisher(
                    msg_type=publisher_type,
                    topic=topic_name,
                    qos_profile=qos_profile_latched()
                )
            else:
                self.__dict__[name] = node.create_publisher(
                    msg_type=publisher_type,
                    topic=topic_name,
                    qos_profile=qos_profile_unlatched()
                )
            resolved_name = resolve_name(node, topic_name)
            message_type = publisher_type.__class__.__module__.split('.')[0] + "/" + publisher_type.__class__.__name__
            self.publisher_details_msg.append(
                py_trees_msgs.PublisherDetails(
                    topic_name=resolved_name,
                    message_type=message_type,
                    latched=latched
                )
            )

        self.introspection_service = node.create_service(
            py_trees_srvs.IntrospectPublishers,
            "~/introspection/" + introspection_topic_name,
            self.introspection_callback
        )

    def introspection_callback(self, unused_request, response):
        response.publisher_details = self.publisher_details_msg
        return response


class Subscribers(object):
    """
    Utility class that groups subscribers together in one convenient structure.

    Args:
        subscriber_details (obj:`tuple`): list of (str, str, msgType, bool, func) tuples representing
                                  (unique_name, topic_name, subscriber_type, latched, callback)
                                  specifications for creating subscribers

    Examples:
        Convert the incoming list of specifications into proper variables of this class.

        .. code-block:: python

           subscribers = py_trees.utilities.Subscribers(
               [
                   ('foo', '~/foo', std_msgs.String, True, foo),
                   ('bar', '/foo/bar', std_msgs.String, False, self.foo),
                   ('foobar', '/foo/bar', std_msgs.String, False, foo.bar),
               ]
           )
    """
    def __init__(self, node, subscriber_details, introspection_topic_name="subscribers"):
        # TODO: check for the correct setting of subscriber_details
        self.subscriber_details_msg = []
        for (name, topic_name, subscriber_type, latched, callback) in subscriber_details:
            if latched:
                self.__dict__[name] = node.create_subscription(
                    msg_type=subscriber_type,
                    topic=topic_name,
                    callback=callback,
                    qos_profile=qos_profile_latched()
                )
            else:
                self.__dict__[name] = node.create_subscription(
                    msg_type=subscriber_type,
                    topic=topic_name,
                    callback=callback,
                    qos_profile=qos_profile_unlatched()
                )

            resolved_name = resolve_name(node, topic_name)
            message_type = subscriber_type.__class__.__module__.split('.')[0] + "/" + subscriber_type.__class__.__name__
            self.subscriber_details_msg.append(
                py_trees_msgs.SubscriberDetails(
                    topic_name=resolved_name,
                    message_type=message_type,
                    latched=latched
                )
            )

        self.introspection_service = node.create_service(
            py_trees_srvs.IntrospectSubscribers,
            "~/introspection/" + introspection_topic_name,
            self.introspection_callback
        )

    def introspection_callback(self, unused_request, response):
        response.subscriber_details = self.subscriber_details_msg
        return response


class Services(object):
    """
    Utility class that groups services together in one convenient structure.

    Args:
        service_details (obj:`tuple`): list of (str, str, srvType, func) tuples representing
                                  (unique_name, topic_name, service_type, callback)
                                  specifications for creating services

    Examples:
        Convert the incoming list of specifications into proper variables of this class.

        .. code-block:: python

           services = py_trees.utilities.Services(
               [
                   ('open_foo', '~/get_foo', foo_interfaces.srv.OpenFoo, open_foo_callback),
                   ('open_foo', '/foo/open', foo_interfaces.srv.OpenFoo, self.open_foo_callback),
                   ('get_foo_bar', '/foo/bar', foo_interfaces.srv.GetBar, self.foo.get_bar_callback),
               ]
           )
    """
    def __init__(self, node, service_details, introspection_topic_name="services"):
        # TODO: check for the correct setting of subscriber_details
        self.service_details_msg = []
        for (name, service_name, service_type, callback) in service_details:
            self.__dict__[name] = node.create_service(
                srv_type=service_type,
                srv_name=service_name,
                callback=callback,
                qos_profile=rclpy.qos.qos_profile_services_default
            )
            resolved_name = resolve_name(node, service_name)
            service_type = service_type.__class__.__module__.split('.')[0] + "/" + service_type.__class__.__name__
            self.service_details_msg.append(
                py_trees_msgs.ServiceDetails(
                    service_name=resolved_name,
                    service_type=service_type,
                )
            )

        self.introspection_service = node.create_service(
            py_trees_srvs.IntrospectServices,
            "~/introspection/" + introspection_topic_name,
            self.introspection_callback
        )

    def introspection_callback(self, unused_request, response):
        response.subscriber_details = self.subscriber_details_msg
        return response
