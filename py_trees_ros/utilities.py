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

import py_trees_ros_interfaces.msg as py_trees_msgs
import py_trees_ros_interfaces.srv as py_trees_srvs
import rclpy
import rclpy.qos
import time

from . import exceptions

##############################################################################
# Methods
##############################################################################


def find_service(node, service_type, namespace=None):
    """
    Discover a service of the specified type and if necessary, under the specified
    namespace.

    Args:
        node (:class:`~rclpy.node.Node`): nodes have the discovery methods
        service_type (:obj:`str`): primary lookup hint
        namespace (:obj:`str`): secondary lookup hint

    Returns:
        :obj:`str`: fully expanded the service name

    Raises:
        :class:`~py_trees_ros.exceptions.NotFoundError`: if no services were found
        :class:`~py_trees_ros.exceptions.MultipleFoundError`: if multiple services were found
    """
    # TODO: follow the pattern of ros2cli to create a node without the need to init
    # rcl (might get rid of the magic sleep this way). See:
    #    https://github.com/ros2/ros2cli/blob/master/ros2service/ros2service/verb/list.py
    #    https://github.com/ros2/ros2cli/blob/master/ros2cli/ros2cli/node/strategy.py

    # Returns a list of the form: [('exchange/blackboard', ['std_msgs/String'])
    service_names_and_types = node.get_service_names_and_types()
    service_names = [name for name, types in service_names_and_types if service_type in types]
    if namespace is not None:
        service_names = [name for name in service_names if namespace in name]

    if not service_names:
        raise exceptions.NotFoundError("service not found [type: {}]".format(service_type))
    elif len(service_names) == 1:
        return service_names[0]
    else:
        raise exceptions.MultipleFoundError("multiple services found [type: {}]".format(service_type))


def find_topic(node, topic_type, namespace=None):
    """
    Discover a topic of the specified type and if necessary, under the specified
    namespace.

    Args:
        node (:class:`~rclpy.node.Node`): nodes have the discovery methods
        topic_type (:obj:`str`): primary lookup hint
        namespace (:obj:`str`): secondary lookup hint

    Returns:
        :obj:`str`: fully expanded the service name

    Raises:
        :class:`~py_trees_ros.exceptions.NotFoundError`: if no services were found
        :class:`~py_trees_ros.exceptions.MultipleFoundError`: if multiple services were found
    """
    # TODO: follow the pattern of ros2cli to create a node without the need to init
    # rcl (might get rid of the magic sleep this way). See:
    #    https://github.com/ros2/ros2cli/blob/master/ros2service/ros2service/verb/list.py
    #    https://github.com/ros2/ros2cli/blob/master/ros2cli/ros2cli/node/strategy.py

    # Returns a list of the form: [('exchange/blackboard', ['std_msgs/String'])
    topic_names_and_types = node.get_topic_names_and_types()
    topic_names = [name for name, types in topic_names_and_types if topic_type in types]
    if namespace is not None:
        topic_names = [name for name in topic_names if namespace in name]

    if not topic_names:
        raise exceptions.NotFoundError("topic not found [type: {}]".format(topic_type))
    elif len(topic_names) == 1:
        return topic_names[0]
    else:
        raise exceptions.MultipleFoundError("multiple topics found [type: {}]".format(topic_type))


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


def qos_profile_latched_topic():
    """
    Convenience retrieval for a latched topic (publisher / subscriber)
    """
    return rclpy.qos.QoSProfile(
        history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1,
        durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
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
        publisher_details (obj:`tuple`): list of (str, str, bool, int) tuples representing
                                  (unique_name, topic_name, publisher_type, latched)
                                  specifications for creating publishers

    Examples:
        Convert the incoming list of specifications into proper variables of this class.

        .. code-block:: python

           publishers = py_trees.utilities.Publishers(
               [
                   ('foo', ~/foo', std_msgs.String, True, 5),
                   ('bar', /foo/bar', std_msgs.String, False, 5),
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
                    qos_profile=qos_profile_latched_topic()
                )
            else:
                self.__dict__[name] = node.create_publisher(
                    msg_type=publisher_type,
                    topic=topic_name
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
    Utility class that groups the publishers together in one convenient structure.

    Args:
        subscriber_details (obj:`tuple`): list of (str, str, bool, func) tuples representing
                                  (unique_name, topic_name, subscriber_type, latched, callback)
                                  specifications for creating subscribers

    Examples:
        Convert the incoming list of specifications into proper variables of this class.

        .. code-block:: python

           subscribers = py_trees.utilities.Subscribers(
               [
                   ('foo', ~/foo', std_msgs.String, True, foo),
                   ('bar', /foo/bar', std_msgs.String, False, self.foo),
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
                    qos_profile=qos_profile_latched_topic()
                )
            else:
                self.__dict__[name] = node.create_subscription(
                    msg_type=subscriber_type,
                    topic=topic_name,
                    callback=callback
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
