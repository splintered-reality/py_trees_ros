#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_ros/devel/LICENSE
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
import py_trees.console as console
import rclpy
import std_msgs.msg as std_msgs

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
    service_names = [name for name, types in service_names_and_types if service_type in types ]
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
    topic_names = [name for name, types in topic_names_and_types if topic_type in types ]
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

##############################################################################
# Convenience Classes
##############################################################################

class Publishers(object):
    """
    Utility class that groups the publishers together in one convenient structure.

    Args:
        publishers (obj:`tuple`): list of (str, str, bool, int) tuples representing (topic_name, publisher_type, latched, queue_size) specifications to create publishers with

    Examples:
        Convert the incoming list of publisher name, type, latched, queue_size specifications into proper variables of this class.

        .. code-block:: python

           publishers = rocon_python_comms.utils.Publishers(
               [
                   ('~foo', std_msgs.String, True, 5),
                   ('/foo/bar', std_msgs.String, False, 5),
                   ('foobar', '/foo/bar', std_msgs.String, False, 5),
               ]
           )

        Note: '~/introspection/dude' will become just 'dude' unless you prepend a field for the name
        as in the third example above.
    """
    def __init__(self, node, publishers, introspection_topic_name="publishers"):
        resolved_names = []
        publisher_details = []
        for info in publishers:
            if len(info) == 3:
                publisher_details.append((basename(info[0]), info[0], info[1], info[2]))
            else:
                # naively assume the user got it right and added exactly 5 fields
                publisher_details.append(info)

        # TODO: handle latched, queue size
        for (name, topic_name, publisher_type, latched) in publisher_details:
            if latched:
                self.__dict__[name] = node.create_publisher(
                    msg_type=publisher_type,
                    topic=topic_name,
                    qos_profile = qos_profile_latched_topic()
                )
            else:
                self.__dict__[name] = node.create_publisher(
                    msg_type=publisher_type,
                    topic=topic_name
                )
            resolved_names.append(resolve_name(node, topic_name))

        # TODO: handle latched, queue size
        self.introspection_publisher = node.create_publisher(
            msg_type=std_msgs.String,
            topic="~/introspection/" + introspection_topic_name,
            qos_profile = qos_profile_latched_topic()
        )
        s = console.bold + "\nPublishers\n\n" + console.reset
        for name in resolved_names:
            s += console.yellow + "  " + name + "\n" + console.reset
        self.introspection_publisher.publish(std_msgs.String(data=s))

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

# ##############################################################################
# # Classes
# ##############################################################################
#
#

# class Subscribers(object):
#     """
#     Converts the incoming list of subscriber name, msg type, callback triples into proper
#     variables of this class. Optionally you can prefix an arg that forces the name of
#     the variable created.
#
#     Args:
#         subscribers (obj:`tuple`): list of (str, str, bool, int) tuples representing (topic_name, subscriber_type, latched, queue_size) specifications to create subscribers with
#
#     Examples:
#
#         .. code-block:: python
#
#            subscribers = rocon_python_comms.utils.Subscribers(
#                [
#                    ('~dudette', std_msgs.String, subscriber_callback),
#                    ('/dudette/jane', std_msgs.String, subscriber_callback),
#                    ('jane', /dudette/jane', std_msgs.String, subscriber_callback),
#                ]
#            )
#
#         Note: '~/introspection/dude' will become just 'dude' unless you prepend a field for the name
#         as in the third example above.
#     """
#     def __init__(self, subscribers, introspection_topic_name="subscribers"):
#         subscriber_details = []
#         for info in subscribers:
#             if len(info) == 3:
#                 subscriber_details.append((basename(info[0]), info[0], info[1], info[2]))
#             else:
#                 # naively assume the user got it right and added exactly 4 fields
#                 subscriber_details.append(info)
#         self.__dict__ = {name: rospy.Subscriber(topic_name, subscriber_type, callback) for (name, topic_name, subscriber_type, callback) in subscriber_details}
#         publisher = rospy.Publisher("~introspection/" + introspection_topic_name, std_msgs.String, latch=True, queue_size=1)
#         publish_resolved_names(publisher, self.__dict__.values())
#         self.introspection_publisher = publisher
