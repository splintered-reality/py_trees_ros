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

from . import exceptions

##############################################################################
# Methods
##############################################################################

def find_service(node, service_type, namespace=None):
    """
    Discover a service of the specified type and optionally, with the specified
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

##############################################################################
# Graveyard
##############################################################################


# def basename(name):
#     """
#     Generate the basename from a ros name.
#
#     Args:
#         name (:obj:`str`): ros name
#
#     Returns:
#         :obj:`str`: name stripped up until the last slash or tilde character.
#     Examples:
#
#         .. code-block:: python
#
#            basename("~dude")
#            # 'dude'
#            basename("/gang/dude")
#            # 'dude'
#     """
#     return name.rsplit('/', 1)[-1].rsplit('~', 1)[-1]
#
#
# def publish_resolved_names(publisher, ros_communication_handles):
#     """
#     Worker that provides a string representation of all the resolved names
#     and publishes it so we can use it as an introspection topic in runtime.
#
#     Args:
#         publisher (:obj:`rospy.Publisher`): use this object to publish with
#         ros_communication_handles ([]): list of handles with their resolved names to to publish
#     """
#     s = console.bold + "\nResolved Names\n\n" + console.reset
#     for handle in ros_communication_handles:
#         s += console.yellow + "  " + handle.resolved_name + "\n" + console.reset
#         publisher.publish(std_msgs.String("%s" % s))
#
# ##############################################################################
# # Classes
# ##############################################################################
#
#
# class Publishers(object):
#     """
#     Utility class that groups the publishers together in one convenient structure.
#
#     Args:
#         publishers (obj:`tuple`): list of (str, str, bool, int) tuples representing (topic_name, publisher_type, latched, queue_size) specifications to create publishers with
#
#     Examples:
#         Convert the incoming list of publisher name, type, latched, queue_size specifications into proper variables of this class.
#
#         .. code-block:: python
#
#            publishers = rocon_python_comms.utils.Publishers(
#                [
#                    ('~foo', std_msgs.String, True, 5),
#                    ('/foo/bar', std_msgs.String, False, 5),
#                    ('foobar', '/foo/bar', std_msgs.String, False, 5),
#                ]
#            )
#
#         Note: '~/introspection/dude' will become just 'dude' unless you prepend a field for the name
#         as in the third example above.
#     """
#     def __init__(self, publishers, introspection_topic_name="publishers"):
#         publisher_details = []
#         for info in publishers:
#             if len(info) == 4:
#                 publisher_details.append((basename(info[0]), info[0], info[1], info[2], info[3]))
#             else:
#                 # naively assume the user got it right and added exactly 5 fields
#                 publisher_details.append(info)
#         self.__dict__ = {name: rospy.Publisher(topic_name, publisher_type, latch=latched, queue_size=queue_size) for (name, topic_name, publisher_type, latched, queue_size) in publisher_details}
#         publisher = rospy.Publisher("~introspection/" + introspection_topic_name, std_msgs.String, latch=True, queue_size=1)
#         publish_resolved_names(publisher, self.__dict__.values())
#         self.introspection_publisher = publisher
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
