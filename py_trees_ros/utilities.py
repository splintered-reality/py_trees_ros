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

import py_trees.console as console
import rospy
import std_msgs.msg as std_msgs

##############################################################################
# Methods
##############################################################################


def basename(name):
    """
    Generate the basename from a ros name, e.g.
    ...code-block:: python
       > basename("~dude")
       > 'dude'
       > basename("/gang/dude")
       > 'dude'
    :param str name: ros name as input
    :returns: name stripped up until the last slash or tilde character.
    :rtype: str
    """
    return name.rsplit('/', 1)[-1].rsplit('~', 1)[-1]


def publish_resolved_names(publisher, ros_communication_handles):
    """
    Worker that provides a string representation of all the resolved names
    and publishes it so we can use it as an introspection topic in runtime.
    """
    s = console.bold + "\nResolved Names\n\n" + console.reset
    for handle in ros_communication_handles:
        s += console.yellow + "  " + handle.resolved_name + "\n" + console.reset
        publisher.publish(std_msgs.String("%s" % s))

##############################################################################
# Classes
##############################################################################


class Publishers(object):
    def __init__(self, publishers, introspection_topic_name="publishers"):
        """
        Converts the incoming list of publisher name, type, latched, queue_size specifications into proper variables of this class.
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
        :param publishers: incoming list of service specifications
        :type publishers: list of (str, str, bool, int) tuples representing (topic_name, publisher_type, latched, queue_size) specifications.
        """
        publisher_details = []
        for info in publishers:
            if len(info) == 4:
                publisher_details.append((basename(info[0]), info[0], info[1], info[2], info[3]))
            else:
                # naively assume the user got it right and added exactly 5 fields
                publisher_details.append(info)
        self.__dict__ = {name: rospy.Publisher(topic_name, publisher_type, latch=latched, queue_size=queue_size) for (name, topic_name, publisher_type, latched, queue_size) in publisher_details}
        publisher = rospy.Publisher("~introspection/" + introspection_topic_name, std_msgs.String, latch=True, queue_size=1)
        publish_resolved_names(publisher, self.__dict__.values())
        self.introspection_publisher = publisher