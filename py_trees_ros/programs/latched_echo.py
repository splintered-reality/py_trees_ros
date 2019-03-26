#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. argparse::
   :module: py_trees_ros.programs.latched_echo
   :func: command_line_argument_parser
   :prog: py-trees-latched-echo

Example interaction with the services of a :class:`Blackboard Exchange <py_trees_ros.blackboard.Exchange>`:

.. image:: images/watcher.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import os
import py_trees.console as console
import py_trees_ros.utilities
import rclpy
import ros2topic.api

##############################################################################
# Module Constants
##############################################################################

DEFAULT_TRUNCATE_LENGTH = 128

##############################################################################
# Argparse
##############################################################################


def description():
    short = "ROS2 topic echo for latched topics\n"
    examples = ['/foo/bar']
    script_name = "py-trees-latched-echo"
    banner_line = console.green + "*" * 79 + "\n" + console.reset

    s = "\n"
    s += banner_line
    s += console.bold_white + "Blackboard Watcher".center(79) + "\n" + console.reset
    s += banner_line
    s += "\n"
    s += short
    s += "\n"
    s += console.bold + "Examples" + console.reset + "\n\n"
    s += '\n'.join(["    $ " + console.cyan + script_name + console.yellow + " {0}".format(example_args) + console.reset for example_args in examples])
    s += "\n\n"
    s += banner_line
    return s


def epilog():
    return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset


def command_line_argument_parser():
    # formatted_for_sphinx is an ugly hack to make sure sphinx does not pick up the colour codes.
    # works only by assuming that the only callee who calls it without setting the arg is sphinx's argparse
    parser = argparse.ArgumentParser(description=description(),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    parser.add_argument(
        'topic_name',
        help="Name of the ROS topic to listen to (e.g. '/chatter')"
    )
    parser.add_argument(
        'message_type', nargs='?',
        help="Type of the ROS message (e.g. 'std_msgs/String')"
    )
    return parser


##############################################################################
# Helpers
##############################################################################


def subscriber(node, topic_name, message_type, callback):
    if message_type is None:
        topic_names_and_types = ros2topic.api.get_topic_names_and_types(node=node, include_hidden_topics=True)
        try:
            expanded_name = rclpy.expand_topic_name.expand_topic_name(
                topic_name,
                node.get_name(),
                node.get_namespace()
            )
        except ValueError as e:
            raise RuntimeError(e)
        try:
            rclpy.validate_full_topic_name.validate_full_topic_name(expanded_name)
        except rclpy.exceptions.InvalidTopicNameException as e:
            raise RuntimeError(e)
        for n, t in topic_names_and_types:
            if n == expanded_name:
                if len(t) > 1:
                    raise RuntimeError(
                        "Cannot echo topic '%s', as it contains more than one type: [%s]" %
                        (topic_name, ', '.join(t))
                    )
                message_type = t[0]
                break
        else:
            raise RuntimeError(
                'Could not determine the type for the passed topic')

    msg_module = ros2topic.api.import_message_type(topic_name, message_type)

    node.create_subscription(
        msg_module,
        topic_name,
        callback,
        qos_profile=py_trees_ros.utilities.qos_profile_latched_topic()
        # qos_profile=rclpy.qos.qos_profile_sensor_data
    )


def echo(msg):
    print("Type: {}".format(type(msg)))
    print("{}".format(msg))


##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the latched echo script.
    """
    parser = command_line_argument_parser()
    args = parser.parse_args()

    rclpy.init()  # picks up sys.argv automagically internally
    node = rclpy.create_node("latched_echo" + "_" + str(os.getpid()))

    subscriber(node, args.topic_name, args.message_type, echo)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
