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
   :module: py_trees_ros.programs.blackboard_watcher
   :func: command_line_argument_parser
   :prog: py-trees-blackboard-watcher

Example interaction with the services of an :class:`py_trees_ros.blackboard.Exchange`:

.. image:: images/blackboard-watcher.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees.console as console
import py_trees_ros
import rclpy
import std_msgs.msg as std_msgs
import sys

##############################################################################
# Classes
##############################################################################


def description(formatted_for_sphinx):
    short = "Open up a window onto the blackboard!\n"
    examples = ["", "--list-variables", "access_point odom/pose/pose/position"]
    script_name = "py-trees-blackboard-watcher"

    if formatted_for_sphinx:
        # for sphinx documentation (doesn't like raw text)
        s = short
        s += "\n"
        s += "**Examples:**\n\n"
        s += ".. code-block:: bash\n"
        s += "    \n"
        s += '\n'.join(["    $ {0} {1}".format(script_name, example_args) for example_args in examples])
        s += "\n"
    else:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Blackboard Watcher".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += "Open up a window onto the blackboard!\n"
        s += "\n"
        s += console.bold + "Examples" + console.reset + "\n\n"
        s += '\n'.join(["    $ " + console.cyan + script_name + console.yellow + " {0}".format(example_args) + console.reset for example_args in examples])
        s += "\n\n"
        s += banner_line
    return s


def epilog(formatted_for_sphinx):
    if formatted_for_sphinx:
        return None
    else:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset


def command_line_argument_parser(formatted_for_sphinx=True):
    # formatted_for_sphinx is an ugly hack to make sure sphinx does not pick up the colour codes.
    # works only by assuming that the only callee who calls it without setting the arg is sphinx's argparse
    parser = argparse.ArgumentParser(description=description(formatted_for_sphinx),
                                     epilog=epilog(formatted_for_sphinx),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    parser.add_argument('-l', '--list-variables', action='store_true', default=None, help='list the blackboard variables')
    parser.add_argument('-n', '--namespace', nargs='?', default=None, help='namespace of blackboard services (if there should be more than one blackboard)')
    parser.add_argument('variables', nargs=argparse.REMAINDER, default=list(), help='space separated list of blackboard variables to watch')
    return parser


def pretty_print_variables(variables):
    s = "\n"
    s += console.bold + console.cyan + "Blackboard Variables:" + console.reset + console.yellow + "\n"
    for variable in variables:
        variable = variable.split('/')
        if len(variable) > 1:
            sep = "/"
        else:
            sep = ""
        s += "    " * len(variable) + sep + variable[-1] + "\n"
    s += console.reset
    print("{}".format(s))


##############################################################################
# Main
##############################################################################


def main(command_line_args=sys.argv[1:]):
    """
    Entry point for the blackboard watcher script.
    """
    # Until there is support for a ros arg stripper
    # command_line_args = rospy.myargv(argv=command_line_args)[1:]
    parser = command_line_argument_parser(formatted_for_sphinx=False)
    args = parser.parse_args(command_line_args)

    rclpy.init(args=None)
    blackboard_watcher = py_trees_ros.blackboard.BlackboardWatcher(
        namespace_hint=args.namespace
    )
    subscription = None
    ####################
    # Setup
    ####################
    try:
        blackboard_watcher.setup(timeout_sec=1.0)
    # setup discovery fails
    except py_trees_ros.exceptions.NotFoundError as e:
        print(console.red + "\nERROR: {}\n".format(str(e)) + console.reset)
        sys.exit(1)
    # setup discovery finds duplicates
    except py_trees_ros.exceptions.MultipleFoundError as e:
        print(console.red + "\nERROR: {}\n".format(str(e)) + console.reset)
        if args.namespace is None:
            print(console.red + "\nERROR: select one with the --namespace argument\n" + console.reset)
        else:
            print(console.red + "\nERROR: but none matching the requested '{}'\n".format(args.namespace) + console.reset)
        sys.exit(1)

    ####################
    # Execute
    ####################
    result = 0
    try:
        if args.list_variables:
            request, client = blackboard_watcher.create_service_client('list')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(blackboard_watcher.node, future)
            if future.result() is None:
                raise py_trees_ros.exceptions.ServiceError(
                    "service call failed [{}]".format(future.exception())
                )
            pretty_print_variables(future.result().variables)
        else:
            # request connection
            request, client = blackboard_watcher.create_service_client('open')
            request.variables = [variable.strip(',[]') for variable in args.variables]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(blackboard_watcher.node, future)
            response = future.result()
            blackboard_watcher.node.destroy_client(client)
            # connect
            watcher_topic_name = response.topic
            blackboard_watcher.node.get_logger().info(
                "creating subscription [{}]".format(watcher_topic_name)
            )
            subscription = blackboard_watcher.node.create_subscription(
                msg_type=std_msgs.String,
                topic=watcher_topic_name,
                callback=blackboard_watcher.echo_blackboard_contents,
                qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
            )
            # stream
            try:
                rclpy.spin(blackboard_watcher.node)
            except KeyboardInterrupt:
                pass
            # close connection
            request, client = blackboard_watcher.create_service_client('close')
            request.topic_name = watcher_topic_name
            future = client.call_async(request)
            rclpy.spin_until_future_complete(blackboard_watcher.node, future)
            if future.result() is None:
                raise py_trees_ros.exceptions.ServiceError(
                    "service call to close connection failed [{}]".format(future.exception())
                )
    # connection problems
    except (py_trees_ros.exceptions.NotReadyError,
            py_trees_ros.exceptions.ServiceError,
            py_trees_ros.exceptions.TimedOutError) as e:
        print(console.red + "\nERROR: {}".format(str(e)) + console.reset)
        result = 1
    if subscription is not None:
        blackboard_watcher.node.destroy_subscription(subscription)
    blackboard_watcher.shutdown()
    rclpy.shutdown()
    sys.exit(result)
