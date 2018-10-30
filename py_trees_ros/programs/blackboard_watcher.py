#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. argparse::
   :module: py_trees_ros.programs.blackboard_watcher
   :func: command_line_argument_parser
   :prog: py-trees-blackboard-watcher

Example interaction with the services of a :class:`Blackboard Exchange <py_trees_ros.blackboard.Exchange>`:

.. image:: images/watcher.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import os
import py_trees.console as console
import py_trees_ros
import rclpy
import sys
import time

##############################################################################
# Classes
##############################################################################


def description(formatted_for_sphinx):
    short = "Open up a window onto the blackboard!\n"
    examples = ["--list-variables", "access_point odom/pose/pose/position"]
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


def echo_blackboard_contents(contents):
    """
    Args:
        contents (:obj:`str`): blackboard contents

    .. note::
        The string comes pre-formatted with bash color identifiers and newlines.
        This is currently not especially good for anything other than debugging.
    """
    print("{}".format(contents))

##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the blackboard watcher script.
    """
    # Until there is support for a ros arg stripper
    # command_line_args = rospy.myargv(argv=sys.argv)[1:]
    command_line_args = None
    parser = command_line_argument_parser(formatted_for_sphinx=False)
    args = parser.parse_args(command_line_args)

    blackboard_watcher = py_trees_ros.blackboard.BlackboardWatcher(
        callback=echo_blackboard_contents,
        namespace_hint=args.namespace
    )

    rclpy.init(args=None)
    node = rclpy.create_node(
        node_name='watcher' + "_" + str(os.getpid()),
    )

    ####################
    # Setup
    ####################
    time.sleep(0.1) # ach, the magic foo before discovery works
    try:
        blackboard_watcher.setup(node, timeout_sec=15)
    except py_trees_ros.exceptions.NotFoundError as e:
        print(console.red + "\nERROR: {}".format(str(e)) + console.reset)
        sys.exit(1)
    except py_trees_ros.exceptions.MultipleFoundError as e:
        print(console.red + "\nERROR: {}".format(str(e)) + console.reset)
        if args.namespace is None:
            print(console.red + "\nERROR: select one with the --namespace argument" + console.reset)
        else:
            print(console.red + "\nERROR: but none matching the requested '%s'" % args.namespace + console.reset)

    ####################
    # Execute
    ####################
    if args.list_variables:
        try:
            pretty_print_variables(blackboard_watcher.list_variables())
        except (py_trees_ros.exceptions.NotReadyError,
                py_trees_ros.exceptions.ServiceError,
                py_trees_ros.exceptions.TimedOutError) as e:
            print(console.red + "ERROR: {}".format(str(e)) + console.reset)
            sys.exit(1)
    else:
        try:
            blackboard_watcher.open_connection(args.variables)
        except (py_trees_ros.exceptions.NotReadyError,
                py_trees_ros.exceptions.ServiceError,
                py_trees_ros.exceptions.TimedOutError) as e:
            print(console.red + "ERROR: {}".format(str(e)) + console.reset)
            sys.exit(1)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        try:
            blackboard_watcher.close_connection()
        except (py_trees_ros.exceptions.NotReadyError,
                py_trees_ros.exceptions.ServiceError,
                py_trees_ros.exceptions.TimedOutError) as e:
            print(console.red + "ERROR: {}".format(str(e)) + console.reset)
            sys.exit(1)
        node.destroy_node()
        rclpy.shutdown()
