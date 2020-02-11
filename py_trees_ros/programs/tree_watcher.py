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
   :module: py_trees_ros.programs.tree_watcher
   :func: command_line_argument_parser
   :prog: py-trees-tree-watcher

Command line utility that introspects on a running
:class:`~py_trees_ros.trees.BehaviourTree` instance over it's snapshot
stream interfaces. Use to visualise the tree as a dot graph or
track tree changes, timing statistics and blackboard variables visited
by the tree on each tick.

.. image:: images/tree-watcher.gif

"""

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees
import py_trees.console as console
import py_trees_ros
import rclpy
import sys

##############################################################################
# Classes
##############################################################################


def description(formatted_for_sphinx):
    short = "Open up a window onto the behaviour tree!\n"
    long = ("\nRender a oneshot snapshot of the tree as a dot graph, or\n"
            "stream it and it's state continuously as unicode art on your console.\n"
            "This utility automatically discovers the running tree and opens\n"
            "interfaces to that, but if there is more than one tree executing\n"
            "use the namespace argument to differentiate between trees.\n"
            )
    examples = {
        "--dot-graph": "render the tree as a dot graph (does not include runtime information)",
        "/tree/snapshots": "connect to an existing snapshot stream (e.g. the default, if it is enabled)",
        "": "open and connect to a snapshot stream, visualise the tree graph and it's changes only",
        "-b": "open a snapshot stream and include visited blackboard variables",
        "-a": "open a snapshot stream and include blackboard access details (activity)",
        "-s": "open a snapshot stream and include timing statistics",
    }
    script_name = "py-trees-tree-watcher"

    if formatted_for_sphinx:
        # for sphinx documentation (doesn't like raw text)
        s = short
        s += long
        s += "\n"
        s += "**Examples:**\n\n"
        s += ".. code-block:: bash\n"
        s += "    \n"
        for command, comment in examples.items():
            s += "    # {}\n".format(comment)
            s += "    $ " + script_name + " {}\n".format(command)
        s += "\n"
    else:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Tree Watcher".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += short
        s += long
        s += "\n"
        s += console.bold + "Examples" + console.reset + "\n\n"
        for command, comment in examples.items():
            s += "    # {}\n".format(comment)
            s += "    $ " + console.cyan + script_name + console.yellow + " {}\n".format(command) + console.reset
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
    # common arguments
    parser.add_argument('topic_name', nargs='?', default=None, help='snapshot stream to connect to, will create a temporary stream if none specified')
    parser.add_argument('-n', '--namespace-hint', nargs='?', const=None, default=None, help='namespace hint snapshot stream services (if there should be more than one tree)')
    parser.add_argument('-a', '--blackboard-activity', action='store_true', help="show logged activity stream (streaming mode only)")
    parser.add_argument('-b', '--blackboard-data', action='store_true', help="show visited path variables (streaming mode only)")
    parser.add_argument('-s', '--statistics', action='store_true', help="show tick timing statistics (streaming mode only)")
    # don't use 'required=True' here since it forces the user to expclitly type out one option
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        '--snapshots',
        dest='mode',
        action='store_const',
        const=py_trees_ros.trees.WatcherMode.SNAPSHOTS,
        help='render ascii/unicode snapshots from a snapshot stream')
    group.add_argument(
        '--dot-graph',
        dest='mode',
        action='store_const',
        const=py_trees_ros.trees.WatcherMode.DOT_GRAPH,
        help='render the tree as a dot graph')
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
    Entry point for the tree watcher script.
    """
    ####################
    # Arg Parsing
    ####################

    # command_line_args = rclpy.utilities.remove_ros_args(command_line_args)[1:]
    command_line_args = None
    parser = command_line_argument_parser(formatted_for_sphinx=False)
    args = parser.parse_args(command_line_args)

    # mode is None if the user didn't specify any option in the exclusive group
    if args.mode is None:
        args.mode = py_trees_ros.trees.WatcherMode.SNAPSHOTS
    args.snapshot_period = 2.0 if (args.statistics or args.blackboard_data or args.blackboard_activity) else py_trees.common.Duration.INFINITE.value
    tree_watcher = py_trees_ros.trees.Watcher(
        namespace_hint=args.namespace_hint,
        topic_name=args.topic_name,
        parameters=py_trees_ros.trees.SnapshotStream.Parameters(
            blackboard_data=args.blackboard_data,
            blackboard_activity=args.blackboard_activity,
            snapshot_period=args.snapshot_period
        ),
        mode=args.mode,
        statistics=args.statistics,
    )

    ####################
    # Setup
    ####################
    rclpy.init(args=None)
    try:
        tree_watcher.setup(timeout_sec=2.0)
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
    except py_trees_ros.exceptions.TimedOutError as e:
        print(console.red + "\nERROR: {}\n".format(str(e)) + console.reset)
        sys.exit(1)

    ####################
    # Execute
    ####################
    try:
        while True:
            if not rclpy.ok():
                break
            if tree_watcher.done:
                if tree_watcher.xdot_process is None:
                    # no xdot found on the system, just break out and finish
                    break
                elif tree_watcher.xdot_process.poll() is not None:
                    # xdot running, wait for it to terminate
                    break
            rclpy.spin_once(tree_watcher.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if tree_watcher.xdot_process is not None:
            if tree_watcher.xdot_process.poll() is not None:
                tree_watcher.xdot_process.terminate()
        tree_watcher.shutdown()
        rclpy.shutdown()
