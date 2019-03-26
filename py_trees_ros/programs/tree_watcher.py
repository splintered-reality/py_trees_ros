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

Command line utility to interact with a running
:class:`~py_trees_ros.trees.BehaviourTree` instance. Print a static ascii
art view of the tree, view snapshots of the tree and tick statistics as
a stream or display the dot graph of the tree.

.. image:: images/tree-watcher.gif

"""

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees.console as console
import py_trees_ros
import rclpy
import sys

##############################################################################
# Classes
##############################################################################


def description(formatted_for_sphinx):
    short = "Open up a window onto the behaviour tree!\n"
    long = ("\nPrint static or dynamic (snapshot) ascii art views of the tree on the console\n"
            "or render a dot graph (static view only). Use the namespace argument to select\n"
            "from trees when there are multiple available.\n"
            )
    examples = [
        "", "--ascii-tree", "--dot-tree", "--namespace=foo --ascii-tree"
    ]
    script_name = "py-trees-tree-watcher"

    if formatted_for_sphinx:
        # for sphinx documentation (doesn't like raw text)
        s = short
        s += long
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
        s += console.bold_white + "Tree Watcher".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += short
        s += long
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
    parser.add_argument('-n', '--namespace', nargs='?', default=None, help='namespace of pytree communications (if there should be more than one tree active)')
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        '-s', '--ascii-snapshot',
        dest='viewing_mode',
        action='store_const',
        const=py_trees_ros.trees.WatcherMode.ASCII_SNAPSHOT,
        help='print an ascii snapshot of the tree state')
    group.add_argument(
        '-a', '--ascii-tree',
        dest='viewing_mode',
        action='store_const',
        const=py_trees_ros.trees.WatcherMode.ASCII_TREE,
        help='print an ascii representation of the tree structure')
    group.add_argument(
        '-d', '--dot-tree',
        dest='viewing_mode',
        action='store_const',
        const=py_trees_ros.trees.WatcherMode.DOT_TREE,
        help='print a dot representation of the tree structure')
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
# Tree Watcher
##############################################################################

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
    # Until there is support for a ros arg stripper
    # command_line_args = rospy.myargv(argv=sys.argv)[1:]
    command_line_args = None
    parser = command_line_argument_parser(formatted_for_sphinx=False)
    args = parser.parse_args(command_line_args)
    if not args.viewing_mode:
        args.viewing_mode = py_trees_ros.trees.WatcherMode.ASCII_SNAPSHOT

    ####################
    # Setup
    ####################
    tree_watcher = py_trees_ros.trees.Watcher(
        namespace_hint=args.namespace,
        mode=args.viewing_mode
    )
    rclpy.init(args=None)
    try:
        tree_watcher.setup()
    except py_trees_ros.exceptions.NotFoundError as e:
        print(console.red + "\nERROR: {}\n".format(str(e)) + console.reset)
        sys.exit(1)
    except py_trees_ros.exceptions.MultipleFoundError as e:
        print(console.red + "\nERROR: {}\n".format(str(e)) + console.reset)
        if args.namespace is None:
            print(console.red + "\nERROR: select one with the --namespace argument\n" + console.reset)
            sys.exit(1)
        else:
            print(console.red + "\nERROR: but none matching the requested '%s'\n" % args.namespace + console.reset)
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
                    break
                elif tree_watcher.xdot_process.poll() is not None:
                    break
            rclpy.spin_once(tree_watcher.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    if tree_watcher.xdot_process is not None:
        if tree_watcher.xdot_process.poll() is not None:
            tree_watcher.xdot_process.terminate()
    tree_watcher.node.destroy_node()
    rclpy.shutdown()
