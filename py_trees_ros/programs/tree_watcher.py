#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. argparse::
   :module: py_trees_ros.programs.tree_watcher
   :func: command_line_argument_parser
   :prog: py-trees-tree-watcher

Interaction with the tree publishers.
"""

##############################################################################
# Imports
##############################################################################

import argparse
import functools
import rospy
import py_trees.console as console
import sys
import std_msgs.msg as std_msgs

from . import utilities

##############################################################################
# Classes
##############################################################################


def description(formatted_for_sphinx):
    short = "Introspect the tree\n"
    examples = ["--tree", "--snapshot", "--namespace my_tree --snapshot"]
    script_name = "py-trees-tree-watcher"

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
        s += console.bold_white + "Tree Watcher".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += short
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
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-t', '--tree', action='store_true', default=None, help='display the entire tree (no running information)')
    group.add_argument('-s', '--snapshot', action='store_true', default=None, help='display the visited state of the tree')
    parser.add_argument('-n', '--namespace', nargs='?', default=None, help='namespace of tree services (if there should be more than one)')
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
    print("%s" % s)


def echo_tree(msg, abort_after_one_message):
    # TODO: have the tree publish yaml/json/pickled data
    # and do the pretty printing here
    print("%s" % msg.data)
    if abort_after_one_message:
        rospy.signal_shutdown("")


def watch_tree(fully_resolved_topic_name, abort_after_one_message):
    rospy.init_node("tree_watcher", anonymous=True)
    # TODO: this will fall over if anyone actually remaps the blackboard name
    # Solution 1: command line argument (pushes the work to the user)
    # Solution 2: have a unique type for the blackboard (not std_msgs/String), look it up
    rospy.Subscriber(fully_resolved_topic_name, std_msgs.String,
                     functools.partial(echo_tree,
                                       abort_after_one_message=abort_after_one_message))
    while not rospy.is_shutdown():
        rospy.spin()


def handle_args(args):
    args.namespace = utilities.discover_namespace(args.namespace)
    # TODO: will completely fall apart if 'ascii', 'tree', or 'snapshot' are remapped
    # SOLN: create specific message types for each
    if args.tree:
        watch_tree(args.namespace + "/ascii/tree", abort_after_one_message=True)
    else:
        watch_tree(args.namespace + "/ascii/snapshot", abort_after_one_message=False)

##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the blackboard watcher script.
    """
    command_line_args = rospy.myargv(argv=sys.argv)[1:]
    parser = command_line_argument_parser(formatted_for_sphinx=False)
    args = parser.parse_args(command_line_args)
    handle_args(args)
