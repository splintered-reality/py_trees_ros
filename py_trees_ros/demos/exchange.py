#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_ros/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. argparse::
   :module: py_trees_ros.demos.blackboard_exchange
   :func: command_line_argument_parser
   :prog: py-trees-demo-blackboard-exchange

.. graphviz:: dot/demo-blackboard.dot

.. image:: images/blackboard.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees
import py_trees_ros

import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description():
    content = "Launches a blackboard exchange.\n"
    content += "\n"
    content += "Interact with it via the py-trees-blackboard-watcher utility.\n"

    if console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Exchange".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def epilog():
    if console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    return parser


##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    unused_args = command_line_argument_parser().parse_args()
    print(description())
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    exchange = py_trees_ros.blackboard.Exchange()
    exchange.blackboard.dude = "Bob"
    exchange.blackboard.dudette = "Sarah"

    ####################
    # Execute
    ####################
    print(exchange.blackboard)
