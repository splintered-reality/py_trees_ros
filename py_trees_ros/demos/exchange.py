#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_ros/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################
from platform import node

"""
.. argparse::
   :module: py_trees_ros.demos.exchange
   :func: command_line_argument_parser
   :prog: py-trees-demo-exchange

.. graphviz:: dot/demo-exchange.dot

.. image:: images/exchange.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import functools
import py_trees_ros
import rclpy
import rclpy.node

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

def periodically_publish(exchange):
    print("Publishing the blackboard")
    exchange.publish_blackboard()

##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    ####################
    # Arg Parsing
    ####################
    # if this gets run with ros-specific command line arguments (e.g. when
    # executed from roslaunch, then we'll need to pass it something like
    # rospy.myargv()[1:])
    # Refer to https://github.com/ros2/demos/blob/master/demo_nodes_py/demo_nodes_py/topics/talker_qos.py
    unused_args = command_line_argument_parser().parse_args()
    print(description())

    ####################
    # Exchange
    ####################
    exchange = py_trees_ros.blackboard.Exchange()
    exchange.blackboard.dude = "Bob"
    exchange.blackboard.dudette = "Sarah"
    print(exchange.blackboard)

    ####################
    # Rclpy
    ####################
    rclpy.init(args=None)
    node = rclpy.node.Node('exchange')
    exchange.setup(node=node, timeout=15)
    timer_period = 1.0
    timer = node.create_timer(
        timer_period,
        functools.partial(periodically_publish, exchange=exchange)
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


