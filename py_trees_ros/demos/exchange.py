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
   :module: py_trees_ros.demos.exchange
   :func: command_line_argument_parser
   :prog: py-trees-ros-demo-exchange

.. graphviz:: dot/demo-exchange.dot
"""

##############################################################################
# Imports
##############################################################################

import argparse
import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys

##############################################################################
# Classes
##############################################################################


def description():
    content = "Demonstrates usage of the Exchange for introspecting a blackboard in ROS\n"
    content += "\n"
    content += "Instantiates a behaviour tree with a very simple pair of behaviours in a\n"
    content += "sequence that increment counters on the blackboard every tick. It\n"
    content += "additionally instantiates a blackboard exchange to provide a ROS API to\n"
    content += "the blackboard which can be used to permit introspection and viewing.\n"
    content += "\n"
    content += "Use with py-trees-blackboard-watcher to introspect the blackboard from ROS.\n"
    content += "\n"

    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Blackboard Exchange".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def epilog():
    if py_trees.console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    """
    Command line parser with an option for rendering.

    Returns:
        argparse.ArgumentParser: the parser
    """
    parser = argparse.ArgumentParser(description=description(),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    parser.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
    return parser


def pre_tick_handler(behaviour_tree):
    """
    This prints a banner will run immediately before every tick of the tree.

    Args:
        behaviour_tree (:class:`~py_trees.trees.BehaviourTree`): the tree custodian

    """
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


def post_tick_handler(snapshot_visitor, exchange, behaviour_tree):
    """
    Prints an ascii tree with the current snapshot status as well
    as a copy of the blackboard.

    Args:
        snapshot_visitor (:class:`~py_trees_ros.visitors.SnapshotVisitor`): carries tick traversal information
        behaviour_tree (:class:`~py_trees.trees.BehaviourTree`): tree information
        exchange (:class:`~py_trees_ros.blackboard.Exchange`): the exchange itself
    """
    print("\n" + py_trees.display.ascii_tree(behaviour_tree.root,
                                             snapshot_information=snapshot_visitor))
    exchange.publish_blackboard()
    print("{0}".format(exchange.blackboard))


class BlackboardWriter(py_trees.behaviour.Behaviour):
    """
    Writes and increments a counter on the blackboard.

    Args:
        name (:obj:`str`): name of the behaviour
        increment (:obj:`int`): how much to increment the counter each tick
    """
    def __init__(self, name="Writer", increment=1):
        super(BlackboardWriter, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.increment = increment

    def setup(self, timeout):
        """
        Setup the blackboard and counter.

        Args:
            timeout (:obj:`float`): time to wait (0.0 is blocking forever)

        Returns:
            :obj:`bool`: suceess or failure of the operation (always True)
        """
        self.blackboard = py_trees.blackboard.Blackboard()
        self.counter = 0
        return True

    def update(self):
        """
        Write a dictionary to the blackboard and return :data:`~py_trees.Status.SUCCESS`.
        """
        self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.counter))
        self.blackboard.set(self.name.lower(), self.counter)
        self.counter += self.increment
        return py_trees.Status.SUCCESS


def create_root():
    """
    Create a sequence with two simple blackboard writers.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = py_trees.composites.Sequence("Sequence")
    carbonara = BlackboardWriter(name="Carbonara")
    gnocchi = BlackboardWriter(name="Gnocchi", increment=2)
    root.add_children([carbonara, gnocchi])
    return root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the demo script.
    """
    parser = command_line_argument_parser()
    myargs = rospy.myargv(argv=sys.argv)
    args = parser.parse_args(args=myargs[1:])
    print(description())

    root = create_root()

    ####################
    # Rendering
    ####################
    if args.render:
        py_trees.display.render_dot_tree(root)
        sys.exit()

    ####################
    # Tree Stewardship
    ####################
    rospy.init_node("demo_exchange")

    exchange = py_trees_ros.blackboard.Exchange()

    # easier if you use py_trees_ros.trees.BehaviourTrees since it takes care of setup and publish
    # but this is useful for demonstration
    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor, exchange))
    behaviour_tree.visitors.append(snapshot_visitor)

    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    behaviour_tree.setup(timeout=15)
    exchange.setup(15)

    ####################
    # Execute
    ####################
    behaviour_tree.tick_tock(500)
