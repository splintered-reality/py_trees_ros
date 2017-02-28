#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
About
^^^^^

A second priority branch and a first example of a very common style of
behaviour in a ROS robot - that of an action client connecting to an
action server in the robot control layer.

Tree
^^^^

.. graphviz:: dot/tutorial-two.dot

The white coloured ellipse shown here is a decorated behaviour (in this
case a :class:`~py_trees.composites.Selector` with the :func:`~py_trees.meta.success_is_failure`
decorator (looking forward to better visualisations for this in the future).

.. literalinclude:: ../py_trees_ros/tutorials/two.py
   :language: python
   :linenos:
   :lines: 104-129
   :caption: py_trees_ros/tutorials/two.py

Here we've added a high priority branch for
dealing with a low battery that causes the hardware strip to flash.

An important point here is to make sure that the flashing behaviour gets
invalidated as soon as the battery becomes ok again. This will trigger the
flashing behaviour's terminate method (see below) to send off a command to
clear the request. To do this we've made use of a higher priority 'Is Battery Ok?'
check underneath the selector, but also had to decorate the selector with
:func:`~py_trees.meta.success_is_failure` to make sure the priority branch is chosen appropriately.

You could have also designed this particular subtree with sequences and parallels
instead of the selector and decorator here.

.. tip::
    When designing, it's very useful to get a visual on what you are
    doing, even before you actually execute or implement
    anything more than a tree of skeleton behaviours. For this tutorial,
    you can render with:

    .. code-block:: python

        py-trees-render py_trees_ros.tutorials.two.create_root

Behaviours
^^^^^^^^^^

Introducing the rotate action client behaviour!

.. literalinclude:: ../py_trees_ros/tutorials/behaviours.py
   :language: python
   :linenos:
   :lines: 27-77
   :caption: py_trees_ros/battery.py

A new feature here is the way it uses the terminate method to put a 'fullstop'
to the commands sent when ticking. Note also that it is permanently in the
:attr:`~py_trees.common.Status.RUNNING` state while ticking. Behaviours do
not *have* to return :attr:`~py_trees.common.Status.SUCCESS` or
:attr:`~py_trees.common.Status.FAILURE`, they can be just as involved in the
decision making via the way they behave when cancelled or interrupted.

Running
^^^^^^^

.. code-block:: bash

    $ roslaunch py_trees_ros tutorial_five.launch --screen

.. image:: images/tutorial-two-battery-ok.png

.. image:: images/tutorial-two-battery-low.png
"""

##############################################################################
# Imports
##############################################################################

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys

##############################################################################
# Behaviours
##############################################################################


def create_root():
    # behaviours
    root = py_trees.composites.Parallel("Tutorial")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    scan2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Scan2BB",
        topic_name="/dashboard/scan",
        variable_name="event_scan_button"
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(name="Battery2BB",
                                                   topic_name="/battery/state",
                                                   threshold=30.0
                                                   )
    priorities = py_trees.composites.Selector("Priorities")
    battery_check = py_trees.meta.success_is_failure(py_trees.composites.Selector)(name="Battery Emergency")
    is_battery_ok = py_trees.blackboard.CheckBlackboardVariable(
        name="Battery Ok?",
        variable_name='battery_low_warning',
        expected_value=False
    )
    flash_led_strip = py_trees_ros.tutorials.behaviours.FlashLedStrip(
        name="Flash Red",
        colour="red")
    scan = py_trees.composites.Parallel(name="Scan", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    scan_rotate = py_trees_ros.tutorials.behaviours.Rotate(name="Rotate")
    scan_flash_led_strip = py_trees_ros.tutorials.behaviours.FlashLedStrip(
        name="Flash Blue",
        colour="blue")
    idle = py_trees.behaviours.Running(name="Idle")

    # tree
    root.add_children([topics2bb, priorities])
    topics2bb.add_children([scan2bb, battery2bb])
    priorities.add_children([battery_check, scan, idle])
    battery_check.add_children([is_battery_ok, flash_led_strip])
    scan.add_children([scan_rotate, scan_flash_led_strip])
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
    rospy.init_node("tree")
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(500)
