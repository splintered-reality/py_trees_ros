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

This tutorial inserts a context switching behaviour to run in tandem
with the scan rotation. It will reconfigure the rotation speed (to be
slower) and enable a hypothetical safety sensor pipeline immediately
prior to running the rotate action and subsequently reset the context
to what it was before commencing upon termination (success/failure or
interruption from above) of the scan.

.. note::

   This context switch could have easily been bundled into the
   scan behaviour itself, however keeping it separate
   promotes visibility and allows the scan behaviour to remain
   eminently reusable. Context switches can tend to be quite
   specific to a use case (i.e. not a behaviour).

Tree
^^^^

.. graphviz:: dot/tutorial-six.dot

.. literalinclude:: ../py_trees_ros/tutorials/six.py
   :language: python
   :linenos:
   :lines: 102-161
   :emphasize-lines: 38,58
   :caption: py_trees_ros/tutorials/six.py#create_root

**Context Switch**

.. graphviz:: dot/tutorial-six-context-switch.dot

The context switch is embedded beneath a parallel. Context will be cached
and reconfigured upon entry to the context and reset when the parallel
finalises or is interrupted from above.

Behaviours
^^^^^^^^^^

Introducing the scan context behaviour!

.. literalinclude:: ../py_trees_ros/tutorials/behaviours.py
   :language: python
   :linenos:
   :lines: 81-168
   :caption: py_trees_ros/tutorials/behaviours.py#scan_context

As you can see, all the action is happening in the :meth:`~py_trees.behaviour.Behaviour.initialise`
and :meth:`~py_trees.behaviour.Behaviour.terminate` methods. This class is intended for use
underneath a parallel with the action(s) that are designed to run in this context. This guarantees
that the context is reset no matter whether the action(s) succeed or fail, or the parallel itself
is interrupted from above.

Running
^^^^^^^

.. code-block:: bash

    $ roslaunch py_trees_ros tutorial_six.launch --screen

**Playing with the Spaghetti**

* Watch the the rotate and safety_sensor namespaces in rqt_reconfigure
* Press the scan button to start a scan
* The *enable* and *duration* variables should change while running
* The *enable* and *duration* variables should reset to initial values when finished

You can see in the diagrams below the relevant dynamic reconfigure variables
switching as the context runs.

.. image:: images/tutorial-six-before.png

.. image:: images/tutorial-six-during.png
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees_ros
import py_trees.console as console
import py_trees_msgs.msg as py_trees_msgs
import rospy
import std_msgs.msg as std_msgs
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

    scan = py_trees.composites.Sequence(name="Scan")
    is_scan_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Scan?",
        variable_name='event_scan_button',
        expected_value=True
    )
    scan_preempt = py_trees.composites.Selector(name="Preempt?")
    is_scan_requested_two = py_trees.meta.success_is_running(py_trees.blackboard.CheckBlackboardVariable)(
        name="Scan?",
        variable_name='event_scan_button',
        expected_value=True
    )
    scanning = py_trees.composites.Parallel(name="Scanning", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    scan_context_switch = py_trees_ros.tutorials.behaviours.ScanContext("Context Switch")
    scan_rotate = py_trees_ros.actions.ActionClient(
        name="Rotate",
        action_namespace="/rotate",
        action_spec=py_trees_msgs.RotateAction,
        action_goal=py_trees_msgs.RotateGoal(),
        override_feedback_message_on_running="rotating"
    )
    scan_flash_blue = py_trees_ros.tutorials.behaviours.FlashLedStrip(name="Flash Blue", colour="blue")
    scan_celebrate = py_trees.composites.Parallel(name="Celebrate", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    scan_flash_green = py_trees_ros.tutorials.behaviours.FlashLedStrip(name="Flash Green", colour="green")
    scan_pause = py_trees.timers.Timer("Pause", duration=3.0)
    idle = py_trees.behaviours.Running(name="Idle")

    # tree
    root.add_children([topics2bb, priorities])
    topics2bb.add_children([scan2bb, battery2bb])
    priorities.add_children([battery_check, scan, idle])
    battery_check.add_children([is_battery_ok, flash_led_strip])
    scan.add_children([is_scan_requested, scan_preempt, scan_celebrate])
    scan_preempt.add_children([is_scan_requested_two, scanning])
    scanning.add_children([scan_context_switch, scan_rotate, scan_flash_blue])
    scan_celebrate.add_children([scan_flash_green, scan_pause])
    return root


class SplinteredReality(object):

    def __init__(self):
        self.tree = py_trees_ros.trees.BehaviourTree(create_root())
        self.tree.add_post_tick_handler(self.publish_reality_report)
        self.report_publisher = rospy.Publisher("~report", std_msgs.String, queue_size=5)

    def setup(self):
        return self.tree.setup(timeout=15)

    def publish_reality_report(self, tree):
        if tree.tip().has_parent_with_name("Battery Emergency"):
            self.report_publisher.publish("battery")
        elif tree.tip().has_parent_with_name("Scan"):
            self.report_publisher.publish("scanning")
        else:
            self.report_publisher.publish("idle")

    def tick_tock(self):
        self.tree.tick_tock(500)

    def shutdown(self):
        self.tree.interrupt()


##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the demo script.
    """
    rospy.init_node("tree")
    splintered_reality = SplinteredReality()
    rospy.on_shutdown(splintered_reality.shutdown)
    if not splintered_reality.setup():
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    splintered_reality.tick_tock()
