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

Whilst the previous tutorial works for running a job ('scanning'), it
becomes untenable if you need to load many job handlers, or require the tree to
support job handlers on different robots. i.e. you need to elimnate the dependency
on the job handler code itself and dynamically load/insert and execute it.

Tree
^^^^

.. graphviz:: dot/tutorial-eight.dot

Dynamic Loading
^^^^^^^^^^^^^^^

Loading job handlers via a string at runtime decouples
the tree implementation from the job provider itself.
Think of the job handlers as apps,
and the core tree as the app launcher.

.. literalinclude:: ../py_trees_ros/tutorials/eight.py
   :language: python
   :linenos:
   :lines: 125-145
   :emphasize-lines: 1,6-7,13-17
   :caption: py_trees_ros/tutorials/eight.py#dynamic_loading

Dynamic Subtrees
^^^^^^^^^^^^^^^^

The second feature of this tutorial is the insertion and deletion of
the job subtree via the tree pre and post tick handlers when a new
goal has arrived, or the currently running job has finished.

This is done in the free time between ticks, but the checking/insertion/deletion
logic could just as easily have been performed inside behaviours inside the tree.

.. literalinclude:: ../py_trees_ros/tutorials/eight.py
   :language: python
   :linenos:
   :lines: 155-197
   :caption: py_trees_ros/tutorials/eight.py#dynamic_subtrees

Job Handler - 'Scan'
^^^^^^^^^^^^^^^^^^^^

.. graphviz:: dot/tutorial-eight-scan.dot

.. literalinclude:: ../py_trees_ros/tutorials/jobs.py
   :language: python
   :linenos:
   :lines: 32-160
   :caption: py_trees_ros/tutorials/jobs.py#scan

Running
^^^^^^^

.. code-block:: bash

    $ roslaunch py_trees_ros tutorial_eight.launch --screen

"""

##############################################################################
# Imports
##############################################################################

import importlib
import py_trees
import py_trees_ros
import py_trees.console as console
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
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        variable_name="event_cancel_button"
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
    idle = py_trees.behaviours.Running(name="Idle")

    # tree
    root.add_children([topics2bb, priorities])
    topics2bb.add_children([cancel2bb, battery2bb])
    priorities.add_children([battery_check, idle])
    battery_check.add_children([is_battery_ok, flash_led_strip])
    return root


class SplinteredReality(object):

    def __init__(self, jobs):
        """
        Initialise a core tree (minus a job) and preload job classes ready to
        be used in spinning up and running the job later when requested.

        Args:
            jobs ([:obj:`str`]): list of module names as strings (e.g. 'py_trees_ros.tutorials.jobs.Scan')
        """
        self.tree = py_trees_ros.trees.BehaviourTree(create_root())
        self.tree.add_pre_tick_handler(self.pre_tick_handler)
        self.tree.add_post_tick_handler(self.post_tick_handler)
        self.report_publisher = rospy.Publisher("~report", std_msgs.String, queue_size=5)
        self.jobs = []
        for job in jobs:
            module_name = '.'.join(job.split('.')[:-1])
            class_name = job.split('.')[-1]
            self.jobs.append(getattr(importlib.import_module(module_name), class_name)())
        self.current_job = None

    def setup(self):
        """
        Redirect the setup function"

        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        return self.tree.setup(timeout=15)

    def pre_tick_handler(self, tree):
        """
        Check if a job is running. If not, spin up a new job subtree
        if a request has come in.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """
        if not self.busy():
            for job in self.jobs:
                if job.goal:
                    job_root = job.create_root(job.goal)
                    if not job_root.setup(timeout=15):
                        rospy.logerr("{0}: failed to setup".format(job.name))
                        continue
                    tree.insert_subtree(job_root, self.priorities.id, 1)
                    rospy.loginfo("{0}: inserted job subtree".format(job_root.name))
                    job.goal = None
                    self.current_job = job

    def post_tick_handler(self, tree):
        """
        Check if a job is running and if it has finished. If so, prune the job subtree from the tree.
        Additionally, make a status report upon introspection of the tree.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """
        # delete the job subtree if it is finished
        if self.busy():
            job = self.priorities.children[-2]
            if job.status == py_trees.common.Status.SUCCESS or job.status == py_trees.common.Status.FAILURE:
                rospy.loginfo("{0}: finished [{1}".format(job.name, job.status))
                tree.prune_subtree(job.id)
                self.current_job = None
        # publish a status report
        if self.busy():
            job = self.priorities.children[-2]
            self.report_publisher.publish(self.current_job.create_report_string(job))
        elif tree.tip().has_parent_with_name("Battery Emergency"):
            self.report_publisher.publish("battery")
        else:
            self.report_publisher.publish("idle")

    def busy(self):
        """
        Check if a job subtree exists and is running.

        Returns:
            :obj:`bool`: whether it is busy with a job subtree or not
        """
        return len(self.priorities.children) == 3

    @property
    def priorities(self):
        return self.tree.root.children[-1]

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
    splintered_reality = SplinteredReality(jobs=['py_trees_ros.tutorials.jobs.Scan'])
    rospy.on_shutdown(splintered_reality.shutdown)
    if not splintered_reality.setup():
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    splintered_reality.tick_tock()
