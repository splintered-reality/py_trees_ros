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

The previous tutorial enables execution of a specific job, 'scanning' upon request.
However, as you start expanding the scope of your development,
you will need to start handling use cases with increasing complexity.

1. Multiple jobs at different priorities, Job A > Job B > Job C
2. Multiple jobs but execution is exclusive, Job A, B, if Job A is running, reject requests for Job B
3. Same core tree, but different jobs on different robots, Job A, B on Robot A, Job C on Robot B
4. Alter the list of permitted jobs dynamically, Job A on Robot A, later Job A, B on Robot A

The first use case is not as common as you'd expect - more often than not
a robot must follow a job through from start to finish - higher
priority switching does not support this.

Here we demonstrate how to (naively) facilitate 2 and 3. A list of jobs destined to
run will be loaded at launch time, but subtrees are not inserted until execution
is requested, i.e. just in time. When an incoming request is received, the corresponding
subtree will be inserted if anoother subtree is not currently running. Once a subtree runs
through to completion, it will be pruned. Insertion and pruning happens in the window
between ticks.

An technical requirement that makes this implementation practical
is to decouple the dependencies on job providers so that your launch does not
become burdened by explicity dependencies on any and all jobs.
Applications should depend on the application launcher, not the other way around.

Tree
^^^^

The core tree is identical to that used in :ref:`tutorial-full-scenario`, but with the
job subtree removed.

.. graphviz:: dot/tutorial-eight.dot

Job Handler
^^^^^^^^^^^

The job subtree create method is moved out into a separate class. Potentially this could be a class
in another module, another package (i.e. decopuled from where the core subtree is defined.
The class includes additional machinery listening for triggers to initiate job subtree insertion
(and subsequently, execution).

.. graphviz:: dot/tutorial-eight-scan.dot

.. literalinclude:: ../py_trees_ros/tutorials/jobs.py
   :language: python
   :linenos:
   :lines: 32-160
   :caption: py_trees_ros/tutorials/jobs.py#scan

Job Loading
^^^^^^^^^^^

Job handlers are loaded via a string at runtime. This ensures decoupling of the
the tree implementation from the job providers. **The SplinteredReality** class here
is responsible for setting up and tick tocking the tree.

.. literalinclude:: ../py_trees_ros/tutorials/eight.py
   :language: python
   :linenos:
   :lines: 152-171
   :emphasize-lines: 3,8-9,15-19
   :caption: py_trees_ros/tutorials/eight.py#job_loading

Just in Time
^^^^^^^^^^^^

Job subtrees are inserted and pruned via the tree pre and post tick handlers.

.. note::

   This is done in the free window between ticks, but the checking/insertion/deletion
   logic could alternatively have been achieved from behaviours inside the tree.

.. literalinclude:: ../py_trees_ros/tutorials/eight.py
   :language: python
   :linenos:
   :lines: 182-224
   :caption: py_trees_ros/tutorials/eight.py#just_in_time

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
                rospy.loginfo("{0}: finished [{1}]".format(job.name, job.status))
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
