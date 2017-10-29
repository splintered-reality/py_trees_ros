#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/stonier/py_trees_ros/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
Test the action client behaviour
"""
##############################################################################
# Imports
##############################################################################

import move_base_msgs.msg
import std_msgs.msg as std_msgs
import operator
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import unittest
import rostest

##############################################################################
# Tree
##############################################################################


class Runner(object):
    def __init__(self):

        self.logger = py_trees.logging.Logger("Testies")
        self.root = py_trees.Sequence(name="Root")
        self.action_client = py_trees_ros.actions.ActionClient(
            "Action Client",
            action_spec=move_base_msgs.msg.MoveBaseAction,
            action_goal=move_base_msgs.msg.MoveBaseGoal(),
            action_namespace="move_base",
        )
        self.root.add_child(self.action_client)
        self.tree = py_trees_ros.trees.BehaviourTree(self.root)
        self.tree.visitors.append(py_trees.visitors.DebugVisitor())
        self.tree.add_pre_tick_handler(self.pre_tick_handler)
        print(console.bold + "\nTree\n" + console.reset)
        py_trees.display.print_ascii_tree(self.root, 0)
        self.tree.setup(15)
        rospy.on_shutdown(self.shutdown)

    def tick_tock(self, number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.tree.count < number_of_iterations:
            self.tree.tick()
            if self.root.status == py_trees.Status.SUCCESS or self.root.status == py_trees.Status.FAILURE:
                break
            rate.sleep()

    def pre_tick_handler(self, behaviour_tree):
        self.logger.debug("")
        self.logger.debug("{:-^30}".format(" Run %s " % behaviour_tree.count))
        self.logger.debug("")

    def shutdown(self):
        self.tree.destroy()  # destroy the tree on shutdown to stop the behaviour
        self.tree.interrupt()


class TestActionClient(unittest.TestCase):
    '''
        Test for CheckSubscriberVariable
    '''
    def __init__(self, *args):
        super(TestActionClient, self).__init__(*args)

    ##############################################################################
    # Tick Tock
    ##############################################################################

    def test_tick_tock(self):
        self.runner = Runner()
        # move_base duration is configured for 1s, at 10hz, 20 is upper bound for the test
        self.runner.tick_tock(number_of_iterations=20)
        self.assertEquals(self.runner.action_client.feedback_message, "goal reached")
        self.assertEquals(self.runner.root.status, py_trees.Status.SUCCESS)


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("test_action_client", log_level=rospy.DEBUG)
    rostest.rosrun('py_trees',
                   'test_action_client',
                   TestActionClient)
