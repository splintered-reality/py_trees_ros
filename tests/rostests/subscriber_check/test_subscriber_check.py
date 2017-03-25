#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/stonier/py_trees_ros/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
Test for CheckSubscriberVariable
"""
##############################################################################
# Imports
##############################################################################

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
        generate_failure = rospy.get_param("~failure", False)
        bad_topic = rospy.get_param("~bad_topic", False)
        topic_name = rospy.get_param("~topic_name", "chatter") if not bad_topic else "wrong_topic"
        expected_string = "Hello Dude" if not generate_failure else "Wrong String"

        print("")
        print(console.bold + "Parameters" + console.reset)
        print(console.cyan + "  generate_failure: " + console.yellow + str(generate_failure) + console.reset)
        print(console.cyan + "  bad_topic       : " + console.yellow + str(bad_topic) + console.reset)
        print(console.cyan + "  topic_name      : " + console.yellow + topic_name + console.reset)
        print(console.cyan + "  expected_string : " + console.yellow + expected_string + console.reset)
        print("")

        self.logger = py_trees.logging.Logger("Testies")
        self.root = py_trees.Sequence(name="Root")
        check_subscriber_variable = py_trees_ros.subscribers.CheckData(
            "Check",
            topic_name=topic_name,
            topic_type=std_msgs.String,
            variable_name="data",
            expected_value=expected_string,
            comparison_operator=operator.eq
        )
        wait_for_subscriber = py_trees_ros.subscribers.WaitForData(
            "Wait",
            topic_name=topic_name,
            topic_type=std_msgs.String,
        )
        self.root.add_child(check_subscriber_variable)
        self.root.add_child(wait_for_subscriber)
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


class TestSubscriberCheck(unittest.TestCase):
    '''
        Test for CheckSubscriberVariable
    '''
    def __init__(self, *args):
        super(TestSubscriberCheck, self).__init__(*args)

    ##############################################################################
    # Tick Tock
    ##############################################################################

    def test_tick_tock(self):
        self.runner = Runner()
        self.runner.tick_tock(number_of_iterations=50)
        self.assertEquals(self.runner.root.status, py_trees.Status.SUCCESS)


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("test_subscriber_check", log_level=rospy.DEBUG)
    rostest.rosrun('py_trees',
                   'test_subscriber_check',
                   TestSubscriberCheck)
