#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.github.com/splintered-reality/py_trees_ros/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
Talker node to assist with subscriber_check test.
"""
##############################################################################
# Imports
##############################################################################

import rospy
import std_msgs.msg as std_msgs

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('subscriber_check_talker')

    # fetch the utterance parameter from our parent namespace
    utterance = "Hello Dude"
    topic_name = "chatter"

    # publish the value of utterance repeatedly
    pub = rospy.Publisher(topic_name, std_msgs.String, queue_size=10, latch=True)
    while not rospy.is_shutdown():
        pub.publish(utterance)
        rospy.loginfo(utterance)
        rospy.sleep(2)
