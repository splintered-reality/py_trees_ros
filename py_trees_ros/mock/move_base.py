#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks the move base action server of the ROS navigation stack.
"""

##############################################################################
# Imports
##############################################################################

import geometry_msgs.msg as geometry_msgs
import move_base_msgs.msg as move_base_msgs
import nav_msgs.msg as nav_msgs
import py_trees_ros
import rospy

from . import action_server

##############################################################################
# Classes
##############################################################################


class MoveBase(action_server.ActionServer):
    """
    Simulates:

    * move base interface
    * publishing on /odom (nav_msgs.msg.Odometry)
    * publishing on /pose (geometry_msgs.msg.PoseWithCovarianceStamped)

    Args:
        odometry_topic (:obj:`str`): name of the odometry topic
        pose_topic (:obj:`str`): name of the pose (with covariance stamped) topic
        duration (:obj:`int`): time for a goal to complete (seconds)
    """
    def __init__(self, odometry_topic='/odom', pose_topic='/pose', duration=None):
        super(MoveBase, self).__init__(action_name="move_base",
                                       action_type=move_base_msgs.MoveBaseAction,
                                       worker=self.worker,
                                       duration=duration
                                       )

        self.odometry = nav_msgs.Odometry()
        self.odometry.pose.pose.position = geometry_msgs.Point(0, 0, 0)
        self.pose = geometry_msgs.PoseWithCovarianceStamped()
        self.pose.pose.pose.position = geometry_msgs.Point(0, 0, 0)

        latched = True
        queue_size_five = 1
        self.publishers = py_trees_ros.utilities.Publishers(
            [
                ('pose', pose_topic, geometry_msgs.PoseWithCovarianceStamped, latched, queue_size_five),
                ('odometry', odometry_topic, nav_msgs.Odometry, latched, queue_size_five)
            ]
        )

        self.publishers.pose.publish(self.pose)
        self.publishers.odometry.publish(self.odometry)
        self.publishing_timer = rospy.Timer(period=rospy.Duration(0.5), callback=self.publish, oneshot=False)
        self.start()

    def worker(self):
        """
        Increment the odometry and pose towards the goal.
        """
        # actually doesn't go to the goal right now...but we could take the feedback from the action
        # and increment this to that proportion
        self.odometry.pose.pose.position.x += 0.01
        self.pose.pose.pose.position.x += 0.01

    def publish(self, unused_event):
        """
        Most things expect a continous stream of odometry/pose messages, so we
        run this in a background thread.
        """
        self.publishers.odometry.publish(self.odometry)
        self.publishers.pose.publish(self.pose)
