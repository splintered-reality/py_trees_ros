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

import actionlib
import geometry_msgs.msg as geometry_msgs
import move_base_msgs.msg as move_base_msgs
import nav_msgs.msg as nav_msgs
import py_trees_ros
import rospy

##############################################################################
# Classes
##############################################################################


class MoveBase(object):
    """
    Simulates:

    - move base interface
    - publishing on /odom
    - publishing on /pose

    Args:
        odometry_topic (:obj:`str`): name of the odometry topic
        pose_topic (:obj:`str`): name of the pose (with covariance stamped) topic
        control_duration (:obj:`int`): time for a goal to complete (seconds)
    """
    def __init__(self, odometry_topic='/odom', pose_topic='/pose', control_duration=5):
        # rospy.get_param("~control_duration", 5)  # secs : time for a goal to complete
        self.control_duration = control_duration
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

        self.action_server = actionlib.SimpleActionServer('move_base',
                                                          move_base_msgs.MoveBaseAction,
                                                          execute_cb=self.execute,
                                                          auto_start=False
                                                          )
        self.percent_completed = 0
        self.result = move_base_msgs.MoveBaseResult()
        self.publishers.pose.publish(self.pose)
        self.publishers.odometry.publish(self.odometry)
        self.action_server.start()
        self.publishing_timer = rospy.Timer(period=rospy.Duration(0.5), callback=self.publish, oneshot=False)

    def publish(self, unused_event):
        """
        Most things expect a continous stream of odometry/pose messages, so we
        run this in a background thread.
        """
        self.publishers.odometry.publish(self.odometry)
        self.publishers.pose.publish(self.pose)

    def execute(self, goal):
        # goal.target_pose = don't care
        frequency = 3.0  # hz
        increment = 100 / (frequency * self.control_duration)
        self.percent_completed = 0
        rate = rospy.Rate(frequency)  # hz
        rospy.loginfo("Move Base: received a goal")
        # if we just received a goal, we erase any previous pre-emption
        self.action_server.preempt_request = False
        while True:
            if rospy.is_shutdown() or self.action_server.is_preempt_requested():
                rospy.loginfo("Move Base: goal preempted")
                self.action_server.set_preempted(self.result, "goal was preempted")
                success = False
                break
            if self.percent_completed >= 100:
                rospy.loginfo("Move Base: feedback 100%")
                success = True
                break
            else:
                rospy.loginfo("Move Base: feedback {0:.2f}%".format(self.percent_completed))
                self.percent_completed += increment
                self.odometry.pose.pose.position.x += 0.01
                self.pose.pose.pose.position.x += 0.01
            rate.sleep()
        if success:
            rospy.loginfo("Move Base: goal success")
            self.action_server.set_succeeded(self.result, "goal reached")
