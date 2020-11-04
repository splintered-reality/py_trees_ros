#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/stonier/py_trees_ros/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Various instantiable templates for action client/server style behaviours.
"""

##############################################################################
# Imports
##############################################################################

import actionlib
import py_trees
import rospy
import actionlib_msgs.msg as actionlib_msgs

##############################################################################
# Behaviours
##############################################################################


class ActionClient(py_trees.behaviour.Behaviour):
    """
    A generic action client interface. This simply sends a pre-configured
    goal to the action client.

    Cases where you might want to subclass and extend this behaviour:

    * Update the goal data in :meth:`~py_trees_ros.actions.ActionClient.initialise()`

      * e.g. use blackboard data to determine the new characteristics of your goal
    * Trigger true pre-emption by sending a new goal in :meth:`~py_trees_ros.actions.ActionClient.update()`

    Args:
        name (:obj:`str`): name of the behaviour
        action_spec (:obj:`any`): spec type for the action (e.g. move_base_msgs.msg.MoveBaseAction)
        action_goal (:obj:`any`): preconfigured action goal (e.g. move_base_msgs.msg.MoveBaseGoal())
        action_namespace (:obj:`str`): where you can find the action topics
        override_feedback_message_on_running (:obj:`str`): override the feedback message from the server

    Feedback messages are often accompanied with detailed messages that continuously change - since these
    are not significant and we don't want to log every change due to these messages, you can provide an override
    here that simply signifies the action is running.

    @todo: a more comprehensive way of filtering/parsing feedback messages to customise a running state so
    that it can identify and react to 'significant' events while running.
    """
    def __init__(self, name="Action Client", action_spec=None, action_goal=None, action_namespace="/action",
                 override_feedback_message_on_running="moving"):
        super(ActionClient, self).__init__(name)
        self.action_client = None
        self.sent_goal = False
        self.action_spec = action_spec
        self.action_goal = action_goal
        self.action_namespace = action_namespace
        self.override_feedback_message_on_running = override_feedback_message_on_running

    def setup(self, timeout):
        """
        Args:
            timeout (:obj:`float`): time to wait (0.0 is blocking forever)

        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True

    def initialise(self):
        """
        Reset the internal variables.
        """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        """
        If running and the current goal has not already succeeded, cancel it.

        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
               (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False
