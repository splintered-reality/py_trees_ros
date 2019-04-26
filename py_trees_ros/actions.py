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
Behaviours for ROS actions.
"""

##############################################################################
# Imports
##############################################################################

import action_msgs.msg as action_msgs  # GoalStatus
import py_trees
import rclpy.action

from typing import Any, Callable

from . import exceptions

##############################################################################
# Behaviours
##############################################################################


class ActionClient(py_trees.behaviour.Behaviour):
    """
    A generic action client interface. This simply sends a pre-configured
    goal to the action client.

    Args:
        action_type (:obj:`any`): spec type for the action (e.g. move_base_msgs.msg.MoveBaseAction)
        action_name (:obj:`str`): where you can find the action topics & services (e.g. "bob/move_base")
        action_goal (:obj:`any`): pre-configured action goal (e.g. move_base_msgs.action.MoveBaseGoal())
        name (:obj:`str`, optional): name of the behaviour defaults to lowercase class name
        generate_feedback_message (:obj:`func`, optional): formatter for feedback messages, takes action_type.Feedback
            messages and returns strings, defaults to None
    """
    def __init__(self,
                 action_type,
                 action_name,
                 action_goal,
                 name: str=py_trees.common.Name.AUTO_GENERATED,
                 generate_feedback_message: Callable[[Any], str]=None,
                 ):
        super().__init__(name)
        self.action_type = action_type
        self.action_name = action_name
        self.action_goal = action_goal
        self.generate_feedback_message = generate_feedback_message

        self.node = None
        self.action_client = None
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None

        self.result_message = None
        self.result_status = None
        self.result_status_string = None

        self.status_strings = {
                action_msgs.GoalStatus.STATUS_UNKNOWN : "STATUS_UNKNOWN",  # noqa
                action_msgs.GoalStatus.STATUS_ACCEPTED : "STATUS_ACCEPTED",  # noqa
                action_msgs.GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",  # noqa
                action_msgs.GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",  # noqa
                action_msgs.GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",  # noqa
                action_msgs.GoalStatus.STATUS_CANCELED : "STATUS_CANCELED",  # noqa
                action_msgs.GoalStatus.STATUS_ABORTED  : "STATUS_ABORTED"  # noqa
            }

    def setup(self, **kwargs):
        """
        Setup the action client services and subscribers.

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            KeyError: if a ros2 node isn't passed under the key 'node' in kwargs
            TimedOutError: if the action server could not be found
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.action_client = rclpy.action.ActionClient(
            node=self.node,
            action_type=self.action_type,
            action_name=self.action_name
        )
        self.node.get_logger().info(
            "waiting for server ... [{}][{}]".format(
                self.action_name, self.qualified_name
            )
        )
        result = self.action_client.wait_for_server(timeout_sec=2.0)
        if not result:
            self.feedback_message = "timed out waiting for the server [{}]".format(self.action_name)
            self.node.get_logger().error("{}[{}]".format(self.feedback_message, self.qualified_name))
            raise exceptions.TimedOutError(self.feedback_message)
        else:
            self.feedback_message = "... connected to action server [{}]".format(self.action_name)
            self.node.get_logger().info("{}[{}]".format(self.feedback_message, self.qualified_name))

    def initialise(self):
        """
        Reset the internal variables.
        """
        self.logger.debug("{}.initialise()".format(self.qualified_name))
        self.send_goal_request()

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """
        self.logger.debug("{}.update()".format(self.qualified_name))

        if self.result_status is None:
            return py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().info("goal result [{}]".format(self.qualified_name))
            self.node.get_logger().info("  status: {}".format(self.result_status_string))
            self.node.get_logger().info("  message: {}".format(self.result_message))
            if self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED:  # noqa
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        If running and the current goal has not already succeeded, cancel it.

        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        self.send_cancel_request()

    def shutdown(self):
        """
        Clean up the action client when shutting down.
        """
        self.action_client.destroy()

    ########################################
    # Action Client Methods
    ########################################
    def feedback_callback(self, msg):
        if self.generate_feedback_message is not None:
            self.feedback_message = "feedback: {}".format(self.generate_feedback_message(msg))
            self.node.get_logger().info(
                '{} [{}]'.format(
                    self.feedback_message,
                    self.qualified_name
                )
            )

    def send_goal_request(self):
        """
        Send the goal and get a future back, but don't do any
        spinning here to await the future result.

        Returns:
            :class:`rclpy.task.Future`
        """
        self.feedback_message = "sending goal ..."
        self.node.get_logger().info("{} [{}]".format(
                self.feedback_message,
                self.qualified_name
            )
        )
        self.send_goal_future = self.action_client.send_goal_async(
                self.action_goal,
                feedback_callback=self.feedback_callback,
                # A random uuid is always generated, since we're not sending more than one
                # at a time, we don't need to generate and track them here
                # goal_uuid=unique_identifier_msgs.UUID(uuid=list(uuid.uuid4().bytes))
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle goal response, proceed to listen for the result if accepted.
        """
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.feedback_message = "goal rejected :( [{}]\n{!r}".format(self.qualified_name, future.exception())
            self.node.get_logger().info('... {}'.format(self.feedback_message))
            return
        else:
            self.feedback_message = "goal accepted :) [{}]".format(self.qualified_name)
            self.node.get_logger().info("... {}".format(self.feedback_message))
            self.node.get_logger().debug("  {!s}".format(future.result()))

        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def send_cancel_request(self):

        self.feedback_message = "cancelling goal ... [{}]".format(self.qualified_name)
        self.node.get_logger().info(self.feedback_message)

        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.feedback_message = "goal successfully cancelled [{}]".format(self.qualified_name)
        else:
            self.feedback_message = "goal failed to cancel [{}]".format(self.qualified_name)
        self.node.get_logger().info('... {}'.format(self.feedback_message))

    def get_result_callback(self, future):
        """
        Handle result.
        """
        print("Done: %s" % future.done())
        self.result_message = future.result()
        self.result_status = future.result().action_status
        self.result_status_string = self.status_strings[self.result_status]
