#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.github.com/splintered-reality/py_trees_ros/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################
from asyncio.tasks import wait_for

"""
Behaviours for ROS actions.
"""

##############################################################################
# Imports
##############################################################################

import typing
import uuid

import action_msgs.msg as action_msgs  # GoalStatus
import py_trees
import rclpy.action

from . import exceptions

##############################################################################
# Behaviours
##############################################################################


class FromBlackboard(py_trees.behaviour.Behaviour):
    """
    An action client interface that draws goals from the blackboard. The
    lifecycle of this behaviour works as follows:

    * :meth:`initialise`: check blackboard for a goal and send
    * :meth:`update`: if a goal was sent, monitor progress
    * :meth:`terminate`: if interrupted while running, send a cancel request

    As a consequence, the status of this behaviour can be interpreted as follows:

    * :data:`~py_trees.common.Status.FAILURE`: no goal was found to send,
      it was rejected or it failed while executing
    * :data:`~py_trees.common.Status.RUNNING`: a goal was sent and is still
      executing on the server
    * :data:`~py_trees.common.Status.SUCCESS`: sent goal has completed with success

    To block on the arrival of a goal on the blackboard, use with the
    :class:`py_trees.behaviours.WaitForBlackboardVariable` behaviour. e.g.

    .. code-block:: python

        sequence = py_trees.composites.Sequence(name="Sequence")
        wait_for_goal = py_trees.behaviours.WaitForBlackboardVariable(
            name="WaitForGoal",
            variable_name="/my_goal"
        )
        action_client = py_trees_ros.aciton_clients.FromBlackboard(
            action_type=py_trees_actions.Dock,
            action_name="dock",
            name="ActionClient"
        )
        sequence.add_children([wait_for_goal, action_client])

    To customise a more interesting feedback message, pass in a method to the
    constructor, for example:

    .. code-block:: python

        action_client = py_trees_ros.action_clients.FromBlackboard(
            action_type=py_trees_actions.Dock,
            action_name="dock",
            name="ActionClient",
            generate_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)
        )

    Args:
        action_type: spec type for the action (e.g. move_base_msgs.action.MoveBase)
        action_name: where you can find the action topics & services (e.g. "bob/move_base")
        key: name of the key on the blackboard
        name: name of the behaviour (default: lowercase class name)
        generate_feedback_message: formatter for feedback messages, takes action_type.Feedback
            messages and returns strings (default: None)
        wait_for_server_timeout_sec: use negative values for a blocking but periodic check (default: -3.0)

    .. note::
       The default setting for timeouts (a negative value) will suit
       most use cases. With this setting the behaviour will periodically check and
       issue a warning if the server can't be found. Actually aborting the setup can
       usually be left up to the behaviour tree manager.
    """
    def __init__(self,
                 action_type: typing.Any,
                 action_name: str,
                 key: str,
                 name: str=py_trees.common.Name.AUTO_GENERATED,
                 generate_feedback_message: typing.Callable[[typing.Any], str]=None,
                 wait_for_server_timeout_sec: float=-3.0
                 ):
        super().__init__(name)
        self.action_type = action_type
        self.action_name = action_name
        self.wait_for_server_timeout_sec = wait_for_server_timeout_sec
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="goal",
            access=py_trees.common.Access.READ,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key)
        )
        self.generate_feedback_message = generate_feedback_message

        self.node = None
        self.action_client = None

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
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
            :class:`~py_trees_ros.exceptions.TimedOutError`: if the action server could not be found
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
        result = None
        if self.wait_for_server_timeout_sec > 0.0:
            result = self.action_client.wait_for_server(timeout_sec=self.wait_for_server_timeout_sec)
        else:
            iterations = 0
            period_sec = -1.0*self.wait_for_server_timeout_sec
            while not result:
                iterations += 1
                result = self.action_client.wait_for_server(timeout_sec=period_sec)
                if not result:
                    self.node.get_logger().warning(
                        "waiting for action server ... [{}s][{}][{}]".format(
                            iterations * period_sec,
                            self.action_name,
                            self.qualified_name
                        )
                    )
        if not result:
            self.feedback_message = "timed out waiting for the server [{}]".format(self.action_name)
            self.node.get_logger().error("{}[{}]".format(self.feedback_message, self.qualified_name))
            raise exceptions.TimedOutError(self.feedback_message)
        else:
            self.feedback_message = "... connected to action server [{}]".format(self.action_name)
            self.node.get_logger().info("{}[{}]".format(self.feedback_message, self.qualified_name))

    def initialise(self):
        """
        Reset the internal variables and kick off a new goal request.
        """
        self.logger.debug("{}.initialise()".format(self.qualified_name))

        # initialise some temporary variables
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None

        self.result_message = None
        self.result_status = None
        self.result_status_string = None

        try:
            self.send_goal_request(self.blackboard.goal)
            self.feedback_message = "sent goal request"
        except KeyError:
            pass  # self.send_goal_future will be None, check on that

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.

        Returns:
            :class:`py_trees.common.Status`
        """
        self.logger.debug("{}.update()".format(self.qualified_name))

        if self.send_goal_future is None:
            self.feedback_message = "no goal to send"
            return py_trees.common.Status.FAILURE
        if self.goal_handle is not None and not self.goal_handle.accepted:
            # goal was rejected
            self.feedback_message = "goal rejected"
            return py_trees.common.Status.FAILURE
        if self.result_status is None:
            return py_trees.common.Status.RUNNING
        elif not self.get_result_future.done():
            # should never get here
            self.node.get_logger().warn("got result, but future not yet done [{}]".format(self.qualified_name))
            return py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().debug("goal result [{}]".format(self.qualified_name))
            self.node.get_logger().debug("  status: {}".format(self.result_status_string))
            self.node.get_logger().debug("  message: {}".format(self.result_message))
            if self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED:  # noqa
                self.feedback_message = "successfully completed"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = "failed"
                return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        """
        If running and the current goal has not already succeeded, cancel it.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        if (
            self.status == py_trees.common.Status.RUNNING and
            new_status == py_trees.common.Status.INVALID
        ):
            self.send_cancel_request()

    def shutdown(self):
        """
        Clean up the action client when shutting down.
        """
        self.action_client.destroy()

    ########################################
    # Action Client Methods
    ########################################
    def feedback_callback(self, msg: typing.Any):
        """
        Default generator for feedback messages from the action server. This will
        update the behaviour's feedback message with a stringified version of the
        incoming feedback message.

        Args:
            msg: incoming feedback message (e.g. move_base_msgs.action.MoveBaseFeedback)
        """
        if self.generate_feedback_message is not None:
            self.feedback_message = "feedback: {}".format(self.generate_feedback_message(msg))
            self.node.get_logger().debug(
                '{} [{}]'.format(
                    self.feedback_message,
                    self.qualified_name
                )
            )

    def send_goal_request(self, goal: typing.Any):
        """
        Send the goal, get a future back and start lining up the
        chain of callbacks that will lead to a result.
        """
        self.feedback_message = "sending goal ..."
        self.node.get_logger().debug("{} [{}]".format(
            self.feedback_message,
            self.qualified_name
        ))
        self.send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
            # A random uuid is always generated, since we're not sending more than one
            # at a time, we don't need to generate and track them here
            # goal_uuid=unique_identifier_msgs.UUID(uuid=list(uuid.uuid4().bytes))
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: rclpy.task.Future):
        """
        Handle goal response, proceed to listen for the result if accepted.

        Args:
            future: incoming goal request result delivered from the action server
        """
        if future.result() is None:
            self.feedback_message = "goal request failed :[ [{}]\n{!r}".format(self.qualified_name, future.exception())
            self.node.get_logger().debug('... {}'.format(self.feedback_message))
            return
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.feedback_message = "goal rejected :( [{}]".format(self.qualified_name)
            self.node.get_logger().debug('... {}'.format(self.feedback_message))
            return
        else:
            self.feedback_message = "goal accepted :) [{}]".format(self.qualified_name)
            self.node.get_logger().debug("... {}".format(self.feedback_message))
            self.node.get_logger().debug("  {!s}".format(future.result()))

        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def send_cancel_request(self):
        """
        Send a cancel request to the server. This is triggered when the
        behaviour's status switches from :attr:`~py_trees.common.Status.RUNNING` to
        :attr:`~py_trees.common.Status.INVALID` (typically a result of a priority
        interrupt).
        """
        self.feedback_message = "cancelling goal ... [{}]".format(self.qualified_name)
        self.node.get_logger().debug(self.feedback_message)

        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future: rclpy.task.Future):
        """
        Immediate callback for the result of a cancel request. This will
        set the behaviour's feedback message accordingly.

        Args:
            future: incoming cancellation result delivered from the action server
        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.feedback_message = "goal successfully cancelled [{}]".format(self.qualified_name)
        else:
            self.feedback_message = "goal failed to cancel [{}]".format(self.qualified_name)
        self.node.get_logger().debug('... {}'.format(self.feedback_message))

    def get_result_callback(self, future: rclpy.task.Future):
        """
        Immediate callback for the result, saves data into local variables so that
        the update method can react accordingly.

        Args:
            future: incoming goal result delivered from the action server
        """
        self.result_message = future.result()
        self.result_status = future.result().status
        self.result_status_string = self.status_strings[self.result_status]


class FromConstant(FromBlackboard):
    """
    Convenience version of the action client that only ever sends the
    same goal.

    .. see-also: :class:`py_trees_ros.action_clients.FromBlackboard`

    Args:
        action_type: spec type for the action (e.g. move_base_msgs.action.MoveBase)
        action_name: where you can find the action topics & services (e.g. "bob/move_base")
        action_goal: the goal to send
        name: name of the behaviour (default: lowercase class name)
        generate_feedback_message: formatter for feedback messages, takes action_type.Feedback
            messages and returns strings (default: None)
        wait_for_server_timeout_sec: use negative values for a blocking but periodic check (default: -3.0)

    .. note::
       The default setting for timeouts (a negative value) will suit
       most use cases. With this setting the behaviour will periodically check and
       issue a warning if the server can't be found. Actually aborting the setup can
       usually be left up to the behaviour tree manager.
    """
    def __init__(self,
                 action_type: typing.Any,
                 action_name: str,
                 action_goal: typing.Any,
                 name: str=py_trees.common.Name.AUTO_GENERATED,
                 generate_feedback_message: typing.Callable[[typing.Any], str]=None,
                 wait_for_server_timeout_sec: float=-3.0
                 ):
        unique_id = uuid.uuid4()
        key = "/goal_" + str(unique_id)
        super().__init__(
            action_type=action_type,
            action_name=action_name,
            key=key,
            name=name,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec
        )
        # parent already instantiated a blackboard client
        self.blackboard.register_key(
            key=key,
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.set(name=key, value=action_goal)
