#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros_tutorials/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Action server/client templates.
"""

##############################################################################
# Imports
##############################################################################

import rclpy
import rclpy.action
import rclpy.callback_groups
import rclpy.parameter
import threading
import time

##############################################################################
# Action Server
##############################################################################
#
# References:
#
#  action client         : https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/action/client.py
#  action server         : https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/action/server.py
#  action client PR      : https://github.com/ros2/rclpy/pull/257
#  action server PR      : https://github.com/ros2/rclpy/pull/270
#  action examples       : https://github.com/ros2/examples/tree/master/rclpy/actions
#
# Note:
#  currently:  py_trees_ros_interfaces.action.Dock_Goal.Request()
#  to come(?): py_trees_ros_interfaces.action.Dock.GoalRequestService.Request()


class GenericServer(object):
    """
    Generic action server that can be subclassed to quickly create action
    servers of varying types in a mock simulation.

    Dynamic Reconfigure:
        * **~duration** (:obj:`float`)

          * reconfigure the duration to be used for the next goal execution

    Args:
        node_name (:obj:`str`): name to use for the node (e.g. docking_controller)
        action_name (:obj:`str`): name of the action server (e.g. dock)
        action_type (:obj:`any`): type of the action server (e.g. py_trees_ros_interfaces.Dock
        custom_execute_callback (:obj:`func`): callback to be executed inside the execute loop, no args
        generate_cancelled_result (action_type.Result): custom result method
        generate_preempted_result (action_type.Result): custom result method
        generate_success_result (action_type.Result): custom result method
        goal_recieved_callback(:obj:`func`): callback to be executed immediately upon receiving a goal
        duration (:obj:`float`): forcibly override the dyn reconf time for a goal to complete (seconds)

    Use the ``dashboard`` to dynamically reconfigure the parameters.

    There are some shortcomings in this class that could be addressed to make it more robust:

     - check for matching goal id's when disrupting execution
     - execute multiple requests in parallel / pre-emption (not even tried)
    """
    def __init__(self,
                 node_name,
                 action_name,
                 action_type,  # e.g. py_trees_ros_interfaces.action.Dock
                 generate_feedback_message=None,
                 generate_cancelled_result=None,
                 generate_preempted_result=None,
                 generate_success_result=None,
                 goal_received_callback=lambda request: None,
                 duration=None):
        self.node = rclpy.create_node(
            node_name,
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    'duration',
                    rclpy.parameter.Parameter.Type.DOUBLE,
                    5.0  # seconds
                ),
            ],
            automatically_declare_parameters_from_overrides=True
        )
        # override
        if duration is not None:
            self.node.set_parameters([
                rclpy.parameter.Parameter(
                    'duration',
                    rclpy.parameter.Parameter.Type.DOUBLE,
                    duration  # seconds
                ),
            ])
        # Needs a member variable to capture the dynamic parameter
        # value when accepting the goal and pass that to the execution
        # of the goal. Not necessary to initialise here, but done
        # for completeness
        self.duration = self.node.get_parameter("duration").value

        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.frequency = 3.0  # hz
        self.percent_completed = 0

        self.action_type = action_type
        if generate_feedback_message is None:
            self.generate_feedback_message = lambda: self.action_type.Feedback()
        else:
            self.generate_feedback_message = generate_feedback_message
        if generate_cancelled_result is None:
            self.generate_cancelled_result = lambda: self.action_type.Result(message="goal cancelled")
        else:
            self.generate_cancelled_result = generate_cancelled_result
        if generate_preempted_result is None:
            self.generate_preempted_result = lambda: self.action_type.Result(message="goal pre-empted")
        else:
            self.generate_preempted_result = generate_preempted_result
        if generate_success_result is None:
            self.generate_success_result = lambda: self.action_type.Result(message="goal executed with success")
        else:
            self.generate_success_result = generate_success_result
        self.goal_received_callback = goal_received_callback
        self.goal_handle = None

        self.action_server = rclpy.action.ActionServer(
            node=self.node,
            action_type=self.action_type,
            action_name=action_name,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),  # needed?
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_goal_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            result_timeout=10
        )

    def goal_callback(self, goal_request):
        """
        Args:
            goal_request: of <action_type>.GoalRequest with members
                goal_id (unique_identifier.msgs.UUID) and those specified in the action
        """
        self.node.get_logger().info("received a goal")
        self.goal_received_callback(goal_request)
        self.percent_completed = 0
        self.duration = self.node.get_parameter("duration").value
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
         ) -> rclpy.action.CancelResponse:
        """
        Cancel any currently executing goal

        Args:
            cancel_request (:class:`~rclpy.action.server.ServerGoalHandle`):
                handle with information about the
                goal that is requested to be cancelled
        """
        self.node.get_logger().info("cancel requested: [{goal_id}]".format(
            goal_id=goal_handle.goal_id))
        return rclpy.action.CancelResponse.ACCEPT

    def execute_goal_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
         ):
        """
        Check for pre-emption, but otherwise just spin around gradually incrementing
        a hypothetical 'percent' done.

        Args:
            goal_handle (:class:`~rclpy.action.server.ServerGoalHandle`): the goal handle of the executing action
        """
        # goal.details (e.g. pose) = don't care
        self.node.get_logger().info("executing a goal")
        increment = 100 / (self.frequency * self.duration)
        while True:
            # TODO: use a rate when they have it
            time.sleep(1.0 / self.frequency)
            self.percent_completed += increment
            with self.goal_lock:
                if goal_handle.is_active:
                    if goal_handle.is_cancel_requested:
                        result = self.generate_cancelled_result()
                        message = "goal cancelled at {percentage:.2f}%%".format(
                            percentage=self.percent_completed)
                        self.node.get_logger().info(message)
                        goal_handle.canceled()
                        return result
                    elif goal_handle.goal_id != self.goal_handle.goal_id:
                        result = self.generate_preempted_result()
                        message = "goal pre-empted at {percentage:.2f}%%".format(
                            percentage=self.percent_completed)
                        self.node.get_logger().info(message)
                        goal_handle.abort()
                        return result
                    elif self.percent_completed >= 100.0:
                        self.percent_completed = 100.0
                        self.node.get_logger().info("sending feedback 100%%")
                        result = self.generate_success_result()
                        message = "goal executed with success"
                        self.node.get_logger().info(message)
                        goal_handle.succeed()
                        return result
                    else:
                        self.node.get_logger().info("sending feedback {percentage:.2f}%%".format(
                            percentage=self.percent_completed))
                        goal_handle.publish_feedback(
                            self.generate_feedback_message()
                        )
                else:  # ! active
                    self.node.get_logger().info("goal is no longer active, aborting")
                    result = self.action_type.Result()
                    return result

    def handle_accepted_callback(self, goal_handle):
        self.node.get_logger().info("handle accepted")
        with self.goal_lock:
            self.goal_handle = goal_handle
            goal_handle.execute()

    def abort(self):
        """
        This method is typically only used when the system is shutting down and
        there is an executing goal that needs to be abruptly terminated.
        """
        with self.goal_lock:
            if self.goal_handle and self.goal_handle.is_active:
                self.node.get_logger().info("aborting...")
                self.goal_handle.abort()

    def shutdown(self):
        """
        Cleanup
        """
        self.action_server.destroy()
        self.node.destroy_node()
