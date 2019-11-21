#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros/devel/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
Various behaviours that enable common interactions with ROS transforms.
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import rclpy.qos
import tf2_ros

import py_trees.console as console

import geometry_msgs.msg as geometry_msgs

##############################################################################
# Behaviours
##############################################################################


class FromBlackboard(py_trees.behaviour.Behaviour):
    """
    Broadcast a transform from the blackboard using the
    transform broadcaster mechanisms.

    If no it fails to find a geometry_msgs.Transform
    object on the blackboard, or the blackboard variable is None,
    this behaviour will update with status
    :attr:`~py_trees.common.Status.FAILURE`.

    Args:
        variable_name: name of the transform variable on the blackboard
        target_frame: name of the frame to transform into
        source_frame: name of the input frame
        static: designate whether it is a static transform or otherwise
        qos_profile: qos profile for the non-static broadcaster
        static_qos_profile: qos profile for the static broadcaster (default: use tf2_ros' defaults)
        name: name of the behaviour
    """
    def __init__(
        self,
        variable_name: str,
        target_frame: str,
        source_frame: str,
        static: bool,
        qos_profile: rclpy.qos.QoSProfile,
        static_qos_profile: rclpy.qos.QoSProfile=None,
        name: str=py_trees.common.Name.AUTO_GENERATED,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name)
        self.variable_name = variable_name
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.static = static
        self.qos_profile = qos_profile
        self.static_qos_profile = static_qos_profile

        self.blackboard.register_key(
            key=self.variable_name,
            access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """
        Initialises the transform broadcaster.

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            KeyError: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        if self.static:
            self.broadcaster = tf2_ros.StaticTransformBroadcaster(
                node=self.node,
                qos=self.static_qos_profile
            )
        else:
            self.broadcaster = tf2_ros.TransformBroadcaster(
                node=self.node,
                qos=self.qos_profile
            )

    def update(self):
        """
        Retrieves the transform from the blackboard, stamps it
        and subsequently broadcasts it.

        Raises:
            TypeError: if the blackboard variable is of the wrong type.
        """
        try:
            transform = self.blackboard.get(self.variable_name)
        except KeyError:
            self.feedback_message = "no transform to broadcast"
            return py_trees.common.Status.FAILURE
        if transform is None:
            self.feedback_message = "no transform to broadcast"
            return py_trees.common.Status.FAILURE
        if type(transform) != geometry_msgs.Transform:
            raise TypeError("'{}' is not of type geometry_msgs/Transform".format(self.variable_name))
        self.feedback_message = "transform sent"
        transform_stamped = geometry_msgs.TransformStamped()
        transform_stamped.header.stamp = rclpy.clock.Clock().now().to_msg()
        transform_stamped.header.frame_id = self.source_frame
        transform_stamped.child_frame_id = self.target_frame
        transform_stamped.transform = transform
        # print(console.green + "Send: {}".format(transform_stamped) + console.reset)
        self.broadcaster.sendTransform(transform_stamped)
        return py_trees.common.Status.SUCCESS


class ToBlackboard(py_trees.behaviour.Behaviour):
    """
    Blocking behaviour that looks for a transform and writes
    it to a variable on the blackboard.

    If it fails to find a transform immediately, it will update
    with status :attr:`~py_trees.common.Status.RUNNING` and write
    'None' to the blackboard.

    .. tip::

       To ensure consistent decision making, use this behaviour
       up-front in your tree's tick to record a transform that
       can be locked in for the remainder of the tree tick.

    **Usage Patterns**

    * clearing_policy == :attr:`~py_trees.common.ClearingPolicy.ON_INTIALISE`

    Use if you have subsequent behaviours that need to make decisions on
    whether the transform was received or not.

    * clearing_policy == :attr:`~py_trees.common.ClearingPolicy.NEVER`

    Never clear the result. Useful for static transforms or if you are doing
    your own lookup on the timestamps for any relevant decision making.

    Args:
        variable_name: name of the key to write to on the blackboard
        target_frame: name of the frame to transform into
        source_frame: name of the input frame
        qos_profile: qos profile for the non-static subscriber
        static_qos_profile: qos profile for the static subscriber (default: use tf2_ros' defaults)
        name: name of the behaviour

    Raises:
        TypeError: if the clearing policy is neither
           :attr:`~py_trees.common.ClearingPolicy.ON_INITIALISE`
           or :attr:`~py_trees.common.ClearingPolicy.NEVER`
    """
    def __init__(
        self,
        variable_name,
        target_frame: str,
        source_frame: str,
        qos_profile: rclpy.qos.QoSProfile,
        static_qos_profile: rclpy.qos.QoSProfile=None,
        clearing_policy: py_trees.common.ClearingPolicy=py_trees.common.ClearingPolicy.ON_INITIALISE,
        name: str=py_trees.common.Name.AUTO_GENERATED,
    ):
        super().__init__(name=name)
        self.variable_name = variable_name
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(
            key=self.variable_name,
            access=py_trees.common.Access.WRITE
        )
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.qos_profile = qos_profile
        self.static_qos_profile = static_qos_profile
        self.clearing_policy = clearing_policy
        if self.clearing_policy == py_trees.common.ClearingPolicy.ON_SUCCESS:
            raise TypeError("ON_SUCCESS is not a valid policy for transforms.ToBlackboard")
        self.buffer = tf2_ros.Buffer()
        # initialise the blackboard
        self.blackboard.set(self.variable_name, None)

    def setup(self, **kwargs):
        """
        Initialises the transform listener.

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            KeyError: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        self.listener = tf2_ros.TransformListener(
            buffer=self.buffer,
            node=self.node,
            # spin_thread=False,
            qos=self.qos_profile,
            static_qos=self.static_qos_profile
        )

    def initialise(self):
        """
        Clear the blackboard variable (set to 'None') if using the
        :attr:`~py_trees.common.ClearingPolicy.ON_INTIALISE` policy.
        """
        if self.clearing_policy == py_trees.common.ClearingPolicy.ON_INITIALISE:
            self.blackboard.set(self.variable_name, None)

    def update(self):
        """
        Checks for the latest transform and posts it to the blackboard
        if available.
        """
        class get_latest(object):
            def __init__(self):
                self.nanoseconds = 0.0

        if self.buffer.can_transform(
            target_frame=self.target_frame,
            source_frame=self.source_frame,
            time=get_latest(),
            # timeout=rclpy.duration.Duration(seconds=5)  # don't block
        ):
            stamped_transform = self.buffer.lookup_transform(
                target_frame=self.target_frame,
                source_frame=self.source_frame,
                time=get_latest(),
                # timeout=rclpy.duration.Duration(seconds=5)  # don't block
            )
            self.blackboard.set(self.variable_name, stamped_transform)
            self.feedback_message = "transform saved to {}".format(self.variable_name)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "waiting for transform".format(
                self.target_frame,
                self.source_frame
            )
            return py_trees.common.Status.RUNNING
