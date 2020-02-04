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
ROS subscribers are asynchronous communication handles whilst py_trees
are by their nature synchronous. They tick, pause, then tick again and
provide an assumption that only one behaviour or function is running at
any single moment. Firing off a subscriber callback in the middle of that
synchronicity to write to a blackboard would break this assumption.

To get around that, subscriber behaviours run the ros callbacks in a
background thread and constrain locking and a local cache inside the behaviour.
Only in the update function is a cached variable unlocked and then
permitted to be used or written to the blackboard.
"""

##############################################################################
# Imports
##############################################################################

import typing

import py_trees
import rclpy.qos

##############################################################################
# Behaviours
##############################################################################


class FromBlackboard(py_trees.behaviour.Behaviour):
    """
    Args:
        topic_name: name of the topic to connect to
        topic_type: class of the message type (e.g. :obj:`std_msgs.msg.String`)
        qos_profile: qos profile for the subscriber
        name: name of the behaviour
        blackboared_variable: name of the variable on the blackboard (can be nested)
    """
    def __init__(self,
                 topic_name: str,
                 topic_type: typing.Any,
                 qos_profile: rclpy.qos.QoSProfile,
                 blackboard_variable: str,
                 name: str=py_trees.common.Name.AUTO_GENERATED,
                 ):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard_variable = blackboard_variable
        self.key = blackboard_variable.split('.')[0]  # in case it is nested
        self.blackboard.register_key(
            key=self.key,
            access=py_trees.common.Access.READ
        )
        self.publisher = None
        self.qos_profile = qos_profile
        self.node = None

    def setup(self, **kwargs):
        """
        Initialises the publisher.

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
        self.publisher = self.node.create_publisher(
            msg_type=self.topic_type,
            topic=self.topic_name,
            qos_profile=self.qos_profile
        )

    def update(self):
        """
        Publish the specified variable from the blackboard.

        Raises:
            TypeError if the blackboard variable is not of the required type

        Returns:
            :data:`~py_trees.common.Status.FAILURE` (variable does not exist on the blackboard) or :data:`~py_trees.common.Status.SUCCESS` (published)
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        try:
            if isinstance(self.blackboard.get(self.blackboard_variable), self.topic_type):
                self.publisher.publish(self.blackboard.get(self.blackboard_variable))
            else:
                raise TypeError("{} is not the required type [{}][{}]".format(
                    self.blackboard_variable,
                    self.topic_type,
                    type(self.blackboard.get(self.blackboard_variable)))
                )
            self.feedback_message = "published"
            return py_trees.common.Status.SUCCESS
        except KeyError:
            self.feedback_message = "nothing to publish"
            return py_trees.common.Status.FAILURE
