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
Getting the most out of your battery.
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import rclpy.qos
import sensor_msgs.msg as sensor_msgs

from . import subscribers

##############################################################################
# Behaviours
##############################################################################


class ToBlackboard(subscribers.ToBlackboard):
    """
    Subscribes to the battery message and writes battery data to the blackboard.
    Also adds a warning flag to the blackboard if the battery
    is low - note that it does some buffering against ping-pong problems so the warning
    doesn't trigger on/off rapidly when close to the threshold.

    When ticking, updates with :attr:`~py_trees.common.Status.RUNNING` if it got no data,
    :attr:`~py_trees.common.Status.SUCCESS` otherwise.

    Blackboard Variables:
        * battery (:class:`sensor_msgs.msg.BatteryState`)[w]: the raw battery message
        * battery_low_warning (:obj:`bool`)[w]: False if battery is ok, True if critically low

    Args:
        topic_name: name of the battery state topic
        qos_profile: qos profile for the subscriber
        name: name of the behaviour
        threshold: percentage level threshold for flagging as low (0-100)
    """
    def __init__(self,
                 topic_name: str,
                 qos_profile: rclpy.qos.QoSProfile,
                 name: str=py_trees.common.Name.AUTO_GENERATED,
                 threshold: float=30.0):
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=sensor_msgs.BatteryState,
                         qos_profile=qos_profile,
                         blackboard_variables={"battery": None},
                         clearing_policy=py_trees.common.ClearingPolicy.NEVER
                         )
        self.blackboard.register_key(
            key="battery_low_warning",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.battery = sensor_msgs.BatteryState()
        self.blackboard.battery.percentage = 0.0
        self.blackboard.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_UNKNOWN  # noqa
        self.blackboard.battery_low_warning = False   # decision making
        self.threshold = threshold

    def update(self) -> py_trees.common.Status:
        """
        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.

        Returns:
            :attr:`~py_trees.common.Status.SUCCESS` if a message was written, :attr:`~py_trees.common.Status.RUNNING` otherwise.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            # we got something
            if self.blackboard.battery.percentage > self.threshold + 5.0:
                self.blackboard.battery_low_warning = False
            elif self.blackboard.battery.percentage < self.threshold:
                    self.blackboard.battery_low_warning = True
                    # TODO: make this throttled
                    self.node.get_logger().error("{}: battery level is low!".format(self.name))
            # else don't do anything in between - i.e. avoid the ping pong problems

            self.feedback_message = "Battery level is low" if self.blackboard.battery_low_warning else "Battery level is ok"
        return status
