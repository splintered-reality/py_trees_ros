#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks a battery provider.
"""


##############################################################################
# Imports
##############################################################################

import argparse
# import dynamic_reconfigure.server
import py_trees_ros
import rclpy
import sensor_msgs.msg as sensor_msgs
import sys

# from py_trees_msgs.cfg import BatteryConfig

##############################################################################
# Class
##############################################################################


class Battery:
    """
    Mocks the processed battery state for a robot (/battery/sensor_state)
    as well as a possible charging source (/battery/charging_source).

    ROS Publishers:
        * **~state** (:class:`sensor_msgs.msg.BatteryState`)

          * full battery state information

    Dynamic Reconfigure:
        * **~charging_percentage** (:obj:`float`)

          * one-step setting of the current battery percentage
        * **~charging** (:obj:`bool`)

          * whether it is currently charging or not
        * **~charging_increment** (:obj:`float`)

          * how fast it charges/discharges

    On startup it is in a DISCHARGING state. Use ``rqt_reconfigure`` to change the battery state.
    """
    def __init__(self):
        # ros communications
        self.node = rclpy.create_node("battery")
        self.battery_publisher = self.node.create_publisher(
            msg_type=sensor_msgs.BatteryState,
            topic="~/state",
            qos_profile=py_trees_ros.utilities.qos_profile_latched_topic()
        )

        # initialisations
        self.battery = sensor_msgs.BatteryState()
        self.battery.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.battery.voltage = float('nan')
        self.battery.current = float('nan')
        self.battery.charge = float('nan')
        self.battery.capacity = float('nan')
        self.battery.design_capacity = float('nan')
        self.battery.percentage = 100.0
        self.battery.power_supply_health = sensor_msgs.BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self.battery.power_supply_technology = sensor_msgs.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
        self.battery.present = True
        self.battery.location = ""
        self.battery.serial_number = ""

        # immediately reconfigured by dynamic_reconfigure
        self.charging_increment = 0.01

        # dynamic_reconfigure
#         self.parameters = None
#         self.dynamic_reconfigure_server = dynamic_reconfigure.server.Server(
#             BatteryConfig,
#             self.dynamic_reconfigure_callback
#         )

    def dynamic_reconfigure_callback(self, config, level):
        """
        Don't use the incoming config except as a read only
        tool. If we write variables, make sure to use
        the server's update_configuration, which will trigger
        this callback here.

        Args:
            config (:obj:`dynamic_reconfigure.encoding.Config`): incoming configuration
            level (:obj:`int`):
        """
        self.parameters = config
        self.battery.percentage = self.parameters.charging_percentage
        self.charging_increment = self.parameters.charging_increment
        if self.battery.percentage >= 100:
            self.battery.percentage = 100
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
        elif self.parameters.charging:
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        return config

    def spin(self):
        """
        Spin around, updating battery state and publishing the result.
        """
        # TODO: with rate and spin_once, once rate is implemented in rclpy
        unused_timer = self.node.create_timer(
            timer_period_sec=0.2,
            callback=self.publish
        )
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()

    def publish(self):
        """
        Update and publish.
        """
#        if self.parameters.charging:
#            self.dynamic_reconfigure_server.update_configuration({"charging_percentage": min(100, self.battery.percentage + self.charging_increment)})
#        else:
#            self.dynamic_reconfigure_server.update_configuration({"charging_percentage": max(0, self.battery.percentage - self.charging_increment)})
        self.battery.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.battery_publisher.publish(msg=self.battery)


def main():
    """
    Entry point for the mock batttery node.
    """
    parser = argparse.ArgumentParser(description='Mock a battery/charging source')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    battery = Battery()
    battery.spin()
    rclpy.shutdown()
