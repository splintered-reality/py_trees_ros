#!/usr/bin/env python
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

import dynamic_reconfigure.server
import rospy
import sensor_msgs.msg as sensor_msgs

from py_trees_msgs.cfg import BatteryConfig

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
        self.battery_publisher = rospy.Publisher('~state', sensor_msgs.BatteryState, queue_size=1, latch=True)

        # initialisations
        self.battery = sensor_msgs.BatteryState()
        self.battery.header.stamp = rospy.Time.now()
        self.battery.voltage = float('nan')
        self.battery.current = float('nan')
        self.battery.charge = float('nan')
        self.battery.capacity = float('nan')
        self.battery.design_capacity = float('nan')
        self.battery.percentage = 100
        self.battery.power_supply_health = sensor_msgs.BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self.battery.power_supply_technology = sensor_msgs.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
        self.battery.present = True
        self.battery.location = ""
        self.battery.serial_number = ""

        # immediately reconfigured by dynamic_reconfigure
        self.charging_increment = 0.01

        # dynamic_reconfigure
        self.parameters = None
        self.dynamic_reconfigure_server = dynamic_reconfigure.server.Server(
            BatteryConfig,
            self.dynamic_reconfigure_callback
        )

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
        rate = rospy.Rate(5)  # hz
        while not rospy.is_shutdown():
            if self.parameters.charging:
                self.dynamic_reconfigure_server.update_configuration({"charging_percentage": min(100, self.battery.percentage + self.charging_increment)})
            else:
                self.dynamic_reconfigure_server.update_configuration({"charging_percentage": max(0, self.battery.percentage - self.charging_increment)})
            self.battery.header.stamp = rospy.get_rostime()  # get_rostime() returns the time in rospy.Time structure
            self.battery_publisher.publish(self.battery)
            rate.sleep()
