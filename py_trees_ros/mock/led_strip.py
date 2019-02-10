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
Mock a hardware LED strip.
"""


##############################################################################
# Imports
##############################################################################

import argparse
import functools
import math
import py_trees.console as console
import py_trees_ros
import rclpy
import std_msgs.msg as std_msgs
import sys
import threading
import uuid

##############################################################################
# Class
##############################################################################


class LEDStrip(object):
    """
    Emulates command/display of an led strip so that it flashes various colours.

    ROS Publishers:
        * **~display** (:class:`std_msgs.msg.String`)

          * colourised string display of the current led strip state

    ROS Subscribers:
        * **~command** (:class:`std_msgs.msg.String`)

          * send it a colour to express, it will flash this for the next 3 seconds
    """
    _pattern = '*'
    _pattern_width = 60  # total width of the pattern to be output
    _pattern_name_spacing = 4  # space between pattern and the name of the pattern

    def __init__(self):
        self.node = rclpy.create_node("led_strip")
        self.command_subscriber = self.node.create_subscription(
            msg_type=std_msgs.String,
            topic='~/command',
            callback=self.command_callback,
            )
        self.display_publisher = self.node.create_publisher(
            msg_type=std_msgs.String,
            topic="~/display",
            qos_profile=py_trees_ros.utilities.qos_profile_latched_topic()
        )
        self.duration_sec = 3.0
        self.last_text = ''
        self.last_uuid = None
        self.lock = threading.Lock()
        self.flashing_timer = None

    def get_display_string(self, width, label="Foo"):
        """Get a string used to display the current state of the base leds

        :param width: The width of the pattern
        :param label: if set, display this in the centre of the pattern rather
            than the pattern name

        """
        # top and bottom of print repeats the pattern as many times as possible
        # in the space specified
        top_bottom = LEDStrip._pattern * int(width / len(LEDStrip._pattern))
        # space for two halves of the pattern on either side of the pattern name
        mid_pattern_space = (width - len(label) - self._pattern_name_spacing * 2) / 2

        # pattern for the mid line
        mid = LEDStrip._pattern * int(mid_pattern_space / len(LEDStrip._pattern))

        # total length of the middle line with pattern, spacing and name
        mid_len = len(mid) * 2 + self._pattern_name_spacing * 2 + len(label)

        # patterns won't necessarily match up with the width, so need to deal
        # with extra space. Odd numbers of extra space handled by putting more
        # spaces on the right side
        extra_space = width - mid_len
        extra_left_space = int(math.floor(extra_space / 2.0))
        extra_right_space = int(math.ceil(extra_space / 2.0))

        # left and right parts of the mid line to go around the name
        left = mid + ' ' * (self._pattern_name_spacing + extra_left_space)
        right = ' ' * (self._pattern_name_spacing + extra_right_space) + mid

        return '\n' + top_bottom + '\n' + left + label.replace('_', ' ') + right + '\n' + top_bottom

    def generate_led_text(self, colour):
        if not colour:
            return ""
        else:
            text = self.get_display_string(self._pattern_width, label=colour)

            # map colour names in message to console colour escape sequences
            console_colour_map = {
                'grey': console.dim + console.white,
                'red': console.red,
                'green': console.green,
                'yellow': console.yellow,
                'blue': console.blue,
                'purple': console.magenta,
                'white': console.white
            }

            coloured_text = console_colour_map[colour] + console.blink + text + console.reset
            return coloured_text

    def command_callback(self, msg):
        with self.lock:
            text = self.generate_led_text(msg.data)
            # don't bother publishing if nothing changed.
            if self.last_text != text:
                print("{}".format(text))
                self.last_text = text
                self.last_uuid = uuid.uuid4()
                self.display_publisher.publish(std_msgs.String(data=msg.data))
            if self.flashing_timer is not None:
                self.flashing_timer.cancel()
                self.node.destroy_timer(self.flashing_timer)
            # TODO: convert this to a one-shot once rclpy has the capability
            # Without oneshot, it will keep triggering, but do nothing while
            # it has the uuid check
            self.flashing_timer = self.node.create_timer(
                timer_period_sec=self.duration_sec,
                callback=functools.partial(
                    self.cancel_flashing,
                    last_uuid=self.last_uuid
                )
            )

    def cancel_flashing(self, last_uuid):
        with self.lock:
            if self.last_uuid == last_uuid:
                # We're still relevant, publish and make us irrelevant
                self.display_publisher.publish(std_msgs.String(data=""))
                self.last_text = ""
                self.last_uuid = uuid.uuid4()

    def spin(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()


def main():
    """
    Entry point for the mock led strip.
    """
    parser = argparse.ArgumentParser(description='Mock an led strip')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init(args=sys.argv)
    led_strip = LEDStrip()
    led_strip.spin()
    rclpy.shutdown()
