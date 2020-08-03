#!/usr/bin/env python
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

import rospy
import math
import std_msgs.msg as std_msgs
import termcolor
import threading

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
    # map colour names in message to termcolor names
    _valid_colours = {'red': 'red', 'green': 'green', 'yellow': 'yellow', 'blue': 'blue', 'purple': 'magenta', 'white': 'white'}

    def __init__(self):
        self.command_subscriber = rospy.Subscriber('~command', std_msgs.String, self.command_callback)
        self.display_publisher = rospy.Publisher('~display', std_msgs.String, queue_size=1, latch=True)
        self.duration = rospy.Duration(3.0)
        self.last_text = ''
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
            return termcolor.colored(text,
                                     colour,
                                     attrs=['blink']
                                     )

    def command_callback(self, msg):
        with self.lock:
            text = self.generate_led_text(msg.data)
            # don't bother publishing if nothing changed.
            if self.last_text != text:
                self.display_publisher.publish(std_msgs.String(text))
            if self.flashing_timer is not None:
                self.flashing_timer.shutdown()
            self.flashing_timer = rospy.Timer(period=self.duration, callback=self.cancel_flashing, oneshot=True)
            self.last_text = text

    def cancel_flashing(self, unused_event):
        with self.lock:
            self.flashing_timer = None
            self.display_publisher.publish(std_msgs.String(""))
            self.last_text = ""

    def spin(self):
        rospy.spin()
