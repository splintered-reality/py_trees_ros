#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Qt support for the tutorials.
"""

##############################################################################
# Imports
##############################################################################

import functools
import py_trees_ros
import rospy
import std_msgs.msg as std_msgs
import threading

from python_qt_binding.QtCore import Signal, Qt, QTimer, Slot
from python_qt_binding.QtWidgets import QWidget, QPushButton, QGridLayout, QSizePolicy, QLabel

##############################################################################
# Dashboard
##############################################################################


class Dashboard(QWidget):

    _go_button_led = Signal(bool)
    _stop_button_led = Signal(bool)

    def __init__(self):
        super(Dashboard, self).__init__()

        not_latched = False
        # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            [
                ('go', "~go", std_msgs.Empty, not_latched, 1),
                ('scan', "~scan", std_msgs.Empty, not_latched, 1),
                ('abort', "~abort", std_msgs.Empty, not_latched, 1),
            ]
        )

        self.go_push_button = QPushButton("Go")
        self.go_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")
        self.go_push_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.go_push_button.pressed.connect(functools.partial(self.publish_button_message, self.publishers.go))

        self.scan_push_button = QPushButton("Scan")
        self.scan_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")
        self.scan_push_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.scan_push_button.pressed.connect(functools.partial(self.publish_button_message, self.publishers.scan))

        self.abort_push_button = QPushButton("Abort")
        self.abort_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")
        self.abort_push_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.abort_push_button.pressed.connect(functools.partial(self.publish_button_message, self.publishers.abort))

        self.led_strip_flashing = True
        self.led_strip_on_count = 1
        self.led_strip_colour = "blue"

        self.led_strip_lock = threading.Lock()
        self.led_strip_timer = QTimer()
        self.led_strip_timer.timeout.connect(self.led_strip_timer_callback)
        self.led_strip_label = QLabel("LED Strip")
        self.led_strip_label.setAlignment(Qt.AlignCenter)
        self.led_strip_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.led_strip_label.setStyleSheet("background-color: %s; font-size: 30pt;" % self.led_strip_colour)

        self.hbox_layout = QGridLayout(self)
        self.hbox_layout.addWidget(self.go_push_button)
        self.hbox_layout.addWidget(self.scan_push_button)
        self.hbox_layout.addWidget(self.abort_push_button)
        self.hbox_layout.addWidget(self.led_strip_label)

        self.subscribers = py_trees_ros.utilities.Subscribers(
            [
                ("led_strip", "/led_strip/display", std_msgs.String, self.led_strip_display_callback)
            ]
        )
        self.led_strip_timer.start(500)  # ms

    def publish_button_message(self, publisher):
        publisher.publish(std_msgs.Empty())

    def led_strip_display_callback(self, msg):
        with self.led_strip_lock:
            if not msg.data:
                self.led_strip_colour = "grey"
                self.led_strip_flashing = False
            else:
                self.led_strip_flashing = True
                self.led_strip_colour = None
                for colour in ["blue", "red", "green"]:
                    if colour in msg.data:
                        self.led_strip_colour = colour
                        break
                if not self.led_strip_colour:
                    self.led_strip_colour = "pink"
                    rospy.loginfo("Dashboard: received unknown LED colour {0}, setting 'pink'".format(msg.data))

    @Slot()
    def led_strip_timer_callback(self):
        with self.led_strip_lock:
            if self.led_strip_flashing:
                if self.led_strip_on_count > 0:
                    self.led_strip_on_count = 0
                    self.led_strip_label.setStyleSheet("background-color: none; font-size: 30pt;")
                else:
                    self.led_strip_on_count += 1
                    self.led_strip_label.setStyleSheet("background-color: %s; font-size: 30pt;" % self.led_strip_colour)
            else:  # solid
                self.led_strip_on_count = 1
                self.led_strip_label.setStyleSheet("background-color: %s; font-size: 30pt;" % self.led_strip_colour)
