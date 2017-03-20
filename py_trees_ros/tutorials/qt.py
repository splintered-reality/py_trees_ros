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

    _scan_button_led = Signal(bool)
    _cancel_button_led = Signal(bool)

    def __init__(self):
        super(Dashboard, self).__init__()

        not_latched = False
        # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            [
                ('scan', "~scan", std_msgs.Empty, not_latched, 1),
                ('cancel', "~cancel", std_msgs.Empty, not_latched, 1),
            ]
        )

        self.scan_push_button = QPushButton("Scan")
        self.scan_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")
        self.scan_push_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.scan_push_button.pressed.connect(functools.partial(self.publish_button_message, self.publishers.scan))

        self.cancel_push_button = QPushButton("Cancel")
        self.cancel_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")
        self.cancel_push_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.cancel_push_button.pressed.connect(functools.partial(self.publish_button_message, self.publishers.cancel))

        self.led_strip_flashing = False
        self.led_strip_on_count = 1
        self.led_strip_colour = "grey"

        self.led_strip_lock = threading.Lock()
        self.led_strip_timer = QTimer()
        self.led_strip_timer.timeout.connect(self.led_strip_timer_callback)
        self.led_strip_label = QLabel("LED Strip")
        self.led_strip_label.setAlignment(Qt.AlignCenter)
        self.led_strip_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.led_strip_label.setStyleSheet("background-color: %s; font-size: 30pt;" % self.led_strip_colour)

        self.hbox_layout = QGridLayout(self)
        self.hbox_layout.addWidget(self.scan_push_button)
        self.hbox_layout.addWidget(self.cancel_push_button)
        self.hbox_layout.addWidget(self.led_strip_label)

        self.subscribers = py_trees_ros.utilities.Subscribers(
            [
                ("report", "/tree/report", std_msgs.String, self.reality_report_callback),
                ("led_strip", "/led_strip/display", std_msgs.String, self.led_strip_display_callback)
            ]
        )
        self.led_strip_timer.start(500)  # ms

    def publish_button_message(self, publisher):
        publisher.publish(std_msgs.Empty())

    def reality_report_callback(self, msg):
        if msg.data == "cancelling":
            self.set_scanning_colour(False)
            self.set_cancelling_colour(True)
            self.cancel_push_button.setEnabled(True)
        elif msg.data == "scanning":
            self.set_scanning_colour(True)
            self.set_cancelling_colour(False)
            self.cancel_push_button.setEnabled(True)
        else:
            self.set_scanning_colour(False)
            self.set_cancelling_colour(False)
            self.cancel_push_button.setEnabled(False)

    def set_cancelling_colour(self, val):
        if val:
            self.cancel_push_button.setStyleSheet("QPushButton { font-size: 30pt; background-color: red}")
        else:
            self.cancel_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")

    def set_scanning_colour(self, val):
        if val:
            self.scan_push_button.setStyleSheet("QPushButton { font-size: 30pt; background-color: green}")
        else:
            self.scan_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")

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
