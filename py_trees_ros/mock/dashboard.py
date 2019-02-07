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
Launch a qt dashboard for the tutorials.
"""
##############################################################################
# Imports
##############################################################################

import functools
import py_trees_ros
import rclpy
import signal
import std_msgs.msg as std_msgs
import sys
import threading

import PyQt5.QtWidgets as qt_widgets
import PyQt5.QtCore as qt_core

##############################################################################
# Main
##############################################################################


class Dash(object):
    def __init__(self):
        self.node = rclpy.create_node("dashboard")

        not_latched = False  # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            self.node,
            [
                ('scan', "~/scan", std_msgs.Empty, not_latched),
                ('cancel', "~/cancel", std_msgs.Empty, not_latched),
            ]
        )

    def spin(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()


class Dashboard(qt_widgets.QWidget):

    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node("dashboard")

        not_latched = False  # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            self.node,
            [
                ('scan', "~/scan", std_msgs.Empty, not_latched),
                ('cancel', "~/cancel", std_msgs.Empty, not_latched),
            ]
        )

        self.scan_push_button = qt_widgets.QPushButton("Scan")
        self.scan_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")
        self.scan_push_button.setSizePolicy(
            qt_widgets.QSizePolicy.Expanding,
            qt_widgets.QSizePolicy.Expanding
        )
        self.scan_push_button.pressed.connect(
            functools.partial(
                self.publish_button_message,
                self.publishers.scan)
        )

        self.cancel_push_button = qt_widgets.QPushButton("Cancel")
        self.cancel_push_button.setStyleSheet("QPushButton { font-size: 30pt; }")
        self.cancel_push_button.setSizePolicy(
            qt_widgets.QSizePolicy.Expanding,
            qt_widgets.QSizePolicy.Expanding
        )
        self.cancel_push_button.pressed.connect(
            functools.partial(
                self.publish_button_message,
                self.publishers.cancel)
        )

        self.led_strip_flashing = False
        self.led_strip_on_count = 1
        self.led_strip_colour = "grey"

        self.led_strip_lock = threading.Lock()
        self.led_strip_timer = qt_core.QTimer()
        self.led_strip_timer.timeout.connect(self.led_strip_timer_callback)
        self.led_strip_label = qt_widgets.QLabel("LED Strip")
        self.led_strip_label.setAlignment(qt_core.Qt.AlignCenter)
        self.led_strip_label.setSizePolicy(
            qt_widgets.QSizePolicy.Expanding,
            qt_widgets.QSizePolicy.Expanding)
        self.led_strip_label.setStyleSheet(
            "background-color: {}; font-size: 30pt;".format(self.led_strip_colour)
        )

        self.hbox_layout = qt_widgets.QGridLayout(self)
        self.hbox_layout.addWidget(self.scan_push_button)
        self.hbox_layout.addWidget(self.cancel_push_button)
        self.hbox_layout.addWidget(self.led_strip_label)

        latched = True
        unlatched = False
        self.subscribers = py_trees_ros.utilities.Subscribers(
            self.node,
            [
                ("report", "/tree/report", std_msgs.String, latched, self.reality_report_callback),
                ("led_strip", "/led_strip/display", std_msgs.String, unlatched, self.led_strip_display_callback)
            ]
        )
        self.led_strip_timer.start(500)  # ms

    def spin(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass

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
                    self.node.get_logger().info("Dashboard: received unknown LED colour {0}, setting 'pink'".format(msg.data))

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

##############################################################################
# Main
##############################################################################


def main():
    rclpy.init()  # picks up sys.argv automagically internally
    app = qt_widgets.QApplication(sys.argv)
    window = qt_widgets.QMainWindow()
    dashboard = Dashboard()
    threading.Thread(target=dashboard.spin).start()
    window.setCentralWidget(dashboard)
    window.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
