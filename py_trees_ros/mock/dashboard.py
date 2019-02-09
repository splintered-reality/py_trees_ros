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
import os
import py_trees_ros
import rclpy
import signal
import std_msgs.msg as std_msgs
import sys
import threading

import PyQt5.QtWidgets as qt_widgets
import PyQt5.QtCore as qt_core
import PyQt5.uic as qt_ui

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

    def __init__(self, ui):
        super().__init__()
        self.ui = ui
        self.node = rclpy.create_node("dashboard")

        not_latched = False  # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            self.node,
            [
                ('scan', "~/scan", std_msgs.Empty, not_latched),
                ('cancel', "~/cancel", std_msgs.Empty, not_latched),
            ]
        )

        self.scan_push_button_stylesheet = self.ui.scan_push_button.styleSheet()
        ui.scan_push_button.pressed.connect(
            functools.partial(
                self.publish_button_message,
                self.publishers.scan)
        )

        self.cancel_push_button_stylesheet = self.ui.cancel_push_button.styleSheet()
        ui.cancel_push_button.pressed.connect(
            functools.partial(
                self.publish_button_message,
                self.publishers.cancel)
        )

        self.led_strip_flashing = False
        self.led_strip_on_count = 1
        self.led_strip_colour = "grey"
        self.led_strip_stylesheet = self.ui.led_strip_label.styleSheet()

        self.led_strip_lock = threading.Lock()
        self.led_strip_timer = qt_core.QTimer()
        self.led_strip_timer.timeout.connect(self.led_strip_timer_callback)
        self.set_led_strip_colour(self.led_strip_colour)

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
            self.ui.cancel_push_button.setEnabled(True)
        elif msg.data == "scanning":
            self.set_scanning_colour(True)
            self.set_cancelling_colour(False)
            self.ui.cancel_push_button.setEnabled(True)
        else:
            self.set_scanning_colour(False)
            self.set_cancelling_colour(False)
            self.ui.cancel_push_button.setEnabled(False)

    def set_cancelling_colour(self, val):
        print("style: {}".format(self.ui.cancel_push_button.styleSheet()))
        background_colour = "green" if val else "none"
        self.ui.cancel_push_button.setStyleSheet(
            self.cancel_push_button_stylesheet + "\n" +
            "background-color: {}".format(background_colour)
        )

    def set_scanning_colour(self, val):
        print("style: {}".format(self.ui.scan_push_button.styleSheet()))
        background_colour = "green" if val else "none"
        self.ui.scan_push_button.setStyleSheet(
            self.scan_push_button_stylesheet + "\n" +
            "background-color: {}".format(background_colour)
        )

    def led_strip_display_callback(self, msg):
        print("Got callback")
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
                    self.set_led_strip_colour("none")
                else:
                    self.led_strip_on_count += 1
                    self.set_led_strip_colour(self.led_strip_colour)
            else:  # solid
                self.led_strip_on_count = 1
                self.set_led_strip_colour(self.led_strip_colour)

    def set_led_strip_colour(self, colour):
        # background-color doesn't line up with the qframe panel border
        # border-radius wipes out the qframe styledpanel raised border
        #
        # Q: How to get the background fill colour, to be merely
        #    embedded in the qframe StyledPanel|Raised style?
        #
        # Workaround: just set the text colour
        self.ui.led_strip_label.setStyleSheet(
            self.led_strip_stylesheet + "\n" +
            "color: {};".format(colour)
        )

##############################################################################
# Main
##############################################################################


def main():
    rclpy.init()  # picks up sys.argv automagically internally
    app = qt_widgets.QApplication(sys.argv)
    resources_directory = os.path.join(os.path.dirname(__file__), '..', 'resources')
    main_window = qt_ui.loadUi(os.path.join(resources_directory, 'main_window.ui'))
    dashboard_group_box = qt_ui.loadUi(os.path.join(resources_directory, 'dashboard.ui'))
    reconfigure_group_box = qt_ui.loadUi(os.path.join(resources_directory, 'reconfigure.ui'))
    main_window.central_layout.addWidget(dashboard_group_box)
    main_window.central_layout.addWidget(reconfigure_group_box)
    dashboard = Dashboard(dashboard_group_box)
    threading.Thread(target=dashboard.spin).start()
    main_window.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
