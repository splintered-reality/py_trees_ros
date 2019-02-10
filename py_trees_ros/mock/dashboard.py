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

# To use generated files instead of loading ui's directly
from . import gui

##############################################################################
# Helpers
##############################################################################


def resources_directory():
    return os.path.join(os.path.dirname(__file__), 'gui')


class MainWindow(qt_widgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # Use generated files - bugfree when using promotions & resources
        self.ui = gui.main_window.Ui_MainWindow()

        # Use ui files directly - pyqt5 has bugs for promotions & resources
#         (Ui_MainWindow, _) = qt_ui.loadUiType(
#             os.path.join(resources_directory(), 'main_window.ui'),
#             from_imports=True  # make sure to use from . import <my_resource_file>
#         )
#         self.ui = Ui_MainWindow()

        self.ui.setupUi(self)


class Dashboard(object):

    def __init__(self):
        super().__init__()
        (Ui_DashboardGroupBox, _) = qt_ui.loadUiType(
            os.path.join(resources_directory(), 'dashboard.ui')
        )
        self.ui = Ui_DashboardGroupBox()
        self.ui.setupUi(self)

        self.node = rclpy.create_node("dashboard")
        print("Dict: %s" % super().__dict__)

        not_latched = False  # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            self.node,
            [
                ('scan', "~/scan", std_msgs.Empty, not_latched),
                ('cancel', "~/cancel", std_msgs.Empty, not_latched),
            ]
        )

        self.scan_push_button_stylesheet = self.ui.scan_push_button.styleSheet()
        self.ui.scan_push_button.pressed.connect(
            functools.partial(
                self.publish_button_message,
                self.publishers.scan)
        )

        self.cancel_push_button_stylesheet = self.ui.cancel_push_button.styleSheet()
        self.ui.cancel_push_button.pressed.connect(
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
                    self.node.get_logger().info("Dashboard: received unknown LED colour {0}, setting 'grey'".format(msg.data))
                    self.led_strip_colour = "grey"
                    self.led_strip_flashing = False

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
    # rclpy.init()  # picks up sys.argv automagically internally
    app = qt_widgets.QApplication(sys.argv)
    main_window = MainWindow()
    # reconfigure_group_box = qt_ui.loadUi(os.path.join(resources_directory(), 'reconfigure.ui'))
    # dashboard = Dashboard()
    # threading.Thread(target=dashboard.spin).start()
    main_window.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
