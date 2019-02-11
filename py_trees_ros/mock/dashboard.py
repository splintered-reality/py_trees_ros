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
# import PyQt5.uic as qt_ui

# To use generated files instead of loading ui's directly
from . import gui

##############################################################################
# Helpers
##############################################################################


def resources_directory():
    return os.path.join(os.path.dirname(__file__), 'gui')


class MainWindow(qt_widgets.QMainWindow):

    request_shutdown = qt_core.pyqtSignal(name="requestShutdown")

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

    def closeEvent(self, unused_event):
        self.request_shutdown.emit()


class Backend(qt_core.QObject):

    led_colour_changed = qt_core.pyqtSignal(str, name="ledColourChanged")

    def __init__(self, dashboard_group_box):
        super().__init__()

        self.ui = dashboard_group_box
        self.node = rclpy.create_node("dashboard")

        self.shutdown_requested = False

        not_latched = False  # latched = True
        self.publishers = py_trees_ros.utilities.Publishers(
            self.node,
            [
                ('scan', "~/scan", std_msgs.Empty, not_latched),
                ('cancel', "~/cancel", std_msgs.Empty, not_latched),
            ]
        )

        self.ui.ui.scan_push_button.pressed.connect(
            functools.partial(
                self.publish_button_message,
                self.publishers.scan)
        )

        self.ui.ui.cancel_push_button.pressed.connect(
            functools.partial(
                self.publish_button_message,
                self.publishers.cancel)
        )

        latched = True
        # unlatched = False
        self.subscribers = py_trees_ros.utilities.Subscribers(
            self.node,
            [
                ("report", "/tree/report", std_msgs.String, latched, self.reality_report_callback),
                ("led_strip", "/led_strip/display", std_msgs.String, latched, self.led_strip_display_callback)
            ]
        )

    def spin(self):
        try:
            while rclpy.ok() and not self.shutdown_requested:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()

    def publish_button_message(self, publisher):
        publisher.publish(std_msgs.Empty())

    def shutdown_requested_callback(self):
        self.shutdown_requested = True

    # TODO: shift to the ui
    def reality_report_callback(self, msg):
        if msg.data == "cancelling":
            self.set_scanning_colour(False)
            self.ui.set_cancel_push_button_colour(True)
            self.ui.ui.cancel_push_button.setEnabled(True)
        elif msg.data == "scanning":
            self.set_scanning_colour(True)
            self.ui.set_cancel_push_button_colour(False)
            self.ui.ui.cancel_push_button.setEnabled(True)
        else:
            self.ui.set_scan_push_button_colour(False)
            self.ui.set_cancel_push_button_colour(False)
            self.ui.ui.cancel_push_button.setEnabled(False)

    def led_strip_display_callback(self, msg):
        print("Got callback")
        colour = "grey"
        if not msg.data:
            self.node.get_logger().info("Dashboard: no color specified, setting '{}'".format(colour))
        elif msg.data not in ["grey", "blue", "red", "green"]:
            self.node.get_logger().info("Dashboard: received unsupported LED colour '{0}', setting '{1}'".format(msg.data, colour))
        else:
            colour = msg.data
        self.led_colour_changed.emit(colour)


##############################################################################
# Main
##############################################################################


def main():
    # picks up sys.argv automagically internally
    rclpy.init()
    # enable handling of ctrl-c (from roslaunch as well)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # the players
    app = qt_widgets.QApplication(sys.argv)
    main_window = MainWindow()
    backend = Backend(main_window.ui.dashboard_group_box)

    # sigslots
    backend.led_colour_changed.connect(
        main_window.ui.dashboard_group_box.set_led_strip_colour
    )
    main_window.request_shutdown.connect(
        backend.shutdown_requested_callback
    )

    # qt ... up
    ros_thread = threading.Thread(target=backend.spin)
    ros_thread.start()
    main_window.show()
    result = app.exec_()

    # shutdown
    ros_thread.join()
    rclpy.shutdown()
    sys.exit(result)
