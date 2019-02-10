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


class Backend(qt_core.QObject):

    led_colour_changed = qt_core.pyqtSignal(str, name="ledColourChanged")

    def __init__(self, dashboard_group_box):
        super().__init__()

        self.ui = dashboard_group_box
        self.node = rclpy.create_node("dashboard")

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
        unlatched = False
        self.subscribers = py_trees_ros.utilities.Subscribers(
            self.node,
            [
                ("report", "/tree/report", std_msgs.String, latched, self.reality_report_callback),
                ("led_strip", "/led_strip/display", std_msgs.String, unlatched, self.led_strip_display_callback)
            ]
        )

    def spin(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass

    def publish_button_message(self, publisher):
        publisher.publish(std_msgs.Empty())

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
        colour = "grey"
        if not msg.data:
            self.node.get_logger().info("Dashboard: no color specified, setting 'grey'")
        elif msg.data not in ["grey", "blue", "red", "green"]:
            self.node.get_logger().info("Dashboard: received unsupported LED colour {0}, setting 'grey'".format(msg.data))
        else:
            colour = msg.data
        self.led_colour_changed.emit(colour)


##############################################################################
# Main
##############################################################################


def main():
    rclpy.init()  # picks up sys.argv automagically internally
    app = qt_widgets.QApplication(sys.argv)
    main_window = MainWindow()

    backend = Backend(main_window.ui.dashboard_group_box)
    backend.led_colour_changed.connect(
        main_window.ui.dashboard_group_box.set_led_strip_colour
    )

    threading.Thread(target=backend.spin).start()
    main_window.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
