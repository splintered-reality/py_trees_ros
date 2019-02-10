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

import os
import PyQt5.QtCore as qt_core
import PyQt5.QtWidgets as qt_widgets
import PyQt5.uic as qt_ui
import threading

##############################################################################
# Helpers
##############################################################################


def resources_directory():
    return os.path.dirname(__file__)


class DashboardGroupBox(qt_widgets.QGroupBox):
    """
    Convenience class that Designer can use to promote
    elements for layouts in applications.
    """
    def __init__(self, parent):
        super(DashboardGroupBox, self).__init__(parent)
        (Ui_DashboardGroupBox, _) = qt_ui.loadUiType(
            os.path.join(resources_directory(), 'dashboard_group_box.ui')
        )
        self.ui = Ui_DashboardGroupBox()
        self.ui.setupUi(self)
        self.stylesheets = {
            "scan_push_button": self.ui.scan_push_button.styleSheet(),
            "cancel_push_button": self.ui.cancel_push_button.styleSheet(),
            "led_strip_label": self.ui.led_strip_label.styleSheet()
        }

        self.led_strip_lock = threading.Lock()
        self.led_strip_flashing = False
        self.led_strip_on_count = 1
        self.led_strip_colour = "grey"
        self.set_led_strip_label_colour(self.led_strip_colour)
        self.led_strip_timer = qt_core.QTimer()
        self.led_strip_timer.timeout.connect(self.led_strip_timer_callback)
        self.led_strip_timer.start(500)  # ms

    def led_strip_timer_callback(self):
        with self.led_strip_lock:
            if self.led_strip_flashing:
                if self.led_strip_on_count > 0:
                    self.led_strip_on_count = 0
                    self.set_led_strip_label_colour("none")
                else:
                    self.led_strip_on_count += 1
                    self.set_led_strip_label_colour(self.led_strip_colour)
            else:  # solid
                self.led_strip_on_count = 1
                self.set_led_strip_label_colour(self.led_strip_colour)

    def set_led_strip_colour(self, colour):
        with self.led_strip_lock:
            self.led_strip_colour = colour
            self.led_strip_flashing = False if self.led_strip_colour == "grey" else True

    def set_cancel_push_button_colour(self, val):
        background_colour = "green" if val else "none"
        self.ui.cancel_push_button.setStyleSheet(
            self.stylesheets["cancel_push_button"] + "\n" +
            "background-color: {}".format(background_colour)
        )

    def set_scan_push_button_colour(self, val):
        print("style: {}".format(self.ui.scan_push_button.styleSheet()))
        background_colour = "green" if val else "none"
        self.ui.scan_push_button.setStyleSheet(
            self.stylesheets["scan_push_button"] + "\n" +
            "background-color: {}".format(background_colour)
        )

    def set_led_strip_label_colour(self, colour):
        # background-color doesn't line up with the qframe panel border
        # border-radius wipes out the qframe styledpanel raised border
        #
        # Q: How to get the background fill colour, to be merely
        #    embedded in the qframe StyledPanel|Raised style?
        #
        # Workaround: just set the text colour
        self.ui.led_strip_label.setStyleSheet(
            self.stylesheets["led_strip_label"] + "\n" +
            "color: {};".format(colour)
        )
