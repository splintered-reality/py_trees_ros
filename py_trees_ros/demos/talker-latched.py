#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
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
import signal
import sys

import PyQt5.QtWidgets as qt_widgets
import PyQt5.uic as qt_ui

# To use generated files instead of loading ui's directly
# import py_trees_ros.gui.gen_main_window

##############################################################################
# Helpers
##############################################################################


def resources_directory():
    return os.path.join(os.path.dirname(__file__), '..', 'gui')


class MainWindow(qt_widgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # Use generated files
        # self.ui = py_trees_ros.gui.gen_main_window.Ui_MainWindow()

        # Use ui files directly
        (Ui_MainWindow, _) = qt_ui.loadUiType(
            os.path.join(resources_directory(), 'main_window.ui')
        )
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)


##############################################################################
# Main
##############################################################################


def main():
    # rclpy.init()  # picks up sys.argv automagically internally
    app = qt_widgets.QApplication(sys.argv)
#    main_window = qt_ui.loadUi(os.path.join(resources_directory(), 'main_window.ui'))
    main_window = MainWindow()
    # reconfigure_group_box = qt_ui.loadUi(os.path.join(resources_directory(), 'reconfigure.ui'))
    # dashboard = Dashboard()
    # main_window.central_layout.addWidget(dashboard)
    # main_window.central_layout.addWidget(reconfigure_group_box)
    # threading.Thread(target=dashboard.spin).start()
    main_window.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
