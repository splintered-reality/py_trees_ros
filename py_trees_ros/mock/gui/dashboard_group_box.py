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
import PyQt5.QtWidgets as qt_widgets
import PyQt5.uic as qt_ui

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
