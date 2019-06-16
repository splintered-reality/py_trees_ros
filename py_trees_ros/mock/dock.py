#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks a docking controller
"""


##############################################################################
# Imports
##############################################################################

import py_trees_ros_interfaces.action as py_trees_actions

from . import actions

##############################################################################
# Class
##############################################################################


class Dock(actions.GenericServer):
    """
    Simple server that docks if the goal is true, undocks otherwise.
    """
    def __init__(self, duration=2.0):
        super().__init__(
            node_name="docking_controller",
            action_name="dock",
            action_type=py_trees_actions.Dock,
            generate_feedback_message=self.generate_feedback_message,
            goal_received_callback=self.goal_received_callback,
            duration=duration
        )

    def goal_received_callback(self, goal):
        if goal.dock:
            self.title = "Dock"
        else:
            self.title = "UnDock"

    def generate_feedback_message(self):
        """
        Create some appropriate feedback.
        """
        # TODO: send some feedback message
        msg = py_trees_actions.Dock.Feedback(  # noqa
            percentage_completed=self.percent_completed
        )
        return msg
