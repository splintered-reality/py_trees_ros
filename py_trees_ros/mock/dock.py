#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks a docking controller.
"""

##############################################################################
# Imports
##############################################################################

import py_trees_msgs.msg as py_trees_msgs

from . import action_server

##############################################################################
# Classes
##############################################################################


class Dock(action_server.ActionServer):
    """
    Simple server that docks if the goal is true, undocks otherwise.
    """
    def __init__(self):
        super(Dock, self).__init__(action_name="dock",
                                   action_type=py_trees_msgs.DockAction,
                                   worker=self.worker,
                                   goal_received_callback=self.goal_received_callback,
                                   duration=2.0
                                   )
        self.start()

    def goal_received_callback(self, goal):
        if goal.dock:
            self.title = "Dock"
        else:
            self.title = "UnDock"

    def worker(self):
        """
        Create some appropriate feedback.
        """
        self.action.action_feedback = py_trees_msgs.DockFeedback(self.percent_completed)
