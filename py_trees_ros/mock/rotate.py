#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks a simple action server that rotates the robot 360 degrees.
"""

##############################################################################
# Imports
##############################################################################

import math
import py_trees_msgs.msg as py_trees_msgs

from . import action_server

##############################################################################
# Classes
##############################################################################


class Rotate(action_server.ActionServer):
    """
    Simple server that controls a full rotation of the robot.

    Args:
        odometry_topic (:obj:`str`): name of the odometry topic
        pose_topic (:obj:`str`): name of the pose (with covariance stamped) topic
        duration (:obj:`int`): time for a goal to complete (seconds)
    """
    def __init__(self, rotation_rate=0.87):
        super(Rotate, self).__init__(action_name="rotate",
                                     action_type=py_trees_msgs.RotateAction,
                                     worker=self.worker,
                                     duration=2.0 * math.pi / rotation_rate
                                     )
        self.start()

    def worker(self):
        """
        Create some appropriate feedback.
        """
        self.action.action_feedback = 2 * math.pi * self.percent_completed / 100.0
