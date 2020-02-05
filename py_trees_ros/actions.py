#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.github.com/splintered-reality/py_trees_ros/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Behaviours for ROS actions.
"""

##############################################################################
# Imports
##############################################################################

from . import action_clients

##############################################################################
# Behaviours
##############################################################################


# to be deprecated in 2.1.x or later
ActionClient = action_clients.FromConstant
