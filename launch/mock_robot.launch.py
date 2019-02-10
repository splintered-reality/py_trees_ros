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
Launch the mock robot.
"""
##############################################################################
# Imports
##############################################################################

import launch
import launch_ros.actions

##############################################################################
# Helpers
##############################################################################


##############################################################################
# Main
##############################################################################

def generate_launch_description():
    """Launch the mock robot."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='py_trees_ros', node_executable='mock-dashboard', output='screen',
            node_name='dashboard'),
        launch_ros.actions.Node(
            package='py_trees_ros', node_executable='mock-led-strip',  output='screen',  # screen is awkward, it's after the fact
            node_name='led_strip'),
    ])
