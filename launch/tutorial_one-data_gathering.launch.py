#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_ros/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Launcher for tutorial one.
"""

##############################################################################
# Imports
##############################################################################

import launch
import launch_ros
import py_trees_ros
import py_trees.console as console

##############################################################################
# Helpers
##############################################################################

##############################################################################
# Main
##############################################################################


def main(argv=sys.argv[1:]):
    """
    Entry point for the launcher.
    """
    ####################
    # Description
    ####################
    launch_description = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp', node_executable='talker', output='screen',
            remappings=[('chatter', 'my_chatter')]),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', node_executable='listener', output='screen',
            remappings=[('chatter', 'my_chatter')]),
    ])
    ####################
    # Rclpy
    ####################
