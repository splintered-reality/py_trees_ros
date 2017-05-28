#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks the move base action server of the ROS navigation stack.
"""

##############################################################################
# Imports
##############################################################################

import dynamic_reconfigure.server

from py_trees_msgs.cfg import MockSafetySensorsConfig

##############################################################################
# Classes
##############################################################################


class SafetySensors(object):
    """
    Mocks the ability to enable/disable a safety sensor processing pipeline.
    This emulates a component which needs to be enabled contextually so that
    cpu resources can be efficiently optimised or to resolve contextual
    conflicts in the usage of the sensors.

    Dynamic Reconfigure:
        * **~enable** (:obj:`bool`)

          * enable/disable the safety sensor pipeline
    """
    def __init__(self):
        # dynamic reconfigure
        self.parameters = None
        # note this instantiation will automatically trigger the callback, so
        # self.parameters *will* get initialised
        self.dynamic_reconfigure_server = dynamic_reconfigure.server.Server(
            MockSafetySensorsConfig,
            self.dynamic_reconfigure_callback
        )

    def dynamic_reconfigure_callback(self, config, unused_level):
        """
        Args:
            config (:obj:`dynamic_reconfigure.encoding.Config`): incoming configuration
            level (:obj:`int`):
        """
        self.parameters = config
        return config
