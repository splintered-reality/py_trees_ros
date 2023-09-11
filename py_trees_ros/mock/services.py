#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros_tutorials/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Service server templates.
"""

##############################################################################
# Imports
##############################################################################

import time

import rclpy

##############################################################################
# Service Server
##############################################################################
#
# References:
#
#  service client        : https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/client.py
#  service server        : https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/service.py
#  service examples      : https://github.com/ros2/examples/tree/rolling/rclpy/services

class GenericServer(object):
    """
    Generic service server that can be used for testing.

    Args:
        node_name (:obj:`str`): name of the node
        service_name (:obj:`str`): name of the service
        service_type (:obj:`type`): type of the service
        sleep_time (:obj:`float`): time to sleep before returning a response
        callback (:obj:`Optional[callable]`): callback to execute when the service is called
    """
    def __init__(self,
                 node_name,
                 service_name,
                 service_type,
                 sleep_time=1.0,
                 callback=None):
        self.node = rclpy.create_node(node_name)

        self.sleep_time = sleep_time
        self.callback = callback

        # Create the service
        self.server = self.node.create_service(
            service_type,
            service_name,
            self.execute_callback
        )

    def execute_callback(self, request, response):
        """
        Execute the callback and populate the response.
        """
        if self.callback is not None:
            return self.callback(request, response)
        
        # Do nothing
        time.sleep(self.sleep_time)
        return response
    
    def shutdown(self):
        """
        Cleanup
        """
        self.server.destroy()
        self.node.destroy_node()