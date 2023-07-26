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
Mocks a SetBool service
"""


##############################################################################
# Imports
##############################################################################

import time

from std_srvs.srv import SetBool

from . import services

##############################################################################
# Class
##############################################################################


class SetBoolServer(services.GenericServer):
    """
    Simple server that docks if the goal is true, undocks otherwise.
    """
    SUCCESS_MESSAGE = "Succeeded in setting bool to True"
    FAILURE_MESSAGE = "Failed to set bool to False"

    def __init__(self, sleep_time=1.0):
        super().__init__(
            node_name="set_bool_server",
            service_name="set_bool",
            service_type=SetBool,
            sleep_time=sleep_time,
            callback=self.callback,
        )

    def callback(self, request, response):
        time.sleep(self.sleep_time)
        if request.data:
            response.success = True
            response.message = SetBoolServer.SUCCESS_MESSAGE
        else:
            response.success = False
            response.message = SetBoolServer.FAILURE_MESSAGE
        return response
