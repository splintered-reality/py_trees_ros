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
Custom exception types for py_trees_ros.
"""

##############################################################################
# Imports
##############################################################################


class MultipleFoundError(Exception):
    """
    Middleware connection found when but one was expected.
    """
    pass


class NotFoundError(Exception):
    """
    Middleware connection not found.
    """
    pass


class NotReadyError(Exception):
    """
    Typically used when methods have been called that expect, but have not
    pre-engaged in the ROS2 specific setup typical of py_trees_ros classes
    and behaviours.
    """
    pass


class ServiceError(Exception):
    """
    Failure in a service request-response process (usually no response).
    """
    pass


class TimedOutError(Exception):
    """
    Timed out waiting (typically) for middleware connections to be established.
    """
    pass
