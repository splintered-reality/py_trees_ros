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
Various instantiable templates for action client/server style behaviours.
"""

##############################################################################
# Imports
##############################################################################

import py_trees

##############################################################################
# Behaviours
##############################################################################


class ActionClient(py_trees.behaviour.Behaviour):
    """
    A generic action client interface. This simply sends a pre-configured
    goal to the action client.
    """
    def __init__(self, name=py_trees.common.Name.AUTO_GENERATED):
        super().__init__(name)
        self.node = None

    def setup(self, **kwargs):
        """
        Setup the action client services and subscribers.

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            KeyError: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

    def initialise(self):
        """
        Reset the internal variables.
        """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        If running and the current goal has not already succeeded, cancel it.

        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
