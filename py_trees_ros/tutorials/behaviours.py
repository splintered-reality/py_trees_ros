#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
A few behaviours to support the tutorials.
"""

##############################################################################
# Imports
##############################################################################

import dynamic_reconfigure.client
import py_trees
import rospy
import std_msgs.msg as std_msgs

##############################################################################
# Behaviours
##############################################################################


class FlashLedStrip(py_trees.behaviour.Behaviour):
    """
    This behavoiur simply shoots a command off to the LEDStrip to flash
    a certain colour and returns :attr:`~py_trees.common.Status.RUNNING`.
    Note that this behaviour will never return with
    :attr:`~py_trees.common.Status.SUCCESS` but will send a clearing
    command to the LEDStrip if it is cancelled or interrupted by a higher
    priority behaviour.

    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`) : name of the battery state topic
        colour (:obj:`str`) : colour to flash ['red', 'green', blue']
    """
    def __init__(self, name, topic_name="/led_strip/command", colour="red"):
        super(FlashLedStrip, self).__init__(name=name)
        self.topic_name = topic_name
        self.colour = colour

    def setup(self, timeout):
        """
        Args:
            timeout (:obj:`float`): time to wait (0.0 is blocking forever)

        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        self.publisher = rospy.Publisher(self.topic_name, std_msgs.String, queue_size=10, latch=True)
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Annoy the led strip to keep firing every time it ticks over (the led strip will clear itself
        if no command is forthcoming within a certain period of time). This behaviour will only finish if it
        is terminated or interrupted from above.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.publisher.publish(std_msgs.String(self.colour))
        self.feedback_message = "flashing {0}".format(self.colour)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Shoot off a clearing command to the led strip.

        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.publisher.publish(std_msgs.String(""))
        self.feedback_message = "cleared"


class ScanContext(py_trees.behaviour.Behaviour):
    """
    This behavoiur simply shoots a command off to the LEDStrip to flash
    a certain colour and returns :attr:`~py_trees.common.Status.RUNNING`.
    Note that this behaviour will never return with
    :attr:`~py_trees.common.Status.SUCCESS` but will send a clearing
    command to the LEDStrip if it is cancelled or interrupted by a higher
    priority behaviour.

    Args:
        name (:obj:`str`): name of the behaviour
    """
    def __init__(self, name):
        super(ScanContext, self).__init__(name=name)
        self.initialised = False
        self._namespaces = ["safety_sensors",
                            "rotate",
                            ]
        self._dynamic_reconfigure_clients = {}
        for name in self._namespaces:
            self._dynamic_reconfigure_clients[name] = None
        self._dynamic_reconfigure_configurations = {}

    def setup(self, timeout):
        """
        Try and connect to the dynamic reconfigure server on the various namespaces.
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        for namespace in self._namespaces:
            if not self._dynamic_reconfigure_clients[namespace]:
                try:
                    self._dynamic_reconfigure_clients[namespace] = dynamic_reconfigure.client.Client(
                        name=namespace,
                        timeout=timeout
                    )
                except rospy.ROSException:
                    rospy.logwarn("ScanContext [%s" % self.name + "]: could not connect to dynamic reconfigure server [%s][%s secs]" % (namespace, timeout))
                    self.feedback_message = "could not connect to dynamic reconfigure server [%s][%s secs]" % (namespace, timeout)
                    return False
        return True

    def initialise(self):
        """
        Get various dyn reconf configurations and cache/set the new variables.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)

        for name, client in self._dynamic_reconfigure_clients.items():
            self._dynamic_reconfigure_configurations[name] = client.get_configuration()
        try:
            self.safety_sensors_enable = self._dynamic_reconfigure_configurations["safety_sensors"]["enable"]
            self._dynamic_reconfigure_clients["safety_sensors"].update_configuration({"enable": True})
        except dynamic_reconfigure.DynamicReconfigureParameterException:
            self.feedback_message = "failed to configure the 'enable' parameter [safety_sensors]"
            self.initialised = False
        try:
            self.rotate_duration = self._dynamic_reconfigure_configurations["rotate"]["duration"]
            self._dynamic_reconfigure_clients["rotate"].update_configuration({"duration": 8.0})
        except dynamic_reconfigure.DynamicReconfigureParameterException:
            self.feedback_message = "failed to configure the 'duration' parameter [rotate]"
            self.initialised = False

        self.initialised = True
        self.feedback_message = "reconfigured the context for scanning"

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        if not self.initialised:
            return py_trees.common.Status.FAILURE
        # used under a parallel, never returns success
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Regardless of whether it succeeed or failed or is getting set to invalid we have to be absolutely
        sure to reset the navi context.
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if self.initialised:
            try:
                self._dynamic_reconfigure_clients["safety_sensors"].update_configuration({"enable": self.safety_sensors_enable})
            except dynamic_reconfigure.DynamicReconfigureParameterException:
                self.feedback_message = "failed to reset the 'enable' parameter [safety_sensors]"
            try:
                self._dynamic_reconfigure_clients["rotate"].update_configuration({"duration": self.rotate_duration})
            except dynamic_reconfigure.DynamicReconfigureParameterException:
                self.feedback_message = "failed to reset the 'duration' parameter [rotate]"
            self.initialised = False
