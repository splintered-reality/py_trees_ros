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
