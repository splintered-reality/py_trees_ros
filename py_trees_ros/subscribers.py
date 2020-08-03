#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_ros/devel/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
ROS subscribers are asynchronous communication handles whilst py_trees
are by their nature synchronous. They tick, pause, then tick again and
provide an assumption that only one behaviour or function is running at
any single moment. Firing off a subscriber callback in the middle of that
synchronicity to write to a blackboard would break this assumption.

To get around that, subscriber behaviours run the ros callbacks in a
background thread and constrain locking and a local cache inside the behaviour.
Only in the update function is a cached variable unlocked and then
permitted to be used or written to the blackboard.
"""

##############################################################################
# Imports
##############################################################################

import copy
import operator
import py_trees
import rospy
import std_msgs.msg as std_msgs
import threading

##############################################################################
# Behaviours
##############################################################################


class Handler(py_trees.behaviour.Behaviour):
    """
    Not intended for direct use, as this just absorbs the mechanics of setting up
    a subscriber for inheritance by user-defined behaviour classes. There are several
    options for the mechanism of clearing the data so that a new result can be processed.

    **Always Look for New Data**

    This will clear any currently stored data upon entry into the behaviour (i.e. when
    :meth:`initialise` is called). This is useful for a behaviour in a sequence
    that is looking to start fresh as it is about to tick and be
    in a :attr:`~py_trees.common.Status.RUNNING` state
    until new data arrives.

    **Look for New Data only after SUCCESS**

    This will clear any currently stored data as soon as the behaviour returns
    :attr:`~py_trees.common.Status.SUCCESS`. This is useful for catching new
    triggers/events from things like buttons where you don't want to miss things
    even though you may not actually be ticking.

    **Use the Latest Data**

    Even if this data was received before entry into the behaviour. In this case
    :meth:`initialise` does not do anything with the currently stored data. Useful
    as a blocking behaviour to wait on some some topic having been initialised with
    some data (e.g. CameraInfo).

    .. warning::
        Do not use - it will always return
        :attr:`~py_trees.common.Status.INVALID`. Subclass it to create a functional
        behaviour.

    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`): name of the topic to connect to
        topic_type (:obj:`any`): class of the message type (e.g. :obj:`std_msgs.msg.String`)
        clearing_policy (:class:`~py_trees.common.ClearingPolicy`): when to clear the data
    """
    def __init__(self,
                 name="Subscriber Handler",
                 topic_name="/foo",
                 topic_type=None,
                 clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
                 ):
        super(Handler, self).__init__(name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.msg = None
        self.subscriber = None
        self.data_guard = threading.Lock()
        self.clearing_policy = clearing_policy

    def setup(self, timeout):
        """
        Initialises the subscriber.

        Args:
            timeout (:obj:`float`): time to wait (0.0 is blocking forever)

        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        # ros doesn't care if it is init'd or not for subscriber construction, but
        # good to have here anyway so initialisation can occur before the callback is connected
        self.subscriber = rospy.Subscriber(self.topic_name, self.topic_type, self._callback, queue_size=5)
        return True

    def initialise(self):
        """
        If the clearing policy is set to :attr:`~py_trees.common.ClearingPolicy.ON_INITIALISE` it
        will clear the internally saved message, otherwise it does nothing.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        with self.data_guard:
            if self.clearing_policy == py_trees.common.ClearingPolicy.ON_INITIALISE:
                self.msg = None

    def _callback(self, msg):
        """
        The subscriber callback, just stores a copy of the message internally.

        Args:
            msg (:obj:`any`): the incoming message
        """
        with self.data_guard:
            self.msg = msg
            # else ignore it


class CheckData(Handler):
    """
    Check a subscriber to see if it has received data.

    It optionally checks whether that data, or part of it has a specific value.

    **Usage Patterns**

    *Sequence Guard*: :attr:`~py_trees.common.Status.RUNNING` until there is a successful comparison

    - fail_if_no_data=False
    - fail_if_bad_comparison=False

    *Selector Priority Chooser*: :attr:`~py_trees.common.Status.FAILURE` until there is a successful comparison

    - fail_if_no_data=True
    - fail_if_bad_comparison=True

    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`): name of the topic to connect to
        topic_type (:obj:`any`): class of the message type (e.g. :obj:`std_msgs.msg.String`)
        variable_name (:obj:`str`): name of the variable to check
        expected_value (:obj:`any`): expected value of the variable
        fail_if_no_data (:obj:`bool`): :attr:`~py_trees.common.Status.FAILURE` instead of :attr:`~py_trees.common.Status.RUNNING` if there is no data yet
        fail_if_bad_comparison (:obj:`bool`): :attr:`~py_trees.common.Status.FAILURE` instead of :attr:`~py_trees.common.Status.RUNNING` if comparison failed
        comparison_operator (:obj:`func`): one from the python `operator module`_
        clearing_policy (:class:`~py_trees.common.ClearingPolicy`): when to clear the data

    .. tip::

        Prefer :class:`~py_trees_ros.subscribers.ToBlackboard` and the various blackboard checking behaviours instead. It will provide you with
        better introspection capability and less complex behaviour construction to manage.

    .. include:: weblinks.rst

    """
    def __init__(self,
                 name="Check Data",
                 topic_name="/foo",
                 topic_type=None,
                 variable_name="bar",
                 expected_value=None,
                 fail_if_no_data=False,
                 fail_if_bad_comparison=False,
                 comparison_operator=operator.eq,
                 clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
                 ):
        super(CheckData, self).__init__(
            name,
            topic_name=topic_name,
            topic_type=topic_type,
            clearing_policy=clearing_policy,
        )
        self.variable_name = variable_name
        self.expected_value = expected_value
        self.comparison_operator = comparison_operator
        self.fail_if_no_data = fail_if_no_data
        self.fail_if_bad_comparison = fail_if_bad_comparison

    def update(self):
        """
        Handles all the logic for determining what result should go back.

        Returns:
            :class:`~py_trees.common.Status`: depending on the checking results
        """
        self.logger.debug("%s.update()]" % self.__class__.__name__)
        with self.data_guard:
            msg = copy.copy(self.msg)
        if msg is None:
            self.feedback_message = "have not yet received any messages"
            return py_trees.common.Status.FAILURE if self.fail_if_no_data else py_trees.common.Status.RUNNING

        check_attr = operator.attrgetter(self.variable_name)
        try:
            value = check_attr(msg)
        except AttributeError:
            rospy.logerr("Behaviours [%s" % self.name + "]: variable name not found [%s]" % self.variable_name)
            print("%s" % msg)
            with self.data_guard:
                self.feedback_message = "variable name not found [%s]" % self.variable_name
                return py_trees.common.Status.FAILURE

        success = self.comparison_operator(value, self.expected_value)

        if success:
            self.feedback_message = "'%s' comparison succeeded [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
            if self.clearing_policy == py_trees.common.ClearingPolicy.ON_SUCCESS:
                with self.data_guard:
                    self.msg = None
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "'%s' comparison failed [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
            return py_trees.common.Status.FAILURE if self.fail_if_bad_comparison else py_trees.common.Status.RUNNING


class WaitForData(Handler):
    """
    Waits for a subscriber's callback to be triggered. This doesn't care about
    the actual data, just whether it arrived or not, so is useful for catching
    triggers (std_msgs/Empty messages) or blocking (in the behaviour sense)
    until some data has arrived (e.g. camera configuration). There are
    two use cases:

    **Usage Patterns**

    *Got Something, Sometime*

    * clearing_policy == :attr:`~py_trees.common.ClearingPolicy.NEVER`

    Don't care when data arrived, just that it arrived. This could be for something
    like a map topic, or a configuration that you need to block. Once it returns :attr:`~py_trees.common.Status.SUCCESS`,
    it will always return :attr:`~py_trees.common.Status.SUCCESS`.

    *Wating for the Next Thing, Starting From Now*

    * clearing_policy == :attr:`~py_trees.common.ClearingPolicy.ON_INTIALISE`

    Useful as a gaurd at the start of a sequence, that is waiting for an event to
    trigger after the sequence has started (i.e. been initialised).

    *Wating for the Next Thing, Starting from the Last*

    * clearing_policy == :attr:`~py_trees.common.ClearingPolicy.ON_SUCCESS`

    Useful as a guard watching for an event that could have come in anytime, but for
    which we do with to reset (and subsequently look for the next event). e.g. button events.

    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`): name of the topic to connect to
        topic_type (:obj:`any`): class of the message type (e.g. :obj:`std_msgs.msg.String`)
        clearing_policy (:class:`~py_trees.common.ClearingPolicy`): when to clear the data
    """
    def __init__(self,
                 name="Wait For Data",
                 topic_name="chatter",
                 topic_type=None,
                 clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
                 ):
        super(WaitForData, self).__init__(
            name,
            topic_name=topic_name,
            topic_type=topic_type,
            clearing_policy=clearing_policy
        )

    def update(self):
        """
        Returns:
            :class:`~py_trees.common.Status`: :attr:`~py_trees.common.Status.RUNNING` (no data) or :attr:`~py_trees.common.Status.SUCCESS`
        """
        self.logger.debug("%s.update()]" % self.__class__.__name__)
        with self.data_guard:
            if self.msg is None:
                self.feedback_message = "no message received yet"
                return py_trees.common.Status.RUNNING
            else:
                self.feedback_message = "got incoming"
                if self.clearing_policy == py_trees.common.ClearingPolicy.ON_SUCCESS:
                    self.msg = None
                return py_trees.common.Status.SUCCESS


class ToBlackboard(Handler):
    """
    Saves the latest message to the blackboard and immediately returns success.
    If no data has yet been received, this behaviour blocks (i.e. returns
    RUNNING).

    Typically this will save the entire message, however sub fields can be
    designated, in which case they will write to the specified keys.

    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`): name of the topic to connect to
        topic_type (:obj:`any`): class of the message type (e.g. :obj:`std_msgs.msg.String`)
        blackboard_variables (:obj:`dict`): blackboard variable string or dict {names (keys) - message subfields (values)}, use a value of None to indicate the entire message
        initialise_variables (:obj:`bool`): initialise the blackboard variables to some defaults
        clearing_policy (:class:`~py_trees.common.ClearingPolicy`): when to clear the data

    Examples:

        To capture an entire message:

        .. code-block:: python

            chatter_to_blackboard = ToBlackboard(topic_name="chatter",
                                                 topic_type=std_msgs.msg.String,
                                                 blackboard_variables = {'chatter': None}
                                                 )

        or to get rid of the annoying sub-data field in std_msgs/String:

        .. code-block:: python

            chatter_to_blackboard = ToBlackboard(topic_name="chatter",
                                                 topic_type=std_msgs.msg.String,
                                                 blackboard_variables = {'chatter': 'data'}
                                                 )

        combinations of multiple entities inside the message can also be saved:

        .. code-block:: python

           blackboard_variables={"pose_with_covariance_stamped": None, "pose": "pose.pose"}
    """
    def __init__(self,
                 name="ToBlackboard",
                 topic_name="chatter",
                 topic_type=None,
                 blackboard_variables={"chatter": None},
                 initialise_variables={},
                 clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
                 ):
        super(ToBlackboard, self).__init__(
            name,
            topic_name=topic_name,
            topic_type=topic_type,
            clearing_policy=clearing_policy
        )
        self.logger = py_trees.logging.Logger("%s" % self.name)
        self.blackboard = py_trees.blackboard.Blackboard()
        if isinstance(blackboard_variables, str):
            self.blackboard_variable_mapping = {blackboard_variables: None}
            if not isinstance(initialise_variables, dict):
                self.blackboard_initial_variable_mapping = {blackboard_variables: initialise_variables}
            else:
                self.blackboard_initial_variable_mapping = initialise_variables
        elif not isinstance(blackboard_variables, dict):
            self.logger.error("blackboard_variables is not a dict, please rectify")
            self.blackboard_variable_mapping = {}
            self.blackboard_initial_variable_mapping = {}
        else:
            self.blackboard_variable_mapping = blackboard_variables
            self.blackboard_initial_variable_mapping = initialise_variables
        # initialise the variables
        for name, value in self.blackboard_initial_variable_mapping.items():
            if not self.blackboard.set(name, value):
                # do we actually want to log an error?
                self.logger.error("tried to initialise an already initialised blackboard variable '{0}', check that you do not have a conflict with another behaviour [{1}]".format(name, self.name))

    def setup(self, timeout):
        """
        Initialise the subscriber.

        Args:
            timeout (:obj:`float`): time to wait (0.0 is blocking forever)

        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        return super(ToBlackboard, self).setup(timeout)

    def update(self):
        """
        Writes the data (if available) to the blackboard.

        Returns:
            :class:`~py_trees.common.Status`: :attr:`~py_trees.common.Status.RUNNING` (no data) or :attr:`~py_trees.common.Status.SUCCESS`
        """
        with self.data_guard:
            if self.msg is None:
                self.feedback_message = "no message received yet"
                return py_trees.common.Status.RUNNING
            else:
                for k, v in self.blackboard_variable_mapping.items():
                    if v is None:
                        self.blackboard.set(k, self.msg, overwrite=True)
                    else:
                        fields = v.split(".")
                        value = copy.copy(self.msg)
                        for field in fields:
                            value = getattr(value, field)
                            self.blackboard.set(k, value, overwrite=True)
                self.feedback_message = "saved incoming message"
                # this is of dubious worth, since the default setting of ClearingPolicy.ON_INITIALISE
                # covers every use case that we can think of.
                if self.clearing_policy == py_trees.common.ClearingPolicy.ON_SUCCESS:
                    self.msg = None
                return py_trees.common.Status.SUCCESS


class EventToBlackboard(Handler):
    """
    Listen for events (:obj:`std_msgs.msg.Empty`) on a
    subscriber and writes the result to the blackboard.

    This will write True if at least one message was received,
    False otherwise to a bool. This can then be consumed
    by the tree's tick. No need to clean up, it will write anew on the next tick.

    .. tip::
        Ideally you need this at the very highest part of the tree so that it
        gets triggered every time - once this happens, then the rest of the behaviour
        tree can utilise the variables.

    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`): name of the topic to connect to
        variable_name (:obj:`str`): name to write the boolean result on the blackboard
    """
    def __init__(self,
                 name="Event to Blackboard",
                 topic_name="/event",
                 variable_name="event"
                 ):
        super(EventToBlackboard, self).__init__(
            name=name,
            topic_name=topic_name,
            topic_type=std_msgs.Empty,
            clearing_policy=py_trees.common.ClearingPolicy.ON_SUCCESS
        )
        self.variable_name = variable_name
        self.blackboard = py_trees.Blackboard()

    def update(self):
        """
        Check for data and write to the board.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        with self.data_guard:
            self.blackboard.set(self.variable_name, self.msg is not None, overwrite=True)
            # ON_SUCCESS is the only clearing_policy that subclasses of SubscriberHandler must implement themselves
            self.msg = None
        return py_trees.common.Status.SUCCESS
