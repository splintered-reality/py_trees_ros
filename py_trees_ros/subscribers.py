#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros/devel/LICENSE
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
import threading
import typing

import py_trees
import rclpy.qos
import std_msgs.msg as std_msgs

##############################################################################
# Behaviours
##############################################################################


class Handler(py_trees.behaviour.Behaviour):
    """
    .. warning::
        Do not use - it will always return
        :attr:`~py_trees.common.Status.INVALID`. Subclass it to create a functional
        behaviour.

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

    Args:
        topic_name: name of the topic to connect to
        topic_type: class of the message type (e.g. :obj:`std_msgs.msg.String`)
        qos_profile: qos profile for the subscriber
        name: name of the behaviour
        clearing_policy: when to clear the data
    """
    def __init__(self,
                 topic_name: str,
                 topic_type: typing.Any,
                 qos_profile: rclpy.qos.QoSProfile,
                 name: str=py_trees.common.Name.AUTO_GENERATED,
                 clearing_policy: py_trees.common.ClearingPolicy=py_trees.common.ClearingPolicy.ON_INITIALISE
                 ):
        super(Handler, self).__init__(name=name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.msg = None
        self.subscriber = None
        self.data_guard = threading.Lock()
        self.clearing_policy = clearing_policy
        self.qos_profile = qos_profile
        self.node = None

    def setup(self, **kwargs):
        """
        Initialises the subscriber.

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
        self.subscriber = self.node.create_subscription(
            msg_type=self.topic_type,
            topic=self.topic_name,
            callback=self._callback,
            qos_profile=self.qos_profile
        )

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
        topic_name: name of the topic to connect to
        topic_type: class of the message type (e.g. :obj:`std_msgs.msg.String`)
        qos_profile: qos profile for the subscriber
        variable_name: name of the variable to check
        expected_value: expected value of the variable
        comparison_operator: one from the python `operator module`_
        fail_if_no_data: :attr:`~py_trees.common.Status.FAILURE` instead of :attr:`~py_trees.common.Status.RUNNING` if there is no data yet
        fail_if_bad_comparison: :attr:`~py_trees.common.Status.FAILURE` instead of :attr:`~py_trees.common.Status.RUNNING` if comparison failed
        name: name of the behaviour
        clearing_policy: when to clear the data

    .. tip::

        Prefer :class:`~py_trees_ros.subscribers.ToBlackboard` and the various blackboard checking behaviours instead. It will provide you with
        better introspection capability and less complex behaviour construction to manage.

    .. include:: weblinks.rst

    """
    def __init__(self,
                 topic_name: str,
                 topic_type: typing.Any,
                 qos_profile: rclpy.qos.QoSProfile,
                 variable_name: str,
                 expected_value: typing.Any,
                 comparison_operator=operator.eq,
                 fail_if_no_data=False,
                 fail_if_bad_comparison=False,
                 name=py_trees.common.Name.AUTO_GENERATED,
                 clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
                 ):
        super(CheckData, self).__init__(
            name=name,
            topic_name=topic_name,
            topic_type=topic_type,
            qos_profile=qos_profile,
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
            self.node.get_logger().error("Behaviour [{}]: variable name not found [{}]".format(self.name, self.variable_name))
            print("{}".format(msg))
            with self.data_guard:
                self.feedback_message = "variable name not found [{}]".format(self.variable_name)
                return py_trees.common.Status.FAILURE

        success = self.comparison_operator(value, self.expected_value)

        if success:
            self.feedback_message = "'{}' comparison succeeded [v: {}][e: {}]".format(
                self.variable_name, value, self.expected_value)
            if self.clearing_policy == py_trees.common.ClearingPolicy.ON_SUCCESS:
                with self.data_guard:
                    self.msg = None
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "'{}' comparison failed [v: {}][e: {}]".format(
                self.variable_name, value, self.expected_value)
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
        topic_name: name of the topic to connect to
        topic_type: class of the message type (e.g. :obj:`std_msgs.msg.String`)
        qos_profile: qos profile for the subscriber
        name: name of the behaviour
        clearing_policy: when to clear the data
    """
    def __init__(self,
                 topic_name: str,
                 topic_type: typing.Any,
                 qos_profile: rclpy.qos.QoSProfile,
                 name=py_trees.common.Name.AUTO_GENERATED,
                 clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
                 ):
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=topic_type,
            qos_profile=qos_profile,
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
        topic_name: name of the topic to connect to
        topic_type: class of the message type (e.g. :obj:`std_msgs.msg.String`)
        qos_profile: qos profile for the subscriber
        blackboard_variables: blackboard variable string or dict {names (keys) - message subfields (values)}, use a value of None to indicate the entire message
        initialise_variables: initialise the blackboard variables to some defaults
        name: name of the behaviour
        clearing_policy: when to clear the data

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
                 topic_name: str,
                 topic_type: typing.Any,
                 qos_profile: rclpy.qos.QoSProfile,
                 blackboard_variables: typing.Dict[str, typing.Any]={},  # e.g. {"chatter": None}
                 initialise_variables: typing.Dict[str, typing.Any]={},
                 name=py_trees.common.Name.AUTO_GENERATED,
                 clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
                 ):
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=topic_type,
            qos_profile=qos_profile,
            clearing_policy=clearing_policy
        )
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.logger = py_trees.logging.Logger("%s" % self.name)
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
        # register the variables
        for name in self.blackboard_variable_mapping:
            self.blackboard.register_key(
                key=name,
                access=py_trees.common.Access.WRITE
            )
        # initialise the variables
        for name, value in self.blackboard_initial_variable_mapping.items():
            if not self.blackboard.set(name, value):
                # do we actually want to log an error?
                self.logger.error("tried to initialise an already initialised blackboard variable '{0}', check that you do not have a conflict with another behaviour [{1}]".format(name, self.name))

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
    Listen for events (:class:`std_msgs.msg.Empty`) on a
    subscriber and writes the result to the blackboard.

    This will write True to the variable on the blackboard if a message
    was received since the last tick, False otherwise. The behaviour itself
    always returns :attr:`~py_trees.common.Status.SUCCESS`.

    .. tip::
        Ideally you need this at the very highest part of the tree so that it
        gets triggered every time - once this happens, then the rest of the behaviour
        tree can utilise the variables.

    Args:
        topic_name: name of the topic to connect to
        qos_profile: qos profile for the subscriber
        variable_name: name to write the boolean result on the blackboard
        name: name of the behaviour
    """
    def __init__(self,
                 topic_name: str,
                 qos_profile: rclpy.qos.QoSProfile,
                 variable_name: str,
                 name=py_trees.common.Name.AUTO_GENERATED,
                 ):
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=std_msgs.Empty,
            qos_profile=qos_profile,
            clearing_policy=py_trees.common.ClearingPolicy.ON_SUCCESS
        )
        self.variable_name = variable_name
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=self.variable_name,
            access=py_trees.common.Access.WRITE
        )

    def update(self):
        """
        Check for data and write to the board.

        Returns:
            Always returns :attr:`~py_trees.common.Status.SUCCESS`
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        with self.data_guard:
            self.blackboard.set(self.variable_name, self.msg is not None, overwrite=True)
            # ON_SUCCESS is the only clearing_policy that subclasses of SubscriberHandler must implement themselves
            self.msg = None
        return py_trees.common.Status.SUCCESS
