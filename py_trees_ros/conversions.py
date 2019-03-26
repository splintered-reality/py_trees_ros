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
Converter methods for transferring information back and forth between
py_trees objects and ros messages.
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees_ros_interfaces.msg as py_trees_msgs
import rclpy
import unique_identifier_msgs.msg as unique_identifier_msgs
import uuid

##############################################################################
# <etjpds
##############################################################################


def behaviour_type_to_msg_constant(behaviour):
    """
    Convert a behaviour class type to a message constant.

    Args:
        behaviour (:class:`~py_trees.behaviour.Behaviour`): investigate the type of this behaviour

    Returns:
        :obj:`uint8`: from the type constants in :class:`~py_trees_msgs.msg.Behaviour`
    """
    # TODO: problems with decorators?
    # TODO: throw an exception if parent is not Behaviour?
    if isinstance(behaviour, py_trees.composites.Sequence):
        return py_trees_msgs.Behaviour.SEQUENCE
    elif isinstance(behaviour, py_trees.composites.Chooser):
        return py_trees_msgs.Behaviour.CHOOSER
    elif isinstance(behaviour, py_trees.composites.Selector):
        return py_trees_msgs.Behaviour.SELECTOR
    elif isinstance(behaviour, py_trees.composites.Parallel):
        return py_trees_msgs.Behaviour.PARALLEL
    elif isinstance(behaviour, py_trees.decorators.Decorator):
        return py_trees_msgs.Behaviour.DECORATOR
    elif isinstance(behaviour, py_trees.behaviour.Behaviour):
        return py_trees_msgs.Behaviour.BEHAVIOUR
    else:
        return py_trees_msgs.Behaviour.UNKNOWN_TYPE


def msg_constant_to_behaviour_type(value):
    """
    Convert one of the behaviour type constants in a
    :class:`~py_trees_ros_interfaces.msg.Behaviour` message to
    a type.

    Args:
        value (obj:`int``: see the message definition for details

    Returns:
        a behaviour class type (e.g. :class:`~py_trees.composites.Sequence`)

    Raises:
        TypeError: if the message type is unrecognised
    """
    if value == py_trees_msgs.Behaviour.SEQUENCE:
        return py_trees.composites.Sequence
    elif value == py_trees_msgs.Behaviour.CHOOSER:
        return py_trees.composites.Chooser
    elif value == py_trees_msgs.Behaviour.SELECTOR:
        return py_trees.composites.Selector
    elif value == py_trees_msgs.Behaviour.PARALLEL:
        return py_trees.composites.Parallel
    elif value == py_trees_msgs.Behaviour.DECORATOR:
        return py_trees.decorators.Decorator
    elif value == py_trees_msgs.Behaviour.BEHAVIOUR:
        return py_trees.behaviour.Behaviour
    else:
        raise TypeError("invalid type specified in message [{}]".format(value))


def status_enum_to_msg_constant(status):
    """
    Convert a status to a message constant.

    Args:
        status (:class:`~py_trees.common.Status`): status enum of a behaviour

    Returns:
        :obj:`uint8`: from the status constants in :class:`~py_trees_msgs.msg.Behaviour`
    """
    if status == py_trees.common.Status.INVALID:
        return py_trees_msgs.Behaviour.INVALID
    elif status == py_trees.common.Status.RUNNING:
        return py_trees_msgs.Behaviour.RUNNING
    elif status == py_trees.common.Status.SUCCESS:
        return py_trees_msgs.Behaviour.SUCCESS
    elif status == py_trees.common.Status.FAILURE:
        return py_trees_msgs.Behaviour.FAILURE
    else:
        return 0  # unknown status


def msg_constant_to_status_enum(value: int) -> py_trees.common.Status:
    """
    Convert one of the status constants in a
    :class:`~py_trees_ros_interfaces.msg.Behaviour` message to
    a py_trees status enum.

    Args:
        value (obj:`int``: see the message definition for details

    Returns:
        :class:`~py_trees.common.Status`: a py_trees status

    Raises:
        TypeError: if the status type is unrecognised
    """
    if value == py_trees_msgs.Behaviour.INVALID:
        return py_trees.common.Status.INVALID
    elif value == py_trees_msgs.Behaviour.RUNNING:
        return py_trees.common.Status.RUNNING
    elif value == py_trees_msgs.Behaviour.SUCCESS:
        return py_trees.common.Status.SUCCESS
    elif value == py_trees_msgs.Behaviour.FAILURE:
        return py_trees.common.Status.FAILURE
    else:
        raise TypeError("invalid status specified in message [{}]".format(value))


def blackbox_enum_to_msg_constant(blackbox_level):
    """
    Convert a blackbox level enum to a message constant.

    Args:
        blackbox_level (:class:`~py_trees.common.BlackboxLevel`): blackbox level of a behaviour

    Returns:
        :obj:`uint8`: from the type constants in :class:`~py_trees_msgs.msg.Behaviour`
    """
    if blackbox_level == py_trees.common.BlackBoxLevel.DETAIL:
        return py_trees_msgs.Behaviour.BLACKBOX_LEVEL_DETAIL
    elif blackbox_level == py_trees.common.BlackBoxLevel.COMPONENT:
        return py_trees_msgs.Behaviour.BLACKBOX_LEVEL_COMPONENT
    elif blackbox_level == py_trees.common.BlackBoxLevel.BIG_PICTURE:
        return py_trees_msgs.Behaviour.BLACKBOX_LEVEL_BIG_PICTURE
    else:
        return py_trees_msgs.Behaviour.BLACKBOX_LEVEL_NOT_A_BLACKBOX


def msg_constant_to_blackbox_level_enum(value: int) -> py_trees.common.BlackBoxLevel:
    """
    Convert one of the blackbox level constants in a
    :class:`~py_trees_ros_interfaces.msg.Behaviour` message to
    a py_trees status enum.

    Args:
        value (obj:`int``: see the message definition for details

    Returns:
        :class:`~py_trees.common.BlackBoxLevel`: a py_trees status

    Raises:
        TypeError: if the status type is unrecognised
    """
    if value == py_trees_msgs.Behaviour.BLACKBOX_LEVEL_DETAIL:
        return py_trees.common.BlackBoxLevel.DETAIL
    elif value == py_trees_msgs.Behaviour.BLACKBOX_LEVEL_COMPONENT:
        return py_trees.common.BlackBoxLevel.COMPONENT
    elif value == py_trees_msgs.Behaviour.BLACKBOX_LEVEL_BIG_PICTURE:
        return py_trees.common.BlackBoxLevel.BIG_PICTURE
    elif value == py_trees_msgs.Behaviour.BLACKBOX_LEVEL_NOT_A_BLACKBOX:
        return py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX
    else:
        raise TypeError("invalid blackbox level specified in message [{}]".format(value))


def uuid4_to_msg(uuid4: uuid.UUID=uuid.uuid4()):
    """
    Convert a uuid4 python object to a ros unique identifier, UUID type.

    Args:
        uuid4 (:class:`uuid.UUID`), optional: unique identifier to convert, defaults to auto-generated uuid4

    Returns:
        :class:`unique_identifier_msgs.msg.UUID`: the ros message type
    """
    return unique_identifier_msgs.UUID(uuid=list(uuid4.bytes))


def msg_to_uuid4(msg: unique_identifier_msgs.UUID):
    """
    Convert a uuid4 python object to a ros unique identifier, UUID type.

    Args:
        msg (:class:`unique_identifier_msgs.msg.UUID`): the ros message type

    Returns:
        :class:`uuid.UUID`: the behaviour's uuid, python style
    """
    return uuid.UUID(bytes=bytes(msg.uuid), version=4)


def behaviour_to_msg(behaviour):
    """
    Convert a behaviour to a message.

    Args:
        behaviour (:class:`~py_trees.behaviour.Behaviour`): behaviour to convert

    Returns:
        :class:`~py_trees_msgs.msg.Behaviour`: a ros message representation of a behaviour
    """
    msg = py_trees_msgs.Behaviour()
    msg.name = behaviour.name
    msg.class_name = str(behaviour.__module__) + '.' + str(type(behaviour).__name__)
    msg.own_id = uuid4_to_msg(behaviour.id)
    msg.parent_id = uuid4_to_msg(behaviour.parent.id) if behaviour.parent else unique_identifier_msgs.UUID()
    msg.child_ids = [uuid4_to_msg(child.id) for child in behaviour.iterate(direct_descendants=True) if not child.id == behaviour.id]

    tip = behaviour.tip()
    # tip_id is empty if the behaviour is invalid or if it is a valid
    # leaf
    if tip is not None and tip != behaviour:
        msg.tip_id = uuid4_to_msg(tip.id)

    msg.type = behaviour_type_to_msg_constant(behaviour)
    msg.blackbox_level = blackbox_enum_to_msg_constant(behaviour.blackbox_level)
    msg.status = status_enum_to_msg_constant(behaviour.status)
    msg.message = behaviour.feedback_message

    return msg


def msg_to_behaviour(msg: py_trees_msgs.Behaviour) -> py_trees.behaviour.Behaviour:
    """
    Convert behaviour message to a py_trees behaviour. This doesn't completely
    recreate the original behaviour (doesn't have any of the custom state), but
    allows the user to compose a tree that can be utilised for debugging or
    visualisation applications.

    Args:
        msg (:class:`~py_trees_msgs.msg.Behaviour`): a ros message representation of a behaviour

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: converted, skeleton of the original behaviour
    """
    BehaviourType = msg_constant_to_behaviour_type(msg.type)
    behaviour = BehaviourType(name=msg.name)
    behaviour.id = msg_to_uuid4(msg.own_id)
    # parent, children and tip have to be filled in via a second pass on the
    # list of behaviours since they directly reference other objects
    behaviour.status = msg_constant_to_status_enum(msg.status)
    behaviour.blackbox_level = msg_constant_to_blackbox_level_enum(msg.blackbox_level)
    behaviour.feedback_message = msg.message
    return behaviour


def rclpy_time_to_float(time: rclpy.time.Time) -> float:
    """
    Convert a ros2 time (seconds/nanoseconds) to a float.

    Args:
        time (:class:`~rclpy.time.Time`): time to convert
    """
    return float(time.nanoseconds) / 1e9


def rclpy_duration_to_float(duration: rclpy.time.Duration) -> float:
    """
    Convert a ros2 duration (seconds/nanoseconds) to a float.

    Args:
        time (:class:`~rclpy.time.Time`): time to convert
    """
    return float(duration.nanoseconds) / 1e9
