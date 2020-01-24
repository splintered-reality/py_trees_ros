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
import py_trees_ros_interfaces.msg  # noqa
import rclpy
import typing
import unique_identifier_msgs.msg
import uuid


##############################################################################
# <etjpds
##############################################################################


def activity_stream_to_msgs() -> typing.List[py_trees_ros_interfaces.msg.ActivityItem]:
    """
    Convert the blackboard activity stream to a message.

    Returns:
        A list of activity item messages.
    """
    activity_stream = py_trees.blackboard.Blackboard.activity_stream
    activity_stream_msgs = []
    for item in activity_stream.data:
        msg = py_trees_ros_interfaces.msg.ActivityItem()
        msg.key = item.key
        msg.client_name = item.client_name
        msg.activity_type = item.activity_type
        msg.previous_value = str(item.previous_value)
        msg.current_value = str(item.current_value)
        activity_stream_msgs.append(msg)
    return activity_stream_msgs


def behaviour_type_to_msg_constant(behaviour: py_trees.behaviour.Behaviour):
    """
    Convert a behaviour class type to a message constant.

    Args:
        behaviour: investigate the type of this behaviour

    Returns:
        :obj:`uint8`: from the type constants in :class:`py_trees_ros_interfaces.msg.Behaviour`
    """
    # TODO: problems with decorators?
    # TODO: throw an exception if parent is not Behaviour?
    if isinstance(behaviour, py_trees.composites.Sequence):
        return py_trees_ros_interfaces.msg.Behaviour.SEQUENCE
    elif isinstance(behaviour, py_trees.composites.Chooser):
        return py_trees_ros_interfaces.msg.Behaviour.CHOOSER
    elif isinstance(behaviour, py_trees.composites.Selector):
        return py_trees_ros_interfaces.msg.Behaviour.SELECTOR
    elif isinstance(behaviour, py_trees.composites.Parallel):
        return py_trees_ros_interfaces.msg.Behaviour.PARALLEL
    elif isinstance(behaviour, py_trees.decorators.Decorator):
        return py_trees_ros_interfaces.msg.Behaviour.DECORATOR
    elif isinstance(behaviour, py_trees.behaviour.Behaviour):
        return py_trees_ros_interfaces.msg.Behaviour.BEHAVIOUR
    else:
        return py_trees_ros_interfaces.msg.Behaviour.UNKNOWN_TYPE


def msg_constant_to_behaviour_type(value: int) -> typing.Any:
    """
    Convert one of the behaviour type constants in a
    :class:`py_trees_ros_interfaces.msg.Behaviour` message to
    a type.

    Args:
        value: see the message definition for details

    Returns:
        a behaviour class type (e.g. :class:`py_trees.composites.Sequence`)

    Raises:
        TypeError: if the message type is unrecognised
    """
    if value == py_trees_ros_interfaces.msg.Behaviour.SEQUENCE:
        return py_trees.composites.Sequence
    elif value == py_trees_ros_interfaces.msg.Behaviour.CHOOSER:
        return py_trees.composites.Chooser
    elif value == py_trees_ros_interfaces.msg.Behaviour.SELECTOR:
        return py_trees.composites.Selector
    elif value == py_trees_ros_interfaces.msg.Behaviour.PARALLEL:
        return py_trees.composites.Parallel
    elif value == py_trees_ros_interfaces.msg.Behaviour.DECORATOR:
        return py_trees.decorators.Decorator
    elif value == py_trees_ros_interfaces.msg.Behaviour.BEHAVIOUR:
        return py_trees.behaviour.Behaviour
    else:
        raise TypeError("invalid type specified in message [{}]".format(value))


def additional_detail_to_str(behaviour: py_trees.behaviour.Behaviour) -> str:
    """
    Provide, e.g. policy information about the behaviour (i.e. black magic
    details under the hood). Useed for debugging, so only strings needed.

    Args:
        behaviour: investigate the policies for this behaviour

    Returns:
        an informative additional detail string
    """
    if isinstance(behaviour, py_trees.composites.Parallel):
        return type(behaviour.policy).__name__
    else:
        return ""


def status_enum_to_msg_constant(status: py_trees.common.Status):
    """
    Convert a status to a message constant.

    Args:
        status: status enum of a behaviour

    Returns:
        :obj:`uint8`: from the status constants in :class:`py_trees_ros_interfaces.msg.Behaviour`
    """
    if status == py_trees.common.Status.INVALID:
        return py_trees_ros_interfaces.msg.Behaviour.INVALID
    elif status == py_trees.common.Status.RUNNING:
        return py_trees_ros_interfaces.msg.Behaviour.RUNNING
    elif status == py_trees.common.Status.SUCCESS:
        return py_trees_ros_interfaces.msg.Behaviour.SUCCESS
    elif status == py_trees.common.Status.FAILURE:
        return py_trees_ros_interfaces.msg.Behaviour.FAILURE
    else:
        return 0  # unknown status


def msg_constant_to_status_enum(value: int) -> py_trees.common.Status:
    """
    Convert one of the status constants in a
    :class:`py_trees_ros_interfaces.msg.Behaviour` message to
    a py_trees status enum.

    Args:
        value: see the message definition for details

    Returns:
        a py_trees status

    Raises:
        TypeError: if the status type is unrecognised
    """
    if value == py_trees_ros_interfaces.msg.Behaviour.INVALID:
        return py_trees.common.Status.INVALID
    elif value == py_trees_ros_interfaces.msg.Behaviour.RUNNING:
        return py_trees.common.Status.RUNNING
    elif value == py_trees_ros_interfaces.msg.Behaviour.SUCCESS:
        return py_trees.common.Status.SUCCESS
    elif value == py_trees_ros_interfaces.msg.Behaviour.FAILURE:
        return py_trees.common.Status.FAILURE
    else:
        raise TypeError("invalid status specified in message [{}]".format(value))


def blackbox_enum_to_msg_constant(blackbox_level: py_trees.common.BlackBoxLevel):
    """
    Convert a blackbox level enum to a message constant.

    Args:
        blackbox_level: blackbox level of a behaviour

    Returns:
        :obj:`uint8`: from the type constants in :class:`py_trees_ros_interfaces.msg.Behaviour`
    """
    if blackbox_level == py_trees.common.BlackBoxLevel.DETAIL:
        return py_trees_ros_interfaces.msg.Behaviour.BLACKBOX_LEVEL_DETAIL
    elif blackbox_level == py_trees.common.BlackBoxLevel.COMPONENT:
        return py_trees_ros_interfaces.msg.Behaviour.BLACKBOX_LEVEL_COMPONENT
    elif blackbox_level == py_trees.common.BlackBoxLevel.BIG_PICTURE:
        return py_trees_ros_interfaces.msg.Behaviour.BLACKBOX_LEVEL_BIG_PICTURE
    else:
        return py_trees_ros_interfaces.msg.Behaviour.BLACKBOX_LEVEL_NOT_A_BLACKBOX


def msg_constant_to_blackbox_level_enum(value: int) -> py_trees.common.BlackBoxLevel:
    """
    Convert one of the blackbox level constants in a
    :class:`~py_trees_ros_interfaces.msg.Behaviour` message to
    a py_trees status enum.

    Args:
        value: see the message definition for details

    Returns:
        a py_trees blackbox level

    Raises:
        TypeError: if the status type is unrecognised
    """
    if value == py_trees_ros_interfaces.msg.Behaviour.BLACKBOX_LEVEL_DETAIL:
        return py_trees.common.BlackBoxLevel.DETAIL
    elif value == py_trees_ros_interfaces.msg.Behaviour.BLACKBOX_LEVEL_COMPONENT:
        return py_trees.common.BlackBoxLevel.COMPONENT
    elif value == py_trees_ros_interfaces.msg.Behaviour.BLACKBOX_LEVEL_BIG_PICTURE:
        return py_trees.common.BlackBoxLevel.BIG_PICTURE
    elif value == py_trees_ros_interfaces.msg.Behaviour.BLACKBOX_LEVEL_NOT_A_BLACKBOX:
        return py_trees.common.BlackBoxLevel.NOT_A_BLACKBOX
    else:
        raise TypeError("invalid blackbox level specified in message [{}]".format(value))


def uuid4_to_msg(uuid4: uuid.UUID=uuid.uuid4()) -> unique_identifier_msgs.msg.UUID:
    """
    Convert a uuid4 python object to a ros unique identifier, UUID type.

    Args:
        uuid4: unique identifier to convert, defaults to auto-generated uuid4

    Returns:
        the ros message type
    """
    return unique_identifier_msgs.msg.UUID(uuid=list(uuid4.bytes))


def msg_to_uuid4(msg: unique_identifier_msgs.msg.UUID) -> uuid.UUID:
    """
    Convert a uuid4 python object to a ros unique identifier, UUID type.

    Args:
        msg: the ros message type

    Returns:
        the behaviour's uuid, python style
    """
    return uuid.UUID(bytes=bytes(msg.uuid), version=4)


def behaviour_to_msg(behaviour: py_trees.behaviour.Behaviour) -> py_trees_ros_interfaces.msg.Behaviour:
    """
    Convert a behaviour to a message.

    Args:
        behaviour: behaviour to convert

    Returns:
        a ros message representation of a behaviour
    """
    msg = py_trees_ros_interfaces.msg.Behaviour()
    msg.name = behaviour.name
    msg.class_name = str(behaviour.__module__) + '.' + str(type(behaviour).__name__)
    msg.own_id = uuid4_to_msg(behaviour.id)
    msg.parent_id = uuid4_to_msg(behaviour.parent.id) if behaviour.parent else unique_identifier_msgs.msg.UUID()
    msg.child_ids = [uuid4_to_msg(child.id) for child in behaviour.iterate(direct_descendants=True) if not child.id == behaviour.id]

    tip = behaviour.tip()
    # tip_id is empty if the behaviour is invalid or if it is a valid leaf
    if tip is not None and tip != behaviour:
        msg.tip_id = uuid4_to_msg(tip.id)
    # else it gets the 'zero' uuid
    if isinstance(behaviour, py_trees.composites.Composite):
        if behaviour.current_child is not None:
            msg.current_child_id = uuid4_to_msg(behaviour.current_child.id)
    msg.type = behaviour_type_to_msg_constant(behaviour)
    msg.blackbox_level = blackbox_enum_to_msg_constant(behaviour.blackbox_level)
    msg.status = status_enum_to_msg_constant(behaviour.status)
    msg.additional_detail = additional_detail_to_str(behaviour)
    msg.message = behaviour.feedback_message
    msg.blackboard_access = []
    for blackboard in behaviour.blackboards:
        for key in blackboard.read:
            access_info = py_trees_ros_interfaces.msg.KeyValue(
                key=key,
                value=py_trees_ros_interfaces.msg.Behaviour.BLACKBOARD_ACCESS_READ  # noqa
            )
            msg.blackboard_access.append(access_info)
        for key in blackboard.write:
            access_info = py_trees_ros_interfaces.msg.KeyValue(
                key=key,
                value=py_trees_ros_interfaces.msg.Behaviour.BLACKBOARD_ACCESS_WRITE  # noqa
            )
            msg.blackboard_access.append(access_info)
        for key in blackboard.exclusive:
            access_info = py_trees_ros_interfaces.msg.KeyValue(
                key=key,
                value=py_trees_ros_interfaces.msg.Behaviour.BLACKBOARD_ACCESS_EXCLUSIVE_WRITE  # noqa
            )
            msg.blackboard_access.append(access_info)
    return msg


def msg_to_behaviour(msg: py_trees_ros_interfaces.msg.Behaviour) -> py_trees.behaviour.Behaviour:
    """
    Convert behaviour message to a py_trees behaviour. This doesn't completely
    recreate the original behaviour (doesn't have any of the custom state), but
    allows the user to compose a tree that can be utilised for debugging or
    visualisation applications.

    Args:
        msg: a ros message representation of a behaviour

    Returns:
        converted, skeleton of the original behaviour
    """
    BehaviourType = msg_constant_to_behaviour_type(msg.type)
    if BehaviourType == py_trees.decorators.Decorator:
        behaviour = BehaviourType(
            name=msg.name,
            # to be replaced with the proper entity in a second pass
            child=py_trees.behaviours.Dummy()
        )
    else:
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
        time: time to convert

    Return:
        time (seconds) as a float
    """
    return float(time.nanoseconds) / 1e9


def rclpy_duration_to_float(duration: rclpy.time.Duration) -> float:
    """
    Convert a ros2 duration (seconds/nanoseconds) to a float.

    Args:
        time: time to convert

    Return:
        time (seconds) as a float
    """
    return float(duration.nanoseconds) / 1e9
