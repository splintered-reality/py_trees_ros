#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_ros/devel/LICENSE
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

import unique_id
import uuid_msgs.msg as uuid_msgs
import py_trees
import py_trees_msgs.msg as py_trees_msgs

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
    msg.own_id = unique_id.toMsg(behaviour.id)
    msg.parent_id = unique_id.toMsg(behaviour.parent.id) if behaviour.parent else uuid_msgs.UniqueID()
    msg.child_ids = [unique_id.toMsg(child.id) for child in behaviour.iterate(direct_descendants=True) if not child.id == behaviour.id]

    tip = behaviour.tip()
    # tip_id is empty if the behaviour is invalid or if it is a valid
    # leaf
    if tip is not None and tip != behaviour:
        msg.tip_id = unique_id.toMsg(tip.id)

    msg.type = behaviour_type_to_msg_constant(behaviour)
    msg.blackbox_level = blackbox_enum_to_msg_constant(behaviour.blackbox_level)
    msg.status = status_enum_to_msg_constant(behaviour.status)
    msg.message = behaviour.feedback_message

    return msg
