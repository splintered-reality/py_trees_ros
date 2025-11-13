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
Behaviours for ROS services.
"""

##############################################################################
# Imports
##############################################################################

import typing
import uuid
from abc import ABC, abstractmethod

import py_trees
import rclpy.callback_groups

from . import exceptions

##############################################################################
# Behaviours
##############################################################################


class FromBlackboard(py_trees.behaviour.Behaviour):
    """
    A service client interface that draws requests from the blackboard. The
    lifecycle of this behaviour works as follows:

    * :meth:`initialise`: check blackboard for a request and send
    * :meth:`update`: if a request was sent, monitor progress
    * :meth:`terminate`: if interrupted while running, send a cancel request

    As a consequence, the status of this behaviour can be interpreted as follows:

    * :data:`~py_trees.common.Status.FAILURE`: no request was found to send,
      the server was not ready, or it failed while executing
    * :data:`~py_trees.common.Status.RUNNING`: a request was sent and is still
      executing
    * :data:`~py_trees.common.Status.SUCCESS`: sent request has completed with success

    To block on the arrival of a request on the blackboard, use with the
    :class:`py_trees.behaviours.WaitForBlackboardVariable` behaviour. e.g.

    Args:
        name: name of the behaviour
        service_type: spec type for the service
        service_name: where you can find the service
        key_request: name of the key for the request on the blackboard
        key_response: optional name of the key for the response on the blackboard (default: None)
        wait_for_server_timeout_sec: use negative values for a blocking but periodic check (default: -3.0)
        callback_group: callback group for the service client

    .. note::
       The default setting for timeouts (a negative value) will suit
       most use cases. With this setting the behaviour will periodically check and
       issue a warning if the server can't be found. Actually aborting the setup can
       usually be left up to the behaviour tree manager.
    """
    def __init__(self,
                 name: str,
                 service_type: typing.Any,
                 service_name: str,
                 key_request: str,
                 key_response: str | None = None,
                 wait_for_server_timeout_sec: float = -3.0,
                 callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None,
                 ):
        super().__init__(name)
        self.service_type = service_type
        self.service_name = service_name
        self.wait_for_server_timeout_sec = wait_for_server_timeout_sec
        self.callback_group = callback_group

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="request",
            access=py_trees.common.Access.READ,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key_request)
        )
        self.write_response_to_blackboard = key_response is not None
        if self.write_response_to_blackboard:
            self.blackboard.register_key(
                key="response",
                access=py_trees.common.Access.WRITE,
                # make sure to namespace it if not already
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key_response)
            )

        self.node = None
        self.service_client = None

    def setup(self, **kwargs):
        """
        Setup the service client and ensure it is available.

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
            :class:`~py_trees_ros.exceptions.TimedOutError`: if the service server could not be found
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.service_client = self.node.create_client(
            srv_type=self.service_type,
            srv_name=self.service_name,
            callback_group=self.callback_group,
        )

        result = None
        if self.wait_for_server_timeout_sec > 0.0:
            result = self.service_client.wait_for_service(timeout_sec=self.wait_for_server_timeout_sec)
        elif self.wait_for_server_timeout_sec == 0.0:
            result = True # don't wait and don't check if the server is ready
        else:
            iterations = 0
            period_sec = -1.0 * self.wait_for_server_timeout_sec
            while not result:
                iterations += 1
                result = self.service_client.wait_for_service(timeout_sec=period_sec)
                if not result:
                    self.node.get_logger().warning(
                        "waiting for service server ... [{}s][{}][{}]".format(
                            iterations * period_sec,
                            self.node.resolve_service_name(self.service_name),
                            self.qualified_name
                        )
                    )
        if not result:
            self.feedback_message = "timed out waiting for the server [{}]".format(
                self.node.resolve_service_name(self.service_name)
            )
            self.node.get_logger().error("{}[{}]".format(self.feedback_message, self.qualified_name))
            raise exceptions.TimedOutError(self.feedback_message)
        else:
            self.feedback_message = "... connected to service server [{}]".format(
                self.node.resolve_service_name(self.service_name)
            )
            self.node.get_logger().info("{}[{}]".format(self.feedback_message, self.qualified_name))

    def initialise(self):
        """
        Reset the internal variables and kick off a new request.
        """
        self.logger.debug("{}.initialise()".format(self.qualified_name))

        # initialise some temporary variables
        self.service_future = None

        try:
            if self.service_client.service_is_ready():
                self.service_future = self.service_client.call_async(self.blackboard.request)
        except TypeError:
            expected_type = self.service_type.Request.__name__
            received_type = type(self.blackboard.request).__name__
            self.logger.error(f"Received a request of type <{received_type}> instead of <{expected_type}>")
        except KeyError as e:
            self.logger.error(f"{e}")
        # self.service_future will be None on either exception, and update will return FAILURE

    def update(self):
        """
        Check only to see whether the underlying service server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.

        Returns:
            :class:`py_trees.common.Status`
        """
        self.logger.debug("{}.update()".format(self.qualified_name))

        if self.service_future is None:
            # either there was no request on the blackboard, the request's type
            # was wrong, or the service server wasn't ready
            return py_trees.common.Status.FAILURE
        elif not self.service_future.done():
            # service has been called but hasn't yet returned a result
            return py_trees.common.Status.RUNNING
        else:
            # service has succeeded; get the result
            self.response = self.service_future.result()
            if self.write_response_to_blackboard:
                self.blackboard.response = self.response
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status):
        """
        If running and the current request has not already succeeded, cancel it.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        if (self.service_future is not None) and (not self.service_future.done()):
            self.service_client.remove_pending_request(self.service_future)

    def shutdown(self):
        """
        Clean up the service client when shutting down.
        """
        self.service_client.destroy()


class FromConstant(FromBlackboard):
    """
    Convenience version of the service client that only ever sends the
    same goal.

    .. see-also: :class:`py_trees_ros.service_clients.FromBlackboard`

    Args:
        name: name of the behaviour
        name: name of the behaviour
        service_type: spec type for the service
        service_name: where you can find the service
        service_request: the request to send
        key_response: optional name of the key for the response on the blackboard (default: None)
        wait_for_server_timeout_sec: use negative values for a blocking but periodic check (default: -3.0)
        callback_group: callback group for the service client

    .. note::
       The default setting for timeouts (a negative value) will suit
       most use cases. With this setting the behaviour will periodically check and
       issue a warning if the server can't be found. Actually aborting the setup can
       usually be left up to the behaviour tree manager.
    """
    def __init__(self,
                 name: str,
                 service_type: typing.Any,
                 service_name: str,
                 service_request: typing.Any,
                 key_response: str | None = None,
                 wait_for_server_timeout_sec: float = -3.0,
                 callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None,
                 ):
        unique_id = uuid.uuid4()
        key_request = "/request_" + str(unique_id)
        super().__init__(
            service_type=service_type,
            service_name=service_name,
            key_request=key_request,
            key_response=key_response,
            name=name,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            callback_group=callback_group,
        )
        # parent already instantiated a blackboard client
        self.blackboard.register_key(
            key=key_request,
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.set(name=key_request, value=service_request)


class AttributesFromBlackboard(FromBlackboard):
    """
    Convenience version of the service client that creates a request with fields read from BB.

    Args:
        name (str): Name of the behaviour
        service_type (typing.Any): Type of the service
        service_name (str): Endpoint of the service
        request_fields (dict[str, typing.Any]): Fields of the request mapped to blackboard variables
        wait_for_server_timeout_sec (float, optional): Wait timeout for the service. Defaults to -3.0.
        callback_group: callback group for the service client
    """

    def __init__(self,
                 name: str,
                 service_type: typing.Any,
                 service_name: str,
                 request_fields: dict[str, typing.Any],
                 wait_for_server_timeout_sec: float = -3.0,
                 callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None,
                 ):
        unique_id = uuid.uuid4()
        self.key_request = "/request_" + str(unique_id)
        super().__init__(
            service_type=service_type,
            service_name=service_name,
            key_request=self.key_request,
            name=name,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            callback_group=callback_group,
        )
        # The parent constructor already instantiated a blackboard client
        self.request_fields = request_fields
        for bb_key in self.request_fields.values():
            self.blackboard.register_key(
                key=bb_key,
                access=py_trees.common.Access.READ,
            )
        self.blackboard.register_key(
            key=self.key_request,
            access=py_trees.common.Access.WRITE,
        )

    def initialise(self):
        """
        Read from the blackboard the attributes to create a Request object; if succeeded, write it to the blackboard.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        request_attributes = {request_attr: self.blackboard.get(bb_key) for request_attr, bb_key in
                              self.request_fields.items()}
        request = self.service_type.Request(**request_attributes)
        self.blackboard.set(name=self.key_request, value=request)

        super().initialise()


class FromCallback(FromBlackboard, ABC):
    """
    Convenience version of the service client that obtains the request from a callback implemented
    by derived classes.

    .. see-also: :class:`py_trees_ros.service_clients.FromBlackboard`

    Args:
        name: name of the behaviour
        name: name of the behaviour
        service_type: spec type for the service
        service_name: where you can find the service
        key_response: optional name of the key for the response on the blackboard (default: None)
        wait_for_server_timeout_sec: use negative values for a blocking but periodic check (default: -3.0)
        callback_group: callback group for the service client

    .. note::
       The default setting for timeouts (a negative value) will suit
       most use cases. With this setting the behaviour will periodically check and
       issue a warning if the server can't be found. Actually aborting the setup can
       usually be left up to the behaviour tree manager.
    """
    def __init__(self,
                 name: str,
                 service_type: typing.Any,
                 service_name: str,
                 key_response: str | None = None,
                 wait_for_server_timeout_sec: float = -3.0,
                 callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None,
                 ):
        unique_id = uuid.uuid4()
        self.key_request = "/request_" + str(unique_id)
        super().__init__(
            service_type=service_type,
            service_name=service_name,
            key_request=self.key_request,
            key_response=key_response,
            name=name,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            callback_group=callback_group,
        )
        # parent already instantiated a blackboard client
        self.blackboard.register_key(
            key=self.key_request,
            access=py_trees.common.Access.WRITE,
        )

    def initialise(self):
        """
        Call derived class `get_request` method and write the returned service request to the blackboard.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        self.blackboard.set(name=self.key_request, value=self.get_request())

        super().initialise()

    @abstractmethod
    def get_request(self):
        pass
