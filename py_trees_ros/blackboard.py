#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/stonier/py_trees_ros/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Using ROS to introspect and monitor the blackboard or points of relevant
interested on the blackboard.
"""

##############################################################################
# Imports
##############################################################################

import operator
import py_trees
import py_trees_msgs.srv as py_trees_srvs
import rospy
import py_trees.console as console
import std_msgs.msg as std_msgs

from cPickle import dumps

##############################################################################
# ROS Blackboard
##############################################################################


class _View(object):
    """
    Utility class that enables tracking and publishing of relevant
    parts of the blackboard for a user. This is used by the
    :class:`~py_trees_ros.blackboard.Exchange` operator.

    Args:
        topic_name (:obj:`str`): name of the topic for the publisher
        attrs (:obj:`???`):
    """
    def __init__(self, topic_name, attrs):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.topic_name = topic_name
        self.attrs = attrs
        self.dict = {}
        self.cached_dict = {}
        self.publisher = rospy.Publisher("~" + topic_name, std_msgs.String, latch=True, queue_size=2)

    def _update_sub_blackboard(self):
        for attr in self.attrs:
            if '/' in attr:
                check_attr = operator.attrgetter(".".join(attr.split('/')))
            else:
                check_attr = operator.attrgetter(attr)
            try:
                value = check_attr(self.blackboard)
                self.dict[attr] = value
            except AttributeError:
                pass

    def _is_changed(self):
        self._update_sub_blackboard()
        current_pickle = dumps(self.dict, -1)
        blackboard_changed = current_pickle != self.cached_dict
        self.cached_dict = current_pickle

        return blackboard_changed

    def __str__(self):
        s = ""
        max_length = 0
        for k in self.dict.keys():
            max_length = len(k) if len(k) > max_length else max_length
        keys = sorted(self.dict)
        for key in keys:
            value = self.dict[key]
            if value is None:
                value_string = "-"
                s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "%s" % (value_string) + console.reset + "\n"
            else:
                lines = ("%s" % value).split('\n')
                if len(lines) > 1:
                    s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ":\n"
                    for line in lines:
                        s += console.yellow + "    %s" % line + console.reset + "\n"
                else:
                    s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "%s" % (value) + console.reset + "\n"
        return s.rstrip()  # get rid of the trailing newline...print will take care of adding a new line


class Exchange(object):
    """
    Establishes ros communications around a :class:`~py_trees.blackboard.Blackboard`
    that enable users to introspect or watch relevant parts of the blackboard.

    ROS Publishers:
        * **~blackboard** (:class:`std_msgs.msg.String`)

          * streams (string form) the contents of the entire blackboard as it updates

    ROS Services:
        * **~get_blackboard_variables** (:class:`py_trees_msgs.srv.GetBlackboardVariables`)

          * list all the blackboard variable names (not values)
        * **~open_blackboard_watcher** (:class:`py_trees_msgs.srv.OpenBlackboardWatcher`)

          * request a publisher to stream a part of the blackboard contents
        * **~close_blackboard_watcher** (:class:`py_trees_msgs.srv.CloseBlackboardWatcher`)

          * close a previously opened watcher

    .. seealso:: This class is utilised in the :class:`~py_trees_ros.trees.BehaviourTree` class.
    """
    def __init__(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.cached_blackboard_dict = {}
        self.sub_blackboards = []
        self.publisher = None

    def setup(self, timeout):
        """
        This is where the ros initialisation of publishers and services happens. It is kept
        outside of the constructor for the same reasons that the familiar py_trees
        :meth:`~py_trees.trees.BehaviourTree.setup` method has - to enable construction
        of behaviours and trees offline (away from their execution environment) so that
        dot graphs and other visualisations of the tree can be created.

        Args:
             timeout (:obj:`float`): time to wait (0.0 is blocking forever)

        Return:
            :obj:`bool`: suceess or failure of the operation

        Examples:

            It is expected that users will use this in their own customised tree custodian:

            .. code-block:: python

                class MyTreeManager(py_trees.trees.BehaviourTree):

                    def __init__(self):
                        pass

                    def setup(self, timeout):
                        super(MyTreeManager, self).setup(timeout)
                        self.exchange = py_trees_ros.blackboard.Exchange()

        .. seealso:: This method is called in the way illustrated above in :class:`~py_trees_ros.trees.BehaviourTree`.
        """
        self.publisher = rospy.Publisher("~blackboard", std_msgs.String, latch=True, queue_size=2)
        rospy.Service('~list_blackboard_variables', py_trees_srvs.BlackboardVariables, self._list_blackboard_variables_service)
        rospy.Service('~spawn_blackboard_watcher', py_trees_srvs.SpawnBlackboardWatcher, self._spawn_blackboard_watcher_service)
        rospy.Service('~destroy_blackboard_watcher', py_trees_srvs.DestroyBlackboardWatcher, self._destroy_blackboard_watcher_service)
        return True

    def _get_nested_keys(self):
        variables = []

        def inner(v, k):
            for attr in dir(type(v)):
                if not isinstance(v, (bool, list, str, int, float)):
                        if not attr.startswith("_"):
                            value = getattr(v, attr)
                            if not callable(value):
                                if not attr.isupper():
                                    variables.append(k + "/" + attr)
                                    inner(value, k + "/" + attr)

        for key in sorted(self.blackboard.__dict__):
            variables.append(key)
            inner(self.blackboard.__dict__[key], key)

        return variables

    def _initialize_sub_blackboard(self, attrs, topic_name=None):
        if isinstance(attrs, list):
            if not topic_name:
                topic_name = "sub_blackboard_" + str(len(self.sub_blackboards))

            sub_blackboard = _View(topic_name, attrs)
            self.sub_blackboards.append(sub_blackboard)

        return topic_name

    def _shutdown_sub_blackboard(self, req):
        for (i, sub_blackboard) in enumerate(self.sub_blackboards):
            if sub_blackboard.topic_name == req.topic_name:
                sub_blackboard.publisher.unregister()
                del self.sub_blackboards[i]
                return True
        return False

    def _is_changed(self):
        current_pickle = dumps(self.blackboard.__dict__, -1)
        blackboard_changed = current_pickle != self.cached_blackboard_dict
        self.cached_blackboard_dict = current_pickle
        return blackboard_changed

    def publish_blackboard(self, tree):
        """
        Lazy string publishing of the blackboard (the contents of
        the blackboard can be of many and varied types, so string form is the only
        way to transmit this across a ros message) on the **~blackboard** topic.

        Typically you would call this from a tree custodian (e.g.
        :class:`py_trees_ros.trees.BehaviourTree`) after each and every tick.

        .. note:: Lazy: it will only do the relevant string processing if there are subscribers present.
        """
        if self.publisher is None:
            rospy.logerr("Blackboard Exchange: no publishers [hint: call setup() on the exchange]")
            return

        # publish blackboard
        if self.publisher.get_num_connections() > 0:
            if self._is_changed():
                self.publisher.publish("%s" % self.blackboard)

        # publish sub_blackboards
        if len(self.sub_blackboards) > 0:
            for (unused_i, sub_blackboard) in enumerate(self.sub_blackboards):
                if sub_blackboard.publisher.get_num_connections() > 0:
                    if sub_blackboard._is_changed():
                        sub_blackboard.publisher.publish("%s" % sub_blackboard)

    def _destroy_blackboard_watcher_service(self, req):
        result = self._shutdown_sub_blackboard(req)
        return result

    def _list_blackboard_variables_service(self, req):
        nested_keys = self._get_nested_keys()
        return py_trees_srvs.BlackboardVariablesResponse(nested_keys)

    def _spawn_blackboard_watcher_service(self, req):
        topic_name = self._initialize_sub_blackboard(req.variables)
        if topic_name:
            absolute_topic_name = rospy.get_name() + "/" + topic_name
        else:
            absolute_topic_name = None
        return py_trees_srvs.SpawnBlackboardWatcherResponse(absolute_topic_name)
