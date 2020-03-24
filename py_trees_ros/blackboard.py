#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/stonier/py_trees_ros/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
The :class:`Blackboard Exchange <py_trees_ros.blackboard.Exchange>` wraps a
:ref:`Blackboard <pt:blackboards-section>` with a ROS API to provide easy
introspection of a blackboard from outside the tree. This includes both
lazily publishing of the entire board when there's a change as well as
services to open windows onto parts of the blackboard for when the
entirity becomes too noisy to track.

You get this for free in the
:class:`ROS Behaviour Tree <py_trees_ros.trees.BehaviourTree>` manager and the
:ref:`py-trees-blackboard-watcher` command line utility provides a convenient
means of interacting with the watching services.
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

import cPickle

##############################################################################
# ROS Blackboard
##############################################################################

def pickle_warning_message():
    """
    Warning message for when the blackboard has unpicklable objects.
    
    Return:
       str: the warning message
    """
    msg = "You have objects on the blackboard that can't be pickled. "
    msg += "Any blackboard watchers will always receive updates, "
    msg += "regardless of whether the data changed or not."
    return msg

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
        self.publisher = rospy.Publisher(topic_name, std_msgs.String, latch=True, queue_size=2)

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
        try:
            current_pickle = cPickle.dumps(self.dict, -1)
            blackboard_changed = current_pickle != self.cached_dict
            self.cached_dict = current_pickle
        except (TypeError, cPickle.PicklingError):
            rospy.logwarn_once(pickle_warning_message())
            blackboard_changed = True
            self.cached_dict = {}

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

    Watching could be more simply enabled by just providing a *get* stye service
    on a particular variable(s), however that would introduce an asynchronous
    interrupt on the library's usage and subsequently, locking. Instead, a
    watcher service requests for a stream, i.e. a publisher to be created for private
    use that streams only a subsection of the blackboard to the user. Additional
    services are provided for closing the stream and listing the variables on the
    blackboard.

    .. seealso::

        :class:`~py_trees_ros.trees.BehaviourTree` (in which it is used) and
        :ref:`py-trees-blackboard-watcher` (working with the watchers).
    """
    _counter = 0
    """Incremental counter guaranteeing unique watcher names"""

    def __init__(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        """
        Internal handle to the blackboard. Users can utilise this, or create their own handles via the
        usual, e.g.

        .. code-block:: python

            my_blackboard = py_trees.blackboard.Blackboard()

        """
        self.cached_blackboard_dict = {}
        self.watchers = []
        self.publisher = None
        self.get_blackboard_variables_srv = None
        self.open_blackboard_watcher_srv = None
        self.close_blackboard_watcher_srv = None

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
                        self.exchange.setup(timeout)

        .. seealso:: This method is called in the way illustrated above in :class:`~py_trees_ros.trees.BehaviourTree`.
        """
        self.publisher = rospy.Publisher("~blackboard", std_msgs.String, latch=True, queue_size=2)
        self.get_blackboard_variables_srv = rospy.Service('~get_blackboard_variables', py_trees_srvs.GetBlackboardVariables, self._get_blackboard_variables_service)
        self.open_blackboard_watcher_srv = rospy.Service('~open_blackboard_watcher', py_trees_srvs.OpenBlackboardWatcher, self._open_blackboard_watcher_service)
        self.close_blackboard_watcher_srv = rospy.Service('~close_blackboard_watcher', py_trees_srvs.CloseBlackboardWatcher, self._close_blackboard_watcher_service)
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

    def _close_watcher(self, req):
        for (i, watcher) in enumerate(self.watchers):
            if watcher.topic_name == req.topic_name:
                watcher.publisher.unregister()
                del self.watchers[i]
                return True
        return False

    def _is_changed(self):
        try:
            current_pickle = cPickle.dumps(self.blackboard.__dict__, -1)
            blackboard_changed = current_pickle != self.cached_blackboard_dict
            self.cached_blackboard_dict = current_pickle
        except (TypeError, cPickle.PicklingError):
            rospy.logwarn_once(pickle_warning_message())
            blackboard_changed = True
            self.cached_blackboard_dict = {}
            
        return blackboard_changed

    def publish_blackboard(self, unused_tree=None):
        """
        Lazy string publishing of the blackboard (the contents of
        the blackboard can be of many and varied types, so string form is the only
        way to transmit this across a ros message) on the **~blackboard** topic.

        Typically you would call this from a tree custodian (e.g.
        :class:`py_trees_ros.trees.BehaviourTree`) after each and every tick.

        .. note:: Lazy: it will only do the relevant string processing if there are subscribers present.

        Args:
            unused_tree (:obj:`any`): if used as a post_tick_handler, needs the argument, but nonetheless, gets unused
        """
        if self.publisher is None:
            rospy.logerr("Blackboard Exchange: no publishers [hint: call setup() on the exchange]")
            return

        # publish blackboard
        if self.publisher.get_num_connections() > 0:
            if self._is_changed():
                self.publisher.publish("%s" % self.blackboard)

        # publish watchers
        if len(self.watchers) > 0:
            for (unused_i, sub_blackboard) in enumerate(self.watchers):
                if sub_blackboard.publisher.get_num_connections() > 0:
                    if sub_blackboard._is_changed():
                        sub_blackboard.publisher.publish("%s" % sub_blackboard)

    def _close_blackboard_watcher_service(self, req):
        result = self._close_watcher(req)
        return result

    def _get_blackboard_variables_service(self, req):
        nested_keys = self._get_nested_keys()
        return py_trees_srvs.GetBlackboardVariablesResponse(nested_keys)

    def _open_blackboard_watcher_service(self, req):
        if isinstance(req.variables, list):
            topic_name = rospy.names.resolve_name("~blackboard/watcher_" + str(Exchange._counter))
            Exchange._counter += 1
            watcher = _View(topic_name, req.variables)
            self.watchers.append(watcher)
        return py_trees_srvs.OpenBlackboardWatcherResponse(topic_name)

    def unregister_services(self):
        """
        Use this method to make sure services are cleaned up when you wish
        to subsequently discard the Exchange instance. This should be a
        fairly atypical use case however - first consider if there are
        ways to modify trees on the fly instead of destructing/recreating
        all of the peripheral machinery.
        """
        for srv in [self.get_blackboard_variables_srv, self.open_blackboard_watcher_srv, self.close_blackboard_watcher_srv]:
            if srv is not None:
                srv.shutdown()
