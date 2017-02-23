Changelog
=========

0.5.0 (2017-02-21)
------------------
* ...

Indigo -> Kinetic Refactor
--------------------------

** Py Trees ROS API **

* subscribers : ``py_trees.subscribers.SubscriberHandler`` -> ``py_trees_ros.subscribers.Handler``
* subscribers : ``py_trees.subscribers.CheckSubscriberVaraible`` -> ``py_trees_ros.subscribers.CheckData``
* subscribers : ``py_trees.subscribers.WaitForSubscriberData`` -> ``py_trees_ros.subscribers.WaitForData``
* convert type : ``py_trees.converters.convert_type`` -> ``py_trees_ros.converters.behaviour_type_to_msg_constant``
* convert status : ``py_trees.converters.convert_status`` -> ``py_trees_ros.converters.status_enum_to_msg_constant``
* convert blackbox : ``py_trees.converters.convert_blackbox`` -> ``py_trees_ros.converters.blackbox_enum_to_msg_constant``
* refactor ros blackboard : ``py_trees.ros.blackboard`` -> ``py_trees_ros.blackboard.Exchange``

** Py Trees ROS Msgs API **

* ros blackboard service : ``~list_blackboard_variables`` -> ``~get_blackboard_variables``
* ros blackboard service : ``py_trees.msgs.srv.BlackboardVariables`` -> ``py_trees_msgs.srv.GetBlackboardVariables``
* ros blackboard service : ``~spawn_blackboard_watcher`` -> ``open_blackboard_watcher``
* ros blackboard service : ``py_trees.msgs.srv.SpawnBlackboardWatcher`` -> ``py_trees_msgs.srv.OpenBlackboardWatcher``
* ros blackboard service : ``~destroy_blackboard_watcher`` -> ``close_blackboard_watcher``
* ros blackboard service : ``py_trees.msgs.srv.DestroyBlackboardWatcher`` -> ``py_trees_msgs.srv.CloseBlackboardWatcher``

* refactor ros classes : ``py_trees`` -> ``py_trees.ros``

** Py Trees **

* refactor visitor classes : ``py_trees.trees`` -> ``py_trees.visitors``
