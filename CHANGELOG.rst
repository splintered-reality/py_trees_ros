=========
Changelog
=========

Forthcoming
-----------
* ...

1.2.0 (2019-10-02)
------------------
* [blackboards] sweeping changes to accomodate the new blackboards with tracking, `#109 <https://github.com/splintered-reality/py_trees_ros/pull/109>`_
* [backend] ensure tree modifications are published with an updated timestamp, `#100 <https://github.com/splintered-reality/py_trees_ros/pull/100>`_
* [behaviours] subscriber related behaviours now *require* qos_profile args, `#104 <https://github.com/splintered-reality/py_trees_ros/pull/104>`_
* [trees] ros parameterisation of the setup timeout, `#101 <https://github.com/splintered-reality/py_trees_ros/pull/101>`_
* [trees] make use of the new `DisplaySnapshotVisitor`, `#102 <https://github.com/splintered-reality/py_trees_ros/pull/102>`_

1.1.2 (2019-08-10)
------------------
* [utilities] permit discovery of multiples with find_topics, `#97 <https://github.com/splintered-reality/py_trees_ros/pull/97>`_

1.1.1 (2019-06-22)
------------------
* [tests] add missing tests/__init.py,  `#94 <https://github.com/splintered-reality/py_trees_ros/pull/94>`_
* [infra] add missing ros2topic dependency,  `#94 <https://github.com/splintered-reality/py_trees_ros/pull/94>`_

1.1.0 (2019-06-19)
------------------

* [actions] bugfix action client, don't cancel if not RUNNING
* [conversions] bugfix msg_to_behaviour for decorators
* [watchers] bugfix tree-watchers dot-graph to string functionality
* [watchers] bugfix missing tip in deserialised tree-watcher tree

1.0.0 (2019-04-28)
------------------

Stripped down and rebuilt for ROS2:

* [behaviours] the familiar subscriber and action client behaviours
* [blackboard] the exchange, mostly unmodified
* [infra] colcon build environment
* [trees] simpler communications, just one serialised tree snapshot, watchers do the rest
* [watchers] revamped 'blackboard' and new 'tree' watcher

What's missing:

* [logging] the basic mechanisms have moved to py_trees, the rosbag implementation is to come

0.5.13 (2017-05-28)
-------------------
* [doc] add many missing packages to satiate autodoc

0.5.9 (2017-04-16)
------------------
* [doc] add missing rqt-py-trees image
* [infra] bugfix missing install rule for mock sensors script

0.5.5 (2017-03-31)
------------------
* [infra] missed the py_trees exec dependency, fixed.

0.5.4 (2017-03-25)
------------------
* [docs] faq added
* [tutorials] 9 - bagging
* [infra] various dependency fixes for tests and autodoc
* [tests] fix broken subscrirber test

0.5.3 (2017-03-21)
------------------
* [tutorials] 8 - dynamic loading, insertion and execution
* [tutorials] 7 - docking, undocking, cancelling and recovery

0.5.2 (2017-03-19)
------------------
* [infra] add missing actionlib dependencies

0.5.1 (2017-03-19)
------------------
* [tutorials] 6 - context switching
* [tutorials] re-insert missing images

0.5.0 (2017-02-21)
------------------
* [docs] new and shiny index added
* [tutorials] qt dashboard support
* [tutorials] 5 - tree scanning added
* [tutorials] 4 - tree introspection added
* [tutorials] 3 - blackboards added
* [tutorials] 2 - battery low branch added
* [tutorials] 1 - data gathering added
* [mock] a mock robot for tutorials and testing
* [behaviours] action client, battery behaviours added
* [infra] refactoring for kinetic

Indigo -> Kinetic Changelist
----------------------------

**Py Trees ROS API**

* **subscribers**

  * py_trees.subscribers.SubscriberHandler -> py_trees_ros.subscribers.Handler
  * py_trees.subscribers.CheckSubscriberVariable -> py_trees_ros.subscribers.CheckData
  * py_trees.subscribers.WaitForSubscriberData -> py_trees_ros.subscribers.WaitForData
* **conversions**

  * py_trees.converters.convert_type -> py_trees_ros.converters.behaviour_type_to_msg_constant
  * py_trees.converters.convert_status -> py_trees_ros.converters.status_enum_to_msg_constant
  * py_trees.converters.convert_blackbox -> py_trees_ros.converters.blackbox_enum_to_msg_constant
* **blackboard**

  * py_trees.ros.blackboard -> py_trees_ros.blackboard.Exchange
  * ~list_blackboard_variables -> ~get_blackboard_variables
  * ~spawn_blackboard_watcher -> ~open_blackboard_watcher
  * ~destroy_blackboard_watcher -> ~close_blackboard_watcher
* **visitors** : classes moved from py_trees.trees -> py_trees_ros.visitors

**Py Trees ROS Msgs API**

* **blackboard services**

  * py_trees.msgs.srv.BlackboardVariables -> py_trees_msgs.srv.GetBlackboardVariables
  * py_trees.msgs.srv.SpawnBlackboardWatcher -> py_trees_msgs.srv.OpenBlackboardWatcher
  * py_trees.msgs.srv.DestroyBlackboardWatcher -> py_trees_msgs.srv.CloseBlackboardWatcher

**Py Trees**

* **program** : py-trees-render added
* **imposter** : bugfix to permit visitors to the children of a composite original
* **visitors** : py_trees.trees -> py_trees.visitors
