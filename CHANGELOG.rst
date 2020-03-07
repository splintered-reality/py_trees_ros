=========
Changelog
=========

Forthcoming
-----------
* ...

2.0.10 (2020-03-06)
-------------------
* [mock] server bugfixed to accommodate custom result handlers, `#154 <https://github.com/splintered-reality/py_trees_ros/pull/154>`_

2.0.9 (2020-03-05)
------------------
* [behaviours] default to periodic checks with warnings for action client timeouts, `#153 <https://github.com/splintered-reality/py_trees_ros/pull/153>`_

2.0.8 (2020-03-04)
------------------
* [behaviours] wait_for_server_timeout_sec in action clients, `#152 <https://github.com/splintered-reality/py_trees_ros/pull/152>`_
* [trees] avoid irregular snapshot streams by avoiding timing jitter, `#151 <https://github.com/splintered-reality/py_trees_ros/pull/151>`_

2.0.7 (2020-02-10)
------------------
* [programs] graph discovery timeouts 1.0->2.0s, `#149 <https://github.com/splintered-reality/py_trees_ros/pull/149>`_

2.0.6 (2020-02-06)
------------------
* [behaviours] action client goals from blackboard behaviour, `#148 <https://github.com/splintered-reality/py_trees_ros/pull/148>`_
* [behaviours] publish from blackboard behaviour, `#146 <https://github.com/splintered-reality/py_trees_ros/pull/146>`_

2.0.5 (2020-01-24)
------------------
* [serialisation] include parallel policy details

2.0.4 (2019-12-30)
------------------
* [blackboard] bugfix protection against activity stream not activitated for views, `#140 <https://github.com/splintered-reality/py_trees_ros/pull/140>`_
* [trees] bugfix toggle for activity client on reconfigure, `#139 <https://github.com/splintered-reality/py_trees_ros/pull/139>`_

2.0.3 (2019-12-26)
------------------
* [infra] add a marker file to assist with ros2cli discovery `#138 <https://github.com/splintered-reality/py_trees_ros/pull/138>`_

2.0.2 (2019-12-26)
------------------
* [trees] on-demand snapshot stream services, similar to blackboard streams, `#135 <https://github.com/splintered-reality/py_trees_ros/pull/135>`_, `#136 <https://github.com/splintered-reality/py_trees_ros/pull/136>`_

2.0.1 (2019-12-03)
------------------
* [trees] periodically publish snapshots, `#131 <https://github.com/splintered-reality/py_trees_ros/pull/131>`_,
* [trees] permit setup visitors, provide a default with timings, `#129 <https://github.com/splintered-reality/py_trees_ros/pull/129>`_
* [trees] bugfix non-infinite timeout arg getting ignored, `#129 <https://github.com/splintered-reality/py_trees_ros/pull/129>`_
* [trees] snapshot now publishing tree changed flag and key access info, `#128 <https://github.com/splintered-reality/py_trees_ros/pull/128>`_
* [utilities] deprecate myargv for rclpy.utilities.remove_ros_args, `#130 <https://github.com/splintered-reality/py_trees_ros/pull/130>`_

2.0.0 (2019-11-20)
------------------
* [blackboards] updated pretty printing to differentiate namespace vs attribute access, `#123 <https://github.com/splintered-reality/py_trees_ros/pull/123>`_
* [blackboards] api updates for namespaced clients, `#122 <https://github.com/splintered-reality/py_trees_ros/pull/122>`_,
* [tests] migrated tests from unittest to pytest
* [transforms] behaviours for writing to and broadcasting from the blackboard, `#121 <https://github.com/splintered-reality/py_trees_ros/pull/121>`_
* [transforms] add missing mocks and update to latest blackboard api, `#125 <https://github.com/splintered-reality/py_trees_ros/pull/125>`_

1.2.1 (2019-10-08)
------------------
* [trees] bugfix KeyError on publication of missing keys, `#118 <https://github.com/splintered-reality/py_trees_ros/pull/118>`_
* [utilities] a ros myargv stipper, a'la ROS1 style, until something is available upstream

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
