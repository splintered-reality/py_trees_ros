=========
Changelog
=========

Forthcoming
-----------
* ...

0.6.1 (2020-08-03)
------------------
* [infra] python-setuptools -> python3-setuptools dependency fix

0.6.0 (2020-08-03)
------------------
* [infra] python3/noetic fixes, `#167 <https://github.com/splintered-reality/py_trees_ros/pull/167>`_

0.5.21 (2020-06-06)
-------------------
* [trees] option for bagging, , `#159 <https://github.com/splintered-reality/py_trees_ros/pull/159>`_

0.5.20 (2020-03-23)
-------------------
* [infra] don't fail for the runtime if qt is not found

0.5.19 (2020-03-23)
-------------------
* [blackboards] log a one-shot warning instead of exceptions when pickle fails, `#157 <https://github.com/splintered-reality/py_trees_ros/pull/157>`_

0.5.18 (2019-03-23)
-------------------
* [infra] merge kinetic and melodic release branches

0.5.17 (2019-02-20)
-------------------
* [actions] remove redundant prints in the action behaviour

0.5.16 (2019-02-02)
-------------------
* [trees] added serialisation to Decorator

0.5.15 (2019-02-02)
-------------------
* [programs] blackboard-watcher can operate on the entire blackboard
* [programs] tree-watcher added

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
