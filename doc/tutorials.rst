.. _py-trees-ros-tutorials-section:

Tutorials
=========

The tutorials here all run atop a very simple :term:`mock` robot with the
following features:

* Battery
* LED Strip
* Move Base Action Server
* Rotation Action Servver

and is launched with:

.. literalinclude:: ../launch/mock_robot.launch
   :language: xml
   :linenos:
   :caption: py_trees_ros/launch/mock_robot.launch

This :term:`mock` robot could just as easily be replaced by a gazebo
simulated robot or even real robot with the same ROS API
abstraction layer.

.. _tutorial-one:

Tutorial One - Data Gathering
-----------------------------

.. automodule:: py_trees_ros.tutorials.one
    :synopsis: data gathering with the battery to blackboard behaviour

.. _tutorial-one:

Tutorial Two - Battery Check
----------------------------

.. automodule:: py_trees_ros.tutorials.two
    :synopsis: check and react to a low battery state
