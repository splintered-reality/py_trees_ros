.. _py-trees-ros-tutorials-section:

Tutorials
=========

Dependencies
------------

The tutorials require installation of a few graphical dependencies that
would otherwise unnecessarily explode the requirements of the core package
and modules. e.g. for kinetic:

.. code-block:: bash

    sudo apt install ros-kinetic-rqt-py-trees ros-kinetic-rqt-reconfigure

The Mock Robot
--------------

The tutorials here all run atop a very simple :term:`mock` robot with the
following features:

* Battery
* LED Strip
* Move Base Action Server
* Rotation Action Server

This :term:`mock` robot could just as easily be replaced by a gazebo
simulated robot or even real robot with the same ROS API
abstraction layer.

The tutorials take care of launching the mock robot, but it can be
launched on its own with:

.. literalinclude:: ../launch/mock_robot.launch
   :language: xml
   :linenos:
   :caption: py_trees_ros/launch/mock_robot.launch

.. _tutorial-one:

Tutorial One - Data Gathering
-----------------------------

.. automodule:: py_trees_ros.tutorials.one
    :synopsis: data gathering with the battery to blackboard behaviour

.. _tutorial-two:

Tutorial Two - Battery Check
----------------------------

.. automodule:: py_trees_ros.tutorials.two
    :synopsis: check and react to a low battery state

.. _tutorial-three:

Tutorial Three - Blackboards!
-----------------------------

About
^^^^^

Tutorial three is a repeat of :ref:`tutorial-two`. The purpose of this
tutorial however is to introduce the publishers and services provided to
allow introspection of the blackboard from ROS. Publishers and services
are provided by the :class:`Blackboard Exchange <py_trees_ros.blackboard.Exchange>`
embedded in the :class:`ROS Behaviour Tree <py_trees_ros.trees.BehaviourTree>`
and interaction via the :ref:`py-trees-blackboard-watcher` command line utility.

Running
^^^^^^^

.. code-block:: bash

    # check the entire board
    $ rostopic echo /tree/blackboard
    # determine what you may stream
    $ py-trees-blackboard-watcher --list-variables
    # pull a simple variable
    $ py-trees-blackboard-watcher battery_low_warning
    # drill down to get a variable
    $ py-trees-blackboard-watcher battery/percentage

.. image:: images/tutorial-three-blackboards.gif

