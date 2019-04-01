.. _faq-section-label:

Frequently Asked Questions
==========================

ROS related frequently asked questions.

.. seealso:: The :ref:`pt:faq-section-label` from the py_trees package.

Parameter/Remap Proliferation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can imagine once you have 50+ re-usable behaviours in a tree for move_base, odometry, ...
that the need for remaps in the behaviour tree launcher will become exceedingly large. In these
situations it is more convenient to provide a single point of call for behaviour tree configuration. Load all
your remappings into a single namespace on the parameter server and apply them as you instantiate
behaviours. A typical tree namespace on the parameter server might look something like:

.. todo:: Update with ROS2 instructions

.. code-block:: python

   /tree/topics/odom                 /gopher/odom
   /tree/topics/pose                 /gopher/pose
   /tree/services/get_global_costmap /move_base/global/get_costmap
   /tree/dyn_reconf/max_speed        /trajectory_controller/max_speed

With an example instantiation of a move base client behaviour:

.. code-block:: python

   odometry_topic=rospy.get_param("~topics/odom", "/odom")
   pose_topic=rospy.get_param("~topics/pose", "/pose")
   move_base = some_navi_package.MoveBaseClient(odometry_topic, pose_topic)


Continuous Tick-Tock?
^^^^^^^^^^^^^^^^^^^^^

Not at all - you can set your own pace. This can be useful if you wish to tick only when
an external trigger is received (a common trick to minimise cpu usage in games).

.. code-block:: python

   ...
   while rclpy.ok():
       rclpy.spin_once(timeout_sec=0.1)
       if some_external_trigger:
           tree.tick_once()

or even better, apply conditions to block the tick.

Control-Level Decision Making
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Our first use case never intended the behaviour trees for use by control level applications.
So it was surprising when the control engineers moved their state machines from for
handling, e.g. docking, elevator entry, navigation recovery to the trees.
Note that none of these require low-latency reactivity.

In hindsight, this makes good sense.
Prior to the behaviour trees there was a different state machine implementation in each
controller - having just one decision making engine with shared code and design patterns
is more efficient. It also frees them of the need to bring dependencies to other
subsystems that need to work in concert with their control algorithm 
(e.g. the LED or sound notification subsystems). Such co-ordination is most easily managed
at the tree level. 
