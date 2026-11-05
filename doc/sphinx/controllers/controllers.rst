.. _controllers:
.. title:: Controllers

Controllers
===========

A controller reads signals from :ref:`sensors` and drives actuators, for
example a PID controller that applies a torque to a joint to hold a measured
speed. Controllers are listed in the ``controllers`` array of an assembly
and are evaluated each time the equations of motion are set up, after the
sensors are updated and before the force modules. A controller may have
internal states, such as the integral of a PID controller's error, which are
integrated with the joint coordinates.

Keys of every controller:

.. shift-parameters:: shift/core/controller/base.cc

PID
---

``"type": "pid"`` reads a sensor and adds the control effort

.. math::

   u = k_p e + k_i \int e\, dt + k_d \dot e, \qquad e = \text{target} -
   \text{signal}

to the force on a joint: a force for a prismatic joint, a torque for a
revolute one. The integral of the error is an internal state. The
derivative term uses the rate of the sensor's signal, so it needs a sensor
that provides one, such as a joint position (whose rate is the joint's
velocity). The signs of the gains follow the signs of the joint and the
sensor: in the vehicle examples, driving the wheels forward turns their
joints negatively, so the gains are negative.

.. code:: javascript

   "sensors": [
     {"type": "joint", "name": "crank_speed", "joint": "crank_bearing",
      "quantity": "velocity"}
   ],
   "controllers": [
     {"type": "pid", "name": "speed_control", "kp": 0.5, "ki": 1.0,
      "kd": 0, "target": 80, "sensor": "crank_speed",
      "joint": "crank_bearing"}
   ]

.. shift-parameters:: shift/core/controller/pid.cc

Examples
--------

.. figure:: ../images/examples/vehicle_control.jpg
   :width: 60%
   :alt: vehicle_control.json at 2 s: PID controllers drive the front wheels to the target speed

   vehicle_control.json at 2 s: PID controllers drive the front wheels to the target speed

* :source:`examples/force_modules/belt.json`: speed control of an engine's
  crankshaft driving a belt.
* :source:`examples/vehicle/vehicle_control.json` and its variants: speed
  control of a car through its front wheels, with a steering rack position
  controller that these models leave inactive.

Inverse kinematics
------------------

``"type": "ik_7dof"`` is reserved for the inverse kinematics of 7-axis
robot arms; it is registered but does not act on the model yet.
