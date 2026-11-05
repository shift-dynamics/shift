.. _sensors:
.. title:: Sensors

Sensors
=======

A sensor measures one scalar signal of the model as it runs, such as a
joint's angle or the speed of a frame, for controllers to read by name.
Sensors are listed in the ``sensors`` array of an assembly and are updated
each time the equations of motion are set up, after the kinematics and
before the controllers and force modules. A sensor may also provide the
time derivative of its signal, which the derivative term of a PID
controller uses.

Keys of every sensor:

.. shift-parameters:: shift/core/sensor/base.cc

Joint
-----

``"type": "joint"`` measures a generalized coordinate (``"quantity":
"position"``, with its velocity as the rate) or a generalized velocity
(``"velocity"``) of a joint. ``index`` selects the degree of freedom of a
joint with several. Positions are available for joints whose coordinates
are not quaternions (not for the rotations of spherical and float joints).

.. code:: javascript

   {"type": "joint", "name": "crank_speed", "joint": "crank_bearing",
    "quantity": "velocity"}

.. shift-parameters:: shift/core/sensor/joint.cc

Frame
-----

``"type": "frame"`` measures one component of the position or the velocity
of a frame on a body. ``dof`` is ``Tx``, ``Ty`` or ``Tz`` (the linear
velocity, or the position, of the frame's origin) or ``Rx``, ``Ry`` or
``Rz`` (the angular velocity). ``coordinate_system`` gives the components in
the fixed axes (``global``) or in the axes of the measured frame
(``local``); positions are in the fixed axes only, with the linear velocity
as their rate.

.. code:: javascript

   {"type": "frame", "name": "chassis_speed",
    "body_frame_pair": ["chassis", "origin"], "quantity": "velocity",
    "coordinate_system": "local", "dof": "Tx"}

.. shift-parameters:: shift/core/sensor/frame.cc

Examples
--------

* :source:`examples/force_modules/belt.json`: a joint sensor measures the
  crankshaft's speed for its speed controller.
* :source:`examples/vehicle/vehicle_control_5mps.json`: a frame sensor
  measures the car's forward speed in its own axes, and a joint sensor the
  position of the steering rack.
