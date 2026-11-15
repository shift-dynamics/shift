.. _sensors:
.. title:: Sensors

Sensors
=======

A sensor reports information about the model as it runs; unlike a force
module it puts nothing back into the model. Sensors are listed in the
``sensors`` array of an assembly and come in two kinds:

* Signal sensors (``joint``, ``frame``) measure one scalar, such as a
  joint's angle or the speed of a frame, for controllers to read by name.
  They are updated each time the equations of motion are set up, after the
  kinematics and before the controllers and force modules. A signal sensor
  may also provide the time derivative of its signal, which the derivative
  term of a PID controller uses. Its outputs are the signal, ``value``, and
  the derivative, ``rate``, if it has one.
* Logging sensors (``body_load``, ``marker``) report several values at each
  output time, as their outputs. Their bodies are found by name anywhere in
  the model, so they can sit in an assembly of their own: a body of the
  sensor's own assembly first, then the body of that fully qualified name,
  then the one body of that name.

The loggers record the sensors' outputs (every one, or those of the sensors
a logger names; see :ref:`logging`): the CSV logger as columns named
``<sensor>__<output>`` and the HDF5 logger as datasets
``/sensors/<sensor>/<output>``, where ``<sensor>`` is the sensor's fully
qualified name (``<assembly>__<name>``).

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

Body load
---------

``"type": "body_load"`` reports the load that the force modules put on each
body in ``bodies``: contact, soil, springs and the rest, but not gravity or
the joints' forces. For a track it gives the load the links put on each
roller, idler and sprocket.

At each output time it reports ``<body>_fx``, ``_fy`` and ``_fz`` in N and
``<body>_mx``, ``_my`` and ``_mz`` in N m, the moment about the body's
origin, all in the fixed axes. The load is the one summed in the last
evaluation of the forces, which belongs to the integrator's last step and may
be a little past the output time. Bodies joined rigidly move as one group,
and the load is that of the whole group.

.. code:: javascript

   {"type": "body_load", "name": "roller_loads",
    "bodies": ["roller1", "roller2", "roller3", "roller4", "roller5"]}

.. shift-parameters:: shift/core/sensor/body_load.cc

Marker
------

``"type": "marker"`` reports the pose and velocity of frames on bodies, as
the markers of a motion-capture system would. ``frames`` lists ``[body,
frame]`` pairs. For each frame it reports, with ``<body>_<frame>`` as the
prefix:

* ``_x``, ``_y``, ``_z``: the position of the frame's origin (m), in the
  fixed axes;
* ``_qw``, ``_qx``, ``_qy``, ``_qz``: the frame's orientation as a unit
  quaternion, with ``qw`` kept positive;
* ``_roll``, ``_pitch``, ``_yaw``: the same orientation as angles (rad):
  turn the fixed axes about z by the yaw, then about the new y by the pitch,
  then about the newest x by the roll, :math:`R = R_z(\psi)\, R_y(\theta)\,
  R_x(\phi)`. With x forward and z down this is the usual vehicle
  convention: pitch is nose up;
* ``_vx``, ``_vy``, ``_vz``: the velocity of the frame's origin (m/s);
* ``_wx``, ``_wy``, ``_wz``: the angular velocity of the frame (rad/s).

``coordinate_system`` gives the velocities in the fixed axes (``global``,
the default) or in the axes of each frame (``local``).

.. code:: javascript

   {"type": "marker", "name": "shoes",
    "frames": [["link10", "origin"], ["link11", "origin"]],
    "coordinate_system": "local"}

.. shift-parameters:: shift/core/sensor/marker.cc

Examples
--------

* :source:`examples/force_modules/belt.json`: a joint sensor measures the
  crankshaft's speed for its speed controller.
* :source:`examples/vehicle/vehicle_control_5mps.json`: a frame sensor
  measures the car's forward speed in its own axes, and a joint sensor the
  position of the steering rack.
