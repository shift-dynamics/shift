.. title:: Electric Motor

Electric Motor
==============

``"type": "electric_motor"`` is a motor on a revolute joint: its stator is
the joint's parent, its rotor the child, and its torque acts on the joint.
It is a DC motor, whose winding of resistance :math:`R` and inductance
:math:`L` carries the current :math:`i` against the back voltage of the
rotor turning at :math:`\omega`, the rate of the joint:

.. math::

   v = R i + L \frac{di}{dt} + k \omega, \qquad \tau = k i

The torque constant :math:`k` is, in SI units, also the back voltage per
rotor speed (N m/A = V s/rad). The same equations describe a brushless DC or
permanent magnet synchronous motor under field-oriented control that keeps
the current along the magnet's axis at zero: :math:`i` is then the current
across the axis, :math:`k = \tfrac{3}{2} p \psi` for :math:`p` pole pairs
and the magnet's flux :math:`\psi`, and :math:`R` and :math:`L` are those of
a phase.

The drive
---------

``drive`` says what the electronics between the supply and the winding
control:

``"current"`` (the default)
   The drive makes the current follow the ``current`` input at once, as a
   drive whose current loop is much faster than the mechanism does. The
   current is limited to ``current_limit``, and then to what the supply can
   drive against the back voltage, :math:`(\pm v_s - k\omega)/R` for the
   ``supply_voltage`` :math:`v_s`. The voltage limit comes last: a rotor
   turned faster than the supply allows pushes current back into the supply
   whatever the current limit. The inductance is not used.

``"voltage"``
   The drive applies the ``voltage`` input, limited to :math:`\pm v_s`;
   ``current_limit`` is not used. With an inductance, the current is an
   internal state, which starts at 0; without one, the current is
   :math:`(v - k\omega)/R`. The inductance's time constant :math:`L/R` is
   often a millisecond or less, much shorter than the mechanism's, and the
   integrator takes steps short enough to follow it: leave the inductance
   out unless the electrical response matters.

Without a ``supply_voltage`` or a ``current_limit`` there is no limit.

Inputs
------

``current`` (A) and ``voltage`` (V) are the motor's inputs: a
:ref:`PID controller <controllers>` sets one with ``"component"`` and
``"input"``, a :ref:`schedule <schedule>` can set one, or the model file
gives a constant. Only the input of the drive is used; giving the other in
the model file, or a controller setting it, is an error, as is two
controllers setting the same input.

.. code:: javascript

   "controllers": [
     {"type": "pid", "name": "speed_control", "kp": 2,
      "sensor": "spindle_speed", "component": "motor", "input": "current"}
   ]

The PID controller has no anti-windup: its integral grows while the drive
holds the current limit, and the speed then overshoots its target. Leave
``ki`` out where the limit is reached.

What it leaves to the rest of the model
---------------------------------------

The rotor's inertia is that of its body. Friction and drag of the rotor
are the joint's (its ``damping_coefficients``) or a
:doc:`joint_friction <joint-forces>` module's. A gearbox between the rotor's
shaft and the load's is a :doc:`gear_mesh <gear-mesh>`.

JSON
----

.. literalinclude:: ../../../examples/force_modules/electric_motor.json
   :language: javascript
   :caption: examples/force_modules/electric_motor.json

.. shift-parameters:: shift/core/force_module/electric_motor.cc

Outputs
-------

Logged as columns ``<assembly>__<module>__<output>`` (see :ref:`logging`)
and read by other components with ``GetOutput``:

.. list-table::
   :header-rows: 1

   * - output
     - meaning
   * - ``winding_current``
     - the current in the winding, A
   * - ``terminal_voltage``
     - the voltage across the winding, V
   * - ``back_voltage``
     - :math:`k\omega`, V
   * - ``torque``
     - the torque on the joint, N m
   * - ``speed``
     - the rotor's speed relative to the stator, rad/s
   * - ``electrical_power``
     - :math:`v i`, the power from the supply, W; negative when the motor
       brakes and returns power to it
   * - ``mechanical_power``
     - :math:`\tau\omega`, the power to the rotor, W

The difference of the two powers is the loss in the winding, :math:`R i^2`,
and, with an inductance, the rate of the energy stored in it.

Example
-------

:source:`examples/force_modules/electric_motor.json`: a spindle on a motor
with a 48 V supply and a 20 A current limit. A proportional speed controller
sets the current, and a schedule steps the target to 310 rad/s and then to
150 rad/s. The spindle accelerates at the current limit, settles 2 rad/s
below the target (the error that, times the gain, gives the current for the
bearings' drag), and brakes at the limit, returning up to 0.88 kW to the
supply.

Tests (:source:`test/test_electric_motor.cc`) compare the speed with closed
forms: the exponential rise of the voltage drive to the no-load speed
:math:`v/k` with the time constant :math:`J R / k^2`, the second order
response with inductance, the constant torque of the current drive, and its
current and voltage limits.

Not yet modelled: field weakening above base speed, induction motors, the
battery or DC bus with its resistance and state of charge, losses in the
iron, and the winding's temperature.
