.. _controllers:
.. title:: Controllers

Controllers
===========

A controller reads signals from :ref:`sensors` and drives actuators, for
example a PID controller that applies a torque to a joint to hold a measured
speed. Controllers are listed in the ``controllers`` array of an assembly
and are evaluated each time the equations of motion are set up, after the
sensors are updated and before the force modules (a schedule, below, before
the other controllers too). A controller may have
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

Instead of a force on a joint, the effort can set an *input* of another
component: a number that the component reads each time it
is evaluated, such as the valve command of a hydraulic cylinder. Give
``component`` and ``input`` in place of ``joint``. The controller finds the
component by its name when the model loads; where two kinds of component
share the name (a sensor and a force module, say), give the kind too, as
``"force_modules/boom_cylinder"``. A component of another assembly is
named by its fully qualified name, the names of its assemblies from the top
and its own, joined by two underscores: ``"arm__boom_cylinder"`` for the
cylinder of assembly ``arm``, ``"machine__arm__boom_cylinder"`` where ``arm``
is nested in ``machine`` (with the kind before it if needed, as
``"force_modules/arm__boom_cylinder"``). An input is set by one controller at
most, so a model in which a PID controller and a schedule both set the same
input does not load. This holds the extension of a cylinder at 0.1 m:

.. code:: javascript

   "sensors": [
     {"type": "joint", "name": "boom_extension", "joint": "boom_slide",
      "quantity": "position"}
   ],
   "controllers": [
     {"type": "pid", "name": "boom_position", "kp": 20, "target": 0.1,
      "sensor": "boom_extension", "component": "boom_cylinder",
      "input": "command"}
   ]

The inputs of a component are listed with its parameters, marked "Input.".
The PID controller's own ``target`` is an input, which a schedule (or
another controller) can set, and its ``error`` and ``effort`` are outputs,
which the loggers record (see :ref:`logging`); a schedule's output is its
``value``.

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

.. _schedule:

Schedule
--------

``"type": "schedule"`` changes parameters of other components as the model
runs, following a list of control points in time: the opening of a valve, the
target speed of a PID controller, the friction of a belt, the height of the
ground. It reads no sensor and applies no force itself; it sets numbers, which
the other components then use in their own calculations. Use it where a model
needs an input that changes in time, such as a valve command or a speed
profile, without writing a script.

.. code:: javascript

   "controllers": [
     {"type": "schedule", "name": "boom_valve", "interpolation": "cubic",
      "points": [0, 0, 0.2, 0, 0.5, -0.4, 1.8, -0.4, 2.1, 0],
      "targets": ["force_modules/boom_cylinder_1/command",
                  "force_modules/boom_cylinder_2/command"]}
   ]

Control points
^^^^^^^^^^^^^^

``points`` is a list of pairs :math:`[t_0, v_0, t_1, v_1, \dots]`: a time and
the value that the schedule has at that time. The times must increase. Before
the first point the schedule holds the first value, and after the last it
holds the last, so a schedule with one point sets a constant.

Interpolation
^^^^^^^^^^^^^

``interpolation`` says what the value does between the points. The default is
``"linear"``.

``"linear"``
   Straight lines between the points. The value is continuous, but its rate of
   change jumps at each point.

``"cubic"``
   A cubic curve in each interval, with the slope at each point chosen so that
   the curve never overshoots the points: it rises where the points rise and
   is level where they are level (the monotone cubic of Fritsch and Carlson
   :cite:`fritsch1980`). Both the value and its rate of change are continuous,
   so it is the gentlest input for the solver and for the machine.

``"step"``
   The value of the last point passed, so that the value jumps at each point.
   A jump in a force or a command is a jump in the equations of motion, and
   the integrator takes small steps around it to find it. If a smooth change
   is acceptable, give ``transition_time``: each jump is then made over that
   time as a smooth cubic from the old value to the new one. It must be shorter
   than the time between any two points. The default is 0, a true jump.

For example, with the points :math:`[0, 0, 1, 2, 3, 2, 4, 10]` (the value
rises to 2 at time 1, stays there until time 3, and rises to 10 at time 4) the
values are:

.. list-table::
   :header-rows: 1

   * - time
     - -1
     - 0
     - 0.5
     - 1
     - 2
     - 3
     - 3.5
     - 4
     - 5
   * - ``linear``
     - 0
     - 0
     - 1
     - 2
     - 2
     - 2
     - 6
     - 10
     - 10
   * - ``cubic``
     - 0
     - 0
     - 1.333
     - 2
     - 2
     - 2
     - 4.667
     - 10
     - 10
   * - ``step``
     - 0
     - 0
     - 0
     - 2
     - 2
     - 2
     - 2
     - 10
     - 10

The cubic stays at 2 from time 1 to time 3, where the points are level, while
a curve through the points that was free to swing (a spline) would dip and rise
between them.

Targets
^^^^^^^

``targets`` lists the parameters to set, each by its path from the assembly
that holds the schedule. The path is made of the keys of the model file:

.. list-table::
   :header-rows: 1

   * - to set
     - path
   * - a parameter of a force module
     - ``"force_modules/<name>/<parameter>"``
   * - a parameter of a controller
     - ``"controllers/<name>/<parameter>"``
   * - an entry of a list of numbers (the second friction coefficient of a belt)
     - ``"force_modules/belt/friction_coefficient/1"``
   * - an entry of a vector (the third entry of a vector with three)
     - ``"force_modules/soil/shoe_center/2"``
   * - a parameter of an assembly inside this one
     - ``"assemblies/<name>/force_modules/<name>/<parameter>"``

Entries are counted from 0. One schedule can set several targets to the same
value, as for the two cylinders of a boom on one valve. A target must be a number,
an entry of a list of numbers, or an entry of a vector.

A schedule can set only parameters that their component reads at every step.
Many components compute values from their parameters only when the model
loads (the matrices of a bushing, the soil of a tire), and changing those
later would have no effect, so they are not allowed: the model does not load, and
the message names the target and says that it cannot be changed while the
model runs. The same happens for a path that does not exist, a list or
vector without its index, an index that is out of range, and a path that
ends at an object and not at a number. A component's inputs (marked "Input."
in its list of parameters) can always be set, but by one controller only: a
schedule refuses an input that a PID controller or another schedule sets. The
parameters that can be changed as the model runs are:

.. list-table::
   :header-rows: 1

   * - component
     - parameters
   * - PID controller
     - ``kp``, ``ki``, ``target`` (an input)
   * - hydraulic cylinder
     - ``command`` (an input), ``supply_pressure``, ``leakage``, ``friction``,
       ``end_stop_stiffness``, ``end_stop_damping``
   * - belt
     - ``friction_coefficient``
   * - track soil
     - ``ground``, ``shoe_center``, ``unloading_ratio``
   * - light of a body
     - ``intensity`` (an input), as ``"bodies/<body>/lights/<light>/intensity"``

A schedule does not check that a value is one that its component accepts (a
negative friction coefficient, for example); the schedule's points are the
user's responsibility.

When it is evaluated
^^^^^^^^^^^^^^^^^^^^

A schedule sets its targets every time the equations of motion are evaluated,
with the time of that evaluation, and does so before all other controllers and
the force modules, wherever it is in the file. A PID controller or a force module
that reads a scheduled parameter therefore always uses the value for the
time it is evaluating, and not the value from the step before. The value is
not recorded in the logs by the schedule; log the quantity that depends on it
(the ``opening`` of a hydraulic cylinder, for instance), or read the
parameter's value from the component.

Examples
^^^^^^^^

A valve that opens fully for one second, 2 s into the run, with the opening
and closing made smooth:

.. code:: javascript

   {"type": "schedule", "name": "valve_pulse", "interpolation": "cubic",
    "points": [0, 0, 2, 0, 2.3, 1, 3.3, 1, 3.6, 0],
    "targets": ["force_modules/cylinder/command"]}

A speed target for a PID controller that starts at 20 and changes in two
steps, to 40 at time 5 and to 30 at time 10, each made over a tenth of a
second:

.. code:: javascript

   {"type": "schedule", "name": "speed_profile", "interpolation": "step",
    "transition_time": 0.1, "points": [0, 20, 5, 40, 10, 30],
    "targets": ["controllers/speed_control/target"]}

Keys of ``schedule``:

.. shift-parameters:: shift/core/controller/schedule.cc
   :exclude: name, type, active

:source:`examples/excavator/excavator.json` drives the valves of its four
cylinders with three schedules. Tests: :source:`test/test_schedule.cc`.

.. _map:

Map
---

``"type": "map"`` sets parameters of other components from an *output* of a
component, through a curve: the brake lamps of a car from the brake pedal,
say. It reads the output named ``output`` of the component named
``component`` (the ``value`` of a schedule, the signal of a sensor, the
``effort`` of a PID controller) and sets its ``targets`` to the value of the
curve at it. The component is found as for a PID controller: by its name in
the same assembly, or by its fully qualified name, as
``"car__brake_pedal"``, in another.

The curve is given as a schedule's control points, with the output in place
of the time: ``points`` is :math:`[x_0, v_0, x_1, v_1, \dots]`, the value
:math:`v` at each value :math:`x` of the output, with :math:`x` increasing.
Between the points the value follows ``interpolation`` (``"linear"``,
``"cubic"`` or ``"step"``, as for a schedule) and beyond them it holds the
value of the nearest point. Without ``points`` the value is the output
itself. The targets are given, and checked, as a schedule's (see
:ref:`schedule`).

A map is evaluated with the other controllers, in the order of the model
file, after the schedules. It reads the output of a schedule, a sensor or a
force module as it is at that evaluation; the output of another controller
that is not a schedule is the one that controller left, so put the map after
it in the file. Its own ``value`` is an output, which the loggers record.

The stop lamps of :source:`examples/polo_r_wrc/polo_r_wrc.json` are lights of
the chassis that start dark; this map lights them, at full brightness (an
intensity of 8) once the brake pedal is pressed 5 %:

.. code:: javascript

   {"type": "map", "name": "brake_lights", "component": "brake_pedal",
    "output": "value", "points": [0, 0, 0.05, 8],
    "targets": ["bodies/chassis/lights/left_brake_lamp/intensity",
                "bodies/chassis/lights/right_brake_lamp/intensity"]}

Keys of ``map``:

.. shift-parameters:: shift/core/controller/map.cc
   :exclude: name, type, active

Tests: :source:`test/test_map.cc`.

Inverse kinematics
------------------

``"type": "ik_7dof"`` is reserved for the inverse kinematics of 7-axis
robot arms; it is registered but does not act on the model yet.
