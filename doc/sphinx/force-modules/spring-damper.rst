.. title:: Spring damper

Spring damper
=============

``"type": "spring_damper"`` is a spring and a damper between two frames on two
bodies, without a joint between them: a coil spring or a shock absorber mounted
between a chassis and an axle, a bump stop, or a torsion spring. Unlike a
:doc:`bushing <bushing>`, its force follows the line between the two points as
that line turns, and it has a free length. Its force can come from measured
curves, it can act on one side only, past a gap, and it can have hysteresis.

The force along the line
------------------------

With :math:`\underline{p}_i` and :math:`\underline{p}_j` the origins of the two
frames, the length is :math:`l = |\underline{p}_j - \underline{p}_i|`, and the
deflection from the ``free_length`` :math:`l_0` is :math:`x = l - l_0`, with the
rate :math:`v = \dot{l}`. The force :math:`f`, positive in tension, pulls the two
points together: :math:`-f \underline{n}` on frame :math:`j` and
:math:`f \underline{n}` on frame :math:`i`, along
:math:`\underline{n} = (\underline{p}_j - \underline{p}_i) / l`. Without
``free_length``, :math:`l_0` is the length as the model starts.

.. math::

   f = w \left( s(x_e) + d(v) + f_y z \right)

The spring :math:`s` is :math:`k x_e + f_0`, from ``stiffness`` and ``preload``, or
a ``force_curve`` of measured points. The damper :math:`d` is :math:`c v`, from
``damping``, or a ``damping_curve``. A curve is a list of pairs
``[x0, f0, x1, f1, ...]`` in increasing order, and takes its own values on each
side of zero, so a shock absorber's curve can differ in compression (negative
rate) and rebound (positive rate). Between the points it is a smooth curve that
does not overshoot them, and past the ends it continues with the end slopes.

Stops
-----

Without a stop, :math:`x_e = x` and :math:`w = 1`. A stop acts only past an edge:
``"acts": "compression"`` when :math:`x < -g`, ``"tension"`` when :math:`x > g`, and
``"both"`` on both sides, with nothing between, for the ``gap`` :math:`g`. Then
:math:`x_e` is the deflection past the edge. The force comes in smoothly over
the width ``smoothing`` :math:`b` around the edge, so that the integrator sees no
jump: for the distance :math:`u` past the edge,

.. math::

   \rho(u) = \begin{cases}
     0 & u \le -b/2 \\
     (u + b/2)^2 / (2 b) & |u| < b/2 \\
     u & u \ge b/2
   \end{cases}

:math:`x_e` is :math:`\rho(u)` and :math:`w` its slope. Beyond the width the
deflection past the edge is exact. A stop only pushes: the force is clipped at
zero when the damper of a fast rebound would make it pull.

Hysteresis
----------

A ``hysteresis`` block adds the Bouc-Wen force :math:`f_y z` :cite:`wen1976`, which
depends on the history of the deflection through the state :math:`z`:

.. math::

   \dot{z} = \frac{v}{x_y} \left( 1 - |z|^n \left( \beta + \gamma \,
     \mathrm{sign}(v z) \right) \right)

with the ``yield_force`` :math:`f_y`, the ``yield_displacement`` :math:`x_y`, the
``exponent`` :math:`n` and the shape parameters ``beta`` and ``gamma``. The force
first rises with the slope :math:`f_y / x_y`, and with :math:`\beta + \gamma = 1`
levels off at :math:`f_y`. Its loop does not depend on how fast it is run, as
with the dry friction between the leaves of a leaf spring.

Crushing
--------

With ``unloading_stiffness`` :math:`k_u` the spring crushes, as a protective
structure does in a rollover. In compression it follows its curve down to the
deepest compression :math:`p` reached, a state that follows the compression
over ``crush_time``. From there it comes back along the line of slope
:math:`k_u`, down to zero force, and stays crushed. The outputs ``crush`` and
``absorbed`` are :math:`p` and the work done on the structure.

Torsion
-------

With ``axis``, in the coordinates of frame :math:`i`, the same law gives the
moment about the axis :math:`\underline{a}`: :math:`-f \underline{a}` on frame
:math:`j` and :math:`f \underline{a}` on frame :math:`i`. The rate :math:`v` is the
relative angular velocity about the axis, and the deflection the angle turned
since the start, less ``free_angle``. The angle is a state, the integral of
:math:`v`, so it counts full turns.

Parameters
----------

.. code:: javascript

   {"type": "spring_damper", "name": "shock",
    "frames": [["chassis", "origin"], ["axle", "origin"]],
    "free_length": 0.5, "stiffness": 25000,
    "damping_curve": [-1.0, -1200, -0.1, -400, 0, 0, 0.1, 1000, 1.0, 3000]}

   {"type": "spring_damper", "name": "bump_stop",
    "frames": [["chassis", "origin"], ["axle", "origin"]],
    "free_length": 0.43, "acts": "compression", "gap": 0.06,
    "stiffness": 200000, "damping": 2000, "smoothing": 0.004}

.. shift-parameters:: shift/core/force_module/spring_damper.cc
   :function: force_module::SpringDamper::DeclareParameters

The ``hysteresis`` block:

.. shift-parameters:: shift/core/force_module/spring_damper.cc
   :function: force_module::BoucWen::DeclareParameters

Outputs: ``length``, ``rate`` and ``force``, or ``angle``, ``rate`` and ``moment``
with an axis. ``examples/force_modules/quarter_car.json`` drops a quarter car on
its wheel, with a shock absorber, a bump stop and a tire that only pushes.
