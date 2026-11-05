.. title:: Belt

Belt
====

``"type": "belt"`` is an elastic belt in a closed loop around two or more
pulleys, in the order given and back to the first, as in an engine's
accessory drive or a belt transmission. Each pulley is a frame on a body:
its origin is the center of the pulley and its z axis the axis it turns
about. The belt lies in the plane through the center of the first pulley,
normal to its z axis; the centers of the other pulleys are projected onto
that plane.

Looking down that z axis, the belt travels counterclockwise around the
pulleys inside the loop, so the pulleys are listed counterclockwise. A
negative radius puts a pulley outside the loop, such as an idler running on
the back of the belt.

Spans and tension
-----------------

Between pulleys the belt is a straight span along the common tangent of
the two pulleys. With :math:`r_a` and :math:`r_b` the (signed) radii and
:math:`D` the distance between the centers, the span's length between its
tangent points is :math:`L = \sqrt{D^2 - (r_a - r_b)^2}`. A span's tension
follows from its stretch :math:`e`, its length less the unstretched length
of the belt between its ends:

.. math::

   T = \frac{EA\, e}{L - e} + \beta \frac{EA}{L - e}\,\dot e, \qquad T \ge 0

A span that would be in compression is slack and carries no force; the
change from slack to taut is smoothed over ``slack_band``. The tension
pulls the pulley at each end of the span along the span, at its tangent
point, so the belt transmits torque, :math:`r(T_\text{out} - T_\text{in})`
on each pulley, and pulls the pulleys toward each other. The belt carries
no bending, and its mass is not modeled.

The unstretched length of a span follows from the turns of the pulleys at
its ends: turning a pulley forward by :math:`\theta` feeds
:math:`r\theta` of belt into the span leaving it and takes as much from the
span arriving. The turn of each pulley, relative to the line from the first
pulley's center to the second's, is an internal state, so the pulleys may
turn any number of times.

Slip
----

Without ``friction_coefficient`` the belt does not slip, as a toothed
belt. With it, the friction around a pulley of wrap angle :math:`\phi`
holds tensions in a ratio up to :math:`e^{\mu\phi}` (the capstan equation);
beyond that the belt slides over the pulley, toward the tighter span, at
the speed

.. math::

   v_s = \frac{|T_\text{out} - T_\text{in}| - T_\text{slack}
   \left(e^{\mu\phi} - 1\right)}{c_s}

with :math:`c_s` the ``slip_damping``. The length of belt slipped over each
pulley is another internal state.

JSON
----

.. literalinclude:: ../../../examples/force_modules/belt.json
   :language: javascript
   :caption: examples/force_modules/belt.json

.. shift-parameters:: shift/core/force_module/belt.cc
   :exclude: name, type, active

Outputs
-------

``span_<k>_tension``: the tension of each span :math:`k`, from pulley
:math:`k` to the next; and, with friction, ``pulley_<k>_slip``, the speed at
which the belt slips forward over each pulley.

Example
-------

.. figure:: ../images/examples/belt.jpg
   :width: 80%
   :alt: belt.json at 1 s, from above: the crankshaft pulley (black), alternator (blue), water pump (aluminium) and the idler (steel), with the tensioner slide (black). The belt is a force between the pulleys and is not drawn.

   belt.json at 1 s, from above: the crankshaft pulley (black), alternator (blue), water pump (aluminium) and the idler (steel), with the tensioner slide (black). The belt is a force between the pulleys and is not drawn.

:source:`examples/force_modules/belt.json`: an accessory drive, from the
crankshaft around the alternator and the water pump, with a spring-loaded
idler on the back of the belt as the tensioner. A PID controller brings the
crankshaft up to 80 rad/s, reading a joint speed sensor. The alternator,
pump and idler settle at the speeds the radius ratios give (213, 128 and
-213 rad/s); during the spin-up the belt slips briefly over the
alternator. Tests: :source:`test/test_belt.cc`.
