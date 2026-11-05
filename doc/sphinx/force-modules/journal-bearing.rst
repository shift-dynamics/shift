.. title:: Journal bearing

Journal bearing
===============

``"type": "journal_bearing"`` is the oil film of a plain journal bearing: the force
of the film on the shaft from the position and motion of the shaft in the bearing,
and the friction torque. The film carries the shaft on a wedge of oil that the
rotation drags into the narrowing gap; it pushes the shaft back to the center, and
also sideways, across the line of centers, which is what makes a lightly loaded
shaft whirl.

The shaft (the *journal*) and the bearing are two bodies, each with a frame at the
center of its circle, whose z axis is the axis of the bearing (the axes are
parallel). The *radial clearance* :math:`c` is the bearing's radius less the
journal's. With the shaft off center by the vector :math:`\mathbf{e}`
perpendicular to the axis, the *eccentricity ratio* is
:math:`\varepsilon = |\mathbf{e}|/c`, and the film thickness round the bearing is

.. math::

   h = c\,(1 + \varepsilon\cos\theta)

with :math:`\theta` measured from the thickest part of the film. The unit vector
:math:`\mathbf{u}_1` points along the line of centers (from the bearing's center to
the shaft's) and :math:`\mathbf{u}_2` across it, in the direction of rotation about
the axis.

The pressure
------------

The module uses the theory of the *short bearing* of Ocvirk :cite:`ocvirk1952`: in a
bearing that is short compared with its diameter (a length up to about half the
diameter, and as an approximation to about the diameter) the oil flows out of the
ends of the bearing much more than it flows round it, and the pressure in the film,
for the oil of viscosity :math:`\mu` and a bearing of length :math:`L`, is

.. math::

   p = \frac{3\mu}{c^2}\Bigl(\frac{L^2}{4} - z^2\Bigr)
       \frac{s\sin\theta - q\cos\theta}{(1 + \varepsilon\cos\theta)^3}

with :math:`z` along the axis from the middle of the bearing and

.. math::

   s = \varepsilon(\omega_j + \omega_b) - \frac{2\,\dot e_2}{c}, \qquad
   q = \frac{2\,\dot e_1}{c}

where :math:`\omega_j` and :math:`\omega_b` are the speeds of the journal and the
bearing about the axis and :math:`\dot e_1,\ \dot e_2` the velocity of the
journal's center relative to the bearing's, along the line of centers and across it.
:math:`s` is the part of the pressure from the rotation, including the whirl of the
line of centers, and :math:`q` the *squeeze*, from the shaft moving along the line of
centers. Where this would be negative the oil cavitates and the pressure is zero (a
film for half the turn, the "half Sommerfeld" solution, or the :math:`\pi`-film).

The force
---------

The pressure integrated over the journal gives the force of the film on it:

.. math::

   \mathbf{F} = \frac{\mu R L^3}{2c^2}\,\bigl(I_1\,\mathbf{u}_1 + I_2\,\mathbf{u}_2\bigr),
   \quad
   I_1 = \int\!\frac{(s\sin\theta - q\cos\theta)\cos\theta}{(1+\varepsilon\cos\theta)^3}\,d\theta,
   \quad
   I_2 = \int\!\frac{(s\sin\theta - q\cos\theta)\sin\theta}{(1+\varepsilon\cos\theta)^3}\,d\theta

over the film, for the journal's radius :math:`R`. The module integrates these
numerically, with a change of variable (Sommerfeld's) that keeps the integrands
smooth even where the film is thin. In steady running, with the shaft still at an
eccentricity, they are the classical results

.. math::

   F_1 = -\frac{\mu R L^3\omega}{c^2}\,\frac{\varepsilon^2}{(1-\varepsilon^2)^2}, \qquad
   F_2 = \frac{\mu R L^3\omega}{c^2}\,\frac{\pi\varepsilon}{4(1-\varepsilon^2)^{3/2}}

The first pushes the shaft back to the center. The second, across the line of
centers, makes the *attitude angle* between the load and the line of centers,
:math:`\tan\phi = \pi\sqrt{1-\varepsilon^2}/(4\varepsilon)`, and the film carries the
load

.. math::

   W = \frac{\mu R L^3 \omega\,\varepsilon}{4c^2(1-\varepsilon^2)^2}
       \sqrt{16\varepsilon^2 + \pi^2(1-\varepsilon^2)}

which sets the eccentricity at which a shaft settles under a load. The tests check
the integrals against a direct integration of the pressure over the film, these
closed forms and the equilibrium of a shaft under its weight. With the squeeze, a
shaft near the center moving at a velocity :math:`\mathbf{v}` feels the damping
force :math:`-(\pi\mu R L^3/2c^3)\,\mathbf{v}`.

The friction torque
-------------------

The torque of the film on the journal about the axis is the shear of the oil,
round the whole bearing (Petroff's formula with the eccentricity), and the work of
the pressure force:

.. math::

   T = -\frac{2\pi\mu R^3 L(\omega_j - \omega_b)}{c\sqrt{1-\varepsilon^2}}
       - \tfrac12\,c\,\varepsilon\,F_2

Both oppose the rotation. The bearing gets the opposite torque and force. The
module also gives the minimum film thickness :math:`c(1-\varepsilon)` (negative if
the journal is in contact with the bearing), the greatest pressure in the film,
and the flow of oil out of the ends of the bearing.

Limits
------

The eccentricity ratio of the film is limited to 0.995 (smoothly beyond 0.9), where
the force of the film is large but finite; give ``wall_stiffness`` for the contact
of the journal with the bearing when it is off center by more than the clearance.
The module does not include the inertia of the oil, heating of the oil
(``viscosity`` can be changed as the model runs, see :ref:`schedule`, to follow a
temperature), misalignment, oil grooves, the pressure of the oil supply, the
elasticity of the shaft and bearing, or the finite length of a bearing that is not
short: Booker's mobility method for bearings of finite length is on the roadmap.

.. code:: javascript

   {"type": "journal_bearing", "name": "bearing",
    "journal": ["shaft", "origin"], "bearing": ["fixed", "bearing_axis"],
    "radius": 0.025, "length": 0.025, "clearance": 5e-5, "viscosity": 0.005}

Outputs, logged as ``<name>_<output>``: ``eccentricity_ratio``,
``min_film_thickness``, ``peak_pressure``, ``friction_torque`` (on the journal),
``side_flow``, ``radial_force`` and ``tangential_force`` (along and across the line
of centers) and ``load``.

.. shift-parameters:: shift/core/force_module/journal_bearing.cc
   :exclude: name, type, active

Example
-------

:source:`examples/force_modules/journal_bearing.json`: a 10 kg shaft turning at
100 rad/s in a bearing of 25 mm radius and length, with a radial clearance of
50 µm and oil of 5 mPa s, its axis horizontal. It settles, within 0.13 s, at an
eccentricity ratio of 0.598, as the formula for the load gives: the film is 20 µm
thick at the thinnest and the greatest pressure is 250 kPa. Tests:
:source:`test/test_journal_bearing.cc`.
