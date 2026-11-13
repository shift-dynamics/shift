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
journal's. With the shaft off center by the vector :math:`\underline{e}`
perpendicular to the axis, the *eccentricity ratio* is
:math:`\varepsilon = |\underline{e}|/c`, and the film thickness round the bearing is

.. math::

   h = c\,(1 + \varepsilon\cos\theta)

with :math:`\theta` measured from the thickest part of the film. The unit vector
:math:`\underline{u}_1` points along the line of centers (from the bearing's center to
the shaft's) and :math:`\underline{u}_2` across it, in the direction of rotation about
the axis.

The pressure
------------

The module uses the theory of the *short bearing* of Ocvirk :cite:`ocvirk1952`: in a
bearing that is short compared with its diameter (a length up to about half the
diameter, and as an approximation to about the diameter) the oil flows out of the
ends of the bearing much more than it flows round it, and the pressure in the film,
for the oil of viscosity :math:`\mu` and a bearing of length :math:`l`, is

.. math::

   p = \frac{3\mu}{c^2}\Bigl(\frac{l^2}{4} - z^2\Bigr)
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

   \underline{f} = \frac{\mu r l^3}{2c^2}\,\bigl(i_1\,\underline{u}_1 + i_2\,\underline{u}_2\bigr),
   \quad
   i_1 = \int\!\frac{(s\sin\theta - q\cos\theta)\cos\theta}{(1+\varepsilon\cos\theta)^3}\,d\theta,
   \quad
   i_2 = \int\!\frac{(s\sin\theta - q\cos\theta)\sin\theta}{(1+\varepsilon\cos\theta)^3}\,d\theta

over the film, for the journal's radius :math:`r`. The module integrates these
numerically, with a change of variable (Sommerfeld's) that keeps the integrands
smooth even where the film is thin. In steady running, with the shaft still at an
eccentricity, they are the classical results

.. math::

   f_1 = -\frac{\mu r l^3\omega}{c^2}\,\frac{\varepsilon^2}{(1-\varepsilon^2)^2}, \qquad
   f_2 = \frac{\mu r l^3\omega}{c^2}\,\frac{\pi\varepsilon}{4(1-\varepsilon^2)^{3/2}}

The first pushes the shaft back to the center. The second, across the line of
centers, makes the *attitude angle* between the load and the line of centers,
:math:`\tan\phi = \pi\sqrt{1-\varepsilon^2}/(4\varepsilon)`, and the film carries the
load

.. math::

   w = \frac{\mu r l^3 \omega\,\varepsilon}{4c^2(1-\varepsilon^2)^2}
       \sqrt{16\varepsilon^2 + \pi^2(1-\varepsilon^2)}

which sets the eccentricity at which a shaft settles under a load. The tests check
the integrals against a direct integration of the pressure over the film, these
closed forms and the equilibrium of a shaft under its weight. With the squeeze, a
shaft near the center moving at a velocity :math:`\underline{v}` feels the damping
force :math:`-(\pi\mu r l^3/2c^3)\,\underline{v}`.

A bearing of finite length
--------------------------

The short bearing leaves out the flow round the bearing, so it overstates the pressure
gradient along the axis and gets worse as the bearing gets longer. With
``"model": "finite"`` the module solves the Reynolds equation for the actual length.
With :math:`\bar{h} = 1 + \varepsilon\cos\theta`, :math:`\bar{z} = 2z/l` and
:math:`p = (\mu r^2/c^2)\bar{p}`, it is

.. math::

   \frac{\partial}{\partial\theta}\Bigl(\bar{h}^3\frac{\partial \bar{p}}{\partial\theta}\Bigr)
   + \frac{4r^2}{l^2}\frac{\partial}{\partial \bar{z}}\Bigl(\bar{h}^3\frac{\partial \bar{p}}{\partial \bar{z}}\Bigr)
   = -6\,(s\sin\theta - q\cos\theta)

with :math:`\bar{p} = 0` at the ends of the bearing. The equation is linear in :math:`s` and
:math:`q`, so the module solves it once for each, for a table of eccentricity ratios,
when the model loads. It then clips the pressure at zero, as above, and integrates it
for a table of directions :math:`\mathrm{atan2}(q, s)`. As the model runs, the force is
looked up in the two tables, so a step costs about the same as the short bearing.

The tests check that the finite model approaches the short bearing as the bearing
narrows, with a gap that falls with the square of the length (at
:math:`\varepsilon = 0.8`, 16 % at a length of 0.2 diameters, 5 % at 0.1, 1.3 % at
0.05 and 0.3 % at 0.025), and Sommerfeld's pressure as it gets long.

The mobility method of Booker :cite:`booker1971`, with the curve fits of Goenka, is the
usual way to get this result. The module does not use it: mobility gives the
velocity of the shaft from the load, and a force module needs the opposite.

An oil groove and the heating of the oil
----------------------------------------

``groove_width`` is a circumferential groove at the middle of the bearing, at no
pressure, which divides it into two bearings, each
:math:`(l - \mathrm{groove\_width})/2` long. It works with either model.

With ``heat_capacity`` :math:`c_h`, the temperature rise :math:`t_r` of the oil above the
temperature of ``viscosity`` is a state of the module, heated by the power of the
friction and cooled in proportion to the rise, and the viscosity falls with it:

.. math::

   c_h\,\dot t_r = |\tau\,(\omega_j - \omega_b)| - k_t\,t_r, \qquad
   \mu = \mu_0\,e^{-\beta t_r}

with ``cooling`` :math:`k_t` and ``viscosity_slope`` :math:`\beta`. This is one
temperature for the whole bearing, not a temperature field through the film.

The friction torque
-------------------

The torque of the film on the journal about the axis is the shear of the oil,
round the whole bearing (Petroff's formula with the eccentricity), and the work of
the pressure force:

.. math::

   \tau = -\frac{2\pi\mu r^3 l(\omega_j - \omega_b)}{c\sqrt{1-\varepsilon^2}}
       - \tfrac12\,c\,\varepsilon\,f_2

Both oppose the rotation. The bearing gets the opposite torque and force. The
module also gives the minimum film thickness :math:`c(1-\varepsilon)` (negative if
the journal is in contact with the bearing), the greatest pressure in the film,
and the flow of oil out of the ends of the bearing.

Limits
------

The eccentricity ratio of the film is limited to 0.995 (smoothly beyond 0.9), where
the force of the film is large but finite; give ``wall_stiffness`` for the contact
of the journal with the bearing when it is off center by more than the clearance.
The module does not include the inertia of the oil, misalignment, axial oil grooves,
the pressure of the oil supply, or the elasticity of the shaft and bearing.
``viscosity`` can also be changed as the model runs (see :ref:`schedule`) to follow a
temperature found elsewhere.

.. code:: javascript

   {"type": "journal_bearing", "name": "bearing",
    "journal": ["shaft", "origin"], "bearing": ["fixed", "bearing_axis"],
    "radius": 0.025, "length": 0.025, "clearance": 5e-5, "viscosity": 0.005}

Outputs, logged as ``<assembly>__<name>__<output>``: ``eccentricity_ratio``,
``min_film_thickness``, ``peak_pressure``, ``friction_torque`` (on the journal),
``side_flow``, ``radial_force`` and ``tangential_force`` (along and across the line
of centers), ``load``, ``effective_viscosity`` (that of the oil at its temperature) and
``temperature_rise``.

.. shift-parameters:: shift/core/force_module/journal_bearing.cc
   :exclude: name, type, active

Example
-------

:source:`examples/force_modules/journal_bearing.json`: a 10 kg shaft turning at
100 rad/s in a bearing of 25 mm radius and length, with a radial clearance of
50 µm and oil of 5 mPa s, its axis horizontal. It settles, within 0.13 s, at an
eccentricity ratio of 0.598, as the formula for the load gives: the film is 20 µm
thick at the thinnest and the greatest pressure is 250 kPa.

:source:`examples/force_modules/journal_bearing_finite.json`: the same shaft in a
bearing as long as its diameter, with the finite model and the heating of the oil.
The shaft settles at an eccentricity ratio of 0.27 within 0.1 s. Over 5 s the oil
warms by 8.2 K, where the 4.1 W of friction balances the cooling. The viscosity falls
from 5 to 3.9 mPa s, and the shaft sinks to 0.315.

Tests: :source:`test/test_journal_bearing.cc`.
