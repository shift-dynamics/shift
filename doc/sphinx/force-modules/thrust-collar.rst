.. title:: Thrust collar

Thrust collar
=============

``"type": "thrust_collar"`` is the axial contact of a collar and a shoulder or
washer: the oil film between their faces, the contact of their surfaces, and the
friction. It sits beside :doc:`journal_bearing <journal-bearing>`, which carries a
shaft across its axis, and carries the shaft along it.

The collar and the shoulder are two bodies, each with a frame on its face, on the
axis. The z axis of the first frame is the axis :math:`\underline{a}`, pointing from
its face to the other. The faces are flat rings of inner radius :math:`a` and outer
radius :math:`b`, square to the axis. The gap between them is

.. math::

   h = (\underline{p}_j - \underline{p}_i)\cdot\underline{a}

with :math:`\underline{p}_i` and :math:`\underline{p}_j` the origins of the frames,
so the faces touch where the origins do. With an ``end_play`` :math:`e` the collar
runs between two shoulders: there is a second gap :math:`e - h`, with its own film
and contact.

Rough surfaces never close the film altogether. With the ``roughness``
:math:`\sigma` its thickness is

.. math::

   h_f = \sigma + \tfrac{1}{2}\Bigl(h + \sqrt{h^2 + \sigma^2}\Bigr)

The squeeze of the film
-----------------------

Oil pressed out of a closing gap carries a load, whether or not the faces turn. The
Reynolds equation :cite:`reynolds1886` for a flat ring gives the force, for the oil
of viscosity :math:`\mu` and the closing speed :math:`v = -\dot{h}`:

.. math::

   f_s = c\,v, \qquad
   c = \frac{3\pi\mu}{2 h_f^3}\Bigl(b^4 - a^4 - \frac{(b^2 - a^2)^2}{\ln(b/a)}\Bigr)

A ring loaded by a weight :math:`w` therefore closes slowly but never quite shuts,
:math:`1/h_f^2` growing evenly in time. The film cavitates rather than pull the
faces together, so an opening gap has no force, and the force comes in smoothly from
rest over ``slip_speed``.

As the film thins, :math:`c` becomes too great for the integrator: a body of mass
:math:`m` on it has the time constant :math:`m/c`. So :math:`c` levels off at
``film_damping_limit`` :math:`c_m`, as :math:`c\,c_m/(c + c_m)`. Below this limit
the squeeze is the Reynolds equation's. To follow a thinner film, raise the limit,
and expect steps below :math:`m/c_m`.

The lift of tapered pads
------------------------

Flat parallel faces carry no steady load: the oil only passes through them, and the
collar settles onto its surfaces however fast it turns. With ``pads`` and ``taper``
the face has :math:`n` pads, each rising by :math:`t` across it, which drag oil into a
narrowing wedge. With the mean radius :math:`r_m = (a + b)/2`, the pad's length
:math:`l = 2\pi r_m/n` and width :math:`w = b - a`, the speed
:math:`u = r_m|\omega|` and :math:`k = t/h_f`, the Reynolds equation of a long plane
pad :cite:`hamrock2004` gives

.. math::

   f_l = n\,\frac{6\mu u l^2 w}{h_f^2 k^2}
         \Bigl(\ln(1 + k) - \frac{2k}{2 + k}\Bigr)

The oil lost past the sides of a pad is left out, so this is the upper limit of the
load. The direction of rotation makes no difference, as for pivoted pads.

The contact of the surfaces
---------------------------

The surfaces carry what the film does not:

.. math::

   f_c = \max\bigl(0,\; k_c\,\rho(-h) + c_c\,\rho'(-h)\,v\bigr)

with ``contact_stiffness`` :math:`k_c`, ``contact_damping`` :math:`c_c` and
:math:`\rho` a ramp from zero at :math:`h = \sigma` to :math:`-h` at
:math:`h = -\sigma`. With a ``contact_damping`` of 0 a collar rings on the surfaces,
with only the film to settle it.

The friction
------------

With :math:`\omega` the speed of the second body relative to the first about the
axis, the torque on the second is the shear of the oil and the friction of the
surfaces, under an even pressure:

.. math::

   m = -\frac{\pi\mu(b^4 - a^4)}{2(h_f + t/2)}\,\omega
       - \mu_b f_c r_e \frac{\omega}{\sqrt{\omega^2 + \omega_s^2}}, \qquad
   r_e = \frac{2}{3}\,\frac{b^3 - a^3}{b^2 - a^2}

with ``friction`` :math:`\mu_b` and ``slip_rate`` :math:`\omega_s`, which keeps the
torque continuous. A collar held by its friction therefore creeps slowly; for a
collar that must hold at rest, add :doc:`joint_friction <joint-forces>` with
bristles on its joint.

Parameters
----------

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Key
     - Default
     - Meaning
   * - ``frames``
     - required
     - ``[[body, frame], [body, frame]]``: a frame on each face, on the axis
   * - ``outer_radius``
     - required
     - of the ring of the faces
   * - ``inner_radius``
     - 0
     - of the ring; 0 is a disc
   * - ``viscosity``
     - 0
     - of the oil, Pa s; 0 is dry faces
   * - ``end_play``
     - 0
     - the collar's travel between two shoulders; 0 is a single face
   * - ``roughness``
     - 1e-6
     - of the faces together: the film they leave, and half the width of the edge of the contact
   * - ``pads``, ``taper``
     - 0
     - number of tapered pads and the rise of each, given together
   * - ``contact_stiffness``
     - 1e9
     - force per depth of the contact of the surfaces
   * - ``contact_damping``
     - 1000
     - force per speed in the contact of the surfaces
   * - ``friction``
     - 0.1
     - coefficient of friction of the surfaces
   * - ``film_damping_limit``
     - 1e5
     - what the squeeze of a thin film levels off at, force per speed
   * - ``slip_speed``
     - 1e-6
     - closing speed over which the squeeze comes in
   * - ``slip_rate``
     - 0.1
     - speed of rotation below which the friction of the surfaces falls to zero

.. code-block:: json

   {"type": "thrust_collar", "name": "collar",
    "frames": [["shaft", "collar_face"], ["housing", "shoulder_face"]],
    "inner_radius": 0.03, "outer_radius": 0.06, "viscosity": 0.05,
    "pads": 6, "taper": 3e-5}

The module acts along the axis and about it only. The tilt of the faces and the
heating of the oil are not modelled.

The outputs are ``gap``, ``film_force``, ``contact_force`` and ``friction_torque``.
The model ``examples/force_modules/thrust_collar.json`` spins a rotor down on tapered
pads until it comes down onto its surfaces.
