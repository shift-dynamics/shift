.. title:: Contact

Contact
=======

``"type": "contact"`` applies contact and friction forces between the
collision geometry of bodies (see :ref:`model-file`, Collision geometry).
Contact detection finds the points where geometry touches, with the depth
and rate of penetration at each; this module turns them into forces. Each
collision geometry names a contact *material*, and the properties of each
pair of materials in contact are combined from the two materials, or given
for the pair.

Contact detection is off unless ``model_parameters`` turns it on with
``"enable_contact_detection": true``.

Normal force
------------

At each contact point with penetration depth :math:`d > 0` and rate of
penetration :math:`v`, the normal force is that of the IMPACT function of
MSC Adams, scaled by the point's weight :math:`w`:

.. math::

   F = w \max\left(0,\; K d^e + \mathrm{STEP}(d, 0, 0, h, c)\, v\right),
   \qquad \mathrm{STEP}(d, 0, 0, h, c) = c\, s^2 (3 - 2 s),\quad
   s = \min(d / h, 1)

The damping coefficient ramps up from zero at first touch to :math:`c` at
the depth :math:`h` (``damping_depth``), so the force is continuous and
depends on the state only. The stiffness depends on the shapes:

* **Spheres, ellipsoids and planes**: Hertz contact :cite:`johnson1985` at
  the deepest point, :math:`K = \tfrac{4}{3} E^* \sqrt{R^*}`,
  :math:`e = 3/2`, with :math:`1/E^* = (1-\nu_a^2)/E_a + (1-\nu_b^2)/E_b`
  from the materials' ``youngs_modulus`` and ``poisson_ratio`` and
  :math:`1/R^*` the sum of the curvatures at the contact points.
* **Cylinders and capsules on a plane**: Hertz line contact
  :cite:`johnson1985`, :math:`F = \tfrac{\pi}{4} E^* L d` at depth
  :math:`d` over a length :math:`L` of the line (see below), :math:`e = 1`.
  Against any other shape they touch as if round, with Hertz contact of the
  radius of the cylinder or capsule at the deepest point.
* **Boxes and convex hulls**: a linear spring, :math:`K = k`, :math:`e = 1`, at each
  point of the contact patch, with ``contact_stiffness`` the stiffness of
  one point (a box resting on its face stands on four of them). The
  stiffnesses of two materials combine in series.
  A box is a convex hull of eight corners (``"type": "box"``, with
  ``half_extents``), so a face resting on a plane or on another hull stands
  on its corners.
* **CAD geometry**: each point found on the exact surfaces has its own
  normal and law: Hertz contact where two curved surfaces touch, and, along
  a line of contact (parallel cylinders, a cylinder on a plane), stations
  of Hertz line contact, :math:`K = \tfrac{\pi}{4} E^* L`, :math:`e = 1`.

A cylinder or a capsule on a plane touches along a line, the lowest
generator of the cylinder or the segment of the capsule, and a single
deepest point could not carry its load as it tilts or rolls. At the station
:math:`s` of its axis, between :math:`-h` and :math:`h`, its lowest point is
at the depth

.. math::

   d(s) = o - \underline{n}\cdot\underline{c} - s\, g + r_\mathrm{eff},
   \qquad g = \underline{n}\cdot\underline{a}

with :math:`\underline{n}` the unit normal of the plane, :math:`o` its
offset, :math:`\underline{c}` and :math:`\underline{a}` the center and axis
of the shape, :math:`r_\mathrm{eff} = r` for a capsule (the lowest point of
the sphere swept along its segment) and :math:`r\sqrt{1 - g^2}` for a
cylinder (the lowest point of its circle). The depth is linear in :math:`s`, so
Hertz line contact integrates in closed form over the stations where it is
positive: a trapezoid of length :math:`L`, equal to one point carrying that
length with the mean depth, at the centroid of the trapezoid. The point
moves along the line as the shape tilts, so the force is continuous in the
state and gives the restoring moment. A cylinder standing on a flat end is
carried by its rim, as by a line of zero length that grows with the depth:
use a box or a convex hull for flat faces.

The damping :math:`c` is given as ``damping``, or follows from the
coefficient of restitution :math:`r` as the damping of a mass on a linear
spring that rebounds with :math:`r`,

.. math::

   c = 2 \zeta \sqrt{k m}, \qquad
   \zeta = \frac{-\ln r}{\sqrt{\pi^2 + \ln^2 r}}

with :math:`m` the effective mass of the two groups of bodies in contact
and :math:`k` the stiffness linearized at ``damping_depth``.

Friction
--------

Coulomb friction, made smooth below ``friction_velocity``, opposes the
sliding velocity :math:`u` at each point:

.. math::

   F_t = -\mu F \frac{u}{\sqrt{|u|^2 + u_f^2}}

Friction and restitution of a pair combine by ``friction_combine`` and
``restitution_combine`` (the geometric mean by default) unless the pair is
listed in ``pairs``.

JSON
----

.. code:: javascript

   {
     "type": "contact", "name": "contact",
     "friction_velocity": 0.1,
     "materials": [
       {"name": "bunny", "youngs_modulus": 1e6, "poisson_ratio": 0.4,
        "restitution": 0.5, "friction": 0.5, "contact_stiffness": 1e4},
       {"name": "floor", "youngs_modulus": 1e8, "poisson_ratio": 0.3,
        "restitution": 0.5, "friction": 0.5, "contact_stiffness": 1e6}
     ],
     "pairs": [{"materials": ["bunny", "floor"], "friction": 0.8}]
   }

and on each body, collision geometry that names its material:

.. code:: javascript

   "collision_geometry": [{"type": "convex_hull", "material": "bunny"}]

The module:

.. shift-parameters:: shift/core/force_module/contact.cc
   :function: force_module::Contact::DeclareParameters
   :exclude: name, type, active

Each entry of ``materials``:

.. shift-parameters:: shift/core/force_module/contact.cc
   :function: ContactMaterial::DeclareParameters

Each entry of ``pairs``:

.. shift-parameters:: shift/core/force_module/contact.cc
   :function: ContactMaterialPair::DeclareParameters

Examples
--------

.. figure:: ../images/examples/collision_e_ch.jpg
   :width: 60%
   :alt: collision_test_e_ch.json: two Stanford bunnies in contact, one through an ellipsoid fitted to its mesh and the other through its convex hull

   collision_test_e_ch.json: two Stanford bunnies in contact, one through an ellipsoid fitted to its mesh and the other through its convex hull

.. figure:: ../images/examples/collision_50.jpg
   :width: 60%
   :alt: collision_test_50.json at 1 s: a pile of 50 bunnies

   collision_test_50.json at 1 s: a pile of 50 bunnies

* :tree:`examples/collision`: pairs of spheres, ellipsoids, planes and
  convex hulls dropped on each other (``collision_test_ss.json``,
  ``collision_test_ee.json``, ``collision_test_ps.json``, ...), and piles
  of 5 to 5000 bodies (``collision_test_5.json``, ...; the large ones are
  benchmarks).
* Tests: :source:`test/test_contact_force.cc`,
  :source:`test/test_contact_geometry.cc`.
