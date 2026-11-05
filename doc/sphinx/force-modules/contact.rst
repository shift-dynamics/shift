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

Stick-slip friction
^^^^^^^^^^^^^^^^^^^

Regularized Coulomb friction cannot hold a body at rest: it gives the friction
that a load needs only at a sliding speed that makes it, so a box on a slope
creeps down it, at a fraction of ``friction_velocity``. A pair of collision
geometries listed in ``stick_slip`` has friction that holds, by the bristle model
of Canudas de Wit, Olsson, Åström and Lischinsky :cite:`canudas1995` (the LuGre
model), at each point of its contact. The bristles at a point are a deflection
:math:`z` in the tangent plane, a tiny stretch of the surface between the two
bodies, that grows as they slide over each other and saturates:

.. math::

   \dot z = u - \sigma_0\,\frac{|u|}{g(|u|)}\,z, \qquad
   g(|u|) = \mu_c + (\mu_s - \mu_c)\,e^{-(|u|/v_s)^2}

for the sliding velocity :math:`u` at the point, and the friction force on body
b is

.. math::

   F_t = -F\,(\sigma_0 z + \sigma_1 \dot z + \sigma_2 u)

for the normal force :math:`F` at the point. The parameters are the stiffness
:math:`\sigma_0` of the bristles (``bristle_stiffness``, in 1/m: the sliding at
which the friction of a body at rest reaches a friction coefficient of 1), their
damping :math:`\sigma_1` (``bristle_damping``), a viscous friction
:math:`\sigma_2` (``viscous_friction``), the coefficient :math:`\mu_c` of the pair
(the Coulomb friction of the materials, ``friction`` of the pair, from the
materials or ``pairs``), the static friction :math:`\mu_s` (``static_friction``, at
least :math:`\mu_c`, default :math:`\mu_c`) and the Stribeck velocity :math:`v_s`
(``stribeck_velocity``), the sliding speed over which the friction falls from
static to Coulomb.

At rest the bristles hold: the deflection settles where :math:`\sigma_0 z` carries
the tangential load, so a load of a fraction :math:`f` of the normal force gives
:math:`z = f/\sigma_0`, and the body moves by that and no more, up to
:math:`\mu_s F`. Beyond it the body slides, with friction that falls from
:math:`\mu_s` to :math:`\mu_c` as it speeds up, and the force is a smooth function
of the state throughout.

The deflections of a pair are states of the module, three numbers for each of
``points`` slots (default 8), the k-th point of the contact (in the order the
contact finds them) having the k-th. A point that is not in contact relaxes to
zero over ``release_time``. A box on its face has four points, a line of contact
of a CAD part up to about ten. The module logs, for each pair, the output
``stick_<a>_<b>``: the largest :math:`|z|\sigma_0/\mu_s` of its points, which is 1
at the limit of holding.

The bristles are stiff: they relax with a time constant :math:`g/(\sigma_0|u|)` at
the sliding speed :math:`|u|`, and the variable step integrator has to resolve
it. A stiffer :math:`\sigma_0` holds closer, with less travel, and takes smaller
steps while the body slides; a body that is suddenly loaded moves
:math:`f/\sigma_0` while the load comes on, and a Stribeck velocity smaller than
the speed it gains in that travel gives the friction no time to hold. Choose
:math:`\sigma_0` so that :math:`\mu_c/\sigma_0` is small against the sizes of the
motion.

.. code:: javascript

   "stick_slip": [
     {"geometry_a": "ground", "geometry_b": "box", "bristle_stiffness": 2000,
      "static_friction": 0.6, "stribeck_velocity": 0.05}
   ]

Each entry of ``stick_slip``:

.. shift-parameters:: shift/core/force_module/contact.cc
   :function: StickSlipPair::DeclareParameters

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
* :source:`examples/collision/stick_slip.json`: a box on a slope held by the
  bristles of its four corners, where regularized Coulomb friction would let it
  creep.
* Tests: :source:`test/test_contact_force.cc`,
  :source:`test/test_contact_geometry.cc` and :source:`test/test_stick_slip.cc`.
