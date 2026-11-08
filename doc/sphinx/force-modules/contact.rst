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

   f = w \max\left(0,\; k d^n + \mathrm{STEP}(d, 0, 0, h, c)\, v\right),
   \qquad \mathrm{STEP}(d, 0, 0, h, c) = c\, s^2 (3 - 2 s),\quad
   s = \min(d / h, 1)

The damping coefficient ramps up from zero at first touch to :math:`c` at
the depth :math:`h` (``damping_depth``), so the force is continuous and
depends on the state only. The stiffness depends on the shapes:

* **Spheres, ellipsoids and planes**: Hertz contact :cite:`johnson1985` at
  the deepest point, :math:`k = \tfrac{4}{3} e^* \sqrt{r^*}`,
  :math:`n = 3/2`, with :math:`1/e^* = (1-\nu_a^2)/e_a + (1-\nu_b^2)/e_b`
  from the materials' ``youngs_modulus`` and ``poisson_ratio`` and
  :math:`1/r^*` the sum of the curvatures at the contact points.
* **Cylinders and capsules on a plane**: Hertz line contact
  :cite:`johnson1985`, :math:`f = \tfrac{\pi}{4} e^* l d` at depth
  :math:`d` over a length :math:`l` of the line (see below), :math:`n = 1`.
  Against any other shape they touch as if round, with Hertz contact of the
  radius of the cylinder or capsule at the deepest point.
* **Boxes and convex hulls**: a linear spring, :math:`n = 1`, at each
  point of the contact patch, with :math:`k` the ``contact_stiffness``, the
  stiffness of one point (a box resting on its face stands on four of them). The
  stiffnesses of two materials combine in series.
  A box is a convex hull of eight corners (``"type": "box"``, with
  ``half_extents``), so a face resting on a plane or on another hull stands
  on its corners.
* **CAD geometry**: each point found on the exact surfaces has its own
  normal and law: Hertz contact where two curved surfaces touch, and, along
  a line of contact (parallel cylinders, a cylinder on a plane, a NURBS
  or extruded face on a cylinder or plane, as a cam on its roller, or two such
  faces on each other, as tooth flanks; the line may bend, as a curved ring on
  a plane does), stations of Hertz line contact, :math:`k = \tfrac{\pi}{4} e^* l`, :math:`n = 1`.
  A sphere or an ellipsoid touches CAD geometry the same way, on its exact
  surfaces: at a point of each face it presses on, with the curvature of the
  shape and of the face there, or at the edge or vertex it reaches where no
  face next to it touches. A ball in a hollow has the gap of the two curvatures,
  and a ball too wide for a hollow stands on its rims. A box, cylinder or convex
  hull touches CAD geometry as another CAD part does, through faces, edges and
  vertices of its own: a corner on a face, a cylinder's side along a line, a rim
  on a face, or two edges crossing. Its corners and edges are taken as rounded
  to its ``edge_radius``, by default 0.002 of its size, which sets their Hertz
  contact. A capsule is the side of a cylinder with a sphere at each end, and as
  it tilts its line of contact gives way smoothly to the point of the end. A
  cylinder or capsule rolling over the edge of a face keeps its line of contact,
  along that edge, and so does a cylinder of another STEP part. All of these use the ``youngs_modulus`` of the materials
  against CAD geometry, not the ``contact_stiffness``. A point deeper than
  ``max_depth`` is taken to be on the far side of the part: set it on the shape
  to use another value than the CAD geometry's, 2% of its size by default.
  Near an edge of a face, within the radius of the Hertz contact patch, a round
  shape's stiffness goes over smoothly to that of the edge.

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
positive: a trapezoid of length :math:`l`, equal to one point carrying that
length with the mean depth, at the centroid of the trapezoid. The point
moves along the line as the shape tilts, so the force is continuous in the
state and gives the restoring moment. A cylinder standing on a flat end is
carried by its rim, as by a line of zero length that grows with the depth:
use a box or a convex hull for flat faces.

The damping :math:`c` is given as ``damping``, or follows from the
coefficient of restitution :math:`r` as the damping of a mass on a linear
spring that rebounds with :math:`r`,

.. math::

   c = 2 \zeta \sqrt{\bar{k} m}, \qquad
   \zeta = \frac{-\ln r}{\sqrt{\pi^2 + \ln^2 r}}

with :math:`m` the effective mass of the two groups of bodies in contact
and :math:`\bar{k}` the stiffness linearized at ``damping_depth``.

Friction
--------

Coulomb friction, made smooth below ``friction_velocity``, opposes the
sliding velocity :math:`\underline{u}` at each point:

.. math::

   \underline{f}_t = -\mu f \frac{u}{\sqrt{|\underline{u}|^2 + u_f^2}}

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
:math:`\underline{z}` in the tangent plane, a tiny stretch of the surface between the two
bodies, that grows as they slide over each other and saturates:

.. math::

   \dot{\underline{z}} = \underline{u} - \sigma_0\,\frac{|\underline{u}|}{g(|\underline{u}|)}\,\underline{z}, \qquad
   g(|\underline{u}|) = \mu_c + (\mu_s - \mu_c)\,\exp\bigl(-(|\underline{u}|/v_s)^2\bigr)

for the sliding velocity :math:`\underline{u}` at the point, and the friction force on body
b is

.. math::

   \underline{f}_t = -f\,(\sigma_0 \underline{z} + \sigma_1 \dot{\underline{z}} + \sigma_2 \underline{u})

for the normal force :math:`f` at the point. The parameters are the stiffness
:math:`\sigma_0` of the bristles (``bristle_stiffness``, in 1/m: the sliding at
which the friction of a body at rest reaches a friction coefficient of 1), their
damping :math:`\sigma_1` (``bristle_damping``), a viscous friction
:math:`\sigma_2` (``viscous_friction``), the coefficient :math:`\mu_c` of the pair
(the Coulomb friction of the materials, ``friction`` of the pair, from the
materials or ``pairs``), the static friction :math:`\mu_s` (``static_friction``, at
least :math:`\mu_c`, default :math:`\mu_c`) and the Stribeck velocity :math:`v_s`
(``stribeck_velocity``), the sliding speed over which the friction falls from
static to Coulomb.

At rest the bristles hold: the deflection settles where :math:`\sigma_0 \underline{z}` carries
the tangential load, so a load of a fraction :math:`\eta` of the normal force gives
:math:`|\underline{z}| = \eta/\sigma_0`, and the body moves by that and no more, up to
:math:`\mu_s f`. Beyond it the body slides, with friction that falls from
:math:`\mu_s` to :math:`\mu_c` as it speeds up, and the force is a smooth function
of the state throughout.

The deflections of a pair are states of the module, three numbers for each of
``points`` slots (default 8), the k-th point of the contact (in the order the
contact finds them) having the k-th. A point that is not in contact relaxes to
zero over ``release_time``. A box on its face has four points, a line of contact
of a CAD part up to about ten. The module logs, for each pair, the output
``stick_<a>_<b>``: the largest :math:`|\underline{z}|\sigma_0/\mu_s` of its points, which is 1
at the limit of holding.

The bristles are stiff: they relax with a time constant :math:`g/(\sigma_0|\underline{u}|)` at
the sliding speed :math:`|\underline{u}|`, and the variable step integrator has to resolve
it. A stiffer :math:`\sigma_0` holds closer, with less travel, and takes smaller
steps while the body slides; a body that is suddenly loaded moves
:math:`\eta/\sigma_0` while the load comes on, and a Stribeck velocity smaller than
the speed it gains in that travel gives the friction no time to hold. Choose
:math:`\sigma_0` so that :math:`\mu_c/\sigma_0` is small against the sizes of the
motion.

``bristle_damping`` defaults to 5 s/m, because a body held on several points needs
some. Setting it to 0 is allowed, but the body may then creep though it is below the
limit of holding. The friction at a point is its normal force times the deflection of its bristles, so as a
body rocks the friction changes with it, which can feed energy into the rocking.
Without damping, the smallest disturbance can start a box on a slope rocking more and
more and slipping a little each time. The box of the example, at 0.58 of the limit of
holding, walks 4 mm down the slope in 6 s in the optimized build, where rounding is
enough to set it off, and stays put in the debug build. With :math:`\sigma_1` = 5 s/m it stays put, and the integrator takes a quarter of the
steps. A body nearer the limit needs more than the default: the box at 0.89 of the limit, with
:math:`\sigma_0` = 20000 1/m, needs about 500 s/m, where 150 s/m still lets it move. If
the output ``stick_<a>_<b>`` wanders while the body should be at rest, raise
:math:`\sigma_1`.

.. code:: javascript

   "stick_slip": [
     {"geometry_a": "ground", "geometry_b": "box", "bristle_stiffness": 2000,
      "bristle_damping": 5, "static_friction": 0.6, "stribeck_velocity": 0.05}
   ]

Each entry of ``stick_slip``:

.. shift-parameters:: shift/core/force_module/contact.cc
   :function: StickSlipPair::DeclareParameters

.. shift-parameters:: shift/core/force_module/contact.cc
   :function: BristleModel::DeclareParameters

With ``stick_slip_all``, every other pair of a model holds in the same way, without
being listed. Setting aside states for every possible pair would take a number that
grows with the square of the geometries, so the module keeps a fixed number of sets
of bristles, ``pairs``, for the pairs in contact at the time. A pair takes a free set
when it comes into contact and gives it back once it has left contact and its
bristles have relaxed, so a free set always has no deflection and it does not matter
which one a pair takes. If no set is free, the pair has regularized friction, and the
module warns once. Give ``pairs`` at least the number of pairs that touch at once.
The friction of a pair comes from its materials, and the static friction is
:math:`\mu_s` = ``static_ratio`` :math:`\mu_c`. A pair listed in ``stick_slip`` keeps its
own bristles. The module logs ``stick_all``, the largest ratio over the sets, and
``stick_pairs``, the number of sets in use.

.. code:: javascript

   "stick_slip_all": {"bristle_stiffness": 2000, "bristle_damping": 5,
                      "static_ratio": 1.2, "stribeck_velocity": 0.05,
                      "pairs": 32}

Its parameters, with those of the bristles above:

.. shift-parameters:: shift/core/force_module/contact.cc
   :function: StickSlipAll::DeclareParameters

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
* :source:`examples/collision/stick_slip_all.json`: the same box with no pair named,
  held by a set of bristles from ``stick_slip_all``.
* Tests: :source:`test/test_contact_force.cc`,
  :source:`test/test_contact_geometry.cc` and :source:`test/test_stick_slip.cc`.
