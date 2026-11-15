.. _dem_theory:
.. title:: Discrete Element Method

Discrete Element Method
=======================

The discrete element method (DEM) models a granular material, such as sand,
gravel, ore or seeds, as a large number of rigid grains that touch, push and
rub on each other :cite:`cundall1979`. Each grain is a rigid body with its own
mass and inertia; the forces between grains come from contact laws at the
points where they touch. Unlike the SPH soil, which treats the soil as a
continuum carried by particles (see :doc:`../force-modules/soil`), DEM
resolves the grains themselves, so that a heap, a flow through a hopper or
the pile in front of a blade follows from the shapes and the friction of the
grains rather than from a constitutive law of the bulk.

The method follows the soft-sphere DEM of Chrono, as implemented in
Chrono::GPU :cite:`fang2021` and Chrono DEM-Engine :cite:`zhang2024`, the GPU
modules of Project Chrono :cite:`tasora2016`. The grains interpenetrate
slightly at a contact, and the overlap, its rate and the history of the
sliding at the contact give the force. The equations of motion of each grain
are then integrated explicitly with a step small enough to resolve each
collision. Every grain is advanced independently of every other, from the
forces of its contacts, which is why the method maps well onto a graphics
processor (GPU): one invocation of a compute shader per grain or per contact,
thousands at a time. Chrono runs these computations in CUDA; *Shift* runs
them as Vulkan compute shaders, as it does the SPH soil, so that they run on
any GPU with a Vulkan driver.

The sections below describe the grains, the contact law, the equations of
motion and their integration, the search for contacts, how the computation
is laid out on the GPU and how the grains exchange forces with the bodies of
a multibody model. The notation follows the :ref:`notation` page: the fixed
inertial reference frame is :math:`f`, a superscript is the coordinate
system of a vector, a subscript gives the frames from and to, and a value
after a comma names a point.


.. _dem_grains:

Grains
------

A grain is a single sphere, a *clump*, a rigid union of spheres
that may overlap each other, used to give grains angular shapes and
interlocking :cite:`zhang2024`, or a convex hull (see
:ref:`dem_hulls`). Grain :math:`a` has its own coordinate
system, also denoted :math:`a`, with its origin at the center of mass and
its axes along the principal axes of inertia, so that its centroidal inertia
:math:`\bar{I}_{aa}` is diagonal. Its state is

* the position :math:`\underline{r}_{fa}^{f}` of its center of mass,
* its orientation, the unit quaternion :math:`\hat{e}^{fa}` (scalar part last,
  as in KSL) of the rotation matrix
  :math:`R^{fa}`, which transforms the coordinates of a vector from :math:`a`
  to :math:`f`,
* the velocity :math:`\underline{v}_{fa}^{f}` of its center of mass, and
* its angular velocity :math:`\underline{\omega}_{fa}^{a}`, in the
  coordinates of the grain, the coordinates in which the inertia is
  constant and diagonal.

Together they form the velocity screw of the grain at its center of mass,
:math:`\mathbf{\underline{v}}_{fa} = \{\underline{v}_{fa}^{f};\
R^{fa}\underline{\omega}_{fa}^{a}\}`.

Sphere :math:`s` of a clump has radius :math:`r_s` and its center at the
fixed offset :math:`\underline{r}_{as}^{a}` from the center of mass of the
clump, so that in the fixed frame

.. math::
    :label: eq:dem_sphere_center

    \underline{r}_{fs}^{f} = \underline{r}_{fa}^{f} + R^{fa}\,\underline{r}_{as}^{a}

A single sphere is a clump of one sphere with
:math:`\underline{r}_{as}^{a} = \underline{0}`. The mass :math:`m_a` and
inertia :math:`\bar{I}_{aa}` of a clump are given with its spheres, as with
any body of a model (see :doc:`../model-file/model-file`); the density of
the material is only a convenience for computing them. Clump shapes are
templates: a template holds the offsets, radii and materials of its spheres
and the mass and inertia, and each grain refers to its template, so that
millions of grains of a few shapes need little memory.


Contact Kinematics
------------------

Two spheres :math:`s` of grain :math:`a` and :math:`t` of grain :math:`b`
touch when the distance between their centers is less than the sum of their
radii. With the vector between the centers
:math:`\underline{r}_{ts}^{f} = \underline{r}_{fs}^{f} - \underline{r}_{ft}^{f}`, the
unit normal :math:`\underline{n}`, pointing from :math:`b` to :math:`a`, and
the overlap :math:`d` are

.. math::
    :label: eq:dem_overlap

    \underline{n} = \frac{\underline{r}_{ts}^{f}}{|\underline{r}_{ts}^{f}|},
    \qquad
    d = r_s + r_t - |\underline{r}_{ts}^{f}|

and the spheres are in contact while :math:`d > 0`. The contact point
:math:`p` is taken in the middle of the overlap, on the line between the
centers,

.. math::
    :label: eq:dem_contact_point

    \underline{r}_{fp}^{f} = \underline{r}_{ft}^{f} +
    \left(r_t - \frac{d}{2}\right)\underline{n}

The velocity of the material of grain :math:`a` at :math:`p` follows from
the velocity screw of the grain,

.. math::
    :label: eq:dem_point_velocity

    \underline{v}_{fa,p}^{f} = \underline{v}_{fa}^{f} +
    \tilde{\omega}_{fa}^{f}\,\underline{r}_{ap}^{f},
    \qquad
    \underline{\omega}_{fa}^{f} = R^{fa}\underline{\omega}_{fa}^{a},
    \qquad
    \underline{r}_{ap}^{f} = \underline{r}_{fp}^{f} - \underline{r}_{fa}^{f}

and likewise for :math:`b`. The relative velocity of :math:`a` with respect
to :math:`b` at the contact, :math:`\underline{u}`, has a normal component
:math:`u_n` and a tangential part :math:`\underline{u}_t`:

.. math::
    :label: eq:dem_relative_velocity

    \underline{u} = \underline{v}_{fa,p}^{f} - \underline{v}_{fb,p}^{f},
    \qquad
    u_n = \underline{n}\cdot\underline{u},
    \qquad
    \underline{u}_t = \underline{u} - u_n\,\underline{n}

A negative :math:`u_n` means the spheres approach. The same expressions
hold for a sphere against the surface of a body of the multibody model (see
:ref:`dem_bodies`), with :math:`\underline{n}` the outward normal of the
surface at its point nearest the center of the sphere and :math:`d` the
radius less the signed distance of the center from the surface.


.. _dem_contact:

Contact Forces
--------------

The contact law is the Hertz–Mindlin model with tangential displacement
history used by Chrono :cite:`fang2021,zhang2024,fleischmann2016`: Hertz's
normal force between elastic spheres :cite:`hertz1882,johnson1985`,
Mindlin's tangential stiffness :cite:`mindlin1949,mindlin1953`, and damping
chosen from a coefficient of restitution :cite:`tsuji1992`. Each contact
depends on the radii, the masses and the materials of the pair through the
effective radius, mass, Young's modulus and shear modulus

.. math::
    :label: eq:dem_effective

    \frac{1}{r^*} = \frac{1}{r_s} + \frac{1}{r_t},
    \qquad
    \frac{1}{m^*} = \frac{1}{m_a} + \frac{1}{m_b},
    \qquad
    \frac{1}{e^*} = \frac{1-\nu_a^2}{e_a} + \frac{1-\nu_b^2}{e_b},
    \qquad
    \frac{1}{g^*} = \frac{2(2-\nu_a)(1+\nu_a)}{e_a} +
                    \frac{2(2-\nu_b)(1+\nu_b)}{e_b}

with :math:`e_a`, :math:`\nu_a` the Young's modulus and Poisson's ratio of
the material of :math:`s` (the masses are those of the whole grains, not of
the spheres). For a sphere on a body, the radius and mass of the body are
infinite, so that :math:`r^* = r_s` on a plane and :math:`m^* = m_a`. The
radius of the circle of contact, :math:`r_c = \sqrt{r^* d}`, sets the
stiffnesses of the contact.

Normal force
************

The normal force on :math:`a` is a nonlinear spring and damper along
:math:`\underline{n}`,

.. math::
    :label: eq:dem_normal

    f_n = k_n d - c_n u_n,
    \qquad
    k_n = \tfrac{4}{3}\, e^* r_c,
    \qquad
    c_n = -2\sqrt{\tfrac{5}{6}}\,\beta\sqrt{2 e^* r_c\, m^*},
    \qquad
    \beta = \frac{\ln\varepsilon}{\sqrt{\ln^2\varepsilon + \pi^2}}

so that the elastic part is Hertz's force,
:math:`k_n d = \tfrac{4}{3} e^* \sqrt{r^*} d^{3/2}`. The damping coefficient
grows with the stiffness of the contact, :math:`2 e^* r_c` being the
tangent stiffness :math:`\partial (k_n d) / \partial d`, and follows from the
coefficient of restitution :math:`\varepsilon` of the pair through
:math:`\beta \le 0`, so that a collision rebounds with approximately
:math:`\varepsilon` for any impact speed :cite:`tsuji1992,kruggelemden2007`.
The normal force on :math:`b` is opposite. As in Chrono, the force is not
clipped at zero: as the spheres separate quickly, the damping can make it
slightly attractive near the end of a collision, which is what gives the
rebound its coefficient of restitution.

Tangential force
****************

Friction requires the contact to remember how far it has been sheared since
the grains first touched. Without this history a grain cannot rest on a slope
or carry a shear load, and a direct shear test of a granular sample comes
out wrong :cite:`fleischmann2016`. Each contact carries the tangential
displacement :math:`\underline{\xi}`, zero when the contact forms, and
updated at each step of duration :math:`h` from the tangential velocity, then
returned to the tangent plane, which turns as the grains roll:

.. math::
    :label: eq:dem_history

    \underline{\xi}' = \underline{\xi} + h\,\underline{u}_t,
    \qquad
    \underline{\xi} \leftarrow \underline{\xi}' -
      (\underline{n}\cdot\underline{\xi}')\,\underline{n}

The tangential force on :math:`a` is a spring and damper on this displacement,

.. math::
    :label: eq:dem_tangential

    \underline{f}_t = -k_t\,\underline{\xi} - c_t\,\underline{u}_t,
    \qquad
    k_t = 8\, g^* r_c,
    \qquad
    c_t = -2\sqrt{\tfrac{5}{6}}\,\beta\sqrt{k_t\, m^*}

with :math:`k_t` the tangential stiffness of Mindlin :cite:`mindlin1949`.
(Chrono::GPU uses this :math:`c_t`; the default model of DEM-Engine has
:math:`\sqrt{5/3}` in place of :math:`\sqrt{5/6}`.) Coulomb's law bounds the
force by the coefficient of friction :math:`\mu` of the pair. When the force
exceeds :math:`\mu f_n`, it is scaled back onto the bound and the displacement
is set to the one that, with the same velocity, gives the scaled force, so
that the contact slides without storing more spring than it can hold
:cite:`fleischmann2016,zhang2024`:

.. math::
    :label: eq:dem_coulomb

    \text{if } |\underline{f}_t| > \mu f_n: \qquad
    \underline{f}_t \leftarrow \mu f_n\,
      \frac{\underline{f}_t}{|\underline{f}_t|},
    \qquad
    \underline{\xi} \leftarrow
      -\frac{\underline{f}_t + c_t\,\underline{u}_t}{k_t}

The displacement is a state of the contact, not of the grains: it lives as
long as the two spheres overlap and is dropped when they part (see
:ref:`dem_history_mapping`).

Rolling resistance
******************

Real grains are not spheres, and a sphere on a flat surface rolls forever.
Rolling resistance stands in for the shape: a pair of equal and opposite
moments that resist the rolling of :math:`a` and :math:`b` on each other.
*Shift* uses the constant torque model of Chrono::GPU :cite:`fang2021,ai2011`,
about the part in the tangent plane of the angular velocity of :math:`b`
relative to :math:`a`:

.. math::
    :label: eq:dem_rolling

    \underline{w} = \underline{\omega}_{fb}^{f} - \underline{\omega}_{fa}^{f},
    \qquad
    \underline{w}_t = \underline{w} - (\underline{n}\cdot\underline{w})\,\underline{n},
    \qquad
    \underline{\tau}_r = \mu_r\, |f_n|\, r^*\,
      \frac{\underline{w}_t}{|\underline{w}_t|}

where :math:`\mu_r` is the dimensionless coefficient of rolling resistance of
the pair. The moment :math:`\underline{\tau}_r` acts on :math:`a` and
:math:`-\underline{\tau}_r` on :math:`b`, so that the pair keeps its angular
momentum, with the effective radius :math:`r^*` as the arm of the resistance
:cite:`ai2011`; on a wall, :math:`r^*` is the radius of the sphere, as in
Chrono. The rotation of the two grains about the normal (spinning) is not
resisted. As in Chrono, the moment is applied only once the contact has lasted
longer than the duration of an elastic collision,

.. math::
    :label: eq:dem_rolling_delay

    t = \pi\sqrt{\frac{m^*}{k(1 - \zeta^2)}},
    \qquad
    k = \tfrac{4}{3}e^*\sqrt{r^*},
    \qquad
    \zeta = \frac{-2\sqrt{\tfrac{5}{3} m^* e^*}\,\beta\,{r^*}^{1/4}}{2\sqrt{k\, m^*}}

so that it resists sustained rolling and not the spin a grain picks up in a
brief impact. (DEM-Engine instead resists the velocity at the contact point
that the two rotations give, :math:`\tilde{\omega}_{fb}^{f}\underline{r}_{bp}^{f}
- \tilde{\omega}_{fa}^{f}\underline{r}_{ap}^{f}`, which vanishes when two grains
roll on each other like gears :cite:`zhang2024`.) The constant moment
reverses with the rolling, so a grain held by it at rest rocks by a tiny angle
each step; the elastic–plastic spring–damper model of :cite:`ai2011`, which
carries an angular displacement like :math:`\underline{\xi}`, does not.

Contact wrench
**************

The force on :math:`a` at :math:`p` is
:math:`\underline{f} = f_n\,\underline{n} + \underline{f}_t`, and its wrench
about the center of mass of :math:`a`, in the coordinates of the fixed frame,
is the coscrew

.. math::
    :label: eq:dem_wrench

    \mathbf{\underline{f}}^*_{a} = \begin{Bmatrix}
      \underline{f} \\
      \tilde{r}_{ap}^{f}\,\underline{f} + \underline{\tau}_r
    \end{Bmatrix},
    \qquad
    \mathbf{\underline{f}}^*_{b} = \begin{Bmatrix}
      -\underline{f} \\
      -\tilde{r}_{bp}^{f}\,\underline{f} - \underline{\tau}_r
    \end{Bmatrix}

Contacts between spheres of the same clump are not evaluated. Two
clumps may touch at several pairs of spheres at once; each pair is a contact
of its own, with its own history. Where the spheres of a clump overlap, two
pairs can report what is physically one contact, which stiffens it; the
spheres of a template should overlap little where they meet the surface of
the clump :cite:`berry2023`.

Walls
*****

A wall is a fixed plane, :math:`\underline{n}_w\cdot\underline{p} \le o`
for the solid, with :math:`\underline{n}_w` the unit normal out of it, as a
plane of collision geometry (see :doc:`../contact/surfaces`). A sphere touches
it when the height of its center above the plane is less than its radius:

.. math::
    :label: eq:dem_wall

    d = r_s - (\underline{n}_w\cdot\underline{r}_{fs}^{f} - o),
    \qquad
    \underline{n} = \underline{n}_w

The contact law is that of two spheres with the radius and mass of the wall
infinite, :math:`r^* = r_s` and :math:`m^* = m_a`, the wall's material in
:math:`e^*`, :math:`g^*` and the pair's coefficients, and the wall at rest.
Each sphere keeps a tangential displacement for each wall.

.. _dem_hulls:

Convex hulls
************

A grain can also be a convex polyhedron, the hull of the vertices
:math:`\underline{x}_{i}^{a}`, in the coordinates of the grain, with the
planes of its faces :math:`\underline{n}_k^{a}\cdot\underline{x}^{a} \le
o_k` and the polygon of each face, its vertices counterclockwise about
:math:`\underline{n}_k^{a}`. Its mass properties are those of the solid hull
(Eberly's polyhedral integrals), and its coordinates are its principal axes
from its center of mass, as for a clump. For the search the hull is the
sphere about its center of mass that holds it, of radius
:math:`R_a = \max_i|\underline{x}_i^{a}|`; for the contact law, the step
and the margin it is the sphere of its volume :math:`V_a`,

.. math::
    :label: eq:dem_hull_radius

    r_a = \left(\frac{3V_a}{4\pi}\right)^{1/3}

Two grains :math:`a` and :math:`b`, of which at least one is a hull and the
other a hull or a sphere, overlap where the Minkowski difference of the two
holds the origin. With :math:`\underline{c} = \underline{r}_{fa}^{f} -
\underline{r}_{fb}^{f}` the center of :math:`a` from that of :math:`b`, the
point of a hull farthest along the direction :math:`\underline{d}` (its
support) and the support of the difference are

.. math::
    :label: eq:dem_hull_support

    \underline{s}_a(\underline{d}) = R^{fa}\,\underline{x}_{j}^{a},\quad
      j = \arg\max_i\,(R^{af}\underline{d})\cdot\underline{x}_i^{a},
    \qquad
    \underline{s}(\underline{d}) = \underline{c} +
      \underline{s}_a(\underline{d}) - \underline{s}_b(-\underline{d})

and a sphere's is :math:`r_s\underline{d}/|\underline{d}|`. Minkowski
portal refinement :cite:`snethen2008`, as Chrono's ``ChNarrowphaseMPR``,
starts from :math:`\underline{c}`, inside the difference, and finds a
triangle of support points :math:`(\underline{v}_1, \underline{v}_2,
\underline{v}_3)`, the portal, that the ray from :math:`\underline{c}`
through the origin passes. It then replaces one of them by the support
:math:`\underline{v}_4 = \underline{s}(\hat{\underline{d}})` along the
portal's outward normal :math:`\hat{\underline{d}}` until the portal is
within the tolerance of the surface,

.. math::
    :label: eq:dem_hull_mpr

    \min_{j=1,2,3}\,(\underline{v}_4 - \underline{v}_j)\cdot
      \hat{\underline{d}} \le \epsilon,
    \qquad \epsilon = 10^{-6}(R_a + R_b)

or for at most 32 iterations; it stops without a contact as soon as a
support does not pass the origin, :math:`\underline{v}\cdot
\hat{\underline{d}} \le 0`. The overlap and the normal from :math:`b` to
:math:`a` are those of the portal's plane,

.. math::
    :label: eq:dem_hull_depth

    d = \underline{v}_1\cdot\hat{\underline{d}},
    \qquad \underline{n} = -\hat{\underline{d}}

The point of contact of two hulls is the center of the patch where their
faces meet. The face of each most nearly facing the other,
:math:`\max_k (R^{fb}\underline{n}_k^{b})\cdot\underline{n}` on
:math:`b` and :math:`\max_k -(R^{fa}\underline{n}_k^{a})\cdot\underline{n}`
on :math:`a`, is found, and the better aligned of the two is the
reference, with outward normal :math:`\underline{N}` and vertices
:math:`\underline{P}_j`; the other, the incident face, is clipped against
the planes through the reference's sides (Sutherland and Hodgman
:cite:`sutherland1974`), keeping each point :math:`\underline{q}` with
:math:`(\underline{q} - \underline{P}_j)\cdot((\underline{P}_{j+1} -
\underline{P}_j)\times\underline{N}) \le 0`. Each point
:math:`\underline{q}_k` of the clipped polygon below the reference face, at
depth :math:`\delta_k = \underline{N}\cdot(\underline{P}_0 -
\underline{q}_k) > 0`, takes part with its depth as weight, half its depth
below the face:

.. math::
    :label: eq:dem_hull_point

    \underline{p} = \frac{\sum_k \delta_k\,(\underline{q}_k +
      \tfrac{1}{2}\delta_k\underline{N})}{\sum_k \delta_k},
    \qquad
    \rho^2 = \frac{\sum_k \delta_k\,|\underline{t}_k|^2}{\sum_k
      \delta_k},\quad
    \underline{t}_k = (\underline{q}_k - \underline{p}) -
      \left((\underline{q}_k - \underline{p})\cdot\underline{N}\right)
      \underline{N}

A tilted face touches only along its lower side, where its points are
deepest, so the point moves toward that side and the contact holds the
grain up as a face does, not on a single corner. With no clipped point
below the face (two edges crossing), or a sphere in the pair, the point is
the midpoint of the two deepest points,
:math:`\tfrac{1}{2}(\underline{c} + \underline{s}_a(-\underline{n}) +
\underline{s}_b(\underline{n}))`, and :math:`\rho = 0`. The contact law is
that of two spheres :eq:`eq:dem_normal`, with the radii
:eq:`eq:dem_hull_radius` in :math:`r^*` (a sphere's own radius) and the
arms from the centers of mass to :math:`\underline{p}`.

One point carries the force of a whole patch, and the dampers at that point
cannot resist the grain rocking about it, nor spinning about the normal,
which a patch of dampers would. The damping :math:`c_n` of the normal law
:eq:`eq:dem_normal` and, with friction, :math:`c_t` of the tangential law
:eq:`eq:dem_tangential` are therefore spread over the patch, as dampers at
its points in proportion to their weights: their sums are :math:`c_n u_n`
and :math:`c_t \underline{u}_t` as before, and the moment they add on
:math:`a` is

.. math::
    :label: eq:dem_hull_rocking

    \underline{m}_a = c_n\,\rho^2\left(\underline{\omega}_{ba} -
      (\underline{\omega}_{ba}\cdot\underline{n})\underline{n}\right)
      + c_t\,\rho^2\,(\underline{\omega}_{ba}\cdot\underline{n})\,
      \underline{n},
    \qquad
    \underline{\omega}_{ba} = R^{fb}\underline{\omega}_{fb}^{b} -
      R^{fa}\underline{\omega}_{fa}^{a}

with the opposite on :math:`b`, as for rolling resistance; the second term
is absent without friction (:math:`\mu = 0`). The first term is the moment
of the normal dampers, :math:`c_n` times the second moment of the patch
about the point, for a rocking about a tangent axis; the second that of the
tangential dampers, which a spin about the normal slides at
:math:`|\underline{t}_k|\,\omega_n` across the arms
:math:`\underline{t}_k`. Without the first two cubes stacked on a floor
rocked for the length of the test. Without the second, nothing resisted a
cube's spin on a face, which friction at a single point under its center
cannot: the spin that rounding left after the cube landed, different with
and without fused multiply-adds, turned it on the floor for as long as the
test ran.

A hull touches a wall with its vertices beyond it, each at depth
:math:`\delta_i = o - \underline{n}_w\cdot\underline{r}_{fi}^{f} > 0`: the
overlap is the deepest, :math:`d = \max_i\delta_i`, the point is their mean
weighted by depth, and :math:`\rho^2` their spread about it, as in
:eq:`eq:dem_hull_point`.

Two grains whose bounding spheres overlap are evaluated as above; most such
pairs of hulls do not touch, and the refinement stops at its first support.
Each pair is evaluated by both grains with the lower index first, as for
spheres (see :ref:`dem_gpu_sums`), so that the forces are exactly opposite
and the GPU repeats its runs bit for bit. Hulls have at most 32 vertices, 64
faces and 12 vertices a face, and take the smooth method only. A hull
touches the triangles of bodies as described in :ref:`dem_hull_triangle`.

``Template::OfHull`` makes the template of the hull of given points (with
qhull), its mass and inertia of a density, and ``Template::ClumpOfHull``
the clump of spheres that fills it: spheres inscribed in the hull, centered
on a lattice of 12 along its longest side, each in turn the one that covers
the most points of a lattice of 32 not yet covered. The clump keeps the
hull's mass, inertia and triangles, which the viewer draws, so that the two
kinds of grain look alike; ``RockPoints`` gives the points of a rock, on an
ellipsoid of random semi-axes, moved in and out at random.


Equations of Motion
-------------------

Each grain is a free rigid body. Its equations of motion are those of
:doc:`../dynamics/dynamics` for a single body at its center of mass, with the
translation in the fixed frame and the rotation in the coordinates of the
grain, where :math:`\bar{I}_{aa}` is constant:

.. math::
    :label: eq:dem_eom

    m_a\,\dot{\underline{v}}_{fa}^{f} = \sum_k \underline{f}_k + m_a\,\underline{g},
    \qquad
    \bar{I}_{aa}\,\dot{\underline{\omega}}_{fa}^{a} +
      \tilde{\omega}_{fa}^{a}\,\bar{I}_{aa}\,\underline{\omega}_{fa}^{a} =
      R^{af} \sum_k \underline{\tau}_k

where the sums run over the contacts :math:`k` of the grain, with
:math:`\underline{f}_k` and :math:`\underline{\tau}_k` the force and moment
parts of their wrenches :eq:`eq:dem_wrench`, :math:`R^{af} = R^{faT}`, and
:math:`\underline{g}` is the acceleration of gravity
(:math:`\{0, 0, 9.81\}` m/s² with *z* down). For a sphere the inertia is
:math:`\tfrac{2}{5} m_a r_s^2 I` and the gyroscopic term
:math:`\tilde{\omega}\bar{I}\underline{\omega}` vanishes. DEM-Engine omits it
for clumps too; *Shift* keeps it, since a clump of unequal principal moments
precesses and, spun about its intermediate axis, tumbles.


Time Integration
----------------

The grains are integrated with the semi-implicit (symplectic) Euler method,
the *centered difference* integrator of Chrono::GPU :cite:`fang2021` and the
method of the SPH soil. With the accelerations from the forces at step
:math:`k`, the velocities are advanced first, and the positions and
orientations then advance with the new velocities:

.. math::
    :label: eq:dem_integration

    \begin{aligned}
    \underline{v}_{fa}^{f}(k+1) &= \underline{v}_{fa}^{f}(k) +
      h\,\dot{\underline{v}}_{fa}^{f}(k) \\
    \underline{\omega}_{fa}^{a}(k+1) &= \underline{\omega}_{fa}^{a}(k) +
      h\,\dot{\underline{\omega}}_{fa}^{a}(k) \\
    \underline{r}_{fa}^{f}(k+1) &= \underline{r}_{fa}^{f}(k) +
      h\,\underline{v}_{fa}^{f}(k+1) \\
    \hat{e}^{fa}(k+1) &= \hat{e}^{fa}(k)\,
      \exp\!\left\{\tfrac{h}{2}\,\underline{\omega}_{fa}^{a}(k+1);\ 0\right\}
    \end{aligned}

The orientation is advanced by the rotation through the angle
:math:`h|\underline{\omega}_{fa}^{a}|` about :math:`\underline{\omega}_{fa}^{a}`,
the exponential of the pure quaternion of half the rotation vector,
:math:`\{\hat{u}\sin\phi;\ \cos\phi\}` with
:math:`\phi = \tfrac{h}{2}|\underline{\omega}_{fa}^{a}|` and :math:`\hat{u}`
the unit vector along :math:`\underline{\omega}_{fa}^{a}`. It multiplies on
the right because the angular velocity is in the coordinates of the grain, and
the result is normalized after each step. The method is the leapfrog method
with the velocities taken at the half steps; it is symplectic, which keeps the
energy of an elastic system bounded instead of letting it drift, and it needs
a single evaluation of the forces per step, as every explicit DEM integrator
does.

Step size
*********

The explicit integration is stable, and each collision resolved, only if the
step is a small fraction of the time a contact lasts. Two estimates bound it.
The Rayleigh time is the time a surface wave takes to cross a grain
:cite:`li2005`,

.. math::
    :label: eq:dem_rayleigh

    t_R = \frac{\pi\, r_{\min}\sqrt{\rho/g}}{0.1631\,\nu + 0.8766}

with :math:`r_{\min}` the smallest radius, :math:`\rho` the density and
:math:`g = e/(2(1+\nu))` the shear modulus of the softest material, and the
duration of a Hertz collision at the impact speed :math:`u_0`
:cite:`johnson1985` is

.. math::
    :label: eq:dem_collision_time

    t_c = 2.87\left(\frac{{m^*}^2}{r^*\, {e^*}^2\, u_0}\right)^{1/5}

The step is :math:`h = c_h \min(t_R, t_c)`, with :math:`c_h` of 0.1 to 0.3,
and :math:`u_0` the largest speed expected in the model
(``dem::StableStep``). As in Chrono, the Young's modulus of the grains is
commonly taken a hundred to a thousand times lower than that of the real
material, which lengthens the step by a factor of :math:`(e/e')^{2/5}` to
:math:`(e/e')^{1/2}` while changing the bulk behavior little as long as the
overlaps stay well below a percent of the radius.


Contact Search
--------------

Each step needs the pairs of spheres that overlap. Testing every pair is out
of the question for millions of grains; *Shift* finds them with a uniform
grid, as Chrono::GPU does with its subdomains :cite:`fang2021` and the SPH
soil does with its cells :cite:`green2010,teschner2003`, and keeps them for
many steps with a margin, as DEM-Engine does with its *delayed contact
detection* :cite:`zhang2024,verlet1967`.

Grid and sort
*************

Space is divided into cubic cells of width :math:`w = 2 r_{\max} + m`, with
:math:`r_{\max}` the largest radius and :math:`m` the margin. Each sphere
belongs to the cell containing its center, so two spheres within
:math:`r_s + r_t + m` of each other are in the same or adjacent cells, and
each sphere need examine only the 27 cells around its own. The cells are
hashed into a table of a power of two entries, at least twice the number of
spheres :cite:`teschner2003`, so that only occupied cells take memory. The
spheres are then sorted by cell, and within a cell by their index, with a
counting sort: count the spheres of each cell, take the running sum of the
counts (a parallel prefix sum, ``gpu::Scan``) to find where each cell starts,
place each sphere in its cell's range, and sort the few spheres of each cell
by index.

Cells sized for the largest sphere hold many small ones when the sizes vary
widely. DEM-Engine then places a sphere in every cell its enlarged bounding
box overlaps, in cells sized for the small spheres (eight times the smallest
radius by default :cite:`zhang2024`); *Shift* does not yet.

Neighbors and the margin
************************

Each sphere keeps a list of its *neighbors*: the spheres of other grains whose
distance is less than :math:`r_s + r_t + m`, in the order of their indices,
up to ``max_neighbors`` of them (a sphere with more is an error, rather than
a contact silently lost), as Chrono::GPU keeps the spheres each sphere
touches. A pair is in the lists of both its spheres. Because the margin admits
pairs that are not yet in contact, the lists stay complete for as many steps
as it takes any two spheres to close the margin. With :math:`\Delta_{\max}`
the largest displacement of any sphere since the lists were made, measured
on the GPU at each step, they are made again once

.. math::
    :label: eq:dem_skin

    2\,\Delta_{\max} \ge m

the criterion of Verlet's neighbor lists :cite:`verlet1967`. The margin trades
the cost of the search against the cost of evaluating pairs not in contact:
DEM-Engine sizes it from the speeds of the grains, the step and the number of
steps between searches :cite:`zhang2024`; *Shift* takes it from the settings,
or a fifth of the smallest radius. A pair of a list whose overlap
:eq:`eq:dem_overlap` is not positive gives no force, and its history is set
to zero.

.. _dem_history_mapping:

Carrying the history across searches
************************************

The tangential displacements belong to the pairs, so a new list inherits
them from the old one. Both lists of a sphere are in the order of the indices
of the neighbors, so one pass through the two finds each neighbor of the new
list in the old, if it is there: it takes the old :math:`\underline{\xi}` and
the duration of the contact, and a new neighbor starts from zero. A neighbor
of the old list missing from the new one is a contact that has ended.


Computation on the GPU
----------------------

The computation is a fixed sequence of passes, each a compute shader written
in GLSL (``shift/core/dem/shaders``), compiled to SPIR-V with ``glslc`` and
run through the headless Vulkan layer of ``shift/core/gpu``, as the SPH soil
is. Up to 256 steps go in one submission, with the bodies' poses at each of
them, so that the host waits for the GPU once per submission: it then reads
back the flags of the search and, after the last, the bodies' impulses.

.. list-table:: Passes of a step
   :header-rows: 1
   :widths: 25 75

   * - pass
     - work, one invocation per
   * - spheres
     - sphere: its center, arm, velocity and angular velocity from its grain,
       Eq. :eq:`eq:dem_sphere_center`; whether it has moved half the margin,
       Eq. :eq:`eq:dem_skin`
   * - triangles
     - triangle: its vertices from its body's pose at the substep; whether
       it has moved half the margin
   * - count, scatter, cell sort
     - sphere, triangle or cell: the counting sorts of the spheres and of the
       triangles by cell
   * - neighbors
     - sphere: its neighbors and the triangles near it in the 27 cells
       around it, and their histories
   * - flags
     - one: count the search, and clear its request
   * - forces
     - sphere: the force on it and the moment on its grain from each
       neighbor, wall and triangle, Eqs. :eq:`eq:dem_overlap` to
       :eq:`eq:dem_wrench` and :eq:`eq:dem_triangle`; its impulse on each
       body, Eq. :eq:`eq:dem_body_wrench`
   * - grains
     - grain: the sum of the forces and moments on its spheres; its motion,
       Eqs. :eq:`eq:dem_eom` and :eq:`eq:dem_integration`

Whether a step searches is decided on the GPU: the search passes run at every
step and return at once unless a sphere has asked for a search, so the host
need not wait to find out.

.. _dem_gpu_sums:

Repeatable sums
***************

Chrono adds each contact's force into its grains with atomic floating-point
additions. Their order varies from run to run, and floating-point addition is
not associative, so two runs of the same model drift apart. Atomic float
addition is also an optional feature of Vulkan. *Shift* adds no floats
atomically. As in Chrono::GPU, each sphere evaluates its own contacts, so
each pair is evaluated twice, once by each sphere, and each sphere adds its
contacts in the order of its neighbors; each grain then adds the forces of its
spheres in order. Both spheres evaluate a pair with the same arithmetic, the
sphere of the lower index first, and take the force and moment of their own
side: the forces are exactly opposite, so the pair keeps the momentum to the
last bit, and both copies of its history stay the same. A run gives the same
result bit for bit each time, and the CPU solver (``dem::CpuSolver``), doing
the same arithmetic in the same order, agrees with the GPU to the rounding of
the multiplications and additions the GPU's compiler fuses.

Positions
*********

Single-precision floats keep seven significant digits: about a micron at ten
meters from the origin, the size of the overlaps the contact law depends on.
Double precision runs at a small fraction of the speed of single precision on
consumer GPUs. Chrono::GPU stores positions as integers within subdomains,
and DEM-Engine as a 64-bit voxel index with three 16-bit offsets within the
voxel :cite:`fang2021,zhang2024`. *Shift* stores the center of each grain as
an anchor, three 32-bit integers counting cells of the grid (of width
:math:`w`), and a single-precision offset within the cell:

.. math::
    :label: eq:dem_anchor

    \underline{r}_{fa}^{f} = w\,\underline{\eta}_a + \underline{\delta}_a,
    \qquad
    \underline{r}_{fs}^{f} - \underline{r}_{ft}^{f} =
      w\,(\underline{\eta}_a - \underline{\eta}_b) +
      (\underline{\delta}_s - \underline{\delta}_t)

with :math:`\underline{\eta}_a` the integer anchor of grain :math:`a`,
:math:`\underline{\delta}_a` its offset, and
:math:`\underline{\delta}_s = \underline{\delta}_a + R^{fa}\underline{r}_{as}^{a}`
the offset of sphere :math:`s` from the anchor of its grain. The difference
of the anchors of two nearby grains is a small exact integer, so the vector
between two spheres, and their overlap, keep the precision of the offsets
wherever the grains are; an offset that leaves its cell after a step moves to
the next anchor. A wall is anchored by its point nearest the origin, so the
height of a sphere above it keeps that precision when the wall is a plane of
the coordinates, and otherwise loses it in proportion to the distance from
that point. Velocities, forces and histories are single precision, as in
Chrono.

Contact laws
************

DEM-Engine compiles user-written contact laws into its CUDA code at run time
:cite:`zhang2024`. *Shift* has the one law above, and a pair skips the
friction and the rolling resistance when their coefficients are zero.


.. _dem_bodies:

Coupling with the Multibody Model
---------------------------------

The ``dem`` force module couples the grains to the bodies of a model (see
:ref:`dem_model_file`). The grains touch a body on triangles fixed in it,
and push back on it with the opposite of the forces they take.

Triangles
*********

Each body the grains touch is a set of triangles in the coordinates of its
origin: those of its mesh geometry, as they are, or the hull of its collision
geometry of a convex hull or a box. A triangle with an edge longer than the
width :math:`w` of a cell is divided at the midpoints of its edges into four,
until none is, so that its box spans at most two cells along each axis and
the triangle is in at most eight cells. At each substep the vertices of the
triangles follow their body's pose, anchored as the grains are
:eq:`eq:dem_anchor`. The triangles are counted, sorted and hashed by cell in
a grid of their own, of the same cells as the spheres', and each sphere lists
the triangles within :math:`r_s + m` of its center in the 27 cells around its
own, each once and in the order of their indices, up to ``max_triangles`` of
them, with the tangential displacement of each carried across searches as
for the neighbors (see :ref:`dem_history_mapping`). The lists are made again
when a sphere or a vertex of a triangle has moved half the margin
:eq:`eq:dem_skin`.

Contact with a triangle
***********************

Sphere :math:`s` of grain :math:`a` touches triangle :math:`t` of body
:math:`b` at the point :math:`q` of the triangle nearest its center
:cite:`ericson2005`, where it overlaps it:

.. math::
    :label: eq:dem_triangle

    d = r_s - |\underline{r}_{qs}^{f}|,
    \qquad
    \underline{n} = \frac{\underline{r}_{qs}^{f}}{|\underline{r}_{qs}^{f}|}

with :math:`\underline{n}` from the triangle to the sphere. The contact point
:math:`p` is halfway into the overlap, :math:`\underline{r}_{qp}^{f} =
-\tfrac{1}{2}d\,\underline{n}`, and the body moves there with the velocity
of its origin :math:`o` and its angular velocity,

.. math::
    :label: eq:dem_body_velocity

    \underline{v}_{fb,p}^{f} = \underline{v}_{fb,o}^{f} +
      \tilde{\omega}_{fb}^{f}\,\underline{r}_{op}^{f}

The contact law is that of two spheres with the radius and mass of the body
infinite, :math:`r^* = r_s` and :math:`m^* = m_a`, as for a wall, with the
body's velocity in the relative velocity :eq:`eq:dem_relative_velocity` and
its angular velocity in the rolling resistance. Where a sphere is nearest an
edge or a vertex that several triangles of a body share, it finds the same
point on each; the contact counts once, at the first of them, so that a
sphere on a mesh is pushed as it is by a smooth surface.

.. _dem_hull_triangle:

A hull and a triangle
~~~~~~~~~~~~~~~~~~~~~

A hull :math:`a` touches triangle :math:`t` where the two overlap, found
with the refinement of :eq:`eq:dem_hull_mpr` and :eq:`eq:dem_hull_depth`
with the triangle as the shape :math:`b`: its support is its corner farthest
along :math:`\underline{d}`, and its interior point is :math:`q`, the
point of the triangle nearest the hull's center, so that the refinement
starts along :math:`\underline{r}_{qa}^{f}` as the contact of a sphere
does. A triangle has two faces, one along its normal :math:`\underline{N}_t`
(of its corners in order) and one against it, with its corners in the
reverse order, and the point of contact and the spread are those of
:eq:`eq:dem_hull_point`, the triangle's face the reference or the incident
face as it is better or worse aligned. A hull whose center is less than
:math:`R_a` from the triangle is evaluated; the triangles within
:math:`R_a + m` of it are in its list, as for a sphere.

A hull on a face of a body lies across several of its triangles, each of
which finds the face's normal and the hull's whole overlap; a hull on a
gently curved surface finds nearly the same normal on each. The contacts of
the hull with the triangles of one body are therefore gathered into
patches: a contact joins the patch of that body whose normal is most nearly
its own if :math:`\underline{n}\cdot\underline{n}_P > \cos 15^\circ`, and
otherwise starts a new one (at most 8 per hull). A patch :math:`P` pushes
the hull once, along the normal of its deepest contact, with that overlap,
at the mean of its contacts' points :math:`\underline{p}_j` (from the hull's
center) weighted by their sums of depths :math:`w_j = \sum_k\delta_k` of
:eq:`eq:dem_hull_point` (the overlap, for a point not from clipped faces),
with the spread of all their points about it:

.. math::
    :label: eq:dem_hull_patch

    d_P = \max_{j\in P} d_j,
    \qquad
    \underline{p}_P = \frac{\sum_{j\in P} w_j\,\underline{p}_j}
      {\sum_{j\in P} w_j},
    \qquad
    \rho_P^2 = \frac{\sum_{j\in P} w_j\,(\rho_j^2 +
      |\underline{p}_j|^2)}{\sum_{j\in P} w_j} - |\underline{p}_P|^2

Clipped against each triangle in turn, the hull's face is cut into the
parts over each, so that the patch's point is that of the whole face. The
contact law is that of a hull and a wall, :math:`r^* = R_a` and :math:`m^* =
m_a`, with the rocking damping :eq:`eq:dem_hull_rocking` of the spread and
the body's velocity at the point :eq:`eq:dem_body_velocity`. A patch keeps
its tangential history in the slot of its first triangle, taking that of
the first of its triangles that has one, so that a hull sliding from one
triangle of a face to the next keeps it. In the tests a cube across the four
triangles that meet at a plate's center rests at the height it rests at on
a wall, to the last digit printed (4.99583 mm, a 0.01 m cube of glass), and
a moving plate pushes a cube as it pushes a sphere. A hull whose center has
passed through a triangle is pushed out the far side, as a sphere is; the
triangles of a body should face the grains with some thickness behind them.

Impulses and the load on a body
*******************************

The force on the body is the opposite of that on the sphere, at :math:`p`,
and the moment of rolling resistance is opposite too, so that the wrench on
the body about its origin, in the coordinates of the fixed frame, is

.. math::
    :label: eq:dem_body_wrench

    \mathbf{\underline{f}}^*_{b,o} = \begin{Bmatrix}
      -\underline{f} \\
      -\tilde{r}_{op}^{f}\,\underline{f} - \underline{\tau}_r
    \end{Bmatrix}

Each sphere adds :math:`h\,\mathbf{\underline{f}}^*_{b,o}` of its contacts
with body :math:`b` to an impulse of its own for that body, so that no two
spheres write the same sum; after the substeps, the impulses of the spheres
are summed in a fixed order (``gpu::SegmentSum``), so that the body's impulse,
too, is the same on every run.

The grains need a step far shorter than the multibody integrator takes, so
they run as substeps between the integrator's accepted steps, as the SPH soil
does (see :doc:`../force-modules/soil`) and as Chrono couples DEM-Engine to
its vehicles :cite:`zhang2024`. After the accepted step from :math:`t_0` to
:math:`t_1`, the grains advance by whole substeps to :math:`t_1`, the rest of
a substep waiting for the next step. Over those substeps each body moves
along the cubic Hermite path between its poses and velocity screws at
:math:`t_0` and :math:`t_1` (``sph::Interpolate``). The mean of its impulse
over the substeps is the load on it, its moment transformed from the origin
of the body at :math:`t_1` to the origin of the fixed frame with
:math:`[Ad]^*`, and the force module applies that load to the body over the
next step.

The coupling is therefore explicit: the grains' load is a step behind the
body. For a light body on stiff grains the delay pumps energy into the
contact, and the body rattles on the grains unless the integrator's steps are
short against the period of the body on them, :math:`2\pi\sqrt{m_b/k}` for
a body of mass :math:`m_b` on grains of contact stiffness :math:`k` in all.
In practice the steps should be no longer than a few substeps, set with the
integrator's ``hmax``: a plate of 0.1 kg on glass beads of 50 MPa, about
:math:`k = 2\times10^5` N/m in all, comes to rest with steps of one substep
(40 µs) and keeps rattling at 0.1 to 0.2 m/s with steps of 100 µs.

.. _dem_model_file:

In a model
**********

.. code-block:: json

    {"type": "dem", "name": "gravel", "device": "gpu",
     "materials": [{"name": "rock", "youngs_modulus": 1e8,
                    "poisson_ratio": 0.3, "restitution": 0.4,
                    "friction": 0.6, "rolling_resistance": 0.05}],
     "grains": [{"name": "pebble", "radius": 0.01, "material": "rock",
                 "mass": 0.011}],
     "fill": [{"grain": "pebble", "min": [-0.5, -0.5, -0.5],
               "max": [0.5, 0.5, -0.02], "spacing": 0.025, "jitter": 0.1}],
     "walls": [{"normal": [0, 0, -1], "offset": 0, "material": "rock"}],
     "bodies": [{"body": "bucket", "geometry": ["bucket_mesh"],
                 "material": "rock"}]}

A shape of grains is a sphere, with ``radius``, ``material`` and ``mass``
(or ``density``), or a clump, with ``spheres`` (each with ``center``,
``radius`` and ``material``), ``mass`` and the principal moments of
``inertia``. Each box of ``fill`` places grains on a lattice of ``spacing``
from ``min`` to ``max``, moved at random by up to ``jitter`` spacings. Walls
are the planes ``normal`` · :math:`\underline{p}` = ``offset``. The substep is
``step``, or ``step_fraction`` of the shorter of :eq:`eq:dem_rayleigh` and
:eq:`eq:dem_collision_time` at ``max_speed``; ``margin``,
``max_neighbors`` and ``max_triangles`` are as above, and ``device`` is
``"gpu"`` or ``"cpu"``. ``examples/dem/plate_on_grains.json`` drops a plate
onto beads and clumps.


.. _dem_nsc:

Non-smooth Contact
------------------

The method above is the *smooth* contact method (SMC) of Chrono: the grains
are stiff springs and dampers, and the step must resolve each collision.
Chrono also offers the *non-smooth* contact method (NSC), in which the grains
are rigid and do not overlap, and the contact forces are impulses that keep
them apart over a step :cite:`anitescu2006,anitescu2010`. *Shift* has both,
chosen with ``contact_method``. NSC needs no stiffness, and its step is set
by how far a grain may move in a step, not by the duration of a collision;
in exchange each step solves a coupled problem over all the contacts at
once, where SMC evaluates each contact on its own.

Impulses at the contacts
************************

Over a step :math:`h`, contact :math:`k` between :math:`a` and :math:`b`,
with the normal :math:`\underline{n}_k` of :eq:`eq:dem_overlap` and two unit
tangents :math:`\underline{t}_{1k}` and :math:`\underline{t}_{2k}`, transmits
the impulse

.. math::
    :label: eq:dem_nsc_impulse

    \underline{P}_k = \gamma_{n,k}\,\underline{n}_k +
      \gamma_{1,k}\,\underline{t}_{1k} + \gamma_{2,k}\,\underline{t}_{2k}

to :math:`a` at :math:`p`, and :math:`-\underline{P}_k` to :math:`b`. The
velocities at the end of the step are those of the free motion, under
gravity and the gyroscopic moment of :eq:`eq:dem_eom`, changed by the
impulses:

.. math::
    :label: eq:dem_nsc_velocity

    \begin{aligned}
    \underline{v}_{fa}^{f}(k+1) &= \underline{v}_{fa}^{f}(k) + h\,\underline{g}
      + \frac{1}{m_a}\sum_k \underline{P}_k \\
    \underline{\omega}_{fa}^{a}(k+1) &= \underline{\omega}_{fa}^{a}(k)
      - h\,\bar{I}_{aa}^{-1}\tilde{\omega}_{fa}^{a}\bar{I}_{aa}
        \underline{\omega}_{fa}^{a}(k)
      + \bar{I}_{aa}^{-1} R^{af}\sum_k
        \left(\tilde{r}_{ap}^{f}\,\underline{P}_k + \underline{\rho}_k\right)
    \end{aligned}

where :math:`\underline{\rho}_k` is the impulse of the rolling resistance
below, and the positions and orientations then advance with the new
velocities as in :eq:`eq:dem_integration`. The relative velocity
:math:`\underline{u}_k` of :eq:`eq:dem_relative_velocity` at the end of the
step is thus linear in the impulses, :math:`\underline{u} = N\underline{\gamma}
+ \underline{u}^{\mathrm{free}}`, with :math:`\underline{\gamma}` the impulses
of all the contacts and :math:`N`, the Delassus operator, the change of the
relative velocities for unit impulses.

The impulse of a contact lies in its friction cone, the normal part pushing
and the tangential part at most :math:`\mu` times it:

.. math::
    :label: eq:dem_nsc_cone

    \mathcal{K}_k = \left\{\underline{\gamma}_k :\ \gamma_{n,k} \ge 0,\
      \sqrt{\gamma_{1,k}^2 + \gamma_{2,k}^2} \le \mu\,\gamma_{n,k}\right\}

The contacts are not only the touching pairs but every pair, wall and
triangle within the margin, with the gap :math:`-d \ge 0` between them. The
normal velocity at the end of the step may close the gap but no more,
:math:`u_n + b \ge 0`, with the bias

.. math::
    :label: eq:dem_nsc_bias

    b = \begin{cases}
      -d/h, & d \le 0 \\
      \max(-d/h,\ -v_r), & d > 0
    \end{cases}

so that a gap closes in the step and an overlap, left by the step before,
is pushed out at most at the recovery speed :math:`v_r` (``recovery_speed``,
0.01 m/s). Since the grains are rigid, a collision is plastic: the grains
stop on contact. The coefficient of restitution :math:`\varepsilon` of the
pair restores the bounce as Newton's law: where the normal velocity before
the step, :math:`u_n^-`, is an approach faster than ``min_bounce_speed``
:math:`v_\varepsilon` (0.15 m/s) that closes the gap within the step, the
bias becomes :math:`\min(b,\ \varepsilon\,u_n^-)`, so that the grains part
at :math:`-\varepsilon u_n^-`. Slower approaches stay plastic, so that a
resting grain does not chatter.

The impulses follow, as in Chrono, from Anitescu's convex relaxation of the
complementarity conditions :cite:`anitescu2006,anitescu2010`: they minimize

.. math::
    :label: eq:dem_nsc_qp

    \min_{\underline{\gamma}_k \in \mathcal{K}_k}\ \
      \tfrac{1}{2}\,\underline{\gamma}^T N\underline{\gamma} +
      \underline{\gamma}^T \underline{r},
    \qquad
    \underline{r}_k = \underline{u}_k^{\mathrm{free}} + b_k\,\underline{n}_k

a quadratic program over a product of cones. Its optimality conditions are
the *cone complementarity problem*: the impulse is in :math:`\mathcal{K}_k`,
the velocity :math:`\underline{u}_k + b_k\underline{n}_k` in its dual cone
:math:`\{u_n \ge \mu|\underline{u}_t|\}`, and the two are orthogonal. A
contact either sticks, with :math:`\underline{u}_k + b_k\underline{n}_k = 0`
and the impulse inside the cone; slides, with the friction at
:math:`\mu\gamma_n` opposite the sliding; or opens, with no impulse. The
relaxation is that a sliding contact separates at :math:`u_n + b =
\mu|\underline{u}_t|` rather than staying closed, a dilation that vanishes
as :math:`h \to 0` and is small where the sliding is slow, as it is in most
of a pile.

Rolling resistance
******************

The rolling resistance is a moment impulse :math:`\underline{\rho}_k` in the
tangent plane, on :math:`a` and opposite on :math:`b`, in the disk of the
normal impulse :cite:`tasora2013`,

.. math::
    :label: eq:dem_nsc_rolling

    \underline{n}\cdot\underline{\rho}_k = 0,
    \qquad
    |\underline{\rho}_k| \le \mu_r\, r^*\, \gamma_{n,k}

that stops the relative rolling :math:`\underline{w}_t` of
:eq:`eq:dem_rolling` while it can, and resists it with the largest moment
when it cannot. Unlike the moment of :eq:`eq:dem_rolling`, it holds a grain
at rest exactly, with no delay of a collision's duration and no rocking, as
static friction does. The spin about the normal is not resisted.

Solving for the impulses
************************

The quadratic program :eq:`eq:dem_nsc_qp` is solved by iterations that need
only products with :math:`N`, which are the velocity changes of the grains
for the impulses, accumulated contact by contact. Each contact is scaled by
:math:`\eta_k = 3/\operatorname{tr}N_{kk}`, the inverse of the mean of the
diagonal of its own block, and its rolling impulse by :math:`\eta_{r,k} =
2/\operatorname{tr}N_{r,kk}`. ``nsc_solver`` chooses among

* ``"gauss_seidel"``, the projected Gauss-Seidel method: each contact in
  turn takes :math:`\underline{\gamma}_k \leftarrow
  \Pi_{\mathcal{K}_k}\left(\underline{\gamma}_k - \omega\,\eta_k(\underline{u}_k
  + b_k\underline{n}_k)\right)` from the velocities of the impulses so far,
  :math:`\Pi_{\mathcal{K}}` being the projection onto the cone of Chrono
  (``ChConstraintTwoTuplesFrictionT::Project``), and then its rolling
  impulse, from the spin its new impulse leaves, projected onto the disk
  :eq:`eq:dem_nsc_rolling`;
* ``"jacobi"``, the projected Jacobi method :cite:`anitescu2010`, the same
  update for every contact at once from the velocities of the last
  iteration, with mass splitting :cite:`tonge2012`: each contact steps as
  if it held only its share of each grain or body it touches, the inverse
  mass and inverse inertia of a grain of :math:`n` contacts taken
  :math:`n` times in :math:`\eta_k` and :math:`\eta_{r,k}`,

  .. math::
      :label: eq:dem_nsc_splitting

      N_{kk} = \sum_{a} n_a\,J_{ka}M_{aa}^{-1}J_{ka}^T

  over the grains (and bodies) :math:`a` of contact :math:`k`, while the
  impulses still change the velocities by the full masses. Neighboring
  contacts that would otherwise each push a grain as if alone, and
  together overshoot, then sum to at most one push, and the relaxation is
  :math:`\omega = 1.3`, the fastest in the tests below (without splitting,
  as a C++ setting ``Nsc::mass_splitting``, :math:`\omega = 0.2`); and
* ``"apgd"``, the accelerated projected gradient descent of Mazhar et al.
  :cite:`mazhar2015`, the default: Nesterov's method on
  :eq:`eq:dem_nsc_qp` in the scaled impulses
  :math:`\underline{z}_k = \underline{\gamma}_k/\sqrt{\eta_k}`, with a
  restart when the step goes uphill and the best iterate kept by its
  residual. Each iteration steps from the extrapolated point
  :math:`\underline{y}` to :math:`\underline{x} =
  \Pi(\underline{y} - \underline{g}_y/L)`, with :math:`\underline{g}` the
  gradient :math:`N\underline{z} + \underline{r}`, and keeps the step if it
  descends enough,

  .. math::
      :label: eq:dem_nsc_descent

      (\underline{x} - \underline{y})\cdot(\underline{g}_x -
        \underline{g}_y) \le L\,|\underline{x} - \underline{y}|^2

  which, the objective being quadratic, is the usual test
  :math:`f(\underline{x}) \le f(\underline{y}) + \underline{g}_y\cdot
  (\underline{x} - \underline{y}) + \tfrac{L}{2}|\underline{x} -
  \underline{y}|^2`, without the difference of the nearly equal
  :math:`f(\underline{x})` and :math:`f(\underline{y})` that single
  precision loses; otherwise the Lipschitz constant :math:`L` doubles, for
  at most four tries, and each iteration it shrinks by 0.9.

Gauss-Seidel and Jacobi converge slowly where the masses differ greatly,
as a body resting on grains does, and a column of grains needs iterations in
proportion to its height. APGD converges far faster in both: a stack of ten
beads at rest moves 4e-6 mm/s after 100 iterations of APGD, 5.6 mm/s after
100 of Gauss-Seidel and 1.7 mm/s after 100 of Jacobi (0.0013 mm/s after
400), and 41 mm/s after 100 of Jacobi without mass splitting. Without it,
Jacobi also diverges where many contacts load a grain at once, as where
clumps falling fast land on a dense pack of them, even with a relaxation of
0.2; with it, Jacobi converges there for any relaxation from 0.2 to 1.9.
The iterations start from the impulses of the contact at the step before
(warm start), kept in its history slot, and stop after
``nsc_iterations``.

Bodies
******

A body the grains touch takes part in the step as a free rigid body, of its
mass :math:`m_b` and inertia :math:`\bar{I}_{bb}` about its center of mass,
transformed to the fixed frame, :math:`R^{fb}\bar{I}_{bb}^{-1}R^{bf}`: its
velocity changes with the impulses of its contacts within the step, so that
a rigid column of grains under it is pushed by a finite impulse, as it is
not by a body of prescribed motion. Its triangles stand where the body is at
the start of the step and move at its velocities at the end, from the path
of :ref:`the coupling <dem_model_file>`; the grains advance in equal steps to
the end of each accepted step of the integrator, and the body's impulses are
applied to it in full over the following steps, whatever their length.

The step of NSC is ``step``, or by default a tenth of the radius of the
smallest sphere at ``max_speed``, and the margin is at least the distance
two spheres close at ``max_speed`` in a step, so that every contact the
step can close is in it.

On the GPU
**********

APGD and Jacobi update all the contacts at once and run on the GPU as well
(``"device": "gpu"``); Gauss-Seidel, one contact after another, runs on the
CPU only. Each step, after the search, the GPU lists the contacts within
the margin sphere by sphere, in the order of the CPU solver: each sphere
counts its own (its pairs with spheres of higher index, the walls and the
triangles), the counts are summed (a prefix sum) to where each sphere's
contacts start, and each sphere writes its own there, into room for
``max_neighbors`` / 2 + 1 contacts per sphere. Each sphere also finds,
for each neighbor of lower index, the contact of their pair, so that each
grain sums the impulses of all its contacts itself, in the order the CPU
adds them, and no two invocations add to the same sum. The sums over all
contacts that APGD needs, and the impulses on each body that takes part,
are trees over workgroups of 1024 contacts, which the CPU solver takes in
the same order. APGD's decisions (whether a step descends enough
:eq:`eq:dem_nsc_descent`, whether the momentum restarts, whether an iterate
is the best) are made on the GPU, by one workgroup that sets the numbers of
workgroups of the dispatches that follow (indirect dispatches): the four
tries of each iteration are recorded in advance, and those after the step
is taken run no workgroups. A step therefore needs no exchange with the
host, and four steps go in one submission.

The GPU computes in single precision, as the CPU solver does, and differs
from it where it fuses a multiplication and an addition. Jacobi follows
the CPU's runs to :math:`10^{-7}` m as grains land in a box; APGD stops at
a residual of about :math:`10^{-6}`, keeping as its best iterate one of
those near it, so that its runs on the two differ by that much from the
first impacts, until a collision near ``min_bounce_speed`` bounces in one
and not the other and the runs part, as any two runs of grains do. On an
NVIDIA RTX 5060 Ti, a step of 100 iterations of APGD takes 16 ms for 1000
grains (53 ms on one core of the CPU), 21 ms for 8000 and 89 ms for 64 000
grains with 214 000 contacts; Jacobi takes 3 ms for 1000, 5 ms for 8000
and 30 ms for 64 000.
Up to some ten thousand grains the time is that of the GPU's dispatches,
about 2700 a step for APGD; beyond, it grows with the number of contacts.

Comparison with the smooth method
*********************************

Both methods are run on the same problems in ``DemSpeed.NscAgainstSmc`` (see
:doc:`../force-modules/dem`): a bed of 1000 jittered glass beads settling for
0.5 s, and a column of 6 by 4 by 24 beads collapsing for 1 s:

.. list-table::
    :header-rows: 1

    * - :math:`\mu_r`
      - method
      - bed height
      - bed overlap
      - runout of the column
    * - 0.05
      - SMC
      - 91.6 mm
      - 0.047 mm
      - 455 mm
    * - 0.05
      - NSC, APGD
      - 97.5 mm
      - 0
      - 334 mm
    * - 0.05
      - NSC, Gauss-Seidel
      - 94.2 mm
      - 0
      - 364 mm
    * - 0
      - SMC
      - 87.4 mm
      - 0.039 mm
      - 484 mm
    * - 0
      - NSC, APGD
      - 94.3 mm
      - 0
      - 485 mm
    * - 0
      - NSC, Gauss-Seidel
      - 93.5 mm
      - 0
      - 485 mm

Without rolling resistance the methods agree on the collapse. With it, NSC
holds the grains more firmly, since its rolling resistance acts at once and
holds a grain still, where that of SMC waits a collision's duration and
rocks; more iterations do not change this. NSC packs a looser bed, as the
dilation of the relaxation predicts, with no overlap. Pazouki et al. found
the same for larger tests, that the two methods predict the same bulk
behavior where the parameters are matched :cite:`pazouki2017`. On the CPU,
NSC took 9 to 12 times as long as SMC on these problems.


Choosing the Parameters
-----------------------

A DEM material is given by the density, Young's modulus and Poisson's ratio
of the grains and, for each pair of materials, the coefficients of
restitution :math:`\varepsilon`, friction :math:`\mu` and rolling resistance
:math:`\mu_r`. As in DEM-Engine, each material has its own coefficients, and a
pair takes the average of its two materials' unless it is given its own. The bulk behavior depends mostly on the friction, the rolling
resistance and the shape of the grains, which together set the angle of
repose and the shear strength, and little on the stiffness once the overlaps
are small. Grains much larger than the real ones (*coarse graining*) keep the
number of grains manageable, at the price of fewer grains across the features
of the bodies, which should be at least several grain diameters wide. The
parameters are usually calibrated against simple bulk tests, such as the angle
of repose of a heap and a direct shear test :cite:`fleischmann2016`.
