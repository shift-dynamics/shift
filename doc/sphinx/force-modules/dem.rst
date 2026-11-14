.. title:: Grains (DEM)

Grains (DEM)
============

``"type": "dem"`` models a granular material, such as gravel, ore or seeds,
as rigid grains that touch and rub on each other (the discrete element
method), and couples the grains to bodies of the model: the grains touch a
body on the triangles of its geometry and push back on it. The grains run on
the GPU, or on the CPU for a few thousand of them. The contact law, the
integration of the grains, the search for contacts and the coupling are
described on the theory page :doc:`../dem/dem`.

.. code:: javascript

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

Positions and directions are in the inertial frame, and gravity is the
model's.

.. _dem_grain_shapes:

Grain shapes
------------

.. figure:: ../images/examples/dem_grain_shapes.jpg
   :width: 100%

   The three kinds of grain, from left to right: a sphere, a clump of 12
   spheres and a convex hull, resting on a floor
   (:source:`examples/dem/grain_shapes.json`). The clump's spheres fill the
   hull of the same rock and have its mass and inertia.

Each of ``grains`` names a shape of grain, which ``fill`` places many times.
A shape is one of three kinds; one model can mix all three, and each kind
touches the others.

**A sphere** has a ``radius``, a ``material``, and a ``mass`` (or a
``density`` for it). It is the cheapest grain: one contact per pair, the
contact law of :ref:`dem_contact`, and no orientation to keep. Round grains
roll freely, so give them ``rolling_resistance`` in their material to stand
in for the interlocking of angular grains.

.. code:: javascript

    {"name": "bead", "radius": 0.005, "material": "glass", "mass": 0.0013}

**A clump** is a set of ``spheres``, rigid together, each with its own
``center`` (in the clump's principal axes, from its center of mass),
``radius`` and ``material``. The clump has a ``mass`` and the principal
moments of ``inertia`` about its center of mass, stated as for a body,
because the spheres overlap and their own masses do not add up to the
clump's. Each sphere touches as a sphere, so a clump takes both contact
methods, SMC and NSC, and approximates any shape, convex or not, with
enough spheres. Its surface is lumpy, and its smallest spheres set the
substep. ``mesh`` names a mesh file that the viewer draws in place of the
spheres, in the same coordinates (see :ref:`dem_grains`).

.. code:: javascript

    {"name": "pair", "mass": 0.0018, "inertia": [1.6e-8, 3.4e-8, 3.4e-8],
     "spheres": [{"center": [-0.003, 0, 0], "radius": 0.004, "material": "glass"},
                 {"center": [0.003, 0, 0], "radius": 0.004, "material": "glass"}]}

**A convex hull** is the hull of the vertices of the mesh file ``hull``,
given in the grain's principal axes from its center of mass, with its
``material``, ``mass`` and ``inertia``. Its flat faces and sharp edges touch
as they are: hulls touch each other, spheres and walls where their shapes
overlap (see :ref:`dem_hulls`), and the triangles of bodies (see
:ref:`dem_hull_triangle`). A hull has at most 32 vertices, 64 faces and 12
vertices on a face. Hulls take SMC only.

.. code:: javascript

    {"name": "rock_00", "hull": "rock_00.obj", "material": "granite",
     "mass": 0.000781, "inertia": [5.44e-09, 5.01e-09, 6.43e-09]}

Choose the kind by what the grains' shape must do:

* Spheres, for round grains (glass beads, seeds, shot), or wherever the
  bulk behavior can be calibrated with friction and rolling resistance
  alone. They are the fastest, and the only kind tested at a million
  grains.
* Hulls, for angular grains (gravel, ore, crushed rock) with SMC. A hull
  touches with one contact per pair, at the substep a sphere of its size
  would take: 100,000 rocks settled for 1.5 s in 173 s on an NVIDIA RTX
  5060 Ti, where clumps of the same rocks, each 12 spheres of about 1 mm
  that need a substep some six times shorter, took 3981 s.
* Clumps, for elongated or non-convex grains, and for angular grains that
  need NSC.

The mass and inertia in the model file are the grain's own, as for a body
(see :ref:`dem_grains`); *Shift* does not compute them from a density for a
clump or a hull. To make them, ``shift::core::dem::Template::OfHull`` in C++
takes the points of a hull and a density and returns the hull in its
principal axes with its mass and inertia; ``Template::ClumpOfHull`` fills
such a hull with spheres; and ``RockPoints`` generates the points of a
random rock from a seed (see :source:`shift/core/dem/dem.h`). The benchmark
``dem_rocks`` (:source:`test/dem/test_dem_model.cc`) uses them to write a
model of 16 rocks.

Bodies
------

Each of ``bodies`` names a body of the model and the ``geometry`` the grains
touch. A body's ``geometry`` names its mesh geometry (type ``mesh``), whose
triangles the grains touch as they are, or its collision geometry of type
``convex_hull`` or ``box``, whose hull they touch: a box's from its half
extents, a convex hull's from its half extents or else from the body's mesh.
Set ``"contact": false`` on collision geometry that only the grains should
touch. A sphere lists at most ``max_triangles`` triangles near it (24, at
most 32): a hull fitted to a fine mesh, its triangles thin or many meeting at
a vertex, can bring more within reach of one sphere, which stops the
simulation with an error; fit the hull to a coarser mesh.

The grains advance after each accepted step of the integrator, in substeps,
and their mean load on each body over those substeps acts on the body over
the next step. The load is therefore a step behind the body: keep the
integrator's steps no longer than a few of the grains' substeps with its
``hmax`` (see :ref:`dem_bodies`), or a light body rattles on stiff grains.
The substep is printed when the simulation starts.

Contact method: SMC or NSC
--------------------------

``contact_method`` chooses how the grains touch, for the whole module:

* ``"smc"``, the default (smooth contact): grains overlap slightly, and a
  spring and damper on the overlap give the force (Hertz–Mindlin, see
  :ref:`dem_contact`). The substep resolves each collision: it is
  ``step_fraction`` (default 0.2) of the shorter of the Rayleigh time and
  the time of a collision at ``max_speed``, or ``step`` if given.
* ``"nsc"`` (non-smooth contact): the grains are rigid, and each step
  finds the impulses that keep them from overlapping, with Coulomb friction
  and rolling resistance as limits on them, as Chrono's ``ChSystemNSC``
  does (see :ref:`dem_nsc`). It needs no stiffness: the step (``step``, by
  default a tenth of the smallest radius at ``max_speed``) is set by how far
  a grain moves in it, not by the duration of a collision. Each step is an
  iterative solve over all the contacts, set by ``nsc_solver``,
  ``nsc_iterations`` (default 100) and ``nsc_relaxation``;
  ``recovery_speed`` and ``min_bounce_speed`` set how fast overlaps are
  undone and which collisions bounce.

.. code:: javascript

   {"type": "dem", "name": "gravel", "device": "gpu",
    "contact_method": "nsc", "nsc_solver": "apgd", "nsc_iterations": 100,
    ...}

``nsc_solver`` is one of:

* ``"apgd"``, the default (accelerated projected gradient descent), on the
  CPU or the GPU. It converges in far fewer iterations than the others
  where a body rests on grains or grains stand in a tall column; 100 are
  enough in the tests.
* ``"jacobi"`` (projected Jacobi, with mass splitting), on the CPU or the
  GPU. An iteration is the cheapest on the GPU, but it needs some four
  times APGD's iterations.
* ``"gauss_seidel"`` (projected Gauss–Seidel, one contact after another),
  on the CPU only, for small problems; it needs thousands of iterations
  where masses differ much.

A body on the grains takes part in each step's solve with its mass and
inertia, and its impulse is applied to it in full over the integrator's
steps; keep the same ``hmax`` as with SMC. On an NVIDIA RTX 5060 Ti a step
of 100 iterations of APGD takes 16 ms for 1000 grains, against 53 ms on one
core of the CPU, and 89 ms for 64 000 grains; below about ten thousand
grains the GPU's time is mostly the cost of starting its passes.

.. list-table:: The two methods on the tests (CPU, release build for the bed and column)
    :header-rows: 1

    * - test
      - SMC
      - NSC (APGD)
    * - plate on grains: plate above the top grain
      - a radius less the overlap, 8.4 mm/s at the end, 4.1 s (debug)
      - a radius, 4.3 mm/s at the end, 28.8 s (debug)
    * - wedge: edge below the bed top
      - 20.6 mm, 2.3 s (debug)
      - 23.1 mm, 51.9 s (debug)
    * - bed of 1000 beads, :math:`\mu_r` 0.05: height, time
      - 91.6 mm, 2.25 s
      - 97.5 mm, 21.4 s
    * - column collapse, :math:`\mu_r` 0.05: runout, time
      - 455 mm, 2.2 s
      - 334 mm, 19.6 s
    * - column collapse, no rolling resistance: runout, time
      - 484 mm, 2.0 s
      - 485 mm, 22.4 s

In these tests NSC takes 8 to 22 times as long as SMC on the CPU, rests a
plate on the grains without overlap, packs a bed a few percent looser, and
holds grains more firmly with rolling resistance; without rolling
resistance the two agree on the collapse of a column. The recommendations
that follow from them:

* **Use SMC by default.** It is an order of magnitude faster, runs to a
  million grains on the GPU, is the only method for convex hulls, and its
  parameters (Young's modulus, restitution) are measurable properties of the
  material.
* **Use NSC** where the grains must not overlap (a body resting on grains
  sits exactly on them), where a stiff material would force an SMC substep
  far shorter than the motion needs, or to check an SMC result with a
  second method. Use spheres or clumps, with ``"nsc_solver": "apgd"``; try
  ``"jacobi"`` on the GPU for many grains, with more iterations.
* **Calibrate rolling resistance per method.** The two methods resist
  rolling differently: SMC applies a moment of :math:`\mu_r |f_n| r^*`
  against the rolling, which waits a collision's duration and lets a grain
  rock, while NSC finds a moment impulse up to :math:`\mu_r r^* \gamma_n`
  that holds a grain still at once, as static friction does (see
  :ref:`dem_nsc`). The same :math:`\mu_r` therefore holds grains more
  firmly with NSC.

Settling and filling
--------------------

Grains placed on a lattice are loosely packed and fall into place when the
simulation starts. To start from a settled bed, run a model that only lets
the grains settle, with ``"save": "settled.csv"``: at its end (or at
``save_time``) it writes every grain to that file, in the model's
directory: its shape, position, orientation and velocities, as comma
separated values. A ``fill`` of another model then names the file in place
of a lattice, ``{"file": "settled.csv"}``, and its grains start exactly as
they were saved. The history of their contacts is not saved, so grains that
friction held on a slope may slip a little at the start. Keep bodies out of
reach of the settling grains (for example, start them high on their joint
with ``initial_displacement``), or the grains settle under their load.

Viewing the grains
------------------

The viewer (``build-release/shift/main/shift``) draws the grains while the
model runs and when you play it back: each shape of grain in its own color,
its spheres as they are, or its hull or ``mesh``, flat shaded. At each output time it records every grain's
position, orientation and velocities, 64 bytes a grain, so 10 000 grains over
1 000 output times take 640 MB; lengthen ``time_step`` for long runs of many
grains. Playback interpolates the grains between output times. **Run**
continues from the grains shown, without the history of their contacts (the
tangential spring of each contact starts again from zero), and **Reset**
returns them to where the ``fill`` placed them. An HDF5 log does not hold
the grains, so playing back a log draws them where they were placed. The
walls are not drawn; give the fixed body mesh geometry to show a container.

Parameters
----------

.. shift-parameters:: shift/core/force_module/dem.cc
   :function: force_module::Dem::DeclareParameters
   :exclude: name, type, active

Each of ``materials``:

.. shift-parameters:: shift/core/force_module/dem.cc
   :function: DemMaterial::DeclareParameters

Each of ``pairs``:

.. shift-parameters:: shift/core/force_module/dem.cc
   :function: DemMaterialPair::DeclareParameters

Each of ``grains``, a sphere, a clump of ``spheres`` or a ``hull``:

.. shift-parameters:: shift/core/force_module/dem.cc
   :function: DemGrain::DeclareParameters

.. shift-parameters:: shift/core/force_module/dem.cc
   :function: DemSphere::DeclareParameters

Each of ``fill``, ``walls`` and ``bodies``:

.. shift-parameters:: shift/core/force_module/dem.cc
   :function: DemFill::DeclareParameters

.. shift-parameters:: shift/core/force_module/dem.cc
   :function: DemWall::DeclareParameters

.. shift-parameters:: shift/core/force_module/dem.cc
   :function: DemBody::DeclareParameters

Examples and tests
------------------

* :source:`examples/dem/plate_on_grains.json`: a plate on a vertical slide
  drops onto a bed of beads and clumps and comes to rest on it.
* :source:`examples/dem/grain_shapes.json`: a sphere, a clump and a convex
  hull side by side on a floor (the figure above).
* :source:`test/dem/test_dem.cc` (the grains alone and against bodies whose
  motion is given: restitution, Hertz's overlap, rolling, momentum, a sphere
  resting on a body's triangles, a moving body pushing a sphere, the GPU
  against the CPU; with NSC, a rebound with restitution, a stack at rest
  without overlap under each solver, rolling down a slope and held by
  rolling resistance, sliding with Coulomb friction, momentum in a
  collision, grains settling in a box; and on the GPU, APGD and Jacobi
  against the CPU, alone and with a body of mass, and grains settling the
  same twice; convex hulls: a cube's mass properties, a rock's hull and its
  clump, a cube resting flat on the floor and on another cube, momentum in
  collisions of rocks and a sphere, rocks settling in a box, a cube resting
  on a body's triangles as on a wall and pushed by a moving body, and the
  GPU against the CPU, alone and with a body), :source:`test/dem/test_dem_model.cc` (a plate in a model
  coming to rest on grains, its weight carried by them, with SMC and with
  NSC on the CPU and the GPU; a wedge, the convex hull of its mesh,
  splitting a bed of beads, with SMC and NSC; settling and filling from a
  file; grains of a hull and a clump drawn as a mesh, read from a file) and
  :source:`test/dem/test_dem_playback.cc` (the grains in the viewer: played
  back forward and back, Reset repeating the run, Run from the grains
  shown).
* Benchmarks, not run with the tests, each run with
  ``meson test -C build --benchmark <name>``: ``dem_speed`` (the GPU solver
  alone on up to 885,000 grains), ``dem_rock`` (a rock of 41 kg, the convex
  hull of a lumpy ellipsoid's mesh, thrown into 186,000 beads, 0.3 s),
  ``dem_million`` (a million beads falling into place, 0.05 s) and
  ``dem_rocks`` (100,000 convex hull rocks of 16 shapes about 1 cm across
  settling for 1.5 s from a lattice, then a block of 20 kg dropped on them
  from 0.1 m, making a crater, 0.5 s). On an
  NVIDIA RTX 5060 Ti in the debug build, a substep takes 2.8 ms with the rock
  and 3.8 ms with a million beads. In the release build ``dem_rocks`` takes
  about 5 minutes: 3.4 ms a substep of 29 µs while the rocks settle, and
  7 ms with the block, whose integrator steps of at most 40 µs each bring
  the GPU's results back (the block sinks 32 mm below the top of the bed
  and still creeps at 1.4 cm/s at the end). ``dem_nsc`` runs the bed and the column
  of the table above with each method and solver, on the CPU, and
  ``dem_nsc_gpu`` times APGD and Jacobi on the GPU against the CPU for up
  to 64 000 grains.
