.. _model-file:
.. title:: Model File

Model File
==========

A *Shift* model is a JSON file whose structure follows the objects it
creates: each object in the file becomes an object of the same name in the
program, and each key a parameter of it. The tables on this page list
every key, and are generated from the source code, where each object
declares its keys, so they show what this version of *Shift* reads.

A model may be written in JSON or in YAML: a file ending in ``.json`` is read
as JSON, and one ending in ``.yaml`` or ``.yml`` as YAML. The organization of
the file is the same in both, with the same keys and values, so everything on
this page applies to either. ``examples/pendulum/pendulum.json`` and
``examples/pendulum/pendulum.yaml`` are the same model. In YAML, mappings are
the objects and sequences the arrays; an unquoted ``true``, ``false``,
``null``, number (``12``, ``1.5``, ``1e-3``) is read as that type, and a
quoted value is always text. Anchors, aliases and the merge key ``<<`` can
share values between objects, and comments start with ``#``.

Comments are allowed: ``//`` starts a comment that runs to the end of the
line. A key that is not required may be left out; its default is given in
its description. Arrays of objects (bodies, joints, force modules, ...)
create one object per entry, and the ``type`` of each entry selects which
kind of object it is.

.. contents:: On this page
   :local:
   :depth: 1

Layout
------

.. code:: javascript

   {
     "model_parameters": {...},   // units, gravity, solver, integrator, loggers
     "frames": [...],             // frames on the fixed body (the ground)
     "lights": [...],             // lights of the 3D view
     "terrain": [...],            // elevation maps
     "surface_materials": [...],  // the materials of the ground
     "scenery": [...],            // trees, bushes, grass and rocks on the ground
     "environment": {...},        // sky, sun, fog and water
     "contact": {...},            // contact model of the environment
     "collision_geometry": [...], // collision geometry of the fixed body
     "assemblies": [              // the model itself
       {
         "name": "machine",
         "bodies": [...],
         "joints": [...],
         "force_modules": [...],
         "sensors": [...],
         "controllers": [...],
         "assemblies": [...]      // subassemblies, nested to any depth
       }
     ]
   }

The top level of the file is the fixed body. It has a frame named
``origin`` without being given one, and its frames, lights, terrain and
collision geometry are fixed in space.

.. shift-parameters:: shift/core/context.cc

.. shift-parameters:: shift/core/body/fixed.cc
   :function: Fixed::DeclareParameters

Names
-----

Every body, joint, frame and module has a ``name``. A name may be used only
once at one level: among the assemblies, bodies, joints, force modules,
sensors, controllers or constraints of an assembly, or among the frames,
lights, geometry or collision geometry of a body. A model that repeats one does
not load, even if the objects are not active. The same name is allowed in a
different assembly or body, and objects of different kinds, such as a body and
a force module, may share a name, as each object is identified by its fully
qualified name, ``<assembly>__<subassembly>__<name>``. A joint names the
bodies it connects as ``[[body], frame]``: ``[["fixed"], "origin"]`` is the
origin of the fixed body, ``[["boom"], "pivot"]`` the frame ``pivot`` of
the body ``boom``. Force modules and lights name frames as ``[body, frame]``.
The body is looked for in their own assembly first, then anywhere in the
model. If several bodies elsewhere have that name, the model does not load:
give the fully qualified name, such as ``machine__left__load``, to choose one.
Sensors, controllers and force
modules that name a joint or a sensor look for it in their own assembly.
Each body has a frame ``origin`` at its origin, which needs no entry.

Model parameters
----------------

.. shift-parameters:: shift/core/model_parameters.cc

``units``:

.. shift-parameters:: shift/core/units.cc

The units name the units of the numbers in the file; the force modules'
defaults (and the descriptions on these pages) assume SI units.

Solvers
*******

``"type": "rnca"``, the run-time optimized natural constraint algorithm,
sets up and solves the equations of motion (see :ref:`about`).

.. shift-parameters:: shift/core/solver/rnca.cc

Models without closed loops can instead use an articulated-body solver (see
:ref:`articulated_body_algorithms`), which needs no constraint Jacobian or
generalized mass matrix:

* ``"type": "aba"``, the articulated-body algorithm, for serial chains and
  branched trees;
* ``"type": "dca"``, the divide-and-conquer algorithm, for serial chains and
  branched trees with mass and inertia in every body. It is slower than
  ``"aba"`` on one processor; its assemblies are organized for parallel
  evaluation.

Both take only the ``type`` parameter and stop with an error, at
initialization, on a model they cannot solve.

``"type": "auto"`` chooses for each tree of the model: the trees without closed
loops are solved by ``"aba"`` (or ``"dca"``, see ``tree_solver``) and the trees
with closed loops by RNCA, whose parameters it also takes. The tree solver adds
no work to the RNCA solve, so a model of several robots, one of which has a
closed loop, pays the cost of the constraint Jacobian for that one only.

.. shift-parameters:: shift/core/solver/auto.cc

Integrators
***********

Every integrator advances the coordinates over ``time_span`` and reports
the state, to the loggers and the viewer, every ``time_step``. The loop
closure equations of a model with closed loops are kept satisfied by
Baumgarte stabilization, which pulls the coordinates back onto the
constraints at the rate ``omega_n``.

.. shift-parameters:: shift/core/integrator/base.cc

``"type": "ode113"``: variable step, variable order Adams–Bashforth–Moulton
predictor–corrector, which chooses its steps to keep the error of each
within ``relative_error`` and ``absolute_error``. Its steps are not limited
by ``time_step``, which only sets how often results are reported.

.. shift-parameters:: shift/core/integrator/ode113.cc

.. _fast-modes:

Why the steps are small: the fast modes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ODE113 is an explicit method: it stays stable only while each step is short
compared with the fastest motion of the model. Near a state, the model
behaves like the linear equations
:math:`\dot{\underline{x}} = \mathbf{A}\,\underline{x}`, and each
eigenvalue :math:`\lambda = \sigma \pm i\omega` of :math:`\mathbf{A}` is a
mode, a motion that oscillates at :math:`\omega` (rad/s) and grows or
decays at the rate :math:`\sigma` (1/s). The steps stay stable roughly while

.. math::

   h\,|\lambda| \lesssim 1

for every mode, so a stiff spring, a heavily damped joint or a friction law
that changes steeply with slip speed (a clutch, a limited-slip differential)
can force steps far shorter than the motion that matters needs. Raising
``relative_error`` then barely helps: the steps are limited by stability,
not by accuracy.

``shift-cli --linearize <time>`` simulates the model to that time, computes
:math:`\mathbf{A}` there by finite differences of the equations of motion
(the forces of the force modules and controllers and the rates of their
internal states included), and prints the fastest modes, ten by default
(``--modes <n>`` for more). For each mode it lists the eigenvalue, the
frequency and damping ratio, the step :math:`h = 1/|\lambda|` it allows, and
the coordinates and internal states that take part in it most:

.. code:: bash

   build/shift/main/shift-cli --linearize 8 --modes 5 examples/polo_r_wrc/polo_r_wrc.json

A model with closed loops is linearized without Baumgarte stabilization, so
the motions that would break a loop show up as eigenvalues of 0 and do not
hide the modes of the mechanism.

``"type": "rk4"``: the classical fixed step Runge–Kutta method, with steps
of ``step_size``.

.. shift-parameters:: shift/core/integrator/rk4.cc

Loggers
*******

See :ref:`logging`.

Frames
------

A frame is a coordinate system fixed in a body: an origin ``T`` and axes
``R`` relative to the body's origin. The rotation may be given in several
forms; give only one.

.. shift-parameters:: shift/core/frame.cc

.. code:: javascript

   "frames": [
     {"name": "distal", "T": [0, 0, 3]},
     {"name": "pin", "T": [0.5, 0, 0],
      "R": [[0, 0, -1], [0, 1, 0], [1, 0, 0]]}  // row by row
   ]

Assemblies
----------

.. shift-parameters:: shift/core/assembly.cc

An assembly that is not ``active`` is read but left out of the
simulation, with everything in it, which makes it easy to keep variants of
a model in one file.

Bodies
------

The default type, ``rigid``, is a rigid body. Its ``inertia`` gives the
mass and the moments and products of inertia about the center of mass, in
the body's axes; ``center_of_mass`` places the center of mass relative to
the body's origin.

.. shift-parameters:: shift/core/body/base.cc

.. code:: javascript

   {
     "name": "link1",
     "frames": [{"name": "distal", "T": [0, 0, 3]}],
     "geometry": [{"type": "mesh", "filename": ["link.obj", "link.mtl"]}],
     "center_of_mass": [0.0, 1.5, 0.0],
     "inertia": {"mass": 1.0, "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0,
                 "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}
   }

``density`` computes the mass properties from the body's geometry as a
uniform solid. It is a convenience: state the mass, center of mass and
inertia of a body explicitly when they are known, since a solid model of a
hollow part, such as a fabricated boom, overstates its mass.

Geometry and materials
**********************

A body is drawn with the meshes in its ``geometry`` list: OBJ files (with
their MTL material files) or glTF files, placed in the body's axes. These
meshes draw the body (and give ``density`` its volume); contact uses the
body's collision geometry instead (see :doc:`../force-modules/contact`). ``scale`` sizes a mesh, so one mesh of a
unit box or cylinder can draw parts of any size, and a negative factor
mirrors it (``[1, 1, -1]`` is the mirror image of a part across its x-y
plane, as for the right hand part of a left hand one):

.. code:: javascript

   "geometry": [{"type": "mesh", "filename": ["meshes/cylinder_z_black.obj"],
                 "scale": [0.08, 0.08, 0.025],
                 "texture_mapping": "triplanar", "texture_scale": 0.05}]

.. shift-parameters:: shift/core/geometry/mesh.cc

The viewer shades meshes with metallic-roughness materials, read from the
MTL file of an OBJ mesh:

.. code:: text

   newmtl paint
   Kd 0.72 0.40 0.045                            # base color, linear
   Pm 0.0                                        # metalness, 0 to 1
   Pr 0.4                                        # roughness, 0 to 1
   map_Kd qrc:/textures/steel_base_color.png     # color image
   map_Pr qrc:/textures/steel_roughness.png      # roughness image
   norm -bm 0.2 qrc:/textures/steel_normal.png   # normal map and its strength

Image paths are relative to the MTL file, or name a texture built into the
viewer as ``qrc:/textures/<file>``, which needs no file next to the model.
The built-in textures are the images in :tree:`assets/textures`: brushed
steel (``steel_base_color.png``, ``steel_roughness.png``,
``steel_normal.png``), and ground surfaces (``asphalt``, ``asphalt2``,
``concrete``, ``concrete_wall``, ``cracked_sand``, ``grass`` and
``pebbles``, each a ``.jpg`` color image with ``_normal`` and
``_specular`` maps). Terrains name them the same way, e.g.
``"texture_filename": "qrc:/textures/asphalt.jpg"``. A mesh without
texture coordinates, as many OBJ files are, takes images with
``"texture_mapping": "triplanar"``.

``"type": "free_body"`` is a body moving freely in space without a joint,
placed by its own initial pose and velocity:

.. shift-parameters:: shift/core/body/free.cc

Joints
------

A joint connects the frame of a parent body to the frame of a child body
and lets the child move relative to the parent in the ways of its type.
The coordinates of every joint are the generalized coordinates of the
model.

.. shift-parameters:: shift/core/joint/base.cc

``"type": "revolute"``: a hinge about ``axis`` of the parent frame; one
coordinate, the angle.

.. shift-parameters:: shift/core/joint/revolute.cc

``"type": "prismatic"``: a slider along ``axis``; one coordinate, the
displacement.

.. shift-parameters:: shift/core/joint/prismatic.cc

The polynomial stiffness and damping of revolute and prismatic joints act
on the joint as the force (or torque)
:math:`-(a_0 + a_1 q + a_2 q^2 + \dots) - (b_0 + b_1 \dot q + \dots)`,
e.g. ``"stiffness_coefficients": [-50, 2e4]`` is a spring of 20 kN/m
preloaded to push with 50 N where the coordinate is zero.

``"type": "spherical"``: a ball joint; three rotations, represented by a
quaternion.

.. shift-parameters:: shift/core/joint/spherical.cc

``"type": "screw"``: a screw, one coordinate: the angle :math:`q` turned about
``axis`` (in the parent frame), with the child advancing along the axis by
``pitch`` :math:`\cdot q`. The pitch is the advance per radian, so a thread
that advances :math:`l` per turn has a pitch of :math:`l / 2\pi`; a positive
pitch is a right-handed screw, and a pitch of zero is a revolute joint. The
joint's force acts on the angle :math:`q`; a force :math:`f` along the axis
on the child does the work of a torque :math:`f \cdot` ``pitch`` on the
coordinate.

.. shift-parameters:: shift/core/joint/screw.cc

``"type": "gear"``: a gear rolling on a gear of the parent, without
slipping. The parent frame is at the center of the parent gear, with its z
axis along the gear axis. The child frame is at the center of the child gear,
``separation`` away from it in the x-y plane of the parent frame. The
coordinate :math:`q` is the angle of the line of centers from the x axis of
the parent frame. As it grows, the child gear rolls around the parent gear
and turns about its own axis, relative to the parent frame, by
:math:`(1 + n) q`, where the ``gear_ratio`` :math:`n` is the radius of the
parent gear over the radius of the child gear, which is also the ratio of
their numbers of teeth. The ``separation`` of a pair of external gears is the
sum of their pitch radii. A gear inside a ring has a negative ratio and a
separation equal to the difference of the radii. For a parent gear that is
fixed, the joint is a planetary gear (the carrier is the line of centers).

.. shift-parameters:: shift/core/joint/gear.cc

:source:`examples/joints/screw_and_gear.json` shows both: four gears
with involute teeth in a train, each meshed pair joined by a gear joint that
closes a loop (the teeth mesh at every instant), and a right-handed and a
left-handed threaded rod turning through nuts on screw joints, which advance
in opposite directions. It is written by
:source:`examples/joints/make_screw_and_gear.py`, which makes the meshes.

Both coordinates are angles; give the two joints as for a revolute joint
(``initial_displacement`` and ``initial_velocity`` are in radians and radians
per second). A joint that closes a loop may be a screw or a gear joint like
any other.

``"type": "float"``: six degrees of freedom, three translations and three
rotations, for a body that is not otherwise joined, or is held only by
force modules.

A part of a model that no path of joints joins to ``fixed`` does not need
one: *Shift* adds a floating joint named ``_float_joint[k]`` from the origin
of ``fixed`` to the origin of the body of that part with the most joints (the
first in the file among equals), with the identity as its initial
displacement and zero initial velocity, and says so in an information
message. Write the floating joint yourself to choose the body, its initial
pose or its initial velocity.

.. shift-parameters:: shift/core/joint/float.cc

``"type": "rigid"``: no motion; the child is fixed to the parent at
``initial_displacement``. *Shift* merges rigidly joined bodies, so a rigid
joint costs nothing at run time.

.. shift-parameters:: shift/core/joint/rigid.cc

Closed loops need no special treatment: list every joint, including the one
that closes a loop. *Shift* finds a spanning tree, makes each remaining
joint a *chord* whose closure equations constrain the tree's coordinates,
and groups loops that share bodies into substructures.

Geometry
--------

Geometry is what the viewer draws, and what collision geometry is fitted
to. Files are found next to the model file or in its
``include_directories``; a relative include directory is taken from the
directory of the model file.

.. shift-parameters:: shift/core/geometry/base.cc

``"type": "mesh"``: a mesh file (OBJ with its MTL, glTF, ... anything
assimp reads).

.. shift-parameters:: shift/core/geometry/mesh.cc

``"type": "cad"``: the exact geometry of a STEP file (with the ``step``
option), tessellated for display and usable for exact contact.

.. shift-parameters:: shift/core/geometry/cad.cc

Collision geometry
------------------

Collision geometry is what the contact force module (see
:doc:`../force-modules/contact`) tests for contact. Without dimensions, a
collision geometry is fitted to the body's meshes (``fit_to``).

.. shift-parameters:: shift/core/collision_geometry/base.cc

========================= ===============================================
``type``                  shape
========================= ===============================================
``sphere``                a sphere of ``radius``, or the smallest sphere
                          around the meshes
``ellipsoid``             an ellipsoid of ``semi_axes``, or the smallest
                          ellipsoid around the meshes
``convex_hull``           the convex hull of the meshes, or a box of
                          ``half_extents``
``box``                   a box of ``half_extents``, or the smallest box
                          around the meshes along their principal axes
``cylinder``              a cylinder of ``radius`` and ``half_length`` along
                          the z axis of its ``pose``, or the smallest around
                          the meshes
``capsule``               a cylinder with a hemisphere on each end: the
                          points within ``radius`` of a segment of
                          ``half_length`` along the z axis of its ``pose``,
                          or the smallest around the meshes
``plane``                 a half space, for floors and walls
``cad``                   the exact surfaces of the body's CAD geometry
========================= ===============================================

.. shift-parameters:: shift/core/collision_geometry/sphere.cc shift/core/collision_geometry/ellipsoid.cc shift/core/collision_geometry/convex_hull.cc shift/core/collision_geometry/cylinder.cc shift/core/collision_geometry/capsule.cc shift/core/collision_geometry/plane.cc shift/core/collision_geometry/cad.cc

Terrain
-------

Terrains lie on frames of the fixed body. Tires drive on terrain (see
:doc:`../force-modules/tire`).

``"type": "basic"``: an elevation map from a TIFF image.

.. shift-parameters:: shift/core/terrain/base.cc shift/core/terrain/basic.cc

A texture tiled over a large terrain repeats, and the eye picks out the
pattern. With ``texture_variation`` the viewer shifts the tiles by random
amounts in small patches, blending between them, and varies the brightness
slowly over tens of meters, so that the tiles no longer show.

``surfaces`` lay up to two other textures over the terrain's in patches, such
as dirt and grass among gravel. Each covers about the fraction ``coverage``
of the ground, in patches about ``patch_size`` across, placed by a noise of
the position on the terrain; the second lies over the first. The edges of a
patch follow the texture below it, so that bright stones show through the
edge of dirt laid over gravel. Surfaces change only the look, not the shape
of the terrain or its soil.

.. code-block:: json

   "texture_filename": "qrc:/textures/pebbles.jpg", "texture_scale": 0.5,
   "texture_variation": true,
   "surfaces": [{"texture_filename": "qrc:/textures/dry_sand.jpg",
                 "texture_scale": 2.0, "coverage": 0.35, "patch_size": 8.0},
                {"texture_filename": "qrc:/textures/grass.jpg",
                 "texture_scale": 2.0, "coverage": 0.15, "patch_size": 25.0}]

.. shift-parameters:: shift/core/terrain/surface_texture.cc

``"type": "scm"``: deformable soil (the Soil Contact Model), a grid of nodes
``spacing`` apart over the ranges ``x`` and ``y`` of the terrain frame. The
nodes start on the elevation map, or without ``filename`` on the xy plane of
the terrain frame. The soil models lower and raise them, and the terrain's
surface, as tires and other models see it, follows. It takes the keys of
``basic`` and needs a ``soil`` or a ``soil_map``. The soil contact force module
(see :doc:`../force-modules/soil`) puts bodies on it and leaves their ruts; the
nodes change only when the integrator accepts a step.

.. code-block:: json

   "terrain": [{"type": "scm", "name": "site", "parent": [["fixed"], "terrain"],
                "x": [-10, 10], "y": [-10, 10], "spacing": 0.05,
                "soil": {"name": "sandy_loam", "shear_modulus": 0.015}}]

.. shift-parameters:: shift/core/terrain/scm.cc
   :function: Scm::DeclareParameters

``bulldozing`` pushes the soil bodies press down to the rims of their ruts and
erodes it to the angle of repose (see :doc:`../force-modules/soil`):

.. code-block:: json

   "bulldozing": {"enable": true, "flow_factor": 1.0, "erosion_angle": 0.6,
                  "erosion_iterations": 30, "interval": 0.001}

.. shift-parameters:: shift/core/terrain/scm.cc
   :function: Bulldozing::DeclareParameters

``"type": "sph"``: soil as an elastoplastic continuum carried by particles
(smoothed particle hydrodynamics, SPH). The particles fill the ranges ``x``
and ``y`` of the terrain frame, ``spacing`` apart, from the surface (the
elevation map, or the xy plane of the terrain frame without one) down to
``depth``. Fixed markers under them and around them are the floor and walls
of a box. The particles flow, pile up and carry loads as a soil does, at a
far higher cost than the SCM terrain's nodes, so an SPH soil is usually a
region of an SCM terrain (see Regions below). The particles run on the GPU
(``"device": "gpu"``, see :doc:`../installation`) or, for small soils, on
the CPU (``"cpu"``). After each step the integrator accepts, the particles
advance to that step's time, in substeps no longer than their stable step
(set by the spacing and the soil's stiffness). The surface that tires and
other models see is the top of the particles. It is updated at each output
time.

The bodies of the soil contact force module (see :doc:`../force-modules/soil`)
press on the particles directly. Each body's collision geometry is filled
with markers, a spacing apart and half a spacing inside its surface. Over each
accepted step they follow the body, and the force the particles put on them
acts on the body over the next step. A body that starts inside the soil
pushes its particles out of the way first: those particles are removed.

That force is a step behind the body, which puts energy into a light body
resting on the soil when the steps are long. So an SPH soil limits the
integrator's longest step to ``substeps_per_step`` of its stable substeps
(by default 1: the bodies and the particles advance together). The
integrator keeps its own control of the step and order below that limit.
A larger ``substeps_per_step`` runs faster, and suits heavy bodies or bodies
driven along a set path, such as a blade on a prismatic joint.

Its soil needs, besides the friction angle, cohesion and unit weight, the
properties of the soil as a continuum: ``youngs_modulus`` and
``poisson_ratio``, and optionally ``dilatancy_angle`` (see
:doc:`../force-modules/soil`). Bekker's sinkage properties are not needed.

.. code-block:: json

   "terrain": [{"type": "sph", "name": "pit", "parent": [["fixed"], "terrain"],
                "x": [0, 1], "y": [0, 0.5], "depth": 0.2, "spacing": 0.02,
                "soil": {"cohesion": 0, "friction_angle": 0.52,
                         "unit_weight": 15700, "youngs_modulus": 2e6,
                         "poisson_ratio": 0.3}}]

.. shift-parameters:: shift/core/terrain/sph.cc
   :function: Sph::DeclareParameters

Regions
*******

A terrain can hold other terrains in its ``regions``, each over its own
``x`` and ``y``. Where a region lies, it is the terrain: tires and the soil
models see its surface and its soil. A region uses the frame of the terrain
that holds it, so it has no ``parent``. It starts from that terrain's
surface and soils. A region's ``soil`` changes the properties it names in
each of those soils, and keeps the rest. Regions must lie inside the terrain
that holds them, and must not overlap.

An SPH region in an SCM terrain gives SPH soil where a tool digs, and the
cheaper SCM nodes everywhere else. The region's soil adds the properties of
a continuum to the SCM terrain's soil. Under a region, the soil contact force
module leaves the SCM nodes alone, and the SCM surface is drawn at the
region's floor.

Soil a tool carries or throws out of an SPH region lands on the SCM terrain
around it. Every ``landing_interval`` (by default 10 ms), each particle
outside the region's ``x`` and ``y`` that has come down to the SCM surface
is removed, and its volume (its mass over the soil's density) is added to the
level of the nearest SCM node. With the SCM terrain's ``bulldozing`` enabled,
the soil that lands settles at the angle of repose. A particle that lands off
the SCM grid is lost: its volume is counted, with a warning, but goes
nowhere.

.. code-block:: json

   "terrain": [{"type": "scm", "name": "ground", "parent": [["fixed"], "terrain"],
                "x": [0, 20], "y": [0, 10], "spacing": 0.05,
                "soil": {"name": "lete_sand"},
                "regions": [{"type": "sph", "name": "pit", "x": [8, 10],
                             "y": [4, 6], "depth": 0.3, "spacing": 0.02,
                             "soil": {"youngs_modulus": 2e6,
                                      "poisson_ratio": 0.3}}]}]

Soil map
********

``soil`` gives a terrain one soil (see :doc:`../force-modules/soil`).
``soil_map`` gives soils by area: each sample of an 8 or 16 bit integer TIFF
is a value that selects a soil from ``soils``, and samples of 0 with no soil
of their own take the terrain's ``soil``. The soil at a point is that of the
nearest sample, so boundaries between soils stay sharp. The samples are
``x_scale`` by ``y_scale`` apart from the origin of the terrain frame, laid
out as the elevation map's (``coordinate_type``). A value with no soil, two
soils of one value or a float map stops the model from loading.

.. code-block:: json

   "soil": {"name": "dry_sand"},
   "soil_map": {"filename": "soils.tiff", "x_scale": 0.25, "y_scale": 0.25,
                "soils": [{"value": 1, "name": "sandy_loam"},
                          {"value": 2, "name": "clayey_soil_wet",
                           "shear_modulus": 0.02}]}

.. shift-parameters:: shift/core/terrain/soil_map.cc
   :function: SoilMap::DeclareParameters

Each soil of ``soils`` has the keys of a soil and its ``value``:

.. shift-parameters:: shift/core/terrain/soil_map.cc
   :function: MappedSoil::DeclareParameters

Surface materials and material maps
************************************

``surface_materials``, at the root of the model file, are the materials of
the ground: what each looks like, what the cameras will label it, and its
soil. A terrain's ``material_map`` lays them out by area: each sample of an
8 or 16 bit integer TIFF is a value that selects a material from
``materials``, by name. The materials replace the terrain's own texture. A
material map takes the place of a soil map: the soil at a point is that of
the material of the nearest sample, or the terrain's ``soil`` where the
material has none. The samples are ``x_scale`` by ``y_scale`` apart from the
origin of the terrain frame, laid out as the elevation map's
(``coordinate_type``), and a region takes the map of the terrain that holds
it.

The viewer and the ray tracer blend neighboring materials over
``blend_width`` on either side of a boundary. Where a material has a
``height_map_filename`` (a displacement map, light where the surface stands
up, as the stones of a gravel), the blend follows the heights: the
material's stones stand out of its neighbor near the boundary and its
hollows fill with it, instead of the two fading into each other. Each material's share of the
nearby samples is smoothed, so that boundaries do not follow the grid in
steps, and moved about by noise; the line where the share is one half is
where the soil changes. A value with no material, two materials of one value
or a material that is not among the surface materials stops the model from
loading.

.. code-block:: javascript

   "surface_materials": [
     {"name": "dry_sand", "label": "sand",
      "texture_filename": "qrc:/textures/dry_sand.jpg",
      "normal_map_filename": "qrc:/textures/dry_sand_normal.jpg",
      "texture_scale": 1.5, "soil": {"name": "dry_sand"}},
     {"name": "wet_sand", "label": "sand",
      "texture_filename": "qrc:/textures/dry_sand.jpg",
      "texture_scale": 1.5, "color": [0.6, 0.55, 0.48], "roughness": 0.35,
      "soil": {"name": "lete_sand"}}],
   "terrain": [{"type": "basic", "name": "beach", ...,
                "material_map": {"filename": "beach_materials.tiff",
                                 "x_scale": 0.25, "y_scale": 0.25,
                                 "blend_width": 0.3,
                                 "materials": [{"value": 0, "material": "dry_sand"},
                                               {"value": 1, "material": "wet_sand"}]}}]

.. shift-parameters:: shift/core/environment/surface_material.cc
   :function: SurfaceMaterial::DeclareParameters

.. shift-parameters:: shift/core/terrain/material_map.cc
   :function: MaterialMap::DeclareParameters

Each material of ``materials``:

.. shift-parameters:: shift/core/terrain/material_map.cc
   :function: MappedMaterial::DeclareParameters

Scenery
*******

``scenery``, at the root of the model file, strews objects over a terrain:
trees, bushes, grass, rocks. Each scenery is one kind of object, with one or
more meshes in ``geometry``, each in its own coordinates with z down and the
origin where it meets the ground. The items are tried at random over the
terrain, ``density`` per square meter, and kept where the terrain's material
map has one of ``materials`` (anywhere if none are given), with no other
material within ``margin``, and no other item of the scenery, or of the
sceneries named in ``avoid`` (given before it), nearer than ``spacing``.
Each item is one of the meshes, chosen at random, turned at random about
the vertical and scaled by a random factor in ``scale``; it stands on the
surface with its origin ``depth`` below it, so that it does not stand clear
of a slope, upright (``tilt`` 0, trees) or on the slope (``tilt`` 1, grass
and stones). The same ``seed`` gives the same items every time.

The scenery is drawn and seen by cameras, by the viewer and the ray tracer;
it does not collide. A mesh is loaded once for all its items. The viewer's
rasterizer leaves out the items beyond ``draw_distance`` from the camera,
such as grass that would cover too few pixels to matter; the ray tracer
draws them all. Meshes of foliage are cut out by the alpha of their textures
(glTF's ``"alphaMode": "MASK"``).

.. code-block:: javascript

   "scenery": [
     {"name": "trees", "terrain": "forest_stage",
      "materials": ["forest_floor"], "margin": 2.0,
      "density": 0.06, "spacing": 3.5, "scale": [0.75, 1.15], "depth": 0.25,
      "seed": 1, "draw_distance": 250.0,
      "geometry": [
        {"type": "mesh", "filename": ["../../assets/scenery/fir_a.glb"]},
        {"type": "mesh", "filename": ["../../assets/scenery/pine_a.glb"]}]},
     {"name": "verge_grass", "terrain": "forest_stage",
      "materials": ["verge"], "density": 6.0, "spacing": 0.3,
      "scale": [1.4, 2.4], "tilt": 1.0, "seed": 4, "draw_distance": 45.0,
      "geometry": [
        {"type": "mesh", "filename": ["../../assets/scenery/grass_a.glb"]}]}]

.. shift-parameters:: shift/core/environment/scenery.cc
   :function: Scenery::DeclareParameters

:source:`assets/scenery` holds trees, saplings, shrubs, ferns, grass and
rocks to strew (CC0, from Poly Haven), and :source:`assets/environments`
a beach and a forest rally stage built with them, to use in any model. Tests:
:source:`test/test_scenery.cc`.

Environment
-----------

``environment``, at the root of the model file, is what surrounds the model
as the viewer and the cameras see it: the sky, the sun, haze and a body of
water. None of it acts on the dynamics. Without it the viewer draws its own
gradient sky and sun, and the sun gives way to the model's lights; with it,
the sun shines along with them.

.. code-block:: json

   "environment": {
     "sky": {"type": "physical", "turbidity": 2.5},
     "sun": {"azimuth": 125.0, "elevation": 32.0},
     "water": {"height": 0.0, "wave_height": 0.25, "wave_direction": 265.0,
               "wind_speed": 5.0}
   }

The sky is a gradient (``"type": "gradient"``, the default), a clear sky
computed for the sun and the haze (``"physical"``, the model of Preetham,
Shirley and Smits, 1999), or an image of the whole sky (``"hdri"``, an
OpenEXR file in the equirectangular projection; the viewer reads it when
built with OpenEXR). The sky lights the scene too: the viewer and the ray
tracer take their ambient light and reflections from it. Under a physical
sky, sunlight reddens as the sun sets, and distant surfaces fade into the
air: the light from a surface at distance :math:`s` is dimmed, in red,
green and blue, by :math:`e^{-(\beta_R + \beta_A) s}`, with
:math:`\beta_R` the Rayleigh scattering of the air and :math:`\beta_A`
that of the aerosols of the haze, from the same optical depths as the sky
and the sunlight (the turbidity :math:`T` gives Ångström's coefficient
:math:`0.0461 T - 0.0459`, the aerosols' optical depth at 1 µm), the air
thinning with height over 8 km and the aerosols over 1.2 km; and the air
adds the light it scatters toward the camera, that of the sky at the
horizon in the direction of view, which is the light a long level path
through the air scatters (as the aerial perspective of Preetham, Shirley
and Smits). Distant land and sea turn the color of the horizon under them,
bluish away from the sun and brighter toward it: under a sky of turbidity
2.5 a surface 3.6 km away, the horizon of the sea seen from 1 m up, keeps
60 % of its light. Fog adds to the haze of the air.

.. shift-parameters:: shift/core/environment/environment.cc
   :function: Sky::DeclareParameters

The sun's ``azimuth`` is measured from +x toward +y, its ``elevation`` above
the horizon (the xy plane, +z down).

.. shift-parameters:: shift/core/environment/environment.cc
   :function: Sun::DeclareParameters

The fog dims surfaces with distance toward its color, and thins with height:
its density is :math:`\sigma(h) = \sigma_0 e^{-a (h - h_0)}` at height
:math:`h` (upward) for ``density`` :math:`\sigma_0`, ``height_falloff``
:math:`a` and ``height`` :math:`h_0`.

.. shift-parameters:: shift/core/environment/environment.cc
   :function: Fog::DeclareParameters

The water is a level surface at ``height`` with waves, everywhere: the
terrain stands out of it where it is higher. Light enters the water through
the waves, by Fresnel's law, and is absorbed along its path below the
surface at the rates of ``extinction``, red the fastest, so shallow water
shows the ground below it, tinted, and deep water is the water's ``color``.
The ray tracer refracts its rays into the water. The waves are a random
sea of the spectrum of a sea grown by the wind (JONSWAP, Hasselmann et al.,
1973) whose peak is at ``wavelength`` and whose significant height (the
mean crest to trough height of the highest third of the waves) is
``wave_height``, the longest close to ``wave_direction`` and the shorter
spread wider about it, each moving at the speed of deep water waves of its
length. A wind of ``wind_speed`` (10 m above the water) blowing toward
``wave_direction`` raises a sea of shorter waves on top of this swell, of
the same spectrum, its height and peak those that the wind grows over a
fetch of 10 km (Hasselmann et al., 1973, as in the *Shore Protection
Manual*, 1984), at most those of a sea fully grown (Pierson and Moskowitz,
1964): 5 m/s raises a chop 0.25 m high of waves 8 m long. The viewer sums them by fast Fourier transforms over three square
tiles of different sizes, from six peak wavelengths down to 1.6 m, which
holds the ripples a few centimeters long whatever the swell (Tessendorf, *Simulating Ocean Water*, 2001), each tile with levels of
detail that hold the mean height and slopes of the waves a pixel covers and
the spread of those slopes. Near the camera the ray tracer finds where its rays meet the waves
themselves, so that crests hide the troughs behind them and stand against
the horizon; the viewer and the ray tracer beyond tilt the level surface by
their slopes. A face of a wave turned away from a low camera is hidden by
the crest in front of it, which is seen in its place. Waves too short for
the pixels average out, and their slopes widen the reflection of the sky and
the sun instead (as in Bruneton, Neyret and Holzschuch, *Real-time Realistic
Ocean Lighting using Seamless Transitions from Geometry to BRDF*, 2010), so
that the distant water blends into the horizon. Where the water is shallow
over the terrain, the waves break, at a depth of their height over 0.78
(McCowan, 1894), so that no wave is higher there than 0.78 of the depth,
into bores of foam that run toward the shore once a wave
period at the speed of shallow water waves, :math:`\sqrt{g d}` at depth
:math:`d`, over the equilibrium profile of a sandy beach (Dean, 1977). The
period is that of deep water waves of ``wavelength``,
:math:`T = \sqrt{2 \pi \lambda / g}`: 2.1 s for waves 7 m long, 6.2 s for a
swell 60 m long. Each bore that reaches the shore runs up the beach and back
as a thrown body rises and falls, with a line of foam at its front, to a
height of :math:`0.06 \sqrt{H \lambda}` (Stockdon et al., 2006) for the wave
height :math:`H`, varying from wave to wave; the water thins out over the
sand at its edge. The water is only seen:
nothing in the dynamics acts on it yet.

.. shift-parameters:: shift/core/environment/environment.cc
   :function: Water::DeclareParameters

Contact of the environment
--------------------------

``contact``, at the root of the model file, is the contact model of the
environment, with the keys of a contact force module (see
:doc:`../force-modules/contact`): the materials of the ground and of the
objects around the model, and their pairs. It acts on its own when no
assembly has a contact force module. A contact force module of an assembly
overrides it: the materials and pairs it gives replace the environment's of
the same names, the environment's others stay in use, and the settings it
does not give (``damping_depth``, the combine rules, ...) are the
environment's. So a model of a robot or a vehicle carries the materials of
its own parts, and the environment those of the ground.

.. code-block:: json

   "contact": {"damping_depth": 1e-4,
               "materials": [{"name": "sand", "youngs_modulus": 5e7,
                              "poisson_ratio": 0.3, "restitution": 0.1,
                              "friction": 0.8}]}

Lights
------

Lights illuminate the 3D view. ``type`` is ``directional``, ``point``,
``spot`` or ``ies`` (a measured luminaire, from an IES file). A light on a
frame of a moving body moves with it.

A light's ``intensity`` can change while the model runs, set by a
controller (see :ref:`map`), to switch a lamp on and off. ``glow`` names
geometries of the light's body that light up with it, as the lens of a lamp:
the viewer adds to their shading their own color times the light's ``Kd``
and intensity, so that a stop lamp's lens turns bright red as the brake is
pressed. A mesh file that several bodies load is drawn once for all of
them, and glows on all of them.

.. shift-parameters:: shift/core/light/base.cc shift/core/light/spot.cc shift/core/light/ies.cc
