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

Names
-----

Every body, joint, frame and module has a ``name``. A joint names the
bodies it connects as ``[[body], frame]``: ``[["fixed"], "origin"]`` is the
origin of the fixed body, ``[["boom"], "pivot"]`` the frame ``pivot`` of
the body ``boom``. Force modules name frames as ``[body, frame]``, with the
body found by its name among all the active bodies of the model, so body
names should be unique across the model. Sensors, controllers and force
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
~~~~~~~~~~~~~~~~~~~~~~

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
that advances :math:`L` per turn has a pitch of :math:`L / 2\pi`; a positive
pitch is a right-handed screw, and a pitch of zero is a revolute joint. The
joint's force acts on the angle :math:`q`; a force :math:`F` along the axis
on the child does the work of a torque :math:`F \cdot` ``pitch`` on the
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
``include_directories``.

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

``"type": "basic"``: an elevation map from a TIFF image, on a frame of the
fixed body. Tires drive on terrain (see :doc:`../force-modules/tire`).

.. shift-parameters:: shift/core/terrain/base.cc shift/core/terrain/basic.cc

Lights
------

Lights illuminate the 3D view. ``type`` is ``directional``, ``point``,
``spot`` or ``ies`` (a measured luminaire, from an IES file). A light on a
frame of a moving body moves with it.

.. shift-parameters:: shift/core/light/base.cc shift/core/light/spot.cc shift/core/light/ies.cc
