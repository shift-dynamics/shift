.. _soil:
.. title:: Soil

Soil
====

Soft soil carries a wheel or a track shoe by its pressure, and pushes it along
by its shear. The force modules that model it use the same description of the
soil: the Bekker-Wong wheel (``"tire_type": "bekker_wong"`` of the
:doc:`tire`), the track soil module (``"type": "track_soil"``, below), and on
deformable soil, a terrain of type ``scm`` (see the Terrain section of the
:doc:`../model-file/model-file`), the soil contact module
(``"type": "soil_contact"``, below).

Describing a soil
-----------------

A soil is given by name, from the table below, or by its properties, in SI
units and radians. Any property given beside a name replaces the table's
value:

.. code:: javascript

   "soil": {"name": "lete_sand", "shear_modulus": 0.011, "unit_weight": 15000}

   "soil": {"sinkage_exponent": 0.8, "cohesive_modulus": 1e5,
            "frictional_modulus": 5e6, "cohesion": 1300,
            "friction_angle": 0.54, "shear_modulus": 0.01}

.. shift-parameters:: shift/core/force_module/soil.cc
   :function: force_module::Soil::DeclareParameters

The pressure under a plate of width :math:`b` pressed in by :math:`z` is
Bekker's :math:`p = (k_c/b + k_\phi)\,z^n`, with the cohesive modulus
:math:`k_c`, the frictional modulus :math:`k_\phi` and the sinkage exponent
:math:`n`. Its shear strength is :math:`c + p\tan\phi` for the cohesion
:math:`c` and the friction angle :math:`\phi`, reached as the plate slides, by
Janosi and Hanamoto's :math:`1 - e^{-j/k_j}` after the slide :math:`j`
(:math:`k_j` is the shear modulus).

The SPH soil (a terrain of type ``sph``, see :doc:`../model-file/model-file`)
treats the soil as an elastoplastic continuum, not by Bekker's law. It needs
three more properties: ``youngs_modulus`` :math:`E` and ``poisson_ratio``
:math:`\nu`, the soil's elastic stiffness before it yields, and optionally
``dilatancy_angle`` :math:`\psi`, how much the soil swells as it shears (0,
the default, for no change of volume; at most :math:`\phi`). The table does
not give them. A loose sand has :math:`E` of a few MPa; a lower :math:`E`
lets the SPH take longer substeps, at the cost of a soil that is too soft
under small loads. The friction angle, cohesion and unit weight are the same
as for the other models.

Earth pressure
--------------

The soil's unit weight :math:`w` gives the passive earth pressure on the face
of a grouser or the side of a shoe or wheel, the pressure of a face pushing
into the soil. On a vertical face in level soil, down to the depth :math:`h`,
the force normal to the face is
:math:`\tfrac12 w h^2 \kappa_p + 2 c h \sqrt{\kappa_p}` per unit of width. The
coefficient comes from Coulomb's theory, with the friction angle
:math:`\delta` between the soil and the face (``wall_friction``):

.. math::

   \kappa_p = \frac{\cos^2\phi}{(1 - s)^2}, \quad
   \kappa_a = \frac{\cos^2\phi}{(1 + s)^2}, \quad
   s = \sqrt{\frac{\sin(\phi + \delta)\sin\phi}{\cos\delta}}

:math:`\kappa_a` is the active coefficient, of a face giving way to the soil,
and :math:`\kappa_0 = 1 - \sin\phi` (Jaky) the one at rest, against a face that
does not move. Tables of Coulomb's coefficients usually give the thrust along
its inclined line; these are its part normal to the face, smaller by
:math:`\cos\delta`. The term in :math:`c` is Bell's, and is the exact one only
for :math:`\delta = 0`.

The default :math:`\delta = 0`, a smooth face, gives Rankine's
:math:`\kappa_p = \tan^2(45^\circ + \phi/2)`. A rough steel face has
:math:`\delta` of about :math:`\phi/2`. Coulomb's plane failure surface overstates
the passive pressure as :math:`\delta` grows, more so above :math:`\phi/2`, and it
has no finite value when :math:`s` reaches 1 (:math:`\phi = \delta = 45^\circ`),
which the model refuses. In those cases give ``passive_coefficient``, from a
test or a table for curved failure surfaces, in place of the computed value.

The table
---------

The values of the first fourteen soils are those tabulated by Wong
:cite:`wong2001` and reproduced by :cite:`schepelmann2025`; the moisture
content is that of the sample. The last is the lunar soil simulant of
:cite:`kumar2019`. Wong's table gives no shear modulus or unit weight, so
those default to 0.01 m and 16 kN/m\ :sup:`3`: set them for the soil at hand.
The moduli are in kN/m\ :sup:`n+1` and kN/m\ :sup:`n+2`; the properties in the
JSON are in N and m.

.. list-table::
   :header-rows: 1

   * - name
     - moisture
     - :math:`n`
     - :math:`k_c`
     - :math:`k_\phi`
     - :math:`c` (kPa)
     - :math:`\phi` (deg)
   * - ``dry_sand``
     - 0%
     - 1.1
     - 0.99
     - 1528.43
     - 1.04
     - 28
   * - ``sandy_loam``
     - 15%
     - 0.7
     - 5.27
     - 1515.04
     - 1.72
     - 29
   * - ``clayey_soil``
     - 38%
     - 0.5
     - 13.19
     - 692.15
     - 4.14
     - 13
   * - ``clayey_soil_wet``
     - 55%
     - 0.7
     - 16.03
     - 1262.53
     - 2.07
     - 10
   * - ``heavy_clay``
     - 25%
     - 0.13
     - 12.70
     - 1555.95
     - 69.95
     - 34
   * - ``heavy_clay_wet``
     - 40%
     - 0.11
     - 1.84
     - 103.27
     - 20.69
     - 6
   * - ``lean_clay``
     - 22%
     - 0.2
     - 16.43
     - 1724.69
     - 68.95
     - 20
   * - ``lean_clay_wet``
     - 32%
     - 0.15
     - 1.52
     - 119.61
     - 13.79
     - 11
   * - ``lete_sand``
     -
     - 0.79
     - 102
     - 5301
     - 1.3
     - 31.1
   * - ``upland_sandy_loam``
     - 51%
     - 1.10
     - 74.6
     - 2080
     - 3.3
     - 33.7
   * - ``rubicon_sandy_loam``
     - 43%
     - 0.66
     - 6.9
     - 752
     - 3.7
     - 29.8
   * - ``north_gower_clayey_loam``
     - 46%
     - 0.73
     - 41.6
     - 2471
     - 6.1
     - 26.6
   * - ``grenville_loam``
     - 24%
     - 1.01
     - 0.06
     - 5880
     - 3.1
     - 29.8
   * - ``snow``
     -
     - 1.6
     - 4.37
     - 196.72
     - 1.03
     - 19.7
   * - ``tri1_lunar_simulant``
     -
     - 0.404
     - 6.60
     - 139.00
     - 0.36
     - 43.33 (K = 0.0103 m)

Soil contact
------------

``"type": "soil_contact"`` puts bodies on deformable soil, the terrain of type
``scm``, and lowers the soil under them. Each node of the terrain's grid under
a body is tested by a ray cast up its terrain frame's z axis against the
body's collision geometry, from ``max_sinkage`` (of the terrain) below the
surface: the body's lowest point over the node, if it is below the surface,
gives the sinkage :math:`s`, from the undisturbed surface. The nodes in
contact form patches, joined along the grid; a patch of area :math:`A` and
perimeter :math:`P` has the width :math:`b = 2A/P` in Bekker's law.

At each node the pressure follows Bekker's law while the soil is loaded deeper
than ever before, and below its deepest sinkage :math:`z_p` the stiffer
straight line back to zero, as in the track soil module: the soil springs
back by :math:`z_p` / ``unloading_ratio`` (of the terrain), the depth the rut
keeps. The terrain's ``damping`` scales the pressure by
:math:`1 + c \, \dot{s}`, with the speed of sinkage :math:`\dot{s}` (Hunt and
Crossley), so that the force starts from zero as the body touches the soil.

A body set on the surface at the start carries none of its weight there. It
falls, and with little damping it sinks to about twice its depth at rest,
which the soil keeps: dropped onto a soil whose force grows as
:math:`s^{1.1}`, a load overshoots to about 2.1 times its weight. The
terrain's ``settle_time`` settles the bodies onto the soil instead. Over the
first ``settle_time`` seconds, ``settle_damping`` (by default 20 s/m) is
added to the damping and fades smoothly to nothing, as
:math:`\frac{1}{2}(1 + \cos(\pi t / t_s))`, so the force has no jump when
the settling ends. Less damping lets the bodies fall in. Much more holds them
up, so that they are still sinking slowly when the settling ends and fall the
rest of the way. With the default, a ``settle_time`` of 2 s or more settles a
plate to within 1% of Bekker's depth.

On the fine track model (121 kN) with no damping, the soil's force reaches
2.4 times the track's weight as it lands, and the deepest node sinks 171 mm.
Settled over 2 s, the deepest node is at 70 mm after 0.5 s, within 5 mm of
Bekker's law for the mean pressure of 76 kPa, the rest from the higher
pressure under the rollers. As the track then drives, the rollers roll over
links lying on the soil, so every node under the track is loaded by each
roller in turn and sinks to the depth of the pressure under a roller: five
rollers carrying about 20 kN each over one or two links give 86 to 172 kPa,
and the deepest node reaches about 113 mm. This is Bekker's law applied to
the pressure peaks under the rollers, which the mean ground pressure of a
track with few rollers understates.
The shear stress follows Mohr-Coulomb with the curve of Janosi and Hanamoto,
:math:`\tau = (c + p \tan\phi)(1 - e^{-|\underline{j}|/K})`, against the
shear displacement :math:`\underline{j}` of the soil at the node, which grows
by the slip of the body over the node while the body stays on it.

The force is continuous over the evaluations of a step: the nodes change only
when the integrator accepts the step, and :math:`\underline{j}` grows from its
accepted value by the trapezoidal rule on the slip velocity. Each node's soil
is the terrain's there (its ``soil`` or ``soil_map``).

The bodies press on the soil with the collision geometry named in
``geometry``, or with all of theirs. Geometry that should touch only the soil,
such as the shoes of a track, can be left out of contact detection with
``"contact": false``. A body's CAD geometry (``"type": "cad"``) presses on the soil
with its exact faces: the faces are projected onto the grid, so a fine grid
resolves a shoe's grousers. On a grid of 5 mm, a link of the excavator track
(``examples/cad/track_link.step``) runs about 4 times slower than a box of
its shoe.

.. code:: javascript

   {"type": "soil_contact", "name": "soil", "bodies": ["link1", "link2"],
    "geometry": ["shoe"]}

Outputs: ``normal_force`` (the total), ``contact_nodes`` and ``sinkage`` (the
deepest, from the undisturbed surface).

**SPH soil.** On an SPH terrain or region (see
:doc:`../model-file/model-file`), the bodies listed in ``bodies`` are not
cast onto a grid. Their collision geometry is filled with markers that move
among the particles, and the particles push back on them. The soil's force
over each accepted step, averaged over the step, is applied to the body
during the next step. The force of a single step comes from a few SPH
substeps and is noisy. The body's mass smooths it, and its mean carries the
body's weight.

**Bulldozing.** Without it, the soil a body presses down is compacted: the rut
is left and nothing rises around it. With the terrain's ``bulldozing``
enabled, the soil is pushed aside instead, as loose sand is. Every
``interval`` (by default 1 ms; 0 for after every accepted step):

1. The nodes bodies were on since the last time form patches. Gaps up to ``closing_distance``
   wide (by default 5 cm) are part of the patch, such as the cracks between
   the shoes of a track and the wider gaps that open between them as they
   turn up around the sprocket or idler.
2. Compaction: under each node of a patch the soil first compacts. A layer
   ``compaction_depth`` deep (by default the width of the patch, about the
   depth the pressure reaches) can lose up to 1 − ``unit_weight`` /
   ``max_unit_weight`` of its volume, from the soil's unit weight as it lies
   and compacted as far as bodies compact it. Each node keeps how much it has
   compacted. Only the soil pressed down beyond that is pushed aside, so a
   shallow print leaves no rim and a deep one pushes up only what could not
   compact. A soil without ``max_unit_weight`` does not compact: all of the
   soil pressed down is pushed aside.
3. The soil each patch pushed aside since the last time, times
   ``flow_factor``, is spread evenly over its rim: the nodes around it, also
   diagonally. The nodes in the gaps get none, so soil does
   not pile up in the cracks between a body's parts.
4. Erosion: ``erosion_iterations`` times (by default 30), soil flows
   between neighboring nodes outside the patches, from the higher to the
   lower, where the slope between them is steeper than ``erosion_angle`` (by
   default the friction angle of the soil). Soil moves one node per pass. The
   patches are a wall: soil on the rim slides outward, away from the body,
   and none slides back under it or into the gaps between its parts until it
   has left. Erosion goes on at the following times until the slopes settle,
   also once the bodies have left.

The interval matters because bulldozing changes the surface under the bodies.
Changed at every step, the soil force jumps at every step, and the
integrator, which takes steps of microseconds through the contacts, cuts its
steps to follow the jumps: on the fine track model, bulldozing after every
step took twice as many steps as no bulldozing. The wall is what removes most
of those jumps; the interval makes bulldozing itself cheap.

For example, a loose dry sand with void ratios of about 0.8 as it lies and
0.5 compacted (grains of specific gravity 2.65) has unit weights of 14.4 and
17.3 kN/m³. Under a track 0.6 m wide it compacts by about 17% of a 0.5 m
layer, so the first 85 mm or so of sinkage compacts. On the fine track model,
after 0.5 s and with ruts about 112 mm deep, the berms beside the track are
66 mm high on average with compaction and 143 mm without.

Slopes are measured from the undisturbed surface, so the slopes of an
elevation map are not eroded. With a ``flow_factor`` of 1 the volume of soil
pushed aside is conserved, except soil pushed off the grid; more than 1, the pushed soil
loosens and takes more room. A body that goes into a pile, or into soil that
slid into a rut, cuts it down to its own depth and pushes it on, so a blade
drives a pile ahead of it. Piles do not yet carry bodies: the pressure is
from the undisturbed surface, so a body is carried only below it.

.. code:: javascript

   "terrain": [{"type": "scm", "name": "sand", "parent": [["fixed"], "terrain"],
                "x": [0, 6], "y": [0, 2], "spacing": 0.01,
                "soil": {"name": "dry_sand", "unit_weight": 14400,
                         "max_unit_weight": 17300},
                "bulldozing": {"enable": true, "flow_factor": 1.0,
                               "erosion_iterations": 30}}]

.. shift-parameters:: shift/core/force_module/soil_contact.cc
   :exclude: name, type, active

Track soil
----------

``"type": "track_soil"`` puts the soil under the shoes of a track. Each shoe
is a rectangle on its link: ``shoe_center``, ``shoe_normal`` (out of the shoe,
into the soil when it is on the ground), ``shoe_along`` (along the track,
across the grousers), ``shoe_length`` and ``shoe_width``, given in the
coordinates of the link. The ground is the plane :math:`z` = ``ground``, with
the soil below it (z down). The shoe is sampled at ``stations_along`` by
``stations_across`` points, each carrying an equal part of its area.

The pressure at a point sunk :math:`z` below the ground follows Bekker's law
while the soil is loaded further than ever before. Below the deepest sinkage
:math:`z_p` reached there it falls along a straight line back to zero, so that
the soil springs back by :math:`z_p` / ``unloading_ratio``, and a shoe that
is lifted leaves its rut behind. The deepest sinkage is a state that follows
the sinkage from below, over ``plastic_time``, and so there is no jump as the
soil is loaded and unloaded. Off the soil it fades over ``rut_time``: a shoe
lifted for a moment comes back to its rut, and one going round the track comes
down on fresh soil.

The soil also gives traction and side force. The shear on the shoe's plate is
the soil's shear strength under it, built up by the displacement :math:`\underline{j}` of the shoe
over the soil, :math:`1 - e^{-|\underline{j}|/k_j}`. The earth pressure of the soil in its way
adds to it: on the leading grouser along the track (of width ``shoe_width``
and the depth of the grouser's embedding, at most ``grouser_height``), and on
the side of the shoe across it, mobilized by the same factor. The displacement
:math:`\underline{j}` (two states, along and across the ground) is the slip of the shoe's
center over the ground while it is on the soil, and relaxes to zero over
``shear_time`` when the shoe is lifted.

.. code:: javascript

   {"type": "track_soil", "name": "soil", "soil": {"name": "lete_sand"},
    "shoes": ["link1", "link2"], "shoe_center": [0.095, 0, 0.082],
    "shoe_normal": [0, 0, 1], "shoe_along": [1, 0, 0],
    "shoe_length": 0.18, "shoe_width": 0.6, "grouser_height": 0.025}

On deformable soil, a terrain of type ``scm``, the soil contact module
carries the shoes, and must list their links. Each shoe over the terrain's
grid then takes from the track soil module only the earth pressure on its
leading grouser and its side, with the terrain's soil and the sinkage below
its undisturbed surface. A shoe beyond the grid takes all of the forces above,
on the ground plane, with the module's ``soil``, which then defaults to the
terrain's.

Outputs, for each shoe, ``<shoe>_sinkage``, and for all the shoes
``normal_force``, ``traction`` (along the track) and ``side_force`` (across it),
in the ground plane.

``examples/terramechanics/track_frame_soil.json`` drives one track of a 20 t excavator
over dry sand. It settles about 0.1 m into the sand and moves forward at about
0.31 m/s, close to the 0.32 m/s of its sprocket's rim. The track's contacts are
on spheres and boxes, with the STEP parts only drawn, so that it runs in about
25 s in an optimized build; the same track with contact on the STEP surfaces
is a benchmark (``examples/cad/track_frame.json``).

``examples/terramechanics/track_frame_scm.json`` drives the same track on deformable dry
sand (a terrain of type ``scm`` of 0.05 m spacing), the soil contact module
carrying a box of each shoe and its grousers. Its sprocket and idler sit
25 mm above the rollers' line, as on a machine, so they carry little of the
load. It settles for 2 s with the sprocket held, about 60 mm into the sand,
then drives at about 0.32 m/s. The shoes keep sinking while it drives, to
about 95 mm on average (up to 0.13 m) after 3 s, leaving its rut. The cause
is that the shoes rock, about 0.06 to 0.1 rad, as they pass under the
rollers: the soil keeps the deepest sinkage it reaches, so each tilt digs
the edges of a shoe in a little deeper. One shoe of the track rocked by
0.08 rad every 0.6 s under a constant load sinks from 57 to 73 mm in the
first cycle and to 88 mm after 12, with or without bulldozing. It takes
about 50 s in an optimized build, so it is a benchmark.

The sinkage levels off once the whole length of track on the ground has
landed on fresh sand. Driven for 12 s (about 3.8 m), the fine variant,
``track_frame_scm_fine.json``, sinks quickly at first, then more slowly:
its shoes sink on average 75 mm when it starts to drive, 123 mm after 1.2 m,
and a steady 134 mm from about 3 m on, about one ground length. Each shoe
lands flat at the same 75 to 85 mm and sinks to about 190 mm by the
sprocket, over some 17 rocking cycles and five roller passes, so the rut is
deepest at the rear: the frame trims 34 mrad nose up, and the roller next
to the sprocket carries 26 kN to the 17 to 20 kN of the others.

The viewer (``build-release/shift/main/shift``) draws the rut while the track
runs and when you play it back. At each output time it records only the nodes
whose level changed, so it can scrub forward and back without storing the
whole grid at every frame. **View > Smooth deformable terrain** draws a smooth
(Catmull-Rom) surface through the nodes. Unchecked, the surface is bilinear
between the nodes, which is the surface the soil contact module uses. **Run**
continues from the rut shown: each node takes the level drawn and keeps it
when the soil springs back. **Reset** returns the sand to undisturbed. The
ray-traced view draws the rut too, on the same surface: up to four
triangle pairs per cell along each side, fewer on a large grid.
``--no-smooth-terrain`` turns smoothing off for ``--image``.

.. shift-parameters:: shift/core/force_module/track_soil.cc
   :exclude: name, type, active

Example and test
----------------

* :source:`examples/terramechanics/wheel_test_bed.json` and
  ``wheel_test_bed_camber.json``: a Bekker-Wong wheel on a single-wheel test
  bed.
* Tests, in :source:`test/terramechanics`:
  :source:`test/terramechanics/test_terramechanics.cc` (Bekker-Wong and the
  track's soil), :source:`test/terramechanics/test_soil_tool.cc`,
  :source:`test/terramechanics/test_scm.cc` (the SCM terrain and soil maps),
  :source:`test/terramechanics/test_soil_contact.cc` (soil contact and
  bulldozing: the volume is conserved, the rim rises, a crack between two
  plates takes no soil, slopes settle to the
  angle of repose, and a plate cuts through a pile) and
  :source:`test/terramechanics/test_scm_playback.cc` (the SCM terrain in the
  viewer and in HDF5 logs), :source:`test/terramechanics/test_sph.cc` (the
  SPH soil: the GPU matches the CPU bit for bit, a block carries its weight,
  a column collapses as far as experiments show, and a heap settles at the
  friction angle) and :source:`test/terramechanics/test_sph_terrain.cc`
  (the SPH terrain, regions of terrains, and a plate settling on SPH soil).
