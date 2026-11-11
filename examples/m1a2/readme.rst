M1A2 Abrams tank
================

A main battle tank on its tracks and torsion bar suspension, driven by its
sprockets on firm ground: it settles, drives away to 1.5 m/s and turns right
by slowing its right track. There are two variants of the same model:

* ``m1a2.json``: contact on primitive shapes (boxes, spheres and thin
  ellipsoids), with the STEP parts only drawn;
* ``m1a2_step.json``: contact on the exact surfaces of the STEP parts (see
  ``examples/cad``).

Both are benchmarks: with 156 track links they are not run by
``meson test -C build``. In an optimized build (``build-release``) the 7 s of
*m1a2.json* take about 100 s; those of *m1a2_step.json* take about 3 hours,
nearly all of it in contact detection. Run them with ``meson test -C build --benchmark``,
or on their own::

  build-release/shift/main/shift-cli examples/m1a2/m1a2.json

Credit
------

This work is based on "abrams m1a2"
(https://sketchfab.com/3d-models/abrams-m1a2-0e98541f6f684311ac7a8abe08c9449e)
by coderslava402 (https://sketchfab.com/coderslava402), licensed under
CC-BY-4.0 (http://creativecommons.org/licenses/by/4.0/); see *license.txt*.
The hull, turret and gun, the road wheels and the idlers are drawn with its
meshes. The track links, sprockets, road arms and return rollers are STEP parts
written by *generate_m1a2.py*.

Model
-----

The model has 193 bodies. It uses vehicle (SAE) coordinates: x forward, y
right, z down, in meters, with the origin on the ground midway between the
sprocket and idler axles. The hull, with the turret and gun, floats on a float
joint. Each side has:

* seven road wheels, each on a trailing road arm. The arm's pivot is a
  revolute joint with a torsion bar, a linear spring whose preload holds the
  arm at its design angle (25 degrees below the pivot) under the static load.
  The arms of stations 1, 2 and 7 also have a rotary damper. The road wheels of
  the two sides are staggered, as on the tank, where the torsion bars of the
  two sides pass each other across the hull;
* a drive sprocket at the rear, with two rings of 21 teeth that drive the
  track's end connectors at every second tooth;
* an idler at the front, on a fixed axle. Its position along the hull is solved
  so that the loop of links closes, which moves it about 5 cm forward of the
  source's idler (the tank tensions its track by moving the idler);
* two return rollers under the top run of the track;
* a single pin track of 78 links of 190 mm pitch, 635 mm wide, each link on a
  revolute joint to the next. A link has a block that the wheels run on, a
  rubber pad on the ground side, a center guide horn that runs between the
  dual road wheels, and an end connector at each end of its pin, which the
  sprocket drives. Neighboring links are not tested for contact
  (``disabled_pairs``); their joint carries them.

A PID controller holds the speed of each sprocket at the target of a schedule.
A ``track`` module on each side logs the sag of the top spans and how far the
links stray from the middle of the track.

Contact
-------

Contact detection uses a ``contact`` force module and a ground plane.

In *m1a2.json* each link carries a box for its block, a box for its pad, a
box for its guide horn, and a sphere for each end connector. The dual tires of
the road wheels, idlers and return rollers are thin ellipsoids, of the tire's
radius about the axle, that run on the block's box. Each sprocket ring is a
thin ellipsoid under the end connectors, with a small sphere between each two
seats, which holds a connector in its seat. Ellipsoids are used rather than
cylinders because their contact with boxes and spheres has closed forms, while
a cylinder's takes the general GJK and EPA search, many times slower. The
pads stand on the ground on the corners of their boxes.

In *m1a2_step.json* the links, sprockets and return rollers touch on the
surfaces of their STEP files. The road wheels and idlers are drawn with the
meshes of the source and touch on the surfaces of *road_wheel.step*, an
inactive CAD geometry of the same size that is not drawn.

The steel parts use a Young's modulus a hundredth of steel's, as the track
examples in ``examples/cad`` do: real steel needs time steps about ten times
smaller for the impacts of the connectors in the seats.

Estimates
---------

The positions of the road wheels, sprockets and idlers were measured from the
meshes of the source. The overall dimensions match the tank's: a hull 7.8 m
long and 3.67 m wide over the skirts, and 9.65 m long with the gun forward.
Only three figures of the model are published data: the combat weight of
62 t, the road wheel diameter of 25 in (0.635 m) and the track width of
635 mm.

**Everything else is an estimate, not data for the real tank**, and the
model is not validated against it:

* masses: each track link 45 kg, road wheel 180 kg, road arm 120 kg, sprocket
  350 kg, idler 180 kg, return roller 25 kg. The hull, with the turret and gun,
  makes up the rest of the 62 t, about 49.7 t, with its center of mass 1.1 m
  above the ground and 0.2 m behind the origin, over the middle of the road
  wheels, so that the torsion bars, all alike, share its weight evenly;
* inertias: the hull's are those of a uniform box 7.9 by 3.66 by 2.0 m, and the
  wheels' those of uniform discs;
* suspension: a road arm of 0.45 m, and a torsion bar rate of 2.6e5 N/m at the
  wheel, which bounces the hull at about 1.3 Hz. Each damper is 4.5e4 N·s/m at
  the wheel. The model has no bump stops;
* track: the shape of the links, the 190 mm pitch, the sprocket's teeth and the
  return rollers. The real track is a double pin track (T158) with rubber
  bushed pins;
* contact: the stiffness and friction of the rubber of the wheels, of the pads
  and of the ground;
* drive: PID controllers on the sprockets stand in for the engine,
  transmission and steering.

*generate_m1a2.py* lists every value.

Regenerating
------------

*generate_m1a2.py* writes the two models and the STEP parts::

    python3 examples/m1a2/generate_m1a2.py

The meshes are written by *shift/scripts/split_vehicle_gltf.py* with the
configuration *shift/scripts/vehicle_splits/m1a2.json*, from the glTF
download of the source model (Blender 4.2 or later). It also writes
*unused.glb*, the source's one-piece track and its sprockets, which the model
does not use; delete it::

    blender --background --factory-startup \
        --python shift/scripts/split_vehicle_gltf.py -- \
        shift/scripts/vehicle_splits/m1a2.json scene.gltf m1a2
