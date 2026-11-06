Collision modeling
------------------

Collisions are modeled by the `contact force module <https://shift-dynamics.github.io/shift/force-modules/contact.html>`_ between the collision geometry of bodies: spheres, ellipsoids, boxes, cylinders, capsules, planes, convex hulls and the exact surfaces of CAD parts (see `cad <../cad>`_). Without dimensions, a collision geometry is fitted to the body's meshes when the model is loaded.

The models in this directory are pairs and triples of spheres, ellipsoids, planes and convex hulls, and piles of 5 to 5000 bodies. The largest piles are benchmarks: they are left out of ``meson test -C build`` and run with ``meson test -C build --benchmark``. 

.. image:: screenshot.png

Meshes
------

``stanford_bunny.obj`` is the Stanford Bunny, from the Stanford 3D Scanning
Repository.

``human_skull.obj`` is a human cranium (without the mandible), derived from
"Visible Human Male Skull", NIH 3D entry 3DPX-012260
(https://3d.nih.gov/entries/3DPX-012260), licensed under CC BY 4.0
(https://creativecommons.org/licenses/by/4.0/). The mandible and scan debris
were removed, and the mesh was decimated to 70,000 triangles, scaled to a
front-to-back length of 0.19 m, and placed with its origin at the centroid
of its volume, +Y up (superior) and +Z forward (anterior).

Each ``collision_test_*.json`` has a skull version,
``collision_test_skull_*.json``, with the same bodies and orientations. In the
drop scenes (5 to 5000 bodies, with a floor) the skulls have the mass
properties of the solid bounded by ``human_skull.obj`` at 1800 kg/m³, about
the density of skull bone (0.88 kg each), as computed by shift from the
mesh, and start stacked in a square column over the
origin (2, 3, 6 or 12 skulls on a side, 0.27 m apart), in layers 0.81 m
apart from 0.2 m above the floor, each skull at a random height within its
layer (up to 0.54 m higher, so adjacent layers stay at least 0.27 m apart),
every other layer offset by half a cell so that each skull falls into the
hollow between four below it; no body starts in
contact with another or with the floor. In ``collision_test_skull_500.json``
and ``collision_test_skull_500_free.json``, made to pile up, every skull
collides as a convex hull (spheres and ellipsoids roll away); in
``collision_test_skull_500.json`` they are also of a less bouncy material
(restitution 0.1).
