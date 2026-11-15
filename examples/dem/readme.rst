Discrete element method
=======================

Grains as rigid spheres, clumps of spheres and convex hulls (the ``dem`` force module), coupled to the bodies of a model.

* ``plate_on_grains.json``: a steel plate of 0.2 kg on a vertical slide drops onto a bed of glass beads and two-sphere clumps in a box of walls, and comes to rest on it. The grains touch the plate on the triangles of its box. The integrator's steps are no longer than the grains' substep (``hmax``), as the grains' load on the plate is a step behind it. It runs on the CPU; for many grains, set ``"device": "gpu"``.
* ``grain_shapes.json``: the three kinds of grain side by side on a floor, a sphere, a clump of 12 spheres and the convex hull of a rock (``rock_00.obj``), placed from ``grain_shapes.csv`` and settling for 0.2 s. The clump's spheres fill the rock's hull, and both have its mass and inertia.

See the `Discrete Element Method <https://shift-dynamics.github.io/shift/dem/dem.html>`_ theory page of the documentation. The viewer draws the grains, recorded at each output step, and plays them back with the model.
