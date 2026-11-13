Terramechanics
==============

Wheels and tracks on soil.

* ``wheel_test_bed.json``: a single-wheel test bed, a rigid wheel on deformable soil with the ``bekker_wong`` tire model, after the small wheel and TRI-1 lunar soil simulant of P.G. Kumar and S. Jayalekshmi (J. Theoretical and Applied Mechanics 57(3), 2019).
* ``wheel_test_bed_camber.json``: the same test bed with the wheel tilted.
* ``track_frame_soil.json``: one track of a 20 t excavator driving over dry sand, with the ``track_soil`` force module. Its contacts are on spheres and boxes, with the STEP parts (in ``examples/cad``) only drawn.
* ``track_frame_scm.json``: the same track on deformable dry sand, a terrain of type ``scm`` whose undisturbed surface, ``track_sand.tiff``, has gentle swells and hummocks about 1.5 cm high. The ``soil_contact`` force module carries the shoes and the track leaves a rut. A benchmark: it is not run by ``meson test -C build``.
* ``sph_blade.json``: a blade 30 cm wide pushed at 0.5 m/s through a pit of dry sand, an SPH region of an SCM terrain with bulldozing. The sand it pushes out of the pit lands on the SCM nodes and settles to the angle of repose. It needs the GPU (see the installation page).

``scripts/make_track_step.py`` writes the track models and ``scripts/make_track_terrain.py`` the elevation map.

See the `soil <https://shift-dynamics.github.io/shift/force-modules/soil.html>`_ and `tire <https://shift-dynamics.github.io/shift/force-modules/tire.html>`_ pages of the documentation. Another soil model is ``examples/force_modules/dozer_blade.json``.
