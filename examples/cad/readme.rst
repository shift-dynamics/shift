CAD geometry
============

Models that use the exact surfaces of STEP parts, for drawing and for contact (``"type": "cad"`` geometry and collision geometry). The ``.step`` files are in this directory, and ``scripts/make_cam_step.py`` and ``scripts/make_track_step.py`` make the cam and track parts. Reading STEP files needs STEPcode (meson option ``step``).

Models
------

* ``cad_parts.json``: the exact geometry of STEP parts and an assembly, drawn from their surfaces.
* ``cam_follower.json``: a cam turning against a sprung follower, in contact on their exact surfaces.
* ``track_loop.json``: a track of 44 links around a drive sprocket and an idler, resting on soft, damped ground, with the sprocket driven at 1 rad/s by a PID controller. A benchmark: it is not run by ``meson test -C build``.
* ``track_frame.json``: the same track on the track frame of half of a 20 t excavator, with five track rollers and a carrier roller. The frame is on a float joint, held up only by the track on soft, damped ground, so the sprocket drives it along. Also a benchmark.

The same track on soil, ``track_frame_soil.json`` and ``track_frame_scm.json``, is in ``examples/terramechanics``.

Parts
-----

* ``gear_tooth.step`` and ``ring_sector.step``: a tooth of a spur gear with involute flanks and a bent ring, used by the contact tests and written by ``scripts/make_contact_step.py``.

See the `contact <https://shift-dynamics.github.io/shift/force-modules/contact.html>`_ page of the documentation.
