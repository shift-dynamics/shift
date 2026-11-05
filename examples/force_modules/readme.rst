Force modules
=============

Small models, each showing one force module:

* ``bushing.json``: a wheel and hub on a suspension control arm that pivots
  on two bushings, hung from a coil-over strut (a spring and damper) that
  carries the weight. The bushings are stiff in every direction but the
  twist about the pivot axis; after a bump the arm rings down at 3.8 Hz and
  settles within 0.2 s.
* ``rubber_mount.json``: an engine on two rubber mounts, with a hardening
  term and two Maxwell elements on each coordinate, settling 1.95 mm down.
* ``beam.json``: a steel tube 2 m long, clamped at one end with a 10 kg
  mass at the other, as a chain of four beam elements; it swings about its
  static deflection of 28 mm.
* ``belt.json``: an engine's accessory drive, with the crankshaft driving
  the alternator and water pump through a belt with a backside tensioner.
  A PID controller brings the crankshaft to 80 rad/s, and the belt slips
  over the alternator during the spin-up.

The engine of ``rubber_mount.json`` is "V8 Engine"
(https://sketchfab.com/3d-models/v8-engine-7a957b5f9f954fe5b24e685f5e22046f)
by Tahminayeasminmony (https://sketchfab.com/Tahminayeasminmony), licensed
under CC-BY-4.0 (http://creativecommons.org/licenses/by/4.0/), reduced and
turned to the axes of shift by ``scripts/prepare_v8_engine.py``; see
``meshes/v8_engine_license.txt``. The other meshes are made by
``meshes/make_meshes.py``.

Run one from this directory with::

  ../../build/shift/main/shift-cli belt.json

See the documentation of each force module at
https://shift-dynamics.github.io/shift/force-modules/force-modules.html.
