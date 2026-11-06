Force modules
=============

Small models, each showing one force module:

* ``bushing.json``: the front left suspension of the vehicle model (control
  arm, knuckle, strut, tie rod and wheel) with its control arm on two
  bushings instead of a revolute joint. The bushings are stiff in every
  direction but the twist about the pivot axis, and the strut carries the
  weight; after a bump the arm rings down at 7.7 Hz and settles within 0.2 s.
* ``rubber_mount.json``: an engine on two rubber mounts, with a hardening
  term and two Maxwell elements on each coordinate, settling 1.95 mm down.
* ``beam.json``: a steel tube 2 m long, clamped at one end with a 10 kg
  mass at the other, as a chain of four beam elements; it swings about its
  static deflection of 28 mm.
* ``belt.json``: an engine's accessory drive, with the crankshaft driving
  the alternator and water pump through a belt with a backside tensioner.
  A PID controller brings the crankshaft to 80 rad/s, and the belt slips
  over the alternator during the spin-up.
* ``gear_mesh.json``: a 20 tooth pinion driving a wheel, loaded by a viscous
  torque, with a PID controller holding the pinion at 100 rad/s. The mesh
  force ripples about its mean of 100 N at the mesh frequency, 318 Hz.
* ``gear_mesh_helical.json``: the same pair as helical gears in oil, with
  the stiffness computed from the teeth and an axial force on the gears.
* ``journal_bearing.json``: a 10 kg shaft turning at 100 rad/s in a plain
  bearing, settling at an eccentricity ratio of 0.598.
* ``journal_bearing_finite.json``: the same shaft in a bearing as long as its
  diameter, with the finite model and the heating of the oil.
* ``thrust_collar.json``: a rotor spinning down on tapered pads until it comes
  down onto its surfaces.
* ``lead_screw.json``: a screw jack, with a 500 kg load on an Acme screw that
  a PID controller turns at 20 rad/s to raise it.
* ``lead_screw_hold.json``: the same jack left to hold its load with no drive.
* ``slip_spline.json``: a yoke pushed along its spline while the torque rises:
  it slides until the teeth load up, and then the friction stops it.
* ``powertrain.json``: a vehicle of flywheels on two axles, driven by an
  engine and converter through a three speed transmission, with schedule
  controllers for the throttle, gear changes, lockup clutch and brakes.
* ``powertrain_awd.json``: the same drive with a center differential, the
  clutch hydraulics and the losses of the gears.
* ``quarter_car.json``: a quarter car dropped on its wheel, with a shock
  absorber, a bump stop and a tire that only pushes.
* ``dozer_blade.json``: a blade pushed through sandy loam at 1.5 m/s until its
  pile is full.

The engine of ``rubber_mount.json`` is "V8 Engine"
(https://sketchfab.com/3d-models/v8-engine-7a957b5f9f954fe5b24e685f5e22046f)
by Tahminayeasminmony (https://sketchfab.com/Tahminayeasminmony), licensed
under CC-BY-4.0 (http://creativecommons.org/licenses/by/4.0/), reduced and
turned to the axes of shift by ``scripts/prepare_v8_engine.py``; see
``meshes/v8_engine_license.txt``. The other meshes are made by
``meshes/make_meshes.py``.

Run one from the top of the repository with::

  build/shift/main/shift-cli examples/force_modules/belt.json

See the documentation of each force module at
https://shift-dynamics.github.io/shift/force-modules/force-modules.html.
