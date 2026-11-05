Ford F-150 Raptor R
===================

A full-size pickup truck on its own suspension, driving straight ahead at
15 m/s on flat ground. The body, wheels and suspension parts of the source
model were split into separate meshes, one per moving body.

.. image:: raptor.png

Credit
------

This work is based on "2024 Ford F-150 Raptor R"
(https://sketchfab.com/3d-models/2024-ford-f-150-raptor-r-9fbd01c5acfb45e7a7bbdf4c8e2c4aa8)
by VTX (https://sketchfab.com/VTX_car) licensed under CC-BY-NC-SA-4.0
(http://creativecommons.org/licenses/by-nc-sa/4.0/). The meshes in this
directory may not be used commercially, and modified versions must carry the
same license; see *license.txt*.

Model
-----

The model has 55 bodies and 70 joints. It uses vehicle (SAE) coordinates: x
forward, y right, z down, in meters, with the origin on the ground. The
chassis (frame, body, engine and transmission) floats on six joints (three
sliding, three turning).

Front suspension, each side, a double wishbone:

* the lower and upper arms turn on the frame. The pivot axis of the upper arm
  is inclined, as on the truck;
* the knuckle hangs from both arms on ball joints;
* a coil-over stands on the lower arm. It has a universal joint at its top
  mount, a spring and damper along its axis, and a ball joint at the arm;
* a tie rod from the steering rack steers the knuckle. It has a ball joint at
  the rack and a universal joint at the knuckle. The source model has no tie
  rods or rack, so they are not drawn. The inner end of the tie rod lies on
  the line through the inner pivots of the arms, which keeps bump steer small.

The steering rack slides across the truck, held centred by a spring of
300 kN/m and a damper of 6 kN·s/m. These stand in for the steering column
and the driver's hands on the wheel.

Rear suspension, a live axle:

* a lower link on each side and a Panhard rod locate the axle. Each has a
  universal joint at the frame and a ball joint at the axle;
* the upper links are stiff springs along their length (2 MN/m), standing in
  for their rubber bushings, and they hold the pitch of the axle. With four
  rigid parallel links the axle could not roll, while the real axle rolls on
  its bushings;
* a coil-over on each side carries the axle.

The front halfshafts and the prop shaft are drawn on bodies that follow the
suspension: each has a universal joint at its inner end, slides along its
length, and has a ball joint at its outer end. They add no constraint and
carry no torque.

The rear wheels are driven by speed controllers holding 15 m/s. The tires use
the Fiala model.

The source model draws its wheels 7 to 8 cm below the hubs of its suspension.
The split moves the wheel meshes up to the hubs, so the truck sits that much
lower on its wheels than in the source.

Estimates
---------

The hardpoints (joint locations) were measured from the meshes. The masses,
inertias, springs, dampers and tire properties are estimates, not data for the
real truck:

* chassis 2150 kg, its center of mass 0.42 m ahead of the origin and 0.85 m
  above the ground; roll, pitch and yaw inertias 1100, 5000 and
  5300 kg·m². The whole truck is about 2740 kg;
* rear axle 190 kg; wheels 50 kg each, tire radius 0.394 m;
* front coil-overs 118 kN/m and 10 kN·s/m, a wheel rate of about 41 kN/m
  (1.3 Hz); rear coil-overs 56 kN/m and 4.2 kN·s/m, about 41 kN/m (1.5 Hz).
  The preloads hold the modelled lengths with the truck at rest.

*generate_raptor.py* lists every value.

Regenerating
------------

*raptor.json* is written by *generate_raptor.py*::

    python3 generate_raptor.py raptor.json

The meshes are written by *shift/scripts/split_vehicle_gltf.py* with the
configuration *shift/scripts/vehicle_splits/raptor.json*, from the glTF
download of the source model (Blender 4.2 or later)::

    blender --background --factory-startup \
        --python shift/scripts/split_vehicle_gltf.py -- \
        shift/scripts/vehicle_splits/raptor.json scene.gltf raptor
