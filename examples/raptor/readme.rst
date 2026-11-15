Ford F-150 Raptor R
===================

A full-size pickup truck on its own suspension, driven by a powertrain around
a route over rolling ground: away from rest, a double lane change, a bend to
the right, a bend to the left and a stop. The body, wheels and suspension parts of the source
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

The steering rack slides across the truck. A PID controller holds it at the
position a schedule asks for, with a damper of 6 kN·s/m. These stand in for the
steering column
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

The rear wheels are driven by a powertrain module, through a limited slip
differential: an engine and torque converter with a lockup clutch, and a ten
speed automatic, with figures like those of the real truck (a supercharged V8).
All four wheels have brakes.

An engaged clutch is what sets the step of the integrator, so the clutches are
chosen to run fast. They are smoothed over 20 to 30 rad/s, which lets a gear
slip by up to about 13 rad/s at full engine torque, and the inertia of the
transmission's output is well above a real one. Together these leave the truck
about 2% behind the distance it covers with sharper clutches, and the route
runs five to six times faster. More damping does not help: a damper is a fast
term like a clutch, and five times the shaft damping takes three times as long.

The route is a set of schedules in ``ROUTE`` in *generate_raptor.py*: the
throttle, the gear, the lockup clutch, the brakes and the rack position, which
is the target of the steering PID. A rack position of 9 mm turns the truck at
about 20 degrees per second at 17 m/s on level ground, and a positive position
turns it right. The steering is not closed around the path: on the rolling
ground the tires push back on the rack, it holds about 8 mm, and the bends come
out wider than on level ground.

The ground is *raptor_terrain.tiff*, written by *generate_raptor.py* from the
waves in ``TERRAIN``. Long swells pitch and lift the truck, and shorter waves
cross its path at an angle, so the wheels on the two sides ride over different
heights. The suspension travels 14 to 17 cm over the route and no wheel leaves
the ground. The ground is flat around the start, where the truck is placed.

To ride along in the viewer, tick *follow* in the *Camera* box of the
*Simulation* tab with ``chassis`` selected, then run the model. Choose *ground*
next to it to see the truck bounce and tilt on its suspension.

The tires use
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

*raptor.json*, the terrain and the license plate are written by
*generate_raptor.py*::

    python3 generate_raptor.py raptor.json

The source has no license plate: *license_plate.obj* is a plate of 12 by 6
inches reading SHIFT (*license_plate.png*), on the rear bumper above the
hitch, drawn with the chassis. The other meshes are written by *shift/scripts/split_vehicle_gltf.py* with the
configuration *shift/scripts/vehicle_splits/raptor.json*, from the glTF
download of the source model (Blender 4.2 or later)::

    blender --background --factory-startup \
        --python shift/scripts/split_vehicle_gltf.py -- \
        shift/scripts/vehicle_splits/raptor.json scene.gltf raptor
