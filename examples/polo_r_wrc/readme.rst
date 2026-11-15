Volkswagen Polo R WRC
=====================

A World Rally Car of 2013 to 2016 on its own suspension and four wheel drive,
on a hilly gravel stage: away from rest through the gears, a fast left, a
jump, hard braking into a tight right, a left-right chicane, a second jump, a
long left and a stop. The source
model has only the body, the wheels and the brake calipers; the suspension,
steering and driveline are added here, with the published figures of the car
where there are any.

Credit
------

This work is based on "volkswagen polo r wrc"
(https://sketchfab.com/3d-models/volkswagen-polo-r-wrc-41278d76f7184a0c9639ba62a41ff2e3)
by amogusstrikesback2 (https://sketchfab.com/amogusstrikesback2) licensed
under CC-BY-4.0 (http://creativecommons.org/licenses/by/4.0/); see
*license.txt*. The body, wheel and caliper meshes (``.glb``) are split from
it, with the license plate changed to read SHIFT. The meshes of the added
parts (``.obj``) are drawn by *generate_polo_r_wrc.py*.

Published figures
-----------------

From the technical specifications of the Volkswagen Motorsport press kit for
the 2014 FIA World Rally Championship:

* engine: 1.6 l turbocharged straight four, transversally mounted ahead of
  the front axle, 33 mm air restrictor; 232 kW (315 hp) at 6,250 rpm and
  425 Nm at 5,000 rpm;
* transmission: six speed sequential gearbox; hydraulically actuated double
  disc sintered metal clutch (ZF); permanent four wheel drive with a fixed
  drive between the front and rear axles, and multi-plate limited slip
  differentials front and rear;
* suspension: McPherson struts front and rear with ZF dampers, about 275 mm
  of travel on gravel; rack and pinion steering;
* brakes: ventilated discs, 300 mm front and rear on gravel; wheels 7 x 15
  inch on gravel;
* track 1,610 mm, wheelbase 2,480 mm, minimum weight 1,200 kg.

Model
-----

The model has 55 bodies and 71 joints, in vehicle (SAE) coordinates: x
forward, y right, z down, in meters, with the origin on the ground between
the axles. The chassis (body shell, roll cage, engine, gearbox, crew and fuel)
floats on six joints.

Each corner is a McPherson strut:

* the strut has a ball joint at its top mount and slides, sprung and damped,
  along its axis to the knuckle, to which it is clamped;
* a lower arm turns on the body about an axis along x and carries the
  knuckle on a ball joint;
* a tie rod, with a ball joint at its inner end and a universal joint at the
  knuckle, steers the front knuckles from the rack and holds the toe of the
  rear ones from the body;
* a bump stop on the strut's rod meets the top of the damper tube 100 mm into
  the stroke from rest and stiffens as it is squeezed, and a rebound stop
  holds the strut 130 mm out from rest: about 230 mm of strut travel, near
  the published 275 mm at the wheel.

Each end has an anti-roll bar across the car, inboard of the axle: each half
turns on the body, and a drop link, with a ball joint at the top and a
universal joint at the bottom, joins its arm to the lower arm. A torsion
spring between the two halves is the twist of the bar, so the bar resists
roll but not heave.

The tie rods are behind the front axle, so a rack position to the right
(positive) turns the car left. A PID controller holds the rack at the
position a schedule asks for, standing in for the driver's hands on the
wheel.

The halfshafts are drawn on bodies that follow the suspension: each has a
universal joint at its differential, slides along its length and has a ball
joint at the hub. They add no constraint and carry no torque: the powertrain
drives the hub joints.

The drive is a powertrain module: the engine, with the published torque peak
and power; the clutch; six gears; and a fixed drive to both axles (no center
differential), each with a limited slip differential, and a brake at each
wheel. The tires are the Iowa tire, with a relaxation length, on rigid gravel
(friction coefficient 0.75).

The friction of the clutch, of the gears' clutches and of the differentials
is smoothed over a slip speed (``slip_speed``: 40 rad/s, 50 rad/s and
2 rad/s between the wheels of an axle). The smaller the slip speed, the
faster the motion it makes, and the fastest motion of the car sets how short
the integrator's steps must be: with the differentials at the default
0.5 rad/s and the clutches at half these, the car took twice as many steps.
From the top of the repository,

.. code:: bash

   build/shift/main/shift-cli --linearize 8 examples/polo_r_wrc/polo_r_wrc.json

lists the fastest motions at 8 s (see *Why the steps are small* under
Integrators in the model file documentation); they are now the clutches and
the driveline.

The route is a set of schedules in ``ROUTE`` in *generate_polo_r_wrc.py*: the
throttle, the clutch, the gear, the brakes and the rack position. The
steering is not closed around a path, so a change to the car changes its
line; the rack positions are tuned to the car as it is.

The stage is *polo_r_wrc_stage.tiff*, written by *generate_polo_r_wrc.py* from
the waves in ``TERRAIN``: hills 1.5 m high and 190 m long, crests and dips
80 m and 47 m long, and short waves, 1 to 2 cm high and 5 to 7 m long, at
angles to each other for the bumps of a gravel road; the ground rises and
falls about 2.7 m either way. Two crests across the route, 24 m long and
0.35 m and 0.45 m high, are jumps: the car leaves the ground for about 0.6 s
before the first braking and 0.7 s after the chicane, and lands on its bump
stops. They are placed on the route the car drives, so a change to the car
or to the route can miss them. The ground is flat around the start. Its texture is a
photograph of crushed gravel, tiled every 0.5 m, so that the stones are 2 to
4 cm across, with its tiles shifted at random so that they do not repeat
(``texture_variation``). Patches of packed dirt, about 8 m across, cover about
a third of the ground, and patches of grass, about 25 m across, about a
seventh (``surfaces``).

The headlamps and the tail lamps are IES lights on the chassis, at the lamps
of the body mesh, with the photometry of *polo_r_wrc_low_beam.ies* and
*polo_r_wrc_tail_lamp.ies*. These files are not measurements of the car's
lamps, which are not published: *generate_polo_r_wrc.py* draws them to the
photometric points of UN ECE regulations 112 and 7:

* low beam, for traffic on the right: a hot spot of about 29,000 cd 1.5
  degrees down and 2 degrees to the right, a wide, dimmer spread over the
  road just ahead, and the cutoff, flat 0.57 degrees down on the left and
  rising at 15 degrees on the right, above which there are only a few hundred
  candela of glare;
* tail lamp: red, 10 cd straight back (the regulation allows 4 to 17), half
  that at about 53 degrees to the side and 30 degrees up or down;
* stop lamp (*polo_r_wrc_stop_lamp.ies*): red, 80 cd straight back (the
  regulation allows 60 to 185), half that at about 41 degrees to the side and
  21 degrees up or down.

The stop lamps are lights of the chassis body, at the pupils of the tail
lamps (the square in the middle of each), and start dark. The map controller
``brake_lights`` reads the value of the schedule ``brake_pedal`` and sets
their intensity: off with the pedal up, full once it is pressed 5 %, so that
they light at each braking of the route. Their lenses glow with them
(``glow``): *right_stop_lamp_lens.obj* and *left_stop_lamp_lens.obj*, the
pupils, and *high_stop_lamp_lens.obj*, the high mounted stop lamp on the
spoiler, turn bright red as the car brakes. *generate_polo_r_wrc.py* cuts
them from *brake_lamps.glb*, the glow of the brake lights of the source,
whose texture it writes as *polo_r_wrc_lamp_glow.jpg*.

The lights fall off with the square of the distance. Their brightness is
scaled so that 10 lux shows a white surface as white, as eyes adapted to the
dark see it, not against daylight; the viewer's sky stays lit, so the beams
show on the ground ahead of the car and the tail lamps only faintly behind
it.

To see the suspension work, open *polo_r_wrc_suspension.json*: the same car
and route without the body shell. To ride along, tick *follow* in the
*Camera* box of the *Simulation* tab with ``chassis`` selected.

Estimates
---------

The gear ratios and the final drive are not published. They are estimates:
3.60, 2.96, 2.43, 2.00, 1.65 and 1.35 over a final drive of 3.5, so that first
reaches about 70 km/h and sixth about 190 km/h near 7,500 rpm. The hardpoints
were laid out from the meshes (wheels, calipers and arches); the meshes have
a wheelbase of 2.41 m and a track of 1.495 m at the middle of the tires, a
few percent less than the car, and are not rescaled. The masses, inertias,
springs, dampers, brakes and tire properties are estimates:

* chassis 1218 kg with the crew and fuel, its center of mass 0.1 m ahead of
  the middle of the wheelbase and 0.45 m above the ground; roll, pitch and
  yaw inertias 420, 1600 and 1750 kg·m². The whole car is about 1390 kg;
* wheels 23 kg with the gravel tire, radius 0.314 m; knuckles 12 kg;
* struts 36 kN/m and 2.6 kN·s/m at the front, 34 kN/m and 2.3 kN·s/m at the
  rear: about 1.6 and 1.7 Hz in heave, a soft gravel setup. The preloads hold
  the modelled lengths with the car at rest;
* brakes 1500 N·m at the front and 600 N·m at the rear at full pedal, a
  balance that keeps the rear wheels turning under braking;
* anti-roll bars of 800 N·m/rad at the front and 500 N·m/rad at the rear,
  solid steel bars about 17.5 and 15.5 mm across: through the drop links
  they add about half and a third to the roll stiffness of the springs;
* bump stops of polyurethane, 2 kN at 10 mm past contact rising to 60 kN at
  40 mm; rebound stops of 300 kN/m.

*generate_polo_r_wrc.py* lists every value.

Regenerating
------------

*polo_r_wrc.json*, *polo_r_wrc_suspension.json*, the stage, the photometry
of the lamps, the license plate and the meshes of the added parts are written
by *generate_polo_r_wrc.py*::

    python3 generate_polo_r_wrc.py polo_r_wrc.json

The body, wheel and caliper meshes are written by
*shift/scripts/split_vehicle_gltf.py* with the configuration
*shift/scripts/vehicle_splits/polo_r_wrc.json*, from the glTF download of the
source model (Blender 4.2 or later), after *generate_polo_r_wrc.py*: the
configuration replaces the plate texture of the source with
*polo_r_wrc_plate.png*::

    blender --background --factory-startup \
        --python shift/scripts/split_vehicle_gltf.py -- \
        shift/scripts/vehicle_splits/polo_r_wrc.json scene.gltf polo_r_wrc

It also writes *brake_lamps.glb*, from which *generate_polo_r_wrc.py* cuts
the lenses of the stop lamps (so the generator runs again after the split),
and *beams.glb*, the glow of the position lights with the headlight beams of
the source, which reach about 1 m ahead of the bumper; the model does not
use it.
