Excavator
=========

This directory contains the hydraulic excavator model used in the paper *Generalized Coordinate Partitioning for Complex Mechanisms based on Kinematic Substructuring*.

.. image:: excavator.png

The excavator has five independent kinematic chord loops and three non-overlapping clusters of kinematic chord loops. The kinematic loops and *kinematic substructures* (highlighted in red, green and blue below) are identified automatically by the *Shift* software.

.. image:: digraph_w_constraints.png

Once the bodies and joints have been reordered according to their substructure assignments, the Jacobian matrix can be block-diagonalized, allowing for an approximate order of magnitude reduction in FLOPs required for applying GCP to this system.

Models
------

- ``excavator.json``: the excavator with hydraulic cylinders (``hydraulic_cylinder`` force modules) on the boom, stick and bucket. Each cylinder is fed by a proportional valve whose opening follows a dig cycle: the boom lowers, the stick comes in, the bucket curls, then the boom raises while the stick goes out and the bucket dumps.
- ``excavator_simple.json``: the same mechanism without actuators; the linkage moves freely under gravity.
- ``excavator_drive.json``: ``excavator.json`` on its tracks, driving over dry sand (``track_soil``). The sand is an SCM region, 10.5 m by 7.5 m, inside a 60 m square of rigid ground (``excavator_ground.tiff``, a terrain of type ``basic``) that rolls gently, by up to 8 cm in swells 5 m to 14 m long, and is flat under the machine as it starts. The sand's undisturbed surface follows the ground's. The sun is the only light. The undercarriage is free (a ``float`` joint). Each track is a closed chain of 46 links (``excavator_track_link.step``) over a drive sprocket, an idler, seven track rollers and two carrier rollers (``excavator_sprocket.step``, ``excavator_idler.step``, ``excavator_track_roller.step``, ``excavator_carrier_roller.step``). Each sprocket is driven by a PID speed controller, which follows a schedule: settle for 1 s, drive forward, turn on the spot to the left, drive forward again, then stop at 14 s. A brake holds the swing, and the cylinders start at the pressures that hold the arm still. It runs at about 1/20 of real time in a release build, too slow for the regular test suite in a debug build; it runs with ``meson test -C build --benchmark``.

  The parts and the model are generated from ``undercarriage.obj`` by ``scripts/make_excavator_drive.py``. The script cuts cross sections of the mesh: the link is the mesh's planar faces with the bushing at its pin, and each wheel is a prism of its sections. It writes the STEP files, ``undercarriage_frame.obj`` (the mesh without the parts) and the model. The masses are taken from published data, publicly available, for the Caterpillar 323, a machine of the same main dimensions (3650 mm from the sprocket to the idler, a 2380 mm gauge, 600 mm shoes); the mesh was drawn independently and is not a copy of that machine. The component weights are those of Caterpillar's 323 specifications [1]_, an operating weight of 24.1 t. The upper structure is 6.84 t with a 5.4 t counterweight at its back, a block rigidly joined to it; the boom, stick, bucket and cylinders are scaled to the weights of [1]_. The undercarriage is 7.55 t, its links 3.08 t: the wheels have the published masses of parts for machines of the 320 class (track roller 40 kg and carrier roller 17 kg [2]_, sprocket 40 kg [3]_, idler 109 kg [2]_), and the frame gets the rest, spread over its hull. The machine starts resting on the sand's surface, and the sand settles under it in the first second.

  Each idler is on a yoke that slides away from the sprocket, pushed by a preloaded recoil spring (``spring_damper``) that keeps the track tensioned. With the idler fixed, the slack links ahead of the sprocket fold back and forth on the ground.

  Contact is between primitive shapes standing in for the parts: a sphere at each link's bushing, and a box for each of its two rails. The treads of the sprocket and idler are flat ellipsoids under the bushings, and the sprocket has a sphere as a pin between each two seats. Spheres stand for the rollers' treads, and rings of spheres for the flanges of the idlers and track rollers. Each pair of these shapes has an exact contact; cylinders meet boxes and spheres through a search (GJK and EPA), whose varying depth takes the integrator to steps ten times shorter.

  Sources of the masses, all publicly available:

  .. [1] Caterpillar, *Technical Specifications for 323 Hydraulic Excavator*, AEXQ3361-01, 2022, "Major Component Weights" and "Dimensions": base machine 16 400 kg, track shoes (600 mm) 3080 kg, counterweight 5400 kg, swing frame 2090 kg, HD base frame with track and carrier rollers 4470 kg, HD reach boom 2010 kg, reach stick R2.9B1 990 kg, boom cylinders 340 kg. A copy is at https://www.avesco-cat.com/fileadmin/unsorted/323_Specification.pdf.
  .. [2] Caterpillar, *320D L and 320D LN Excavators (PCX) Disassembly and Assembly*, RENR8614, "Track Roller" (SMCS 4180) and "Carrier Roller" (SMCS 4154) and "Front Idler": track roller about 40 kg (88 lb), carrier roller about 17 kg (38 lb), front idler about 109 kg (240 lb). An excerpt is at https://www.slideshare.net/uddksemd8ud/caterpillar-cat-320dln-excavator-prefix-pcx-service-repair-manual-pcx00001-and-uppdf.
  .. [3] VXB Bearings, heavy duty 23 tooth drive sprocket for SH100, SH120, SH260 and SH265 excavators (product 1150-6425), 40 kg: https://vxb.com/products/heavy-duty-23-tooth-drive-sprocket-with-15-mountin.
