.. _examples:
.. title:: Examples

Examples
========

The :tree:`examples` directory holds runnable models. Every one of them is
also a test: ``meson test -C build`` loads and simulates each model in
``examples/`` (a few large collision models are kept as benchmarks, run
with ``meson test -C build --benchmark``). Run any of them from its
directory:

.. code:: bash

   cd examples/force_modules
   ../../build/shift/main/shift-cli belt.json

or open it in the viewer, ``build/shift/main/shift``.

Force modules, sensors and controllers
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 30 25 45

   * -
     - model
     - shows
   * - .. image:: ../images/examples/bushing.jpg
          :width: 220px
          :alt: bushing
     - :source:`examples/force_modules/bushing.json`
     - :doc:`../force-modules/bushing`: the front left suspension of the
       vehicle model, with its control arm on two bushings
   * - .. image:: ../images/examples/rubber_mount.jpg
          :width: 220px
          :alt: rubber_mount
     - :source:`examples/force_modules/rubber_mount.json`
     - :doc:`../force-modules/rubber-mount`: an engine on two rubber mounts,
       with hardening and two Maxwell elements per coordinate
   * - .. image:: ../images/examples/beam.jpg
          :width: 220px
          :alt: beam
     - :source:`examples/force_modules/beam.json`
     - :doc:`../force-modules/beam`: a cantilevered steel tube as a chain
       of four beam elements, with a mass at its tip
   * - .. image:: ../images/examples/belt.jpg
          :width: 220px
          :alt: belt
     - :source:`examples/force_modules/belt.json`
     - :doc:`../force-modules/belt`, :ref:`sensors`, :ref:`controllers`:
       an accessory drive with a backside tensioner, belt slip, and PID
       speed control of the crankshaft
   * - .. image:: ../images/examples/excavator.jpg
          :width: 220px
          :alt: excavator
     - :source:`examples/excavator/excavator.json`
     - :doc:`../force-modules/hydraulic-cylinder`: a 30 t excavator with
       four cylinders, starting held, through a dig cycle; CSV and HDF5
       logs of the cylinders' pressures and forces
   * - .. image:: ../images/examples/vehicle_control.jpg
          :width: 220px
          :alt: vehicle_control
     - :tree:`examples/vehicle`
     - :doc:`../force-modules/tire`, sensors and PID controllers: a
       passenger car with Fiala or PAC2002 tires, on flat ground or a rally
       stage (``vehicle_control.json``, ``vehicle_control_magic.json``,
       ``vehicle_terrain.json``), and single tires (``tire_fiala.json``,
       ``tire_pac2002.json``)
   * - .. image:: ../images/examples/raptor.jpg
          :width: 220px
          :alt: raptor
     - :source:`examples/raptor/raptor.json`
     - a full-size pickup truck, 55 bodies and 70 joints: double wishbone
       front suspension, a live rear axle, Fiala tires and PID speed
       control of the rear wheels at 15 m/s (model by VTX,
       CC-BY-NC-SA-4.0, see :source:`examples/raptor/readme.rst`)
   * - .. image:: ../images/examples/collision_50.jpg
          :width: 220px
          :alt: collision_50
     - :tree:`examples/collision`
     - :doc:`../force-modules/contact`: pairs of spheres, ellipsoids,
       planes and convex hulls, and piles of 5 to 5000 bodies

Mechanisms
----------

.. list-table::
   :header-rows: 1
   :widths: 30 25 45

   * -
     - model
     - shows
   * - .. image:: ../images/examples/double_pendulum.jpg
          :width: 220px
          :alt: double_pendulum
     - :source:`examples/pendulum/pendulum.json`,
       :source:`examples/pendulum/double_pendulum.json`
     - the smallest models; the starting point of :ref:`getting-started`
   * - .. image:: ../images/examples/slider_crank.jpg
          :width: 220px
          :alt: slider_crank
     - :tree:`examples/slider_crank`
     - a slider-crank, and variants split into two or three kinematic
       substructures
   * - .. image:: ../images/examples/bricard.jpg
          :width: 220px
          :alt: bricard
     - :tree:`examples/bricard`
     - Bricard's paradoxical linkage, which no single joint can drive
       through its whole range: *Shift* repartitions its coordinates as it
       moves
   * - .. image:: ../images/examples/klann_linkage.jpg
          :width: 220px
          :alt: klann_linkage
     - :source:`examples/klann_linkage/klann_linkage.json`
     - Klann's walking linkage, a planar multiloop mechanism
   * - .. image:: ../images/examples/sarrus.jpg
          :width: 220px
          :alt: sarrus
     - :source:`examples/sarrus/sarrus.json`
     - the Sarrus linkage, a spatial multiloop mechanism
   * - .. image:: ../images/examples/brake.jpg
          :width: 220px
          :alt: brake
     - :tree:`examples/brake`
     - a bicycle caliper brake, with three loops and one degree of freedom
   * - .. image:: ../images/examples/abiflexx_d4_1300.jpg
          :width: 220px
          :alt: abiflexx_d4_1300
     - :source:`examples/abiflexx_d4_1300/abiflexx_d4_1300.json`
     - a delta robot: 3 revolute and 12 spherical joints, 39 degrees of
       freedom
   * - .. image:: ../images/examples/kuka_iiwa.jpg
          :width: 220px
          :alt: kuka_iiwa
     - :source:`examples/kuka_lbr_iiwa_14_r820/kuka_iiwa.json`
     - a 7-axis collaborative robot arm
   * - .. image:: ../images/examples/excavator_simple.jpg
          :width: 220px
          :alt: excavator_simple
     - :source:`examples/excavator/excavator_simple.json`
     - the excavator linkage alone: five loops in three substructures

Geometry and terrain
--------------------

.. list-table::
   :header-rows: 1
   :widths: 30 25 45

   * -
     - model
     - shows
   * - .. image:: ../images/examples/terrain.jpg
          :width: 220px
          :alt: terrain
     - :tree:`examples/terrain`
     - Bricard's linkage over an elevation map, with asphalt, concrete and
       sand textures
   * - .. image:: ../images/examples/cad_parts.jpg
          :width: 220px
          :alt: cad_parts
     - :source:`examples/cad/cad_parts.json`
     - the exact geometry of STEP parts and an assembly, drawn from their
       surfaces
   * - .. image:: ../images/examples/track_loop.jpg
          :width: 220px
          :alt: track_loop
     - :source:`examples/cad/track_loop.json`
     - a track of 44 links around a drive sprocket and an idler, resting on
       the ground, with the sprocket driven at 1 rad/s by a PID controller:
       contact between the exact STEP surfaces of the links and wheels (a
       benchmark: it is not run in the regular tests)
   * - .. image:: ../images/examples/track_frame.jpg
          :width: 220px
          :alt: track_frame
     - :source:`examples/cad/track_frame.json`
     - the same track on the track frame of half of a 20 t excavator, with
       five track rollers and a carrier roller, free to move forward, up and
       down and to pitch, so the sprocket drives it along the ground (a
       benchmark)
   * - .. image:: ../images/examples/cube.jpg
          :width: 220px
          :alt: cube
     - :tree:`examples/scripting`
     - building a model file from Python (``cube.py``, ``shift.py``)
