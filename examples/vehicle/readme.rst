Vehicle Chassis
===============

This directory contains model files for a generic passenger vehicle chassis with McPherson front suspension and semi-trailing link rear suspension.

.. image:: vehicle_chassis.png

The vehicle chassis contains six independent kinematic chord loops and three *kinematic substructures* that are identified automatically by the *Shift* software.

.. image:: vehicle_graph.png

Models
------

* ``vehicle.json``: the chassis with its suspension.
* ``vehicle_control.json``, ``vehicle_control_magic.json``, ``vehicle_control_iowa.json``: the car driven by sensors and PID controllers on Fiala, PAC2002 and Iowa tires. ``vehicle_control_5mps.json`` and ``vehicle_control_15mps.json`` hold other speeds.
* ``vehicle_terrain.json``, ``vehicle_terrain_iowa.json``: the car on a rally stage, ``rally_stage.tiff``, made by ``generate_rally_stage.py``, on Fiala and Iowa tires.
* ``tire_fiala.json``, ``tire_pac2002.json``, ``tire_iowa.json``: single tires.

For further information, refer to `<https://search.proquest.com/openview/2b96ee91ebc96f840bdada69a21b7cbe>`_
