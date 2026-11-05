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
