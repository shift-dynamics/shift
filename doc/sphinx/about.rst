.. _about:
.. title:: About

About
=====

*Shift* is a general-purpose multibody dynamics engine and visualization
tool for robotics, automation and vehicle dynamics. The input to the tool 
is a file with a model definition specified in a YAML or JSON format. The
file contains a list of bodies and joints connecting them, arranged in assemblies and
subassemblies much as a machine is represented in CAD software. *Shift* automatically
finds the kinematic structure of the model, including the spanning tree, closed
loops, and clusters of overlapping loops. It then sets up and integrates the
equations of motion, with force modules (contact, tires, bushings,
hydraulic cylinders, belts, etc.), sensors and controllers acting on the
model as it runs.

*Shift* is written in C++ on top of the Kinematics Subroutine Library
(`KSL <https://shift-dynamics.github.io/ksl/>`_), a C library of spatial
algebra. It is created and maintained by Kristopher Wehage, PhD.

How it works
------------

A model with closed kinematic loops has more joint coordinates than degrees
of freedom; the loops constrain them. *Shift* keeps every joint coordinate
and splits them into an *independent* set and a *dependent* set, chosen so
that the loop closure equations can be solved for the dependent coordinates
in terms of the independent ones (*generalized coordinate partitioning*,
GCP). The integrator advances all of the coordinates and velocities, of both
sets; at each evaluation the accelerations of both sets are found by solving
the equations of motion in this split form,

.. math::

   \begin{bmatrix} M_{ii} & M_{id} & B_{di}^T \\ M_{di} & M_{dd} & I \\
   B_{di} & I & 0 \end{bmatrix}
   \begin{Bmatrix} \underline{\ddot{q}}_i \\ \underline{\ddot{q}}_d \\
   \underline{\lambda}_d \end{Bmatrix} =
   \begin{Bmatrix} \underline{f}_i \\ \underline{f}_d \\
   \underline{\gamma}_d \end{Bmatrix}

by the run-time optimized natural constraint algorithm (RNCA).

Loops that share bodies are grouped into *kinematic substructures*; loops
in different substructures do not affect each other, so the Jacobian of the
loop equations is block diagonal, with one block per substructure, and each
block is factored on its own. The work is split into three stages:

* **Preprocessing**, once: the bodies and joints are read into a linked
  list, the substructures are found, the bodies and joints of each are
  sorted from the base outward, and their data is copied into contiguous
  arrays in that order, so that the run-time loops walk through memory in
  sequence.
* **Run-time optimization**, as-needed periodically during a simulation: 
  the partition of independent and dependent variables for each
  substructure is chosen by LU decomposition with complete pivoting, and
  lists of source and destination addresses are set up for evaluating the
  Jacobian and the constraints, mapping the mass matrix and solving the
  equations of motion in the partitioned order.
* **Run-time algorithm**, every step: the address lists are followed with a
  few simple operations. The size of each substructure's *B* matrix (its
  infinity norm) is monitored; when it has grown by a set factor from its
  value at the last partition (``infinity_norm_multiplier`` of the
  ``rnca`` solver), the partition has become poorly conditioned and the
  run-time optimization is repeated.

See :ref:`topology` and :ref:`generalized_coordinate_partitioning` for the
theory, and
:cite:`wehage2015generalized,wehage2017` for the
published algorithms.

Features
---------

Faster than real-time simulation
*********************************

.. raw:: html

    <video src="_static/vehicle_chassis.webm" width="640" controls>
    </video>


This simulation of a vehicle chassis uses kinematic substructuring to find
overlapping clusters of chord loops automatically and simplify the
equations of motion. The model runs many times faster than real time on a
desktop workstation.


Automatic assembly of complex mechanisms
****************************************

.. raw:: html

    <video src="_static/klann_linkage.webm" width="640" controls>
    </video>


The model file is a simple list of bodies and joints. The spanning tree and
the overlapping clusters of chord loops (*kinematic substructures*) are
found when the model is loaded.


Repartitioning as the mechanism moves
**************************************

.. raw:: html

    <video src="_static/bricard.webm" width="640" controls>
    </video>


No single joint of Bricard's paradoxical linkage can describe its motion
through its whole range. *Shift* detects when its partition of the
coordinates becomes poorly conditioned and chooses a new one as the
simulation runs, so exceptional and paradoxical linkages need no special
treatment.


Mobility analysis and interactive positioning
**********************************************

.. raw:: html

    <video src="_static/brake.webm" width="640" controls>
    </video>


*Shift* can also analyze each joint's contribution to the mobility of a
mechanism, compute mechanical advantages, and position models
interactively in its 3D viewer.


Programs
--------

``shift-cli``
    Loads a model, simulates it over the time span of its integrator and
    writes the loggers' files. It traps floating point division by zero, so
    a model that divides by zero stops with an error rather than producing
    NaN.

``shift``
    The 3D viewer (Qt 6), which opens a model, runs it and shows the bodies,
    their geometry, terrain, lights and shadows.

Both are thin front ends of the core library ``shift_core``, which holds all
of the modeling and numerics, so other programs can embed it directly
(see :ref:`api`).
