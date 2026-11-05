.. _about:
.. title:: About

About
=====

*Shift* is a general-purpose multibody dynamics engine and visualization
tool for robotics, automation and vehicle dynamics. A model is a JSON file
that lists bodies and the joints between them, arranged in assemblies and
subassemblies much as a machine is assembled in CAD software. *Shift* finds
the kinematic structure of the model itself: the spanning tree, the closed
loops, and the groups of loops that share bodies. It then integrates the
equations of motion, with force modules (contact, tires, bushings,
hydraulic cylinders, belts, ...), sensors and controllers acting on the
model as it runs.

*Shift* is written in C++ on top of the Kinematics Subroutine Library
(`KSL <https://shift-dynamics.github.io/ksl/>`_), a C library of spatial
algebra. It is created and maintained by Kristopher Wehage, PhD.

How it works
------------

A model with closed kinematic loops has more joint coordinates than degrees
of freedom: the loops constrain them. *Shift* keeps every joint coordinate
and splits them into an *independent* set, which the integrator advances,
and a *dependent* set, which follows from the loop closure equations
(*generalized coordinate partitioning*, GCP). The equations of motion are
solved in this split form,

.. math::

   \begin{bmatrix} M_{ii} & M_{id} & B_{di}^T \\ M_{di} & M_{dd} & I \\
   B_{di} & 0 & 0 \end{bmatrix}
   \begin{Bmatrix} \ddot{q}_i \\ \ddot{q}_d \\ \lambda_d \end{Bmatrix} =
   \begin{Bmatrix} f_i \\ f_d \\ \gamma_d \end{Bmatrix}

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
* **Run-time optimization**, now and then: the partition of each
  substructure is chosen by LU decomposition with complete pivoting, and
  lists of source and destination addresses are set up for evaluating the
  Jacobian and the constraints, mapping the mass matrix and solving the
  equations of motion in the partitioned order.
* **Run-time algorithm**, every step: the address lists are followed with a
  few simple operations. The size of each substructure's *B* matrix (its
  infinity norm) is watched; when it has grown by a set factor from its
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
