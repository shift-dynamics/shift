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

   \begin{bmatrix} M^{\ell}_{ii} & M^{\ell}_{id} & B_{di}^T \\
   M^{\ell}_{di} & M^{\ell}_{dd} & I \\
   B_{di} & I & 0 \end{bmatrix}
   \begin{Bmatrix} \underline{\ddot{q}}_i \\ \underline{\ddot{q}}_d \\
   \underline{\lambda}^d \end{Bmatrix} =
   \begin{Bmatrix} \underline{f}^{\ell}_i \\ \underline{f}^{\ell}_d \\
   \underline{\gamma}^d \end{Bmatrix}

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

    <video src="_static/raptor.webm" width="640" controls>
    </video>


This Ford F-150 Raptor R model has 55 bodies and 70 joints and a full drivetrain:
engine, torque converter, ten-speed transmission and limited slip differential.
It runs 5x faster than real time. Kinematic substructuring finds the
overlapping clusters of chord loops in its suspension and steering automatically
and simplifies the equations of motion.

The truck is based on "2024 Ford F-150 Raptor R"
(https://sketchfab.com/3d-models/2024-ford-f-150-raptor-r-9fbd01c5acfb45e7a7bbdf4c8e2c4aa8)
by VTX (https://sketchfab.com/VTX_car), licensed under CC-BY-NC-SA-4.0
(http://creativecommons.org/licenses/by-nc-sa/4.0/).


Automatic assembly of complex mechanisms
****************************************

.. raw:: html

    <video src="_static/klann_walker.webm" width="640" controls>
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


Exact CAD geometry from STEP files
**********************************

.. raw:: html

    <video src="_static/track_frame.webm" width="640" controls>
    </video>


*Shift* reads parts and assemblies from STEP files and keeps their surfaces in the
exact form the file gives them: planes, cylinders, cones, spheres, tori and NURBS
surfaces (the free-form surfaces of CAD), bounded by their trimming curves. No
triangle mesh has to be prepared. The viewer draws the parts by evaluating these
surfaces on the graphics card, so curved faces look smooth at any distance.

Contact uses the same surfaces. Between two STEP parts, or a STEP part and any of
the other collision shapes, the points of contact, their depths and their normals are
solved on the surfaces themselves, so the normals and curvatures are those of the part and not of the
facets of a mesh, and the Hertz contact law uses the true curvature at each point.
In this track frame, half of a 20 t excavator, 44 links run around a drive sprocket,
an idler, five track rollers and a carrier roller, and the sprocket drives the frame
along the ground, with every contact found on the surfaces of the STEP files. Reading STEP
files needs the optional STEPcode library.


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
