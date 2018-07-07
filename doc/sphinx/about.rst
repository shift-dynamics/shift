.. title:: About

About
=====

*Shift* is a general-purpose software tool for simulating and visualizing rigid body dynamics of robotic systems. The underlying algorithms are tailored to process complex robotic systems with multiple overlapping kinematic chord loops, but the software is also capable of efficiently modeling simple systems, such as those consisting of serial chains or *tree* topologies.

A unique aspect of *Shift* is its file format, which is based on the JSON (Javascript Object Notation) specification. In a *Shift* JSON file, the system topology is specified by enumerating a number of bodies and joints. The bodies and joints can be arranged in hierarchies according to their assembly and subassembly assignments. In this way, the user specifies complex topologies in much the same way a physical system is assembled; the software does not burden the user with identifying a *spanning tree* or *scene graph* or with identifying closed kinematic loops if present in the system. If internal loops exist, they are detected automatically by the software when the file is initially loaded.

In *Shift*, model reduction is performed automatically using *Generalized Coordinate Partitioning* (GCP). Additionally, the efficiency of applying the GCP algorithm is increased through the automatic identification of overlapping clusters of kinematic chord loops, referred to as *Kinematic Substructures* throughout this documentation.

Features
---------

Faster than real--time simulation
**********************************

.. raw:: html

    <video src="_static/vehicle_chassis.webm" width="640" controls>
    </video>


This simulation of a vehicle chassis uses the kinematic substructuring algorithm to automatically detect overlapping clusters of chord loops to simplify the dynamics equations. The model runs many times faster than real-time on a desktop workstation.


Automatic assembly of complex mechanisms
****************************************

.. raw:: html

    <video src="_static/klann_linkage.webm" width="640" controls>
    </video>


The *Shift* file format is a simple list of bodies and joints. The spanning tree and overlapping clusters of chord loops (i.e. *kinematic substructures*) are detected automatically at run-time from the user's input file.


Heuristic application of Generalized Coordinate Partitioning
************************************************************

.. raw:: html

    <video src="_static/bricard.webm" width="640" controls>
    </video>


Bricard's paradoxical linkage cannot be parameterized by one independent variable throughout its range of motion. *Shift's* heuristic reapplication of Generalized Coordinate Partitioning (GCP) successfully detects when to reapply GCP on an as--needed basis, even for exceptional or paradoxical linkages.


Mobility analysis and interactive positioning in a 3D environment
******************************************************************

.. raw:: html

    <video src="_static/brake.webm" width="640" controls>
    </video>


In addition to solving dynamics equations, *Shift* has tools to analyze each joint's contribution to overall mechanism mobility, compute mechanical advantages and to interactively position models in a 3D visualization environment.


OpenGL rendering
*****************

.. raw:: html

    <video src="_static/auto_lod.m4v" width="640" height="440" controls>
    </video>


*Shift* uses custom GLSL shaders written to implement the Phong lighting model with multiple light sources and dynamic shadow mapping. 3D models are loaded using the Open Asset Import library (Assimp), which supports a variety of 3D model formats. Other visualization techniques are currently in development, such as automatic level of detail rendering of terrain meshes using tesselation control and evaluation shaders.


Software Design
---------------
The numerical algorithms in *Shift* are written in the C programming language and compiled as a static library and command line interface. A 3D visualization tool is written in C++ using OpenGL and the Qt windowing framework.

Additionally, *Shift* adheres to the following design criteria:

    * The user interfaces (i.e. the C command line interface or C++ graphical interface) are separated from the core functionality, thereby allowing alternative front-ends to be programmed with no changes to the core code-base.

    * The JSON file format is designed to be compatible with modern web technologies.

    * Internal and external datastructures allow a representation of assemblies and subassemblies.

    * Updating the kinematics and dynamics of a hierarchical assembly representation requires minimal logic and computational overhead.

    * The graphical user interface is inspired by modern CAD software. For example, the model tree overview pane at the left of the viewport should be familiar to most engineers.


Workflow
--------

When *Shift* is launched, the system topology is loaded from an external JSON text file. The analysis is split in to the following steps:


Preprocessing
*************

The following operations are performed once at the beginning of simulation and need not be called again, unless the mechanism topology changes:

* The user-defined body-joint-body information is initially loaded into computer working memory as a linked list.

* The topology of the mechanism is analyzed automatically to identify groups of overlapping chord loops, which are referred to as *kinematic substructures* or as `s2_substructure_t <doxid-structs2__substructure__t>`_ in the code.

* Bodies and joints are arranged in level increasing order within each substructure, thereby block-diagonalizing the Jacobian matrix.

* To minimize cache misses, data is copied by value from linked-list representation to a struct-of-arrays data structure in contiguous memory in level-increasing order.


Run-Time Optimization (RTO)
***************************

The following operations may be called once or periodically during the course of a simulation. The operations are performed again only if generalized coordinate partitioning indicates to use a different split of independent/dependent variables during a running simulation:

* Set up source and destination memory pointers to:

      * Evaluate primary Jacobian matrix in permuted order.

      * Evaluate primary constraints (and first and second time derivatives) in permuted order (no need to evaluate secondary constraints).

      * Factor the primary Jacobian matrix.

      * Precondition the primary constraints (i.e. sparse solve for the *B matrix* which is used to find part of the right null space of Jacobian).

      * Map mass matrix in original order to permuted order.

      * Evaluate equations of motion and store the results in unpermuted order.


Run-Time Algorithm (RTA)
************************

During a simulation, the following operations are carried out at every time step. RTA uses only a few primitive functions to set up and sparse solve the equations of motion, stepping through arrays of memory locations without complex logic.

* Carry out operations set up in Run-Time Optimization step using source and destination memory pointers.

* Monitor ratio of infinity norm of the B matrix associated with each block sub-Jacobian matrix of each substructure. If the infinity norm increases more than 2 times from the initial starting value, the simulation is paused and the Run-Time Optimization step is carried out again to regenerate pointer arrays.
