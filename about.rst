About
=====

Overview
--------

Shift, a simulation tool for automation and robotics, is a general-purpose modeling environment targeted to robotics and vehicle dynamics applications. The software was created and is maintained by Kristopher Wehage, PhD.

Refer to Shift's website at `<https://kriswehage.github.io/shift/>`_ for additional information on how to use the software and API.


Workflow
--------

Shift loads and analyzes the topology of a mechanism specified in a JSON
text file.  The analysis is split in to the following steps:

Preprocessing
*************

The following operations are performed once at the beginning of simulation
and need not be called again, unless the mechanism topology changes:

*   The user-defined body-joint-body information is initially loaded into
    computer working memory as a linked list.

*   The topology of the mechanism is analyzed automatically to identify groups
    of overlapping chord loops, which are referred to as *kinematic substructures* -- or as *s2_substructure_t* in the code.

*   Bodies and joints are arranged in level increasing order within each
    substructure, thereby block-diagonalizing the Jacobian matrix.

*   To minimize cache misses, data is copied by value from linked-list
    representation to a struct-of-arrays data structure in contiguous
    memory in level-increasing order.


Run-Time Optimization (RTO)
***************************

The following operations may be called once or periodically during the course
of a simulation. The operations are performed again only if generalized
coordinate partitioning indicates to use a different split of
independent/dependent variables during a running simulation:

*   Set up source and destination memory pointers to:

    *   Evaluate primary Jacobian matrix in permuted order.

    *   Evaluate primary constraints (and first and second time derivatives)
        in permuted order (no need to evaluate secondary constraints).

    *   Factor the primary Jacobian matrix.

    *   Precondition the primary constraints (i.e. sparse solve for the *B
        matrix* which is used to find part of the right null space of Jacobian).

    *   Map mass matrix in original order to permuted order.

    *   Evaluate equations of motion and store the results in unpermuted
        order.


Run-Time Algorithm (RTA)
************************

During a simulation, the following operations are carried out at every time
step. RTA uses only a few primitive functions to set up and sparse solve the
equations of motion, stepping through arrays of memory locations without
complex logic.

*   Carry out operations set up in Run-Time Optimization step using
    source and destination memory pointers.

*   Monitor ratio of infinity norm of the B matrix associated with each
    block sub-Jacobian matrix of each substructure.  If the infinity norm increases more than 2 times from the initial starting value, the simulation is paused and the Run-Time Optimization step is carried out again to
    regenerate pointer arrays.


References
----------

Refer to the following publications for additional details on how the software works:

* Wehage K., Computational Methods for Dynamics and Mobility Analysis of Multiloop Mechanisms and Robotic Systems, Thesis, December 2016 `<https://search.proquest.com/openview/2b96ee91ebc96f840bdada69a21b7cbe>`_

* Wehage K. Ravani B., A Computational Method for Formulation and Solution of Dynamical Equations for Complex Mechanisms and Multibody Systems, Proceedings of IDETC/DIE at Cleveland Ohio, August 2017, `<https://doi.org/10.1115/DETC2017-67766>`_

* Wehage K., Wehage R., Ravani B., Generalized coordinate partitioning for complex mechanisms based on kinematic substructuring, Mechanism and Machine Theory 92:464-483, October 2015 `<https://doi.org/10.1016/j.mechmachtheory.2015.06.006>`_
