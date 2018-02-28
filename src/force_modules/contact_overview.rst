Contact Detection Overview:
===================================

Level 1:  Top level contact detection module
--------------------------------------------------------------

Current status:
*****************
Code stubs written in contact.c for `s2_context_sweepAndPrune()` and `s2_context_contactDetection()`


Functions detailed in:
***********************
* **src/force_module/contact.c/.h**
* **src/force_module/service.h/.h**


Description:
**************
Level 1 sets up and deploys all contact detection events including:

* Performs prune and sweep pass on all collision objects using simplified collision geometry (sphere, obj, or abb - to be determined)

* Provides mechanism for collision events to go in and out of scope (code stub in **contact.c:s2_context_sweepAndPrune**).

* Setup (in preprocessing step) and step through (at runtime) source / destination pointers to evaluate contacts. Code stubs have been written in  **contact.c:s2_context_contactDetection**. The source and destination pointers are stored in a doubly linked list of :code:`s2_simplex_t` datastructures, accessible through :code:`context->simplexHead` and :code:`simplexTail`. Refer to the comments in :code:`s2_context_contactDetection` to see how these would be called.

* Handle contacts as appropriate (feed back forces into simulation or flag gui program to stop NR) or simply print to stdout. No code has been written yet for this step.

Level 1 code interfaces with Level 2 code for each :code:`s2_simplex_t` data structure by calling::

    s2_simplex_t* simplex;
    s2_simplex_support(simplex);

If collision has occured, it will be stored as :code:`true` in the :code:`collision` field of the :code:`s2_simplex_t` datastructure (otherwise :code:`false`) and is available for use by other force modules.

Contact detection is performed before all other force modules.


Level 2: Minkowski difference support mapping functions
---------------------------------------------------------

Current status:
*****************
Code stubs have been written in **simplex.c/.h**. A support mapping data structure has been defined in :code:`s2_simplex_t`; information that should be cached between subsequent calls to the function should be added to the :code:`s2_simplex_t` struct as appropriate.

Functions detailed in:
***********************
* **src/core/simplex.c/.h**
* **src/core/gjk.c/.h**

Description:
***************
Level 2 handles events between two convex objects.

**simplex.c/.h** provides support mapping functions for two convex objects and contains a struct with data that is cached for a pair of collision objects. Will contain a pointer to two :code:`s2_collisionobject_t` data structures (see below).

**gjk.c/.h** contains the core gjk algorithm and calls support mapping functions defined in **simplex.c** and stored in :code:`s2_simplex_t` and extreme vertex support mapping function pointers set in :code:`s2_collisionobject_t`

Level 2 code interfaces with Level 3 code by calling::

    s2_collisionobject_t* collisionobject;
    s2_vector_t* searchVector;
    s2_vector_t* extremeVertex = s2_collisionobject_support(collisionobject, searchVector);

Look in **simplex.c** for additional examples.



Level 3: Extreme vertex support mapping functions
-----------------------------------------------------------------------

Current status:
******************
Partial code in **src/core/collisionobject.c/.h**, some place holder extreme vertex support mapping functions generated for ellipsoids and convex hulls written, many of these will need to be further flushed-out / improved.

Functions detailed in:
*************************
* **src/core/collisionobject.c/.h**
* **src/core/ellipsoid.c/.h**
* **src/core/sphere.c/.h**
* **src/core/convexhull.c/.h**

Description:
***************
:code:`s2_collisionobject_t` provides a common interface to all convex objects. The :code:`s2_collisionobject_t` data structure contains a void pointer to a sphere, ellipsoid, or convex hull data structure, an enum describing what kind of object is stored at the pointer (i.e. the enum provides the necessary information to cast the pointer to the appropriate type if desired) and an extreme vertex support mapping function. So far only code to support spheres, ellipsoids and convexhulls has been generated and it is planned to add support for other convex objects in the future (capped cylinders, obb,abb,etc). The data structures and extreme vertex support mapping functions are located in **sphere.c/.h**, **ellipsoid.c/.h** and **convexhull.c/.h** respectively. Level 1 and Level 2 code should only interface with the more primitive objects (sphere/ellipsoid/convexhull) through the :code:`s2_collisionobject_t` data structure and wrapper functions. In this way if new collision geometry is added later on, the generic interface remains unchanged.
