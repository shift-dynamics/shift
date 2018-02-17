Contact Detection Overview:
===================================

Level 1:  Top level contact detection module
--------------------------------------------------------------

Current status:
*****************
No code / code stubs written yet.


Functions detailed in:
***********************
* **src/force_module/contact.c/.h**
* **src/force_module/service.h/.h**


Description:
**************
Level 1 sets up and deploys all contact detection events including:

* Performs prune and sweep pass on all collision objects using simplified collision geometry (sphere, obj, or abb - to be determined)

* Setup (in preprocessing step) and step through (at runtime) array of function and source / destination pointers to evaluate contacts. Calls functions defined in **gjk.c/.h** or specialized simpler collision tests (sphere/sphere?) Defined in **minkowski.c/.h**

* Handle contacts as appropriate (feed back forces into simulation or flag gui program to stop NR) or simply print to stdout



Level 2: Support mapping functions
---------------------------------------------------

Current status:
*****************
Code stubs written in **minkowski.c/.h**

Functions detailed in:
***********************
* **src/core/minkowski.c/.h** (name subject to change)
* **src/core/gjk.c/.h**

Description:
***************
Level 2 handles events between two convex objects.

**minkowski.c/.h** provides support mapping functions for two convex objects and contains a struct with data that is cached for a pair of collision objects. Provides mechanism for collision events to go in and out of scope (not written yet). Will contain a pointer to two s2_collisionobject_t data structures (see below).

**gjk.c/.h** contains the actual gjk algorithm and calls support mapping functions defined in **minkowski.c** and extreme vertex support mapping function pointers set in :code:`s2_collisionobject_t`


Level 3: Extreme vertex support mapping functions
-----------------------------------------------------------------------

Current status:
******************
Partial code in **src/core/collisionobject.c/.h**, some place holder extreme vertex support mapping functions generated for ellipsoids and convex hulls written, many of these can/will be improved.

Functions detailed in:
*************************
* **src/core/collisionobject.c/.h**
* **src/core/ellipsoid.c/.h**
* **src/core/sphere.c/.h**
* **src/core/convexhull.c/.h**

Description:
***************
:code:`s2_collisionobject_t` provides a common interface to all convex objects. The :code:`s2_collisionobject_t` data structure contains a void pointer to a sphere, ellipsoid, or convex hull data structure, an enum describing what kind of object is stored at the pointer (i.e. the enum provides the necessary information to cast the pointer to the appropriate type if desired) and an extreme vertex support mapping function. So far only code to support spheres, ellipsoids and convexhulls has been generated and it is planned to add support for other convex objects in the future (capped cylinders, obb,abb,etc). The data structures and extreme vertex support mapping functions are located in **sphere.c/.h**, **ellipsoid.c/.h** and **convexhull.c/.h** respectively. Level 1 and Level 2 code should only interface with the more primitive objects (sphere/ellipsoid/convexhull) through the :code:`s2_collisionobject_t` data structure and wrapper functions. In this way if new collision geometry is added later on, the generic interface remains unchanged.
