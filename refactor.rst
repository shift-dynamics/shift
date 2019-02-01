*Shift* C++ Refactoring Overview
================================

Abstract
--------

This working document describes the various *Shift* data structures, how they are created, how they are specified in a model input file, and how they are arranged within the code in the C++ refactoring effort.

Motivations for moving to C++
-----------------------------

* Better organization of code
* Easier for developers to create specializations of *Shift* objects to extend the code. Eventually I will support dynamic loading of user defined specializations so that users can create specializations without touching core code base.
* Use C++ standard library (as opposed to custom stacks, linked lists and queues programmed in C). Simplifies development and debugging, especially for collaborators.
* Allows using other C++ libraries directly without having to develop interface to C code.

Things to keep from C code
--------------------------

In the *Shift* C code, the system topology is analyzed in a preprocessing step, and the instructions to process a model are chained together as a series of function pointers and source and destination memory pointers. Additionally, data that is accessed at run-time is localized in a single block of memory to minimize cache misses.

The *Shift* C++ refactor effort will follow a similar model. C++ classes will primarily be used during preprocessing for convenience, but will not be used extensively at run-time. Any quantities required to process the equations of motion will be stored in the `Shift::Context` data structure in a contiguous block of memory. Function pointers will be used to process the model as in the previous C code. The primary difference is that the function pointers and meta data required to set up the model will be provided by classes during the setup process.

Layout of objects in *Shift*
----------------------------

The layout of data within the *Shift* program follows the hierarchy below. The JSON input files follow a similar structure. See `external/models/bricard/bricard.json` for an example of a JSON file whose syntax has been updated to support the refactoring effort.

* Context

  * Model Parameters

    * Solver
    * Units
    * Integrator
    * Gravity
    * Include Directories
    * Logger

  * Assembly

    * Bodies

      * Fixed Body

        * Frames
        * Lights
        * Terrains

    * Assemblies

      * Assembly

        * Bodies

          * Body

            * Frames
            * Geometry
            * Collision Geometry

        * Lights
        * Behaviors
        * Constraint
        * Joints
        * Sensors
        * Force Modules
        * Assemblies

          * ...

Use of factory method to create *Shift* objects
-----------------------------------------------

Categories of objects created using factory method
**************************************************

The following objects are created using a templated factory method:

=====================  =================================== ====================
Description            Base class                         Description
---------------------  ----------------------------------- --------------------
Behavior               ``Shift::Behavior::Base``           Handling events in a simulation.
Body                   ``Shift::Body::Base``               A simulation object, typically with mass an inertia properties.
Collision geometry     ``Shift::CollisionGeometry::Base``  Simplified geometry used for contact and collision detection.
Constraint             ``Shift::Constraint::Base``         A motion constraint imposed between two bodies or within a joint.
Force module           ``Shift::ForceModule::Base``        Provides a mechanism to feed back forces into a running model.
Geometry               ``Shift::Geometry::Base``           Body geometry, typically used for display in a 3d graphical environment. Simplified collision geometry may also be generated from body's geometry definition.
Integrator             ``Shift::Integrator::Base``         Used to time-step a dynamic simulation.
Joint                  ``Shift::Joint::Base``              A special type of constraint connecting two bodies.
Light                  ``Shift::Light::Base``              Specify a light source in a 3d graphical environment.
Logger                 ``Shift::Logger::Base``             Allows logging simulation data to an external file.
Sensor                 ``Shift::Sensor::Base``             Captures information about a running simulation. The data can be made available to other *Shift* objects. Cameras, IMUs, range sensors are examples of possible sensors.
Solver                 ``Shift::Solver::Base``             Sets up and solves the equations of motion at each time step.
Terrain                ``Shift::Terrain::Base``            A special type of geometry use to describe a ground surface.
=====================  =================================== ====================



Specializations
***************
So far, the following specializations of factory objects are part of the *Shift-core* code. Presently, most of these specializations are stubs, but will be filled out as I fill out the . The "created by user" column indicates whether a user of the software can create these modules in a model input file. *No* in this column indicates that these objects are created and used internally by the *Shift* code only.

``Shift::Body::Base``
^^^^^^^^^^^^^^^^^^^^^

* default ``"type"`` if no keyword provided in JSON file: ``"rigid"``

======================== ================== ================ ============
Class                    ``"type"`` keyword Created by user? Description
------------------------ ------------------ ---------------- ------------
``Shift::Body::Fixed``   ``"fixed"``        No               Special body representing the fixed inertial reference frame, used internally by *Shift* code
``Shift::Body::Free``    ``"free"``         Yes              Floating body with no joints attached to it.
``Shift::Body::Rigid``   ``"rigid"``        Yes              Rigid body, with mass, inertia and geometric properties. Typical body used for modeling articulated multibody systems.
``Shift::Body::Virtual`` ``"virtual"``      No               Special body used to set up and solve equations of motion for topologies with closed loops. Created automatically and used internally by *Shift* code.
======================== ================== ================ ============


``Shift::Behavior::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^

* ``"type"`` is a required keyword in JSON file.

===================================== ======================= ================ ============
Class                                 ``"type"`` keyword      Created by user? Description
------------------------------------- ----------------------- ---------------- ------------
``Shift::Behavior::ContactDetection`` ``"contactdetection"``  Yes              Monitor contact between two or more bodies
``Shift::Behavior::IK7DOF``           ``"ik_7dof"``           Yes              Inverse kinematics for a 7 dof robot arm.
===================================== ======================= ================ ============


``Shift:CollisionGeometry::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* default ``"type"`` if no keyword provided in JSON file: ``"convex_hull"``

======================================== ======================= ================ ============
Class                                    ``"type"`` keyword      Created by user? Description
---------------------------------------- ----------------------- ---------------- ------------
``Shift::CollisionGeometry::ConvexHull`` ``"convex_hull"``       Yes              Automatically fit a convex hull to user specified geometry or provide a mesh.
``Shift::CollisionGeometry::Ellipsoid``  ``"ellipsoid"``         Yes              Fit a minimum bounding ellipsoid to Geometry, or manually specify an ellipsoid by specifying center, focal lengths and orientation.
``Shift::CollisionGeometry::Sphere``     ``"sphere"``            Yes              Fit a minimum bounding sphere to Geometry, or manually specify sphere center and radius.
======================================== ======================= ================ ============


``Shift:Constraint::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^^

* ``"type"`` is a required keyword in JSON file.

============================ ================== ================ ============
Class                        ``"type"`` keyword Created by user? Description
---------------------------- ------------------ ---------------- ------------
``Shift::Constraint::Chord`` ``"chord"``        No               A chord     constraint is imposed for each kinematic chord joint in the model. Chord joints are detected automatically by the software and need not be specified by an analyst in the model input file.
============================ ================== ================ ============


``Shift::ForceModule::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Some force modules may have additional specialization, such as the tire model.
For example, ``Shift::ForceModule::Tire::Fiala`` will inherit from ``Shift::ForceModule::Tire::Base`` which will inherit from ``Shift::ForceModule::Base``. This allows sharing some common functionality between different tire models that will be added later. It is also planned for contact (normal forces) and friction (oblique) forces to eventually be namespaced with their own base classes to simplify development of these specializations.

* `"type"` is a required keyword in JSON file.


=================================== ================== ================ ============
Class                               ``"type"`` keyword Created by user? Description
----------------------------------- ------------------ ---------------- ------------
``Shift::ForceModule::PID``         ``"pid"``          Yes              Specify a PID controller to match a desired set point for a joint position or velocity in a running simulation.
``Shift::ForceModule::Tire::Fiala`` ``"tire_fiala"``   Yes              Fiala tire contact model.
``Shift::ForceModule::Contact``     ``"contact"``      Yes              Specify contact model used between two or more bodies.
=================================== ================== ================ ============


``Shift::Geometry::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^^

* default ``"type"`` if no keyword provided in JSON file: ``"mesh"``


=================================== ================== ================ ============
Class                               ``"type"`` keyword Created by user? Description
----------------------------------- ------------------ ---------------- ------------
``Shift::Geometry::Ellipsoid``      ``"ellipsoid"``    Yes              Ellipsoid used for visualization purposes.
``Shift::Geometry::Mesh``           ``"mesh"``         Yes              Typical triangular soup mesh with vertices loaded from an external model definition (i.e. obj, stl, or gltf format)
``Shift::Geometry::PointCloud``     ``"point_cloud"``  Yes              Point cloud loaded from an external model definition (i.e. ply). In the future may also be generated from simulated sensors.
``Shift::Geometry::Sphere``         ``"sphere"``       Yes              Sphere used for visualization purposes.
=================================== ================== ================ ============


``Shift::Integrator::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

* default ``"type"`` if no keyword provided in JSON file: ``"ode113"``


=================================== ================== ================ ============
Class                               ``"type"`` keyword Created by user? Description
----------------------------------- ------------------ ---------------- ------------
``Shift::Integrator::ODE113``       ``"ode113"``       Yes              The ODE113 integrator is an explicit variable-step, variable-order (VSVO) Adams-Bashforth-Moulton PECE solver of orders 1 to 13. ODE113 is a multistep solver that uses the solutions at several preceding time intervals to compute the current solution. ODE113 is the default integrator used by *Shift* and is suitable for solving non-stiff ordinary differential equations (ODEs). A system with one or more kinematic chord loops is considered to be differential-algebraic (DAE), *Shift* uses an extremely efficient heuristic implementation of *Generalized Coordinate Partitioning* to automatically convert DAE problems with multiple kinematic chord loops to ODEs, which allow this integrator to be used even for complex topologies.

=================================== ================== ================ ============



``Shift::Joint::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^^

* ``"type"`` is a required keyword in JSON file.


=================================== ================== ================ ============
Class                               ``"type"`` keyword Created by user? Description
----------------------------------- ------------------ ---------------- ------------
``Shift::Joint::Float``             ``"float"``        Yes              6DOF Floating joint, used to connect an articulated multibody system to the fixed inertial reference frame. Note that if a ``Shift::Body::Rigid`` body will never be connected to an articulated multibody system, the user should instead use a ``Shift::Body::Free`` body, which does not require explicitly adding a joint. The rotational DOF are modeled using quaternions to avoid Gimball lock.
``Shift::Joint::Prismatic``         ``"prismatic"``    Yes              1DOF Translational joint.
``Shift::Joint::Revolute``          ``"revolute"``     Yes              1DOF rotational joint.
``Shift::Joint::Rigid``             ``"rigid"``        Yes              0DOF Rigid connection between two articulated bodies. Rigid connections between two bodies will be optimized out when solving the equations of motion.
``Shift::Joint::Spherical``         ``"spherical"``    Yes              3DOF Spherical or ball-joint, modeled using quaternions to avoid Gimball lock.
=================================== ================== ================ ============


``Shift::Light::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^^

* ``"type"`` is a required keyword in JSON file.


=================================== ================== ================ ============
Class                               ``"type"`` keyword Created by user? Description
----------------------------------- ------------------ ---------------- ------------
``Shift::Light::Directional``       ``"directional"``  Yes              Distant, directional light source, such as from the sun.
``Shift::Light::IES``               ``"ies"``          Yes              Specify a non-uniform light distribution using an Illuminating Engineering Society (IES) input file.
``Shift::Light::Point``             ``"point"``        Yes              Specify a point light source such as a light bulb.
``Shift::Light::Spot``              ``"spot"``         Yes              Specify a spot light source, such as a lamp.
=================================== ================== ================ ============


``Shift::Logger::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^^

* default ``"type"`` if no keyword provided in JSON file: ``"csv"``


=================================== ================== ================ ============
Class                               ``"type"`` keyword Created by user? Description
----------------------------------- ------------------ ---------------- ------------
``Shift::Logger::CSV``              ``"csv"``          Yes              Log to a comma delimited file.
``Shift::Logger::HDF5``             ``"hdf5"``         Yes              Log to HDF5 file format.
=================================== ================== ================ ============



``Shift::Sensor::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^^

* ``"type"`` is a required keyword in JSON file.

=================================== ================== ================ ============
Class                               ``"type"`` keyword Created by user? Description
----------------------------------- ------------------ ---------------- ------------
``Shift::Logger::IMU``              ``"imu"``          Yes              Model an inertial measurement unit.
``Shift::Logger::Range``            ``"range"``        Yes              Model a laser range sensor.
=================================== ================== ================ ============


``Shift::Solver::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^^

* default ``"type"`` if no keyword provided in JSON file: ``"rnca"``

=================================== ================== ================ ============
Class                               ``"type"`` keyword Created by user? Description
----------------------------------- ------------------ ---------------- ------------
``Shift::Solver::RNCA``             ``"rnca"``         Yes              The run-time optimized natural constraint algorithm (RNCA) is a sparse matrix solver that uses a heuristic application of Generalized Coordinate Partitioning (GCP).
=================================== ================== ================ ============


``Shift::Terrain::Base``
^^^^^^^^^^^^^^^^^^^^^^^^^^

* default ``"type"`` if no keyword provided in JSON file: ``"basic"``

=================================== ================== ================ ============
Class                               ``"type"`` keyword Created by user? Description
----------------------------------- ------------------ ---------------- ------------
``Shift::Terrain::Basic``           ``"basic"``        Yes              Loads a single digital elevation map (DEM) from a rasterized data file to represent the terrain.
=================================== ================== ================ ============

Example: creating a *Rigid* body
********************************

The following example is intended for *Shift* developers. *Shift* creates a ``Shift::Body::Rigid`` specialization of the ``Shift::Body::Base`` class using the ``"rigid"`` keyword noted above as:

.. code-block:: c

   #include "shift-core/factory.h"
   #include "shift-core/body/base.h"
   auto body = Shift::Factory<Shift::Body::Base>::get()->create("rigid");


*Shift* only interfaces with the rigid body specialization through the ``Shift::Body::Base`` base class. This allows other *body* specializations to be created without modifying the core *Shift* code.

The above code creates a ``unique_ptr`` to a ``Shift::Body::Base`` class. Any ``unique_ptr`` created when loading from the JSON file are managed by the ``Shift::Assembly`` or ``Shift::Body`` class the Object is a member of. Any ``unique_ptr`` created automatically when preprocessing the system topology (for example automatically identifying chord constraints), are managed by the ``Shift::Context`` class. Raw pointers to those objects may be used by other *Shift* objects. Shared pointers are not used due to additional overhead required to manage them.

Note that in general, users of the *Shift* software would never create a *Rigid* body using the above code and needn't concern themselves with these implementation details. Instead, they would specify the system topology in a JSON file. In the example above, the default body ``type`` is ``rigid``. If the user does not specify a ``type`` in the JSON file, it is assumed that the type is rigid and only the keys associated with a ``rigid`` body are processed.

A specialization can be created by deriving a new class from the ``Shift::Body::Base`` base class. All body objects use the ``Shift::Body::`` name space. The *rigid* specialization class is ``Shift::Body::Rigid``. All other *Shift* objects are similarly name spaced.

Each body specialization must have a unique keyword identifier. The keyword `rigid` cannot be used to define any another body specialization. The `rigid` key word is registered in the `rigid.cc` file as:

.. code-block:: c

   bool Shift::Body::Rigid::registered_ =
   Shift::Factory<Shift::Body::Base>::get()->registerModule(
     "rigid", Shift::Body::Rigid::create);


Introspection
-------------

To provide a form of introspection, all *Shift* objects contain information about what keys in the model definition file map to what member objects in the class. This functionality is provided in the ``Shift::Serialize`` class which is a base class for all *Shift* objects.  At construction, each class calls the ``addKeys()`` member function which creates a map of keywords to the address of member objects within the class, descriptions of the various objects, information about the type of object and validation. Therefore, all *Shift* objects contain the information necessary to validate and serialize input data. For each supported file format, parsing functions need only be written once. When adding new specializations, it is only necessary to modify the ``addKeys()`` function to describe what keys in the input file map to what member objects.

Example
*******

The following code demonstrates how keys are added to the ``Shift::Body::Rigid`` class.

.. code-block:: c

   void Shift::Body::Rigid::addKeys() {

     Shift::Body::Base::addKeys();

     SerialEntry com("center_of_mass", SerialType::KSL_VEC3,
                     &centerOfMassWRTOrigin_);
     com.setRequired(false);
     com.setDescription(
       "Specifies a vector from the body origin to the body center of mass "
       "expressed in the body-fixed coordinate system.");
     addSerialEntry(com);

     SerialEntry inertia("inertia", SerialType::KSL_INERTIA, &inertia_);
     inertia.setRequired(false);
     inertia.setDescription(
       "Specify body mass and centroidal moment of inertia "
       "expressed in the body-fixed coordinate system. Note that if a zero or "
       "very low mass is specified, it may cause problems for some solvers.");
     addSerialEntry(inertia);
   }


When parsing the JSON file, the data under the ``center_of_mass`` key word will be parsed and loaded into the ``centerOfMassWRTOrigin_`` member object. The ``Shift::Body::Base`` class provides additional keywords which are used by every derived body class.

At present, the following data types are supported:

* STRING
* STRING_ARRAY
* STRING_2DARRAY
* DOUBLE
* DOUBLE_ARRAY
* KSL_VEC3
* KSL_VEC4
* KSL_QUATERNION
* KSL_SCREW
* KSL_COSCREW
* KSL_INERTIA
* KSL_MAT3X3
* KSL_MAT4X4
* EULERANGLES
* BOOLEAN

Loading object specializations from JSON files
**********************************************

In the example for the rigid body above, ``rigid`` is the default keyword which is used if no ``type`` is provided in the input file. Most *Shift* objects do not have a default type, and require that the ``type`` key exist in the JSON file. For example, consider the following definition of a revolute joint in a model input file:


.. code-block:: python

   "joints": [
      {
       "name": "A",
       "parent": [
         ["2"],
         "origin"
       ],
       "child": [
         ["fixed"],
         "distal"
       ],
       "type": "revolute",
       "axis": "x",
       "initial_displacement": 0.776,
       "initial_velocity": 0.02623322119
      },
      ...


First, the key ``"type"`` is processed to obtain what type of joint specialization the user has specified. The key is then used to generate a specialization of the class (``Shift::Joint::Revolute``). Then the JSON file is parsed by calling the ``readJSON()`` member function. ``readJSON()`` simply steps through and processes each `SerialEntry` datastructure added in the ``addKeys()`` function as described above, which loads data under the remaining keys to objects in the class. Developers of new specializations need only populate the ``addKeys()`` member function, they do not need to write logic to parse model input files.

Creating an object specialization is illustrated in the code below. Here, ``value`` represents the first element in the array under ``"joints"``:

.. code-block:: c

   #include "shift-core/factory.h"
   #include "shift-core/joint/base.h"
   std::string jointType(value["type"].asString());
   auto joint = Shift::Factory<Shift::Joint::Base>::get()->create(jointType);
   joint->readJSON(value);
