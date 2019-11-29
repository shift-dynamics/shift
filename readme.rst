*Shift*
=======

.. raw:: html

   <embed>
    <a href='http://104.35.187.125:8090/job/shift-dynamics/job/shift/job/master/'><img src='http://104.35.187.125:8090/buildStatus/icon?job=shift-dynamics/shift/master' alt="Build Status"></a>
   </embed>

A dynamics engine and visualization tool for automation and robotics.


.. contents:: Contents


Overview
--------

*Shift* is a general-purpose modeling environment targeted to robotics, automation and vehicle dynamics applications. The software was created and is maintained by Kristopher Wehage, PhD.

Refer to *Shift's* website at `<http://shift-dynamics.io/>`_ for additional information on how to use the software and API.


Workflow
--------

*Shift* loads and analyzes the topology of a mechanism specified in a JSON
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

RTA uses only a few primitive functions to set up and sparse solve the equations of motion, stepping through arrays of memory locations without complex logic. During a simulation, the following operations are carried out at every time step:

*   Carry out operations set up in Run-Time Optimization step using
    source and destination memory pointers.

*   Monitor ratio of infinity norm of the B matrix associated with each
    block sub-Jacobian matrix of each substructure.  If the infinity norm increases more than a specified threshold from the initial starting value, the simulation is paused and the Run-Time Optimization step is carried out again to regenerate pointer arrays.


References
----------

Refer to the following publications for additional details on how the software works:

* Wehage K., Computational Methods for Dynamics and Mobility Analysis of Multiloop Mechanisms and Robotic Systems, Thesis, December 2016 `<https://search.proquest.com/openview/2b96ee91ebc96f840bdada69a21b7cbe>`_

* Wehage K. Ravani B., A Computational Method for Formulation and Solution of Dynamical Equations for Complex Mechanisms and Multibody Systems, Proceedings of IDETC/DIE at Cleveland Ohio, August 2017, `<https://doi.org/10.1115/DETC2017-67766>`_

* Wehage K., Wehage R., Ravani B., Generalized coordinate partitioning for complex mechanisms based on kinematic substructuring, Mechanism and Machine Theory 92:464-483, October 2015 `<https://doi.org/10.1016/j.mechmachtheory.2015.06.006>`_

*Shift* 2.0
===========

*Shift* has been rewritten from scratch in version 2.0. C code has been ported to C++. The layout of dependencies has been simplified. *Shift* data structures have been modularized to simplify code development. Modern C++ programming techniques, such as a factory design pattern, are used to make the code flexible and easily extensible. Additionally, a new mechanism for introspection has been added to enable rapidly populating class member data from a user's input file. The remainder of this document describes the motivation to move to C++, a new mechanism for introspection via named parameters, and more details of the new *Shift* data structures.

Motivations for moving to C++
-----------------------------

The C++ programming languages provides the following advantages over C:

* Better organization of code using Object-Oriented Programming (OOP)
* Easier for developers to create specializations of *Shift* objects to extend the code.
* Support for dynamic loading of user-defined class specializations. This feature will enable power-users to create and load specializations without modify or recompile the core code base.
* Use the C++ standard library data structures instead of custom stacks, linked lists and queues. This change simplifies development and debugging for collaborators and new developers.
* Allows using other C++ libraries directly without having to develop interface to C code.

A primary disadvantage to using C++ OOP is that there is a small cost associated with accessing data within classes, particularly when the classes are stored in fragmented memory. For this reason, to increase cache efficiency, data used in performance critical code should be stored in a contiguous block of memory (i.e. a struct of arrays or an array of structs). This design pattern is sometimes referred to as Data-Oriented Design (DOD).

Previously, in the *Shift* C code, the system topology was analyzed in a preprocessing step, and the instructions to process the model were chained together as a series of function pointers and source and destination memory pointers. Additionally, data that were accessed at run-time were localized in a single block of memory to minimize cache misses.

The C++ version of *Shift* follows a similar model. C++ classes are used primarily during the preprocessing phase for convenience, but are not used extensively at run-time. Any quantities required to update the kinematics and process the equations of motion are copied out of the classes into a contiguous block of memory. Function pointers are chained together and stepped through to process the model as in the previous C code. The primary difference is that the function pointers and meta data required to set up the model are provided by classes to simplify the setup process.

Simplifying dependencies
------------------------

In Shift 2.0, the layout of dependencies included as submodules has been updated. The layout of submodules in the previous version was:

* shift (C++ GUI) - *private*

  * shift-core (C command line application and C API) - *private*

    * external/ksl (math library) - *public*
    * external/libgeotiff (support for loading terrain files from geotiff format) - *public*

  * shift-models (collection of *Shift* models) - *public*

In the previous version, the GUI and API were separated in two different modules. To facilitate ease of development, the  ``shift`` and ``shift-gui`` modules have now been merged, which simplifies the layout as:

* shift (C++ GUI, C++ Command line interface and C++ API) - *private*

  * external/shift-models (collection of models to display core functionality of *Shift* software, later these can be made into unit tests) - *public*
  * external/ksl (math library) - *public*
  * external/libgeotiff (support for loading terrain files from geotiff format) - *public*

Other possible C++ submodules to add later:

* stepcode - for loading step CAD files
* vhacd - variable approximate convex hull decomposition
* opensubdiv - load subdivision surfaces

Dependencies are provided as submodules if they are not readily available in package managers or if there is a requirement to lock the API to a specific release.

Layout of objects in *Shift*
----------------------------

In *Shift* 2.0, the data structure layout hierarchy and json file format have been modified to have a one-to-one correspondence. See ``external/models/bricard/excavator.json`` for an example of an updated JSON file. The primary change to the JSON file format is the removal of the ``"fixed"`` key word from the root level. The ``"fixed"`` key was used for objects that were referenced to the *fixed* inertial reference frame. In the new file format, the root node of the JSON file *is* the fixed inertial reference frame.

The previous C data structure hierarchy had an additional assembly and body for the fixed inertial reference frame, but the JSON file did not have the fixed assembly. To obtain a closer match between the JSON input file and data layout in code, the *fixed* assembly has now been removed and the ``Shift::Context`` class is derived from the ``Shift::Body::Base`` class. Therefore, the ``Context`` class itself now serves as an interface to the fixed body. This change simplifies the JSON file format, parsing of the JSON file, and obtaining references to other objects during preprocessing.

The layout of classes follows the hierarchy below. For more information on objects created using a factory method, see the next section.

* ``Shift::Context`` (Also serves as interface to Fixed Body which contains one or more fixed inertial reference frame)

  * ``Shift::ModelParameters``

    * Simulation parameters such as
      * Include directories
      * Gravity vector
      * Units
    * ``Shift::Solver::Base`` (one or more methods to set up and solve the equations of motions)
    * ``Shift::Integrator::Base`` (one or more numerical integrators)
    * ``Shift::Logger::Base`` (one or more utilities to support logging)

  * ``Shift::Array<Shift::Frame> frames_`` (static, immobile reference frames attached to fixed body)
  * ``Shift::FactoryArray<Shift::Light::Base> lights_`` (static, immobile lights attached to fixed inertial reference frame)
  * ``Shift::FactoryArray<Shift::Terrain::Base> terrain_`` (static immobile terrain data attached to fixed inertial reference frame)

  * ``Shift::Array<Shift::Assembly> assemblies_``:

    * where a ``Shift::Assembly`` class contains:

      * An array of **bodies** created using a factory method: ``Shift::FactoryArray<Shift::Body::Base> bodies_``

        * where each ``Shift::Body::Base`` contains

          * An array of **frames** ``Shift::Array<Shift::Frame> frames_``
          * An array of **geometry** created using a factory method: ``Shift::FactoryArray<Shift::Geometry::Base> geometry_``
          * An array of **collision geometry** created using a factory method: ``Shift::FactoryArray<Shift::CollisionGeometry::Base> collision_geometry_``

      * An array of **lights** created using a factory method: ``Shift::FactoryArray<Shift::Lights::Base> lights_``
      * An array of **behaviors** created using a factory method: ``Shift::FactoryArray<Shift::Behaviors::Base> behaviors_``
      * An array of **constraints** created using a factory method: ``Shift::FactoryArray<Shift::Constraint::Base> constraints_``
      * An array of **constraints** created using a factory method: ``Shift::FactoryArray<Shift::Joints::Base>  joints_``
      * An array of **joints** created using a factory method: ``Shift::FactoryArray<Shift::Sensors::Base> sensors_``
      * An array of **sensors** created using a factory method: ``Shift::FactoryArray<Shift::ForceModules::Base> force_modules_``
      * An array of **subassemblies** in ``Shift::Array<Shift::Assembly> assemblies_``. Each assembly in turn may contain more subassemblies, nested arbitrarily deep.


Factory design pattern
----------------------

*Shift* objects are now created using a factory design pattern.

Factory base classes
********************

*Shift* provides a number of abstract base class types. Each base class specifies an interface that must be populated in a derived class, or *specialization*. The base classes are described in more detail below:

=====================  =================================== ====================
Description            Base class                          Description
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


Factory specializations
***********************
The following specializations of factory objects are thus far part of the *Shift-core* code. At this time many of the specializations are stubs, but will be filled out as I continue moving code from C over to C++. The "created by user" column indicates whether a user of the software can create these modules in a model input file. *No* in this column indicates that these objects are created and used internally by the *Shift* code only.

``Shift::Body::Base``
^^^^^^^^^^^^^^^^^^^^^

* default ``"type"`` if no keyword provided in JSON file: ``"rigid"``

======================== ================== ================ ============
Class                    ``"type"`` keyword Created by user? Description
------------------------ ------------------ ---------------- ------------
``Shift::Body::Fixed``   ``"fixed"``        No               Special body representing the fixed inertial reference frame, used internally by *Shift* code. The ``Shift::Context`` class is derived from this base class.
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

The above code creates a ``unique_ptr`` to a ``Shift::Body::Base`` class. Any object created when loading from the JSON file are managed by the ``Shift::Assembly`` or ``Shift::Body`` class the object is a member of. Any ``unique_ptr`` created automatically when preprocessing the system topology (for example automatically identifying chord constraints), are managed by the ``Shift::Context`` class. Raw pointers to those objects may be used by other *Shift* objects. Shared pointers are not used due to additional overhead required to manage them.

Note that in general, users of the *Shift* software would never create a *Rigid* body using the above code and needn't concern themselves with these implementation details. Instead, they would specify the system topology in a JSON file and the object would be created automatically during parsing. In the example above, the default body ``type`` is ``rigid``. If the user does not specify a ``type`` in the JSON file, it is assumed that the type is rigid and only the keys associated with a ``rigid`` body are processed.

A specialization can be created by deriving a new class from the ``Shift::Body::Base`` base class. All body objects use the ``Shift::Body::`` name space. The *rigid* specialization class is ``Shift::Body::Rigid``. All other *Shift* objects are similarly name spaced.

Each body specialization must have a unique keyword identifier. The keyword ``"rigid"`` cannot be used to define any other body specialization. The ``"rigid"`` key word is registered in the ``body/rigid.cc`` file as:

.. code-block:: c

   bool Shift::Body::Rigid::registered_ =
   Shift::Factory<Shift::Body::Base>::get()->registerModule(
     "rigid", Shift::Body::Rigid::create);


Introspection via named parameters
----------------------------------

*Shift* now provides a mechanism for introspection. *Introspection* gives *Shift* classes an awareness of the type of data contained within them, and has the following advantages:

* **Simplifies adding new parameters**: Adding new parameters to load from a JSON input file is vastly simplified. New parameters can now added using a simple helper function. The appropriate JSON parsing commands are then called automatically based on the type of data passed to the helper function. This feature eliminates the need for developers of *Shift* objects to write JSON parsing code. All JSON parsing code has now been moved to the ``Shift::Parameter::Value`` class.
* **Simplifies access to parameters**: *Shift* objects contain a ``std::map`` of their keys and values which can be used to retrieve a type safe constant reference or constant pointer to the underlying data. With this new method to access method, it is no longer necessary to write get and set functions to make data accessible to other classes. The method for accessing data should be familiar to developers who have used a Python dictionary.
* **Simplifies debugging**: The JSON parsing code is moved to one file in one location, which simplifies debugging the model parsing pipeline. Additionally, ``Shift::Parameter::*`` classes provide a default ``print()`` method, which can be used to print named parameters to verify correct loading of the model.
* **Flexibility**: The ``Shift::Parameter::Value`` class allows for flexibility in loading objects from JSON files. For example, multiple keys can map to the same object (for example, the keys ``"R"`` and ``"rotation"`` could both map data to a 3x3 rotation matrix internally -- to allow an alternative short-hand version of the key in the input file) and data can be transformed from one type to another during loading (for example, a quaternion can be transformed to a 3x3 rotation matrix)

Classes to support introspection
********************************

Named parameters are supported with the addition of new base classes:

* ``Shift::Parameter::Value``: provides a mechanism for type safe conversion and JSON parsing functionality.
* ``Shift::Parameter::List``: provides a map of keys to a number of ``Shift::Parameter::Value`` objects.
* ``Shift::Parameter::Array`` an array of non-factory *Shift* objects, such as ``Shift::Assembly`` and ``Shift::Frame`` classes.
* ``Shift::Parameter::FactoryArray``: an array of factory *Shift* objects, such as ``Shift::Body::Base`` or ``Shift::Joint::Base``.

Adding new named parameters
***************************

To map keys in a JSON input file to members of a class, populate the ``addParameters()`` function with the key, member variable, description and whether the data is required. During class construction, the ``addParameters()`` function is called to create the mapping.

Example
^^^^^^^

The following code demonstrates how keys are added to the ``Shift::Body::Rigid`` class.

.. code-block:: c

  void Shift::Frame::addParameters() {
    addParameter("name", &name_, "descriptive frame name", true);

    addParameter(
      "active", &active_,
      "indicates whether frame is active, defaults to true if not specified",
      false);

    addParameter(
      "R", &d_wrt_origin_.R,
      "specify orientation as a 3x3 rotation matrix in row-major "
      "order, defaults to identity if not specified; the keys 'rotation', 'R', "
      "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
      "orientation. If multiple key-value pairs are specified for "
      "orientation, only one of the entries will be loaded.",
      false);

    ...
   }


When parsing the JSON input file is parsed, the data under the ``name`` key word in the code block above will be parsed and loaded into the ``name_`` member object. The type passed to overloaded ``addParameter()`` function is used to determine the appropriate way to parse the JSON file. In this example,  ``name_`` is a pointer to a string, so a string is expected in the JSON input file. If the data is incorrectly formatted, an exception will be thrown.

Supported data types
********************

Presently, *Shift* provides support to load strings, doubles, vectors of strings or doubles, and all data structures used in the kinematic subroutine library (KSL) - the math library used internally by *Shift*.

The following data types are supported:

* ``std::string``
* ``std::vector<std::string>``
* ``std::vector<std::vector<std::string>>``
* ``double``
* ``std::vector<double>``
* ``ksl_vec3_t``
* ``ksl_vec4_t``
* ``ksl_quaternion_t``
* ``ksl_screw_t``
* ``ksl_coscrew_t``
* ``ksl_inertia_t``
* ``ksl_mat3x3_t``
* ``ksl_mat4x4_t``
* ``ksl_eulerangles_t``
* ``bool``


Loading factory objects from JSON files
***************************************

When loading objects from a JSON file, *Shift* uses the reserved key word "type" to determine what specialization of the base class to use when creating the corresponding object in C++. The type is then used to create an instance of the object using a factory method.

Some *Shift* objects have a default type. For example, the default type for bodies is ``rigid``. Therefore, if no type is specified, a rigid body will be created. Other objects, such as a joints, do not have a default type, and require the ``type`` key to exist in the JSON file. Consider the following definition of a revolute joint in a model input file:


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
       "axis": [1, 0, 0],
       "initial_displacement": 0.776,
       "initial_velocity": 0.02623322119
      },
      ...


The key ``"type"`` is processed to determine the joint specialization the user has specified. The key is then used to generate a specialization of the class (``Shift::Joint::Revolute``). Then the JSON file is parsed by calling the ``readJSON()`` member function for ``Shift::Joint::Revolute``, stepping through each ``Shift::Parameter::Value`` datastructure within its ``Shift::Parameter::List`` base class. The parameters were added in the ``addParameters()`` function during class construction. During processing of each ``Shift::Parameter::Value``, the data is mapped from the JSON file to the class member variables using classes and class member functions defined in the following files. In this way, there is no need for developers of class specializations to write logic for parsing model input files.

The following code demonstrates how an object specialization is created in *Shift* internal code using the factory method. The example is provided to describe how the classes are generated internally, but it would never be necessary for a developer of a *Shift* extension (such as a class specialization of a ``Shift::Body::Base`` or ``Shift::Joint::Base``) to write this code themselves.

.. code-block:: c

   #include "shift-core/factory.h"
   #include "shift-core/joint/base.h"

   std::string joint_type(value["type"].asString());
   auto joint = Shift::Factory<Shift::Joint::Base>::get()->create(joint_type);
   joint->readJSON(value);


Accessing named parameters
**************************

Lists
^^^^^

The ``[]`` operator for the ``Shift::Parameter::List`` class is overloaded to allow lookup by key. This method of accessing a hierarchical data structure should be familiar to developers who have used a Python dictionary.

.. code-block:: c

   context["model_parameters"]["units"]["length"].print();


Although the syntax for traversing the hierarchy is identical to a Python dictionary, because C++ is a statically typed language, the ``Shift::Value`` object must be cast to its appropriate type before using its underlying data in code. *Shift* provides a type-safe mechanism to request the underlying value. To access the variable ``"length"`` as a string, use the templated ``get()`` function. The ``get()`` functions performs type-safe conversion of its underlying ``std::any`` data structure to the requested format as:

.. code-block:: c

   const std::string& length = context["model_parameters"]["units"]["length"].get<std::string>();


If the requested format does not match the underlying format, an exception will be thrown.

Similarly, it is possible to get a raw pointer to the underlying data as:

.. code-block:: c

   const std::string* length_ptr = context["model_parameters"]["units"]["length"].data<std::string>();


For debugging purposes, it is possible to query the type of object as:

.. code-block:: c

    std::cout << context["model_parameters"]["units"]["length"].typeName();

    >>> std::string


The ``typeName()`` function returns the *demangled* human-readable name of the underlying type.

Similarly, it is also possible to obtain the value's type id or type index. The below code would typically not be used at run-time, but may be helpful for debugging purposes when developing new *Shift* objects.

.. code-block:: c

   std::string value;
   std::cout << (std::type_index(typeid(&value)) ==
                  context["model_parameters"]["units"]["length"].typeIndex()
                 ? "true"
                 : "false")
     << std::endl;

   >>> true

Arrays
^^^^^^

The ``Shift::Parameter::Array`` and ``Shift::Parameter::Factory`` array classes support lookup by name or by index using the `[]` operator. For example, consider the *Shift* hierarchy below:

.. code:: python

   {
     "assemblies": [
       {
         "name": "my_model",
         "bodies": [
           {
             "name": "first_body"
             ...
           },
           {
             "name": "second_body"
             ...
           }
         ]
         ...
       }
     ]
   }


In the above example ``...`` indicates that there would be other key-value pairs which are omitted here. After the above file is loaded into a ``Shift::Context`` data structure, the second body in the array can be accessed by its index as:

.. code:: c

   Shift::Body::Base* body =
     context["assemblies"][0]["bodies"][1].data<Shift::Body::Base>();


The above method is similar to traversing a Python dictionary. It is also possible to look up a value by its descriptive ``"name"`` key as:

.. code:: c

   Shift::Body::Base* body =
     context["assemblies"]["my_model"]["bodies"]["second_body"].data<Shift::Body::Base>();
