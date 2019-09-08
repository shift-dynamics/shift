*Shift* C++ Refactoring Overview
================================

Abstract
--------

This working document describes the various *Shift* data structures, how they are created, how they are specified in a model input file, and how they are arranged within the code in the C++ refactoring effort.

Motivations for moving to C++
-----------------------------

* Better organization of code
* Easier for developers to create specializations of *Shift* objects to extend the code.
* Support dynamic loading of user defined specializations so that users can create specializations without touching core code base.
* Use C++ standard library (as opposed to custom stacks, linked lists and queues programmed in C). Simplifies development and debugging for collaborators and new developers.
* Allows using other C++ libraries directly without having to develop interface to C code.

Things to keep from C code
--------------------------

In the *Shift* C code, the system topology is analyzed in a preprocessing step, and the instructions to process a model are chained together as a series of function pointers and source and destination memory pointers. Additionally, data that is accessed at run-time is localized in a single block of memory to minimize cache misses.

The *Shift* C++ refactor effort will follow a similar model. C++ classes will primarily be used during preprocessing for convenience, but will not be used extensively at run-time. Any quantities required to process the equations of motion will be stored in the `Shift::Context` data structure in a contiguous block of memory. Function pointers will be used to process the model as in the previous C code. The primary difference is that the function pointers and meta data required to set up the model will be provided by classes during the setup process.

Layout of objects in *Shift*
----------------------------

The layout of data within the *Shift* program follows the hierarchy below. The JSON input files follow a similar structure. See ``external/models/bricard/bricard.json`` for an example of a JSON file whose syntax has been updated to support the refactoring effort.

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

Changes to submodule layout
---------------------------

The layout of submodules in the previous version was:

* shift (C++ GUI) - private

  * shift-core (C command line application and C API) - private

    * external/ksl (math library) - public
    * external/libgeotiff (support for loading terrain files from geotiff format) - public

  * shift-models (collection of *Shift* models) - public

Due to licensing, the GUI was separated from the core code. However, after reconsideration, the ``shift`` and ``shift-gui`` modules will be merged together in the refactor effort to simplify the layout as:

* shift (C++ GUI, C++ Command line interface and C++ API) - private

  * external/shift-models (collection of models to display core functionality of *Shift* software, later these can be made into unit tests) - public
  * external/ksl (math library) - public
  * external/libgeotiff (support for loading terrain files from geotiff format) - public

Other possible C++ submodules to added later:

* stepcode - for loading step CAD files
* vhacd - variable approximate convex hull decomposition
* opensubdiv - load subdivision surfaces

Some dependencies are provided as submodules if they are not readily available in package managers or if there is a requirement to lock the API to a specific release.

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
So far, the following specializations of factory objects are part of the *Shift-core* code. Presently, most of these specializations are stubs, but will be filled out as I continue moving code from C over to C++. The "created by user" column indicates whether a user of the software can create these modules in a model input file. *No* in this column indicates that these objects are created and used internally by the *Shift* code only.

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

*Shift* provides a means for introspection via named paramters. Shift objects now inherit from the ``Shift::Parameter::List`` class, and arrays of Shift objects now inherit from the ``Shift::Parameter::Array``, ``Shift::Parameter::FactoryArray`` classes. To map keys in a JSON input file to members of a class, populate the ``addParameters()`` function with the key, member variable, description and whether the data is required. During class construction, the ``addParameters()`` function is called to create the mapping. 

Example
*******

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
      "orientation. If multiple key-value pairs are specified for orientation, "
      "only one of the entries will be loaded.",
      false);

    ...
   }


When parsing a JSON input file, the data under the ``name`` key word will be parsed and loaded into the ``name_`` member object. The type passed to overloaded ``addParameter()`` function is used to determine the appropriate way to parse the JSON file. If the data is incorrectly formatted, an error will be generated. 

At present, the following data types are supported:

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

Loading object specializations from JSON files
**********************************************

When loading objects from a JSON file, *Shift* uses the reserved key word "type" to determine what specialization of the base class to use when creating the corresponding object in C++. Some *Shift* objects have a default type. For example, the default type for bodies is ``rigid``. Therefore, if no type is specified, a rigid body will be created. Other objects, such as a joints, do not have a default type, and require the ``type`` key to exist in the JSON file. For example, consider the following definition of a revolute joint in a model input file:


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


The key ``"type"`` is processed to determine the joint specialization the user has specified. The key is then used to generate a specialization of the class (``Shift::Joint::Revolute``). Then the JSON file is parsed by calling the ``readJSON()`` member function. ``readJSON()`` steps through and processes each ``Shift::Parameter::KeyValue`` datastructure within the ``Shift::Parameter::List`` base class. The parameters were added in the ``addParameters()`` function during clas construction as described above. During processing of each ``Shift::Parameter::KeyValue``, the data is mapped from the JSON file to the class member variables using functions in ``parameter.cc``. The primary advantage to this technique is that developers of new specializations need only populate the ``addParameters()`` member function; there is no need to write logic for parsing model input files.

The following code demonstrates how an object specialization is created in *Shift* internal code. This example is provided to describe how the classes are generated internall, but it would never be necessary for a developer of a *Shift* extension (such as a class specialization of a ``Shift::Body::Base`` or ``Shift::Joint::Base``) to write this code. 

.. code-block:: c

   #include "shift-core/factory.h"
   #include "shift-core/joint/base.h"

   std::string joint_type(value["type"].asString());
   auto joint = Shift::Factory<Shift::Joint::Base>::get()->create(joint_type);
   joint->readJSON(value);

