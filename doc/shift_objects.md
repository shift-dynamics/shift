# Creating shift objects

## Overview
In *Shift*, the data structure layout hierarchy and json file format have a one-to-one correspondence. Refer to `examples/excavator/excavator.json` for an example of an updated JSON file.

The root level of the JSON file represents the "fixed" inertial reference frame, represented by the `shift::core::Context` datastructure. Within the `shift::core::Context` datastructure, the layout of classes follows the hierarchy below. For more information on objects created using a factory method, see the next section.

* `shift::core::Context` (Also serves as interface to Fixed Body which contains one or more fixed inertial reference frames)

  * `Shift::ModelParameters`

    * Simulation parameters such as
      * Include directories
      * Gravity vector
      * Units
    * `shift::core::solver::Base` (one or more methods to set up and solve the equations of motions)
    * `shift::core::integrator::Base` (one or more numerical integrators)
    * `shift::core::logger::Base` (one or more utilities to support logging)

  * `Shift::Array<Shift::Frame> frames_` (static, immobile reference frames attached to fixed body)
  * `shift::core::FactoryArray<shift::core::light::Base> lights_` (static, immobile lights attached to fixed inertial reference frame)
  * `shift::core::FactoryArray<shift::core::terrain::Base> terrain_` (static immobile terrain data attached to fixed inertial reference frame)

  * `Shift::Array<Shift::Assembly> assemblies_`:

    * where a `Shift::Assembly` class contains:

      * An array of **bodies** created using a factory method: `shift::core::FactoryArray<shift::core::body::Base> bodies_`

        * where each `shift::core::body::Base` contains

          * An array of **frames** `Shift::Array<Shift::Frame> frames_`
          * An array of **geometry** created using a factory method: `shift::core::FactoryArray<shift::core::geometry::Base> geometry_`
          * An array of **collision geometry** created using a factory method: `shift::core::FactoryArray<shift::core::collisiongeometry::Base> collision_geometry_`

      * An array of **lights** created using a factory method: `shift::core::FactoryArray<shift::core::lights::Base> lights_`
      * An array of **behaviors** created using a factory method: `shift::core::FactoryArray<shift::core::behaviors::Base> behaviors_`
      * An array of **constraints** created using a factory method: `shift::core::FactoryArray<shift::core::constraint::Base> constraints_`
      * An array of **joints** created using a factory method: `shift::core::FactoryArray<shift::core::joints::Base>  joints_`
      * An array of **sensors** created using a factory method: `shift::core::FactoryArray<shift::core::sensors::Base> sensors_`
      * An array of **force modules** created using a factory method: `shift::core::FactoryArray<shift::core::forcemodules::Base> force_modules_`
      * An array of **subassemblies** in `Shift::Array<Shift::Assembly> assemblies_`. Each assembly in turn may contain more subassemblies, nested arbitrarily deep.


## Factory base classes

*Shift* provides a number of abstract base class types. Each base class specifies an interface that must be populated in a derived class, or *specialization*. The base classes are described in more detail below:

| Description | Base class | Description |
--- | --- | ---
| Behavior | `shift::core::behavior::Base` | Handling events in a simulation. |
| Body     | `shift::core::body::Base` | A simulation object, typically with mass and inertia properties. |
| Collision geometry | `shift::core::collisiongeometry::Base` | Simplified geometry used for collision detection and contact modelling. |
| Constraint | `shift::core::constraint::Base` | A motion constraint imposed between two bodies or within a joint. |
| Force module | `shift::core::forcemodule::Base` | A mechanism to feed back forces into a running model. |
| Geometry | `shift::core::geometry::Base` | Body geometry, used for display in a 3d graphical environment. Simplified collision geometry may also be generated from body's geometry definition. |
| Integrator |           `shift::core::integrator::Base` | Used to time-step a dynamic simulation. |
| Joint      |           `shift::core::joint::Base`      | A special type of constraint connecting two bodies. |
| Light      |           `shift::core::light::Base`      | A light source in a 3d graphical environment. |
| Logger     |           `shift::core::logger::Base`     | Allows logging simulation data to an external file. |
| Sensor     |           `shift::core::sensor::Base`     | Captures information about a running simulation. The data can be made available to other *Shift* objects. Cameras, IMUs, range sensors are examples of possible sensors. |
| Solver     |           `shift::core::solver::Base`     | Sets up and solves the equations of motion at each time step. |
| Terrain    |           `shift::core::terrain::Base`    | A special type of geometry use to describe a ground surface. |

## Factory specializations
The following specializations of factory objects are thus far part of the *Shift-core* code. At this time many of the specializations are stubs, but will be filled out as I continue moving code from C over to C++. The "created by user" column indicates whether a user of the software can create these modules in a model input file. *No* in this column indicates that these objects are created and used internally by the *Shift* code only.

### shift::core::body::Base

* default `"type"` if no keyword provided in JSON file: `"rigid"`

| Class | `"type"` keyword | Created by user? | Description |
--- | --- | --- | ---
| `shift::core::body::Fixed` |  `"fixed"` |       No   |            Special body representing the fixed inertial reference frame, used internally by *Shift* code. The `shift::core::Context` class is derived from this base class. |
| `shift::core::body::Free` |   `"free"` |        Yes  |            Floating body with no joints attached to it. |
| `shift::core::body::Rigid` |  `"rigid"` |       Yes |            Rigid body, with mass, inertia and geometric properties. Typical body used for modeling articulated multibody systems. |
| `shift::core::body::Virtual` | `"virtual"`  |    No   |            Special body used to set up and solve equations of motion for topologies with closed loops. Created automatically and used internally by *Shift* code. |


### shift::core::behavior::Base

* `"type"` is a required keyword in JSON file.

| Class | `"type"` keyword |     Created by user? | Description |
--- | --- | --- | ---
| `shift::core::behavior::ContactDetection` | `"contactdetection"` | Yes |  Monitor contact between two or more bodies |
| `shift::core::behavior::IK7DOF`  |         `"ik_7dof"` |          Yes  |            Inverse kinematics for a 7 dof robot arm. |


### shift:core::collisiongeometry::Base

* default `"type"` if no keyword provided in JSON file: `"convex_hull"`

| Class | `"type"` keyword | Created by user? | Description |
--- | --- | --- | ---
| `shift::core::collisiongeometry::ConvexHull` | `"convex_hull"` |  Yes |             Automatically fit a convex hull to user specified geometry or provide a mesh. |
| `shift::core::collisiongeometry::Ellipsoid` | `"ellipsoid"` |  Yes  |  Fit a minimum bounding ellipsoid to Geometry, or manually specify an ellipsoid by specifying center, focal lengths and orientation. |
| `shift::core::collisiongeometry::Sphere` |  `"sphere"` |  Yes |  Fit a minimum bounding sphere to Geometry, or manually specify sphere center and radius. |


### shift:core::constraint::Base

* `"type"` is a required keyword in JSON file.

| Class  | `"type"` keyword | Created by user? | Description |
--- | --- | --- | ---
| `shift::core::constraint::Chord` | `"chord"` |  No |  A chord constraint is imposed for each kinematic chord joint in the model. Chord joints are detected automatically by the software and need not be specified by an analyst in the model input file. |


### shift::core::forcemodule::Base

Some force modules may have additional specialization, such as the tire model.
For example, `shift::core::forcemodule::tire::Fiala` will inherit from `shift::core::forcemodule::tire::Base` which will inherit from `shift::core::forcemodule::Base`. This allows sharing some common functionality between different tire models that will be added later. It is also planned for contact (normal forces) and friction (oblique) forces to eventually be namespaced with their own base classes to simplify development of these specializations.

* `"type"` is a required keyword in JSON file.


| Class | `"type"` keyword | Created by user? | Description |
--- | --- | --- | ---
| `shift::core::forcemodule::PID` |  `"pid"` |  Yes |  Specify a PID controller to match a desired set point for a joint position or velocity in a running simulation. |
| `shift::core::forcemodule::tire::Fiala` | `"tire_fiala"` |  Yes |             Fiala tire contact model. |
| `shift::core::forcemodule::Contact` | `"contact"` |  Yes  | Specify contact model used between two or more bodies. |


### shift::core::geometry::Base

* default `"type"` if no keyword provided in JSON file: `"mesh"`


| Class | `"type"` keyword | Created by user? | Description |
--- | --- | --- | ---
| `shift::core::geometry::Ellipsoid` | `"ellipsoid"` | Yes | Ellipsoid used for visualization purposes. |
| `shift::core::geometry::Mesh` | `"mesh"` |  Yes | Typical triangular soup mesh with vertices loaded from an external model definition (i.e. obj, stl, or gltf format) |
| `shift::core::geometry::PointCloud` | `"point_cloud"` | Yes | Point cloud loaded from an external model definition (i.e. ply). In the future may also be generated from simulated sensors. |
| `shift::core::geometry::Sphere` | `"sphere"` | Yes | Sphere used for visualization purposes. |


### shift::core::integrator::Base

* default `"type"` if no keyword provided in JSON file: `"ode113"`


| Class | `"type"` keyword | Created by user? | Description |
--- | --- | --- | ---
| `shift::core::integrator::ODE113` | `"ode113"` | Yes | The ODE113 integrator is an explicit variable-step, variable-order (VSVO) Adams-Bashforth-Moulton PECE solver of orders 1 to 13. ODE113 is a multistep solver that uses the solutions at several preceding time intervals to compute the current solution. ODE113 is the default integrator used by *Shift* and is suitable for solving non-stiff ordinary differential equations (ODEs). A system with one or more kinematic chord loops is considered to be differential-algebraic (DAE), *Shift* uses an extremely efficient heuristic implementation of *Generalized Coordinate Partitioning* to automatically convert DAE problems with multiple kinematic chord loops to ODEs, which allow this integrator to be used even for complex topologies. |



### shift::core::joint::Base

* `"type"` is a required keyword in JSON file.


| Class | `"type"` keyword | Created by user? | Description |
--- | --- | --- | ---
| `shift::core::joint::Float` | `"float"` | Yes | 6DOF Floating joint, used to connect an articulated multibody system to the fixed inertial reference frame. Note that if a `shift::core::body::Rigid` body will never be kinematically connected via joints to other bodies, the user should instead use a `shift::core::body::Free` body, which does not require explicitly adding a joint or other kinematic constraint. A `shift::core::body::Free` body may still interact with other bodies via force modules. In a `shift::core::joint::Float` joint, the rotational DOF are modeled using quaternions to avoid Gimball lock. |
| `shift::core::joint::Prismatic` | `"prismatic"` | Yes | 1DOF Translational joint. |
| `shift::core::joint::Revolute` | `"revolute"` | Yes | 1DOF rotational joint. |
| `shift::core::joint::Rigid` | `"rigid"` | Yes | 0DOF Rigid connection between two articulated bodies. Rigid connections between two bodies will be optimized out when solving the equations of motion. |
| `shift::core::joint::Spherical`  | `"spherical"` | Yes | 3DOF Spherical or ball-joint, modeled using quaternions to avoid Gimball lock. |


### shift::core::light::Base

* `"type"` is a required keyword in JSON file.


| Class | `"type"` keyword | Created by user? | Description |
--- | --- | --- | ---
| `shift::core::light::Directional` | `"directional"` | Yes | Distant, directional light source, such as from the sun. |
| `shift::core::light::IES` | `"ies"` | Yes | Specify a non-uniform light distribution using an Illuminating Engineering Society (IES) input file. |
| `shift::core::light::Point` | `"point"` | Yes | Specify a point light source such as a light bulb. |
| `shift::core::light::Spot` | `"spot"` | Yes |  Specify a spot light source, such as a lamp. |


### shift::core::logger::Base

* default `"type"` if no keyword provided in JSON file: `"csv"`


| Class | `"type"` keyword | Created by user? | Description |
--- | --- | --- | ---
| `shift::core::logger::CSV` | `"csv"` | Yes | Log to a comma delimited file. |
| `shift::core::logger::HDF5` | `"hdf5"` | Yes | Log to HDF5 file format. |



### shift::core::sensor::Base

* `"type"` is a required keyword in JSON file.

| Class | `"type"` keyword | Created by user? | Description
--- | --- | --- | ---
| `shift::core::logger::IMU` | `"imu"` | Yes | Model an inertial measurement unit. |
| `shift::core::logger::Range` | `"range"` | Yes | Model a laser range sensor. |


### shift::core::solver::Base

* default `"type"` if no keyword provided in JSON file: `"rnca"`

| Class | `"type"` keyword | Created by user? | Description |
--- | --- | --- | ---
| `shift::core::solver::RNCA` | `"rnca"` | Yes | The run-time optimized natural constraint algorithm (RNCA) is a sparse matrix solver that uses a heuristic application of Generalized Coordinate Partitioning (GCP). |


### shift::core::terrain::Base

* default `"type"` if no keyword provided in JSON file: `"basic"`

| Class | `"type"` keyword | Created by user? | Description |
--- | --- | --- | ---
| `shift::core::terrain::Basic` | `"basic"` | Yes | Loads a single digital elevation map (DEM) from a rasterized data file to represent the terrain. |


## Example

This example illustrates how a `shift::core::body::Rigid` specialization of the `shift::core::body::Base` class is created in the code using the `"rigid"` keyword:

```c
#include "shift/core/factory.h"
#include "shift/core/body/base.h"
auto body = shift::core::Factory<shift::core::body::Base>::get()->Create("rigid");
```

The `shift::core::Context` class is only aware of the base class implementation for each type. It is not aware of the specific details of the rigid body specialization of the `shift::core::body::Base` base class. It only calls functions defined in its base class implementation. This allows other *body* specializations to be created without modifying the core *Shift* code.

The above code creates a `unique_ptr` to a `shift::core::body::Base` class. Any object created when loading from the JSON file are managed by the `Shift::Assembly` or `shift::core::body` class the object is a member of. Any `unique_ptr` created automatically when preprocessing the system topology (for example automatically identifying chord constraints), are managed by the `shift::core::Context` class. Raw pointers to those objects may be used by other *Shift* objects. Shared pointers are not used due to additional overhead required to manage them.

Note that in general, users of the *Shift* software would never create a *Rigid* body using the above code and needn't concern themselves with these implementation details. Instead, they would specify the system topology in a JSON file and the object would be created automatically during parsing. In the example above, the default body `type` is `rigid`. If the user does not specify a `type` in the JSON file, it is assumed that the type is rigid and only the keys associated with a `rigid` body are processed.

A specialization can be created by deriving a new class from the `shift::core::body::Base` base class. All body objects use the `shift::core::body::` name space. The *rigid* specialization class is `shift::core::body::Rigid`. All other *Shift* objects are similarly name spaced.

Each body specialization must have a unique keyword identifier. The keyword `"rigid"` cannot be used to define any other body specialization. The `"rigid"` key word is registered in the `body/rigid.cc` file as:

```c
bool shift::core::body::Rigid::registered_ =
 shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
   "rigid", shift::core::body::Rigid::Create);
```
