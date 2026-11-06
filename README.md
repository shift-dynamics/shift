# Shift

A dynamics engine and visualization tool for automation and robotics.

## Overview

*Shift* is a general-purpose modeling environment targeted to robotics, automation and vehicle dynamics applications. The software was created and is maintained by Kristopher Wehage, PhD.

Refer to *Shift's* website at <https://shift-dynamics.github.io/shift/> for additional information on how to use the software and API.

## Building and running

*Shift* builds with meson and ninja. The libraries it needs are listed in the [installation guide](https://shift-dynamics.github.io/shift/installation.html).

```bash
git clone https://github.com/shift-dynamics/shift.git
cd shift
meson setup build
meson compile -C build
meson test -C build
```

`build/` is a debug build. For simulation work, keep an optimized build next to it, which is 4–8 times faster:

```bash
meson setup build-release --buildtype=release -Dcpu=native
meson compile -C build-release
```

Run a model from the top of the repository with the command line program, or open it in the 3D viewer:

```bash
build/shift/main/shift-cli examples/pendulum/pendulum.json
build/shift/main/shift examples/pendulum/pendulum.json
```

The viewer is built when Qt 6.7 or later is found. The models in [examples/](examples/readme.rst) show what the software can do, and each of them is also run by the tests.

## Code layout

| Path | Contents |
|------|----------|
| `shift/core/` | The dynamics engine, a library with no dependency on the viewer |
| `shift/gui/` | The Qt 6 3D viewer library: scene, camera, ray tracing, video export |
| `shift/main/` | The two programs: `shift_cli.cc` (`shift-cli`, no display needed) and `shift.cc` (`shift`, the viewer) |
| `test/` | The GoogleTest suite, which also simulates the models in `examples/` |
| `examples/` | Example models, with their meshes and the scripts that generate them |
| `doc/` | The source of the website (`doc/sphinx`) and developer notes |
| `assets/` | Textures, icons, geometry and other resources built into the viewer |
| `scripts/` | Utility scripts for plotting logs, preparing meshes and rendering the documentation images |
| `subprojects/` | meson wraps for [KSL](https://github.com/shift-dynamics/ksl), the spatial algebra library, and argagg, the command line parser |

Within `shift/core/`:

| Path | Contents |
|------|----------|
| `context`, `assembly` | `Context` loads a model, analyzes its topology and owns the simulation; `Assembly` is a container of bodies, joints and other objects, and assemblies can be nested |
| `substructure`, `topology`, `jacobian` | Kinematic substructures, the run-time arrays of displacements, velocities and accelerations (`topology::DHVA`), and the constraint Jacobian with its split into dependent and independent variables |
| `body/`, `joint/`, `constraint/` | Fixed, free, rigid and virtual bodies; revolute, prismatic, spherical, screw, gear, float and rigid joints; chord constraints that close kinematic loops |
| `solver/`, `integrator/` | The RNCA equations of motion solver; the ODE113 and RK4 integrators |
| `force_module/` | Forces on bodies and joints: springs, bushings, beams, belts, gears, bearings, hydraulics, powertrains, tires, soil and contact |
| `controller/`, `sensor/`, `logger/` | PID and schedule controllers and 7-DOF inverse kinematics; joint and frame sensors; CSV and HDF5 logs |
| `geometry/`, `collision_geometry/`, `cad/`, `terrain/`, `light/` | Display geometry and mass properties; shapes and contact detection; STEP files and exact surfaces; elevation maps; lights |
| `parameter/`, `factory` | Reading and writing objects as JSON, and creating them by their `type` |


## Workflow
*Shift* loads and analyzes the topology of a mechanism specified in a JSON
text file.  The analysis is split in to the following steps:

### Preprocessing

The following operations are performed once at the beginning of simulation
and need not be called again, unless the mechanism topology changes:

*   The user-defined bodies and joints are read into the arrays of the
    assemblies that contain them, and `shift::core::Context` gathers the ones
    that take part in the simulation into arrays of its own.

*   Starting from the fixed body, joints are sorted breadth-first into level
    increasing order along a spanning tree. A joint that reaches a body
    already in the tree closes a loop: it gets a virtual body and a chord
    constraint.

*   The topology of the mechanism is analyzed automatically to identify groups
    of overlapping chord loops, which are referred to as *kinematic substructures* -- (referred to as `shift::core::Substructure` in the code)

*   Bodies and joints are arranged in level increasing order within each
    substructure, thereby block-diagonalizing the Jacobian matrix.

*   To minimize cache misses, the data used at run time is held in a
    struct of arrays (`shift::core::topology::DHVA`, with the displacements,
    influence coefficients, velocities and quadratic accelerations) in
    level-increasing order, and lookup tables give each joint's place in them.


### Run-Time Optimization (RTO)

The following operations may be called once or periodically during the course
of a simulation. The operations are performed again only if generalized
coordinate partitioning indicates to use a different split of
independent/dependent variables during a running simulation:

*   Set up source and destination memory pointers to:

    *   Evaluate primary Jacobian matrix in permuted order.

    *   Evaluate primary constraints (and first and second time derivatives)
        in permuted order (no need to evaluate secondary constraints).

    *   Factor the primary Jacobian matrix.

    *   Precondition the primary constraints (i.e. sparse solve for the
        *B matrix* which is used to find part of the right null space of Jacobian).

    *   Map mass matrix in original order to permuted order.

    *   Evaluate equations of motion and store the results in unpermuted
        order.


### Run-Time Algorithm (RTA)

RTA uses only a few primitive functions to set up and sparse solve the equations of motion, stepping through arrays of memory locations without complex logic. During a simulation, the following operations are carried out at every time step:

*   Carry out operations set up in Run-Time Optimization step using
    source and destination memory pointers.

*   Monitor ratio of infinity norm of the B matrix associated with each
    block sub-Jacobian matrix of each substructure.  If the infinity norm increases more than a specified threshold from the initial starting value, the simulation is paused and the Run-Time Optimization step is carried out again to regenerate pointer arrays.


## References

Refer to the following publications for additional details on how the software works:

* Wehage K., Computational Methods for Dynamics and Mobility Analysis of Multiloop Mechanisms and Robotic Systems, Thesis, December 2016 <https://search.proquest.com/openview/2b96ee91ebc96f840bdada69a21b7cbe>

* Wehage K. Ravani B., A Computational Method for Formulation and Solution of Dynamical Equations for Complex Mechanisms and Multibody Systems, Proceedings of IDETC/DIE at Cleveland Ohio, August 2017, <https://doi.org/10.1115/DETC2017-67766>

* Wehage K., Wehage R., Ravani B., Generalized coordinate partitioning for complex mechanisms based on kinematic substructuring, Mechanism and Machine Theory 92:464-483, October 2015 <https://doi.org/10.1016/j.mechmachtheory.2015.06.006>

## Contributing

Shift uses the Google style guide: Some of the highlights are as follows:
* `lower_snake_case` for temporary variables
* `lower_snake_case_` with trailing underscore for class member variables
* `lower_snake_case()` for get and set class methods
* `UpperCamelCase` for struct and class definitions
* `UpperCamelCase()` for general class methods / functions

Refer to <https://google.github.io/styleguide/cppguide.html> for full details.
