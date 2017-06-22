.. title:: Introduction

Introduction
============

Three-dimensional CAD (Computer Aided Design) packages have become standard tools in the workflow of modern engineers. In modern CAD software, an engineer builds complex virtual mechanisms and multibody systems in a manner similar to how one would assemble real mechanisms. A user specifies a number of parts and connects them together at joints. Bodies may be grouped together in assemblies or subassemblies to facilitate rapidly building complex machines from more primitive components. In this way, engineers can rapidly design highly sophisticated mechanisms, yet most CAD users lack the training to set up and program detailed three-dimensional rigid body dynamic simulations or analyze the transmission performance of the complex multibody systems they create.

Indeed, programming three-dimensional dynamic simulations is tedious and error-prone, even for the experienced analyst, and therefore automatic methods for setting up and solving a system of dynamic equations and analyzing transmission performance from a user’s CAD representation are desired. Automatic methods should be general-purpose – capable of loading a wide range of parametric and topological variations, robust – capable of handling singular configurations such as bifurfactions and end-of-stroke conditions, and numerically efficient – allowing engineers to quickly generate simulation results for complex systems on standard engineering workstations.

Additionally, the author recognizes the following additional requirements for modern simulation software: data structures (both internal and external) should allow hierarchical organization of bodies and joints into assemblies and subassemblies, allowing users to build complex models from primitives, a concept which is familiar to CAD users; an analyst should not require specialized knowledge of graph theoretic concepts, such as scene graphs or spanning trees, to set up a model - software should analyze the topology of an analyst's natural body-joint-body representation, which may not be the same as a scene graph; and finally, with the pervasive influence of the world-wide-web, and more recently, the internet of things, external data structures should be compatible with modern web technologies.


Expected Contributions
----------------------

    * Design of an ASCII text-based file format based on the Javascript Object Notation (JSON) syntax to describe a multibody system as a directed-acyclic graph (DAG), which allows bodies and joints to be arranged in user-defined assembly hierarchies.
    * Development of a sparse, vectorized kinematics library written in the C programming language - used for transforming displacements, velocities, accelerations and forces. Vectorization is achieved through the use of BLAS (Basic Linear Algebra Subroutines).
    * Discussion of numerical best practices for arranging multibody system data to maximize cache and memory efficiency and enhance throughput of floating point operations.
    * Demonstration of *automata* programming method for updating mechanism topology and solving equations of motion.
    * Demonstration of *dual graph representation* for simultaneous representation of a mechanism as a directed acyclic graph (DAG) and a directed-graph (DG) within a program.
    * Create a desktop-based graphical user interface written in Qt and C++ as a platform to visualize simulation output and benchmark algorithms.
    * Design and implementation of a graph-partitioning algorithm, called *Kinematic Substructuring*, which is used to automatically identify non-overlapping groups of independent chord loops (i.e. *substructures*), thereby block-diagonalizing the Jacobian matrix. Block-diagonalization allows for a significant reduction in floating point operations required for generalized coordinate partitioning (GCP).
    * Introduce a new measure of transmission performance called *Mobility Numbers*.
    * Demonstrate use of mobility numbers for computing mechanical advantages and velocity ratios.
    * Demonstrate use of mobility numbers for computing joint reaction forces.
    * Demonstrate use of mobility numbers for path planning and optimization for redundantly actuated and parallel mechanisms.
