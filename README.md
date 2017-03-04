# A Simulation Tool for Automation and Robotics

## Dependencies

This code requires that the following libraries are installed on your system.

*   [Assimp](<http://assimp.sourceforge.net/>)
*   [Jansson](<http://www.digip.org/jansson/>)
*   [Qt5](<http://www.qt.io/developers/>)
*   [Graphviz](<http://www.graphviz.org/>)
*   [Cblas](<http://www.netlib.org/blas/>)

### Macintosh

On a Macintosh, these dependencies can be installed using the *brew*
package manager:

    brew install assimp jansson qt5 graphviz

More information can be found at these links.

*   <http://brewformulas.org/Assimp>
*   <http://brewformulas.org/Jansson>
*   <http://brewformulas.org/qt5>
*   <http://brewformulas.org/graphviz>

The cblas package is already available on a Macintosh using the *accelerate*
framework, the CMakeLists.txt file may need to be modified to bring in this
dependency correctly.

### Arch Linux

On arch linux, the dependencies can be installed using pacman:

    pacman -S assimp jansson qt5 graphviz cblas

### Ubuntu / Debian-based Linux Distributions

Use apt-get:

    apt-get install assimp jansson qt5 graphviz cblas

## Building the project

From within the top directory of the repository, issue the following command:

    mkdir build
    cd build
    cmake ..
    make

Executables will be located in
${PROJECT_DIRECTORY}/build/bin

Where ${PROJECT_DIRECTORY} is the root directory of the git project on your
system.

You can test if the executable works correctly by running the command line
interface to the program as:

    cd ${PROJECT_DIRECTORY}/build/bin
    ./star2_cli ../models/excavator/excavator.json

Run the graphical user interface component of the program by executing the
following command from within the ${PROJECT_DIRECTORY}/build/bin directory:

    ./star2

## Building documentation

Documentation can be built by issuing the command in the build directory:

    make doc

Optional - documentation can be copied to your /share/doc folder by issuing the
command:

    make install

Alternatively, refer [here](<http://viper.ahmct.ucdavis.edu/static/doxygen/>)
for the latest Doxygen generated documentation. You will need to be issued a
username and password to view this content.

## About

star2 loads and analyzes the topology of a mechanism specified in a JSON
text file.  The analysis is split in to the following steps:

### Preprocessing
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

    *   Precondition the primary constraints (i.e. sparse solve for the *B  
        matrix* which is used to find part of the right null space of Jacobian).

    *   Map mass matrix in original order to permuted order.

    *   Evaluate equations of motion and store the results in unpermuted
        order.


### Run-Time Algorithm (RTA)

During a simulation, the following operations are carried out at every time
step. The RTA uses only a few primitive functions to set up and solve the
equations of motion, stepping through arrays of memory locations without
complex logic.

*   Carry out operations set up in Run-Time Optimization step using
    source and destination memory pointers.

*   Monitor ratio of pivots in LU factors of Jacobian for each substructure.
    If the ratio increases above 10 times the starting value, the simulation is
    paused and the Run-Time Optimization step is carried out again to
    regenerate pointer arrays.
