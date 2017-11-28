star2: A Simulation Tool for Automation and Robotics
====================================================

Dependencies
------------

This code requires that the following libraries are installed on your system.

*   `Assimp <http://assimp.sourceforge.net>`_
*   `Jansson <http://www.digip.org/jansson/>`_
*   `Qt5 <http://www.qt.io/developers/>`_
*   `Graphviz <http://www.graphviz.org/>`_
*   `Cblas <http://www.netlib.org/blas/>`_
*   `ImageMagick <http://www.imagemagick.org>`_
*   `Qhull <http://www.qhull.org>`_

Macintosh
*********

On a Macintosh, these dependencies can be installed using the *brew*
package manager:

.. code::

    brew install assimp jansson qt5 graphviz imagemagick qhull


The cblas package is already available on a Macintosh using the *accelerate*
framework, the CMakeLists.txt file may need to be modified to bring in this
dependency correctly.

Arch Linux
**********

On arch linux, the dependencies can be installed using pacman:

.. code::

    pacman -S assimp jansson qt5 graphviz cblas imagemagick qhull


Ubuntu / Debian-based Linux Distributions
*****************************************

Use apt-get:

.. code::

    apt-get install assimp jansson qt5 graphviz cblas libmagickwand-dev libqhull-dev


Building the project
--------------------

From within the top directory of the repository, issue the following command:

.. code::

    mkdir build
    cd build
    cmake ..
    make


Executables will be located in ${PROJECT_DIRECTORY}/build/bin

Where ${PROJECT_DIRECTORY} is the root directory of the git project on your
system.

You can test if the executable works correctly by running the command line
interface to the program as:

.. code::

    cd ${PROJECT_DIRECTORY}/build/bin
    ./star2_cli ../models/excavator/excavator.json


Run the graphical user interface component of the program by executing the
following command from within the ${PROJECT_DIRECTORY}/build/bin directory:

.. code::

    ./star2

To run the gui program with nvidia-gfx-debugger:

.. code::

    LD_PRELOAD=~/.tgd/libs/libNvidia_gfx_debugger.so ./star2


Building documentation
----------------------

Building the documention requires the dependencies `Doxygen <http://www.stack.nl/~dimitri/doxygen/>`_, `doxyrest <https://github.com/vovkos/doxyrest/>`_ and `Sphinx <http://www.sphinx-doc.org/en/stable/>`_. The documentaion is built in the following three steps.

  1. A Doxygen xml listing is generated from inline Doxygen comments using Doxygen.
  2. Doxygen xml listings are converted to Restructured Text (.rst) format using doxyrest.
  3. Sphinx documentation is generated using Sphinx.


To build the documentation locally:

.. code::

    mkdir build
    cd build
    cmake ..
    make doc


The html documentation will be generated in ${PROJECT_DIRECTORY}/doc/html

Optional - documentation can be copied to your /share/doc folder by issuing the
command:

.. code::

    make install


Syncing documentation with Github
---------------------------------

Issue the following commands to create a git submodule containing the gh-pages branch. This only needs to be done once after the main repository has been cloned.

.. code::

    mkdir -p doc/html
    cd doc/html
    git clone git@github.com:kriswehage/star2.git
    git checkout gh-pages
    git branch -d master


To update the Github hosted website, change directory back to ${PROJECT_DIRECTORY}. Then issue the commands:

.. code::

    mkdir build
    cd build
    cmake ..
    make doc
    cd ../doc/html
    git push
