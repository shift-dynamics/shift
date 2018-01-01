.. title:: Installation

Installation
============

Acquiring Shift
---------------

*Shift* is accessible through a private github repository located at `<https://github.com/kriswehage/shift>`_. You may request access by contacting the author. Once you have been granted access, you will need to add your ssh key to github following the instructions described `here <https://help.github.com/articles/connecting-to-github-with-ssh/>`_. Then clone the project recursively using the following command:

.. code::

   git clone --recursive git@github.com:kriswehage/shift.git

The recursive flag is necessary to initialize the submodules. If you cloned the repository without the recursive flag, you can initialize the submodules later by issuing the command:

.. code::

   git submodule update --init --recursive


Dependencies
------------

In addition to the git submodules that are packaged together with *Shift*, the following additional libraries must be installed on your system.

*   `Cmake <https://cmake.org>`_
*   `Assimp <http://assimp.sourceforge.net>`_
*   `Jansson <http://www.digip.org/jansson/>`_
*   `Qt5 <http://www.qt.io/developers/>`_
*   `Graphviz <http://www.graphviz.org/>`_
*   `Cblas <http://www.netlib.org/blas/>`_
*   `ImageMagick <http://www.imagemagick.org>`_
*   `Qhull <http://www.qhull.org>`_
*   `pandoc <http://pandoc.org>`_


Linux
*****

On most Linux distributions the above packages can be installed using the distribution's package manager.

Arch Linux:
^^^^^^^^^^^

.. code::

   pacman -S cmake assimp jansson qt5 graphviz cblas imagemagick pandoc


Ubuntu / Debian-based Linux Distributions:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code::

   apt-get install cmake assimp jansson qt5 graphviz cblas libmagickwand-dev pandoc


Macintosh
*********

On a Macintosh, the dependencies can be installed using the *brew* package manager:

.. code::

  brew install cmake assimp jansson qt5 graphviz imagemagick pandoc


The cblas package is already available on a Macintosh using the *accelerate* framework, the CMakeLists.txt file may need to be modified to bring in this dependency correctly.


Windows
*******

*Shift* has not been tested on Windows machines.


Building Shift
--------------------

From within the top directory of the repository, issue the following commands:

.. code::

   mkdir build
   cd build
   cmake ..
   make


Executables will be located in ${PROJECT_DIRECTORY}/build/bin

Where ${PROJECT_DIRECTORY} is the root directory of the git project on your system.

You can test if the executable works correctly by running the command line interface to the program as:

.. code::

   cd ${PROJECT_DIRECTORY}/build/bin
   ./shift_cli ../models/excavator/excavator.json


Run the graphical user interface component of the program by executing the following command from within the ${PROJECT_DIRECTORY}/build/bin directory:

.. code::

   ./shift

Building documentation
----------------------

Documentation can be built by issuing the command in the build directory:

.. code::

   make doc


Optional - documentation can be copied to your /share/doc folder by issuing the command:

.. code::

   make install
