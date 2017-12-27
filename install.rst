.. title:: Installation

Installation
==================

Acquiring *Shift*
-----------------

To download the *Shift* repository, you will need to set up your ssh key as described `here <https://help.github.com/articles/connecting-to-github-with-ssh/>`_. Then clone the repository recursively as:

.. code::

  git clone --recursive git@github.com:kriswehage/shift.git


The recursive flag is necessary to initialize the submodules. If you already cloned the repository without the recursive flag, you can initialize the submodules later by issuing the command:

.. code::

  git submodule update --init --recursive


Dependencies
------------

In addition to the git submodules that are packaged with the *Shift* repository, the following additional libraries and tools are required:

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

On most linux variants, the above packages can be installed using the distribution's package manager.

Arch Linux:
^^^^^^^^^^^

.. code::

    pacman -S cmake assimp jansson qt5 graphviz cblas imagemagick qhull pandoc


Ubuntu / Debian-based Linux Distributions:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code::

    apt-get install cmake assimp jansson qt5 graphviz cblas libmagickwand-dev libqhull-dev pandoc


Macintosh
*********

On a Macintosh, the dependencies can be installed using the *brew* package manager:

.. code::

    brew install cmake assimp jansson qt5 graphviz imagemagick qhull pandoc


Cblas package is already available on a Macintosh using the *accelerate*
framework, the CMakeLists.txt file may need to be modified to bring in this
dependency correctly.


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
    ./shift_cli ../models/excavator/excavator.json


Run the graphical user interface component of the program by executing the
following command from within the ${PROJECT_DIRECTORY}/build/bin directory:

.. code::

    ./shift

To run the gui program with nvidia-gfx-debugger:

.. code::

    LD_PRELOAD=~/.tgd/libs/libNvidia_gfx_debugger.so ./shift


Building documentation
----------------------

Building the documention requires the dependencies `Doxygen <http://www.stack.nl/~dimitri/doxygen/>`_, `doxyrest <https://github.com/vovkos/doxyrest/>`_ and `Sphinx <http://www.sphinx-doc.org/en/stable/>`_. The documentaion is built in the following three steps.

1. A Doxygen xml listing is generated from inline Doxygen comments using Doxygen.
2. Doxygen xml listings are converted to Restructured Text (.rst) format using doxyrest.
3. Sphinx documentation is generated using Sphinx.


The cmake *doc* target carries out the three steps above. To build the documentation locally run:

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

This section is targeted to developers of *Shift*. Issue the following commands to create a directory containing the gh-pages branch. This only needs to be done once after the main repository has been cloned.

.. code::

    mkdir -p doc/html
    cd doc/html
    git clone git@github.com:kriswehage/shift.git
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
