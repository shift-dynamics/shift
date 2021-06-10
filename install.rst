.. title:: Installation

Installation
==================

Acquiring *Shift*
-----------------

To download the *Shift* repository, you will need to set up your ssh key as described `here <https://help.github.com/articles/connecting-to-github-with-ssh/>`_. Then clone the repository recursively as:

.. code::

  git clone git@github.com:shift-dynamics/shift.git


Dependencies
------------

In addition to the git submodules that are packaged with the *Shift* repository, the following additional libraries and tools are required:

*   `Cmake <https://cmake.org>`_
*   `Assimp <http://assimp.sourceforge.net>`_
*   `Jsoncpp <https://github.com/open-source-parsers/jsoncpp/>`_
*   `Qt5 <http://www.qt.io/developers/>`_
*   `Graphviz <http://www.graphviz.org/>`_
*   `Cblas <http://www.netlib.org/blas/>`_
*   `ImageMagick <http://www.imagemagick.org>`_
*   `Qhull <http://www.qhull.org>`_
*   `pandoc <http://pandoc.org>`_
*   `glslang <https://www.khronos.org/opengles/sdk/tools/Reference-Compiler/>`_


Linux
*****

On most linux variants, the above packages can be installed using the distribution's package manager.

The Jenkins integration tests currently build and test code with ``gcc`` version 8.2 and ``cmake`` 3.13 on Arch Linux.

Arch Linux:
^^^^^^^^^^^

.. code::

    pacman -S cmake assimp jsoncpp qt5 graphviz cblas imagemagick qhull pandoc
    vulkan-icd-loader vulkan-headers vulkan-validation-layers vulkan-tools glslang sed


Ubuntu / Debian-based Linux Distributions:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code::

    apt-get install cmake assimp jsoncpp qt5 graphviz cblas libmagickwand-dev libqhull-dev pandoc


Macintosh
*********

On a Macintosh, the dependencies can be installed using the *brew* package manager:

.. code::

    brew install cmake assimp jsoncpp qt5 graphviz imagemagick qhull pandoc


The Cblas package is already available on a Macintosh using the *accelerate*
framework. The CMakeLists.txt file may need to be modified to bring in this
dependency correctly.


Building the project
--------------------

From within the top directory of the repository, issue the following command:

.. code::

    mkdir build
    cd build
    cmake ..
    make


Executables will be located in ``${PROJECT_DIRECTORY}/bin`` where 
``${PROJECT_DIRECTORY}`` is the root directory of the git project on your system.

You can test if the executable works correctly by running the command line
interface to the program from within the ``${PROJECT_DIRECTORY}`` as:

.. code::

    ./bin/shift-cli external/models/excavator/excavator.json


Run the graphical user interface component of the program by executing the
following command from within the ${PROJECT_DIRECTORY}/build/bin directory:

.. code::

    ./bin/shift


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
    git clone git@github.com:shift-dynamics/shift.git .
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
