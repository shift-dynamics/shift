.. title:: Installation

Installation
============

Acquiring Shift
---------------

Presently, *Shift* is accessible only through a private github repository located at `<https://github.com/kriswehage/shift>`_. You may request access by contacting the author. Once you have been granted access, you will need to add your ssh key to github following the instructions described `here <https://help.github.com/articles/connecting-to-github-with-ssh/>`_. Then clone the project using the following command:

.. code::

   git clone git@github.com:kriswehage/shift.git


Dependencies
------------

*Shift* requires the following libraries to be installed on your system.

* Assimp

* Jansson

* Qt5

* Graphviz

* Cblas

* ImageMagick

Macintosh
*********

On a Macintosh, the dependencies can be installed using the *brew* package manager:

.. code::

   brew install assimp jansson qt5 graphviz imagemagick

More information can be found at these links.

* http://brewformulas.org/Assimp

* http://brewformulas.org/Jansson

* http://brewformulas.org/qt5

* http://brewformulas.org/graphviz

* http://brewformulas.org/imagemagick

The cblas package is already available on a Macintosh using the *accelerate* framework, the CMakeLists.txt file may need to be modified to bring in this dependency correctly.

Linux
*****

On most Linux distributions the packages can be installed using the distribution's package manager. For example, on Arch Linux, the dependencies can be installed as:

.. code::

   pacman -S assimp jansson qt5 graphviz cblas imagemagick


For Ubuntu / Debian-based Linux Distributions, use apt-get to install the dependencies as:

.. code::

   apt-get install assimp jansson qt5 graphviz cblas libmagickwand-dev


Windows
*******

Shift has not been tested on Windows machines.

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
