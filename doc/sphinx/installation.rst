.. _installation:
.. title:: Installation

Installation
============

Getting the source
------------------

.. code:: bash

   git clone git@github.com:shift-dynamics/shift.git
   cd shift

*KSL* and the command line parser argagg are meson subprojects
(``subprojects/*.wrap``); meson fetches them the first time the build is
set up.

Dependencies
------------

*Shift* builds with `meson <https://mesonbuild.com>`_ and ninja and needs a
C++17 compiler and these libraries:

========================= =================================================
library                   used for
========================= =================================================
jsoncpp                   reading model files in JSON
yaml-cpp                  reading model files in YAML
assimp                    loading meshes (OBJ, glTF, ...)
graphviz (libgvc)         drawing the body and frame graphs
hdf5 (C++)                the HDF5 logger
cblas                     dense linear algebra
LAPACKE                   eigenvalues of ``shift-cli --linearize``
qhull                     convex hulls for collision geometry
libtiff                   terrain elevation maps
OpenMP, threads
STEPcode (optional)       reading STEP CAD files (meson option ``step``)
Vulkan loader and headers calculations on the GPU, such as the SPH soil
and glslc (optional)      (meson option ``gpu``)
Qt 6.7+, ffmpeg libraries the 3D viewer ``shift`` (meson option ``gui``)
GoogleTest                the test suite (meson option ``tests``)
doxygen, Sphinx, breathe, this documentation (meson option ``docs``)
sphinxcontrib-bibtex
========================= =================================================

Building
--------

.. code:: bash

   meson setup build
   meson compile -C build
   meson test -C build

This builds ``build/shift/main/shift-cli``, the viewer
``build/shift/main/shift`` when Qt 6 is found, and the test suite
``build/test/shift-test``. ``meson test`` runs the unit tests and simulates
every model in ``examples/``, except for a few long collision models that
are kept as benchmarks (``meson test -C build --benchmark``).

``build/`` is a debug build (``-O0``, with asserts). For simulation work,
keep an optimized build next to it, which is 4–8 times faster:

.. code:: bash

   meson setup build-release --buildtype=release -Dcpu=native
   meson compile -C build-release

Options
-------

Set options with ``meson configure build -D<option>=<value>``:

==================== ========= =================================================
option               default   purpose
==================== ========= =================================================
``tests``            auto      build the GoogleTest suite
``docs``             auto      add the ``docs`` target (this site)
``gui``              auto      build the Qt 6 viewer ``shift``
``step``             auto      read STEP files with STEPcode
``gpu``              auto      run calculations on the GPU with Vulkan
                               compute (see below)
``log_level``        -1        5 trace, 4 debug, 3 info, 2 warning, 1 error;
                               -1 is 4 in debug builds and 3 otherwise
``cpu``              x86-64    passed to ``-march``; ``native`` optimizes for
                               the build machine (the binaries may not run on
                               older processors)
``fp_trap``          true      ``shift-cli`` stops on a floating point
                               division by zero in every build type;
                               ``false`` for the last bit of speed
``verify_equations`` false     check every solution of the equations of
                               motion against the assembled equations (slow;
                               for debugging the solver)
==================== ========= =================================================

GPU calculations
----------------

Some calculations, starting with the SPH soil, run on the GPU through a small
Vulkan compute layer in ``shift/core/gpu``. It needs no window, so
``shift-cli`` uses it too. It is built when the Vulkan loader and headers and
``glslc`` are found (option ``gpu``); without them, a model that needs the
GPU stops with an error saying so.

The layer picks a discrete GPU if there is one, then an integrated one, then
any other device, such as a software driver. Two environment variables
change that:

* ``SHIFT_GPU_DEVICE=<name>`` picks the first device whose name contains
  ``<name>``, e.g. ``llvmpipe`` for Mesa's software driver lavapipe.
* ``SHIFT_VULKAN_VALIDATION=1`` turns on the Khronos validation layer, whose
  messages go to the log, as in the viewer.

``meson test`` runs the GPU tests on the default device, and skips them if
there is none. When lavapipe is installed (on Arch Linux, the
``vulkan-swrast`` package), it also runs them on lavapipe as the test
``gpu-lavapipe``, so they run on machines without a GPU.

Running a model
---------------

From the top of the repository, give ``shift-cli`` the path to the model file:

.. code:: bash

   build/shift/main/shift-cli examples/pendulum/pendulum.json

You do not need to start *Shift* from the folder of a model. It runs from any
directory, and the paths to the program and to the model file can each be
relative to the current directory or absolute.

``shift-cli --help`` lists the options: ``-d`` writes the graph of bodies
and joints and exits, ``-f`` the graph of frames, ``-V`` only checks the
model file, and ``-v`` sets how much is printed. Files the model refers to
(meshes, terrain, tire property files) are found next to the model file or
in its ``include_directories``, wherever you start from: a relative include
directory is taken from the directory of the model file. A log file is
written only if the model declares a logger, and goes to the current directory.

To watch it run, open the model in the viewer:

.. code:: bash

   build/shift/main/shift examples/pendulum/pendulum.json

Building this documentation
---------------------------

.. code:: bash

   meson compile -C build docs     # -> build/doc/html/index.html

or, without a configured build directory,

.. code:: bash

   sphinx-build -b html doc/sphinx build/doc/html

Either way doxygen reads the headers for the :ref:`api`. The site is
published to GitHub Pages by the ``docs`` workflow
(``.github/workflows/docs.yml``) on every push to ``master``.
