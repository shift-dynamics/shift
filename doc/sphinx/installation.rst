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
qhull                     convex hulls for collision geometry
libtiff                   terrain elevation maps
OpenMP, threads
STEPcode (optional)       reading STEP CAD files (meson option ``step``)
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

Running a model
---------------

.. code:: bash

   cd examples/pendulum
   ../../build/shift/main/shift-cli pendulum.json

``shift-cli --help`` lists the options: ``-d`` writes the graph of bodies
and joints and exits, ``-f`` the graph of frames, ``-V`` only checks the
model file, and ``-v`` sets how much is printed. Files the model refers to
(meshes, terrain, tire property files) are found next to the model file or
in its ``include_directories``; log files are written to the current
directory.

To watch it run, open the model in the viewer:

.. code:: bash

   ../../build/shift/main/shift pendulum.json

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
