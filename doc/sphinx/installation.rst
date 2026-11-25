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

Meshes, textures, terrain maps and other binary files are stored with
`Git LFS <https://git-lfs.com>`_. Install it before cloning (or run
``git lfs install && git lfs pull`` in a clone made without it): without it
those files are small text pointers, models that use them fail to load, and
the viewer has no textures.

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
STEPcode (optional)       reading STEP CAD files (meson option ``step``); an
                          installed one with its AP242 schema, or else
                          built from ``subprojects/stepcode.wrap`` (needs
                          CMake and make)
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
``build/test/shift-test``. ``meson test`` runs the unit tests, split into
24 parts that run in parallel, and simulates the first 0.1 s of every model
in ``examples/`` (enough to check that each loads and runs), except for a few
long collision models that are kept as benchmarks (``meson test -C build
--benchmark``, which simulates them in full). So are the
contact tests that take longest, a dozen of 15 s to 2 minutes each in a
release build, and the long runs of the DEM and of the SPH soil: run them
with ``meson test -C build --benchmark contact_heavy``, ``--benchmark
dem_heavy`` and ``--benchmark sph_heavy``
(they are listed in :source:`test/meson.build`).

``build/`` is a debug build (``-O0``, with asserts). The test suite runs
several times faster in an optimized build that keeps the asserts:

.. code:: bash

   meson setup build-check --buildtype=debugoptimized -Db_ndebug=false
   meson compile -C build-check
   meson test -C build-check

For simulation work, keep an optimized build next to it, which is 4–8 times
faster than ``build/``:

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
``step``             auto      read STEP files with STEPcode (installed in
                               ``/usr`` or ``/usr/local``, or else the
                               subproject)
``gpu``              auto      run calculations on the GPU with Vulkan
                               compute (see below)
``dlss``             auto      upscale the viewer's ray traced view with
                               NVIDIA DLSS (see below)
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

Linux
-----

Install the compiler, the build tools and the libraries with your
distribution's package manager, then build as above (`Building`_).

Arch Linux:

.. code:: bash

   sudo pacman -S --needed base-devel meson ninja cmake pkgconf git-lfs \
     jsoncpp yaml-cpp assimp graphviz hdf5 cblas lapacke qhull libtiff \
     eigen qt6-base qt6-shadertools ffmpeg openexr \
     vulkan-headers vulkan-icd-loader shaderc gtest

Debian and Ubuntu (Debian 13 or Ubuntu 25.04 and newer: the viewer needs Qt
6.7 and FFmpeg 7.1, which older releases do not have; without them the build
leaves out the viewer):

.. code:: bash

   sudo apt install build-essential meson ninja-build cmake pkg-config git-lfs \
     libjsoncpp-dev libyaml-cpp-dev libassimp-dev libgraphviz-dev libhdf5-dev \
     libblas-dev liblapacke-dev libqhull-dev libtiff-dev libeigen3-dev \
     qt6-base-dev qt6-base-private-dev qt6-shader-baker \
     libavformat-dev libavcodec-dev libavutil-dev libswscale-dev libopenexr-dev \
     libvulkan-dev glslc libgtest-dev

Fedora (and, with EPEL, the distributions based on Red Hat Enterprise Linux,
whose Qt may be older than the viewer needs):

.. code:: bash

   sudo dnf install gcc-c++ meson ninja-build cmake pkgconf git-lfs \
     jsoncpp-devel yaml-cpp-devel assimp-devel graphviz-devel hdf5-devel \
     blas-devel lapack-devel qhull-devel libtiff-devel eigen3-devel \
     qt6-qtbase-devel qt6-qtbase-private-devel qt6-qtshadertools-devel \
     ffmpeg-free-devel openexr-devel \
     vulkan-headers vulkan-loader-devel glslc gtest-devel

GCC brings OpenMP. STEPcode is not packaged by these distributions: meson
builds it from its subproject (``subprojects/stepcode.wrap``), or uses one
installed in ``/usr`` or ``/usr/local``. A GPU's Vulkan driver comes with its
graphics driver (Mesa's ``vulkan-radeon`` or ``vulkan-intel`` on Arch,
``mesa-vulkan-drivers`` on Debian and Fedora, or NVIDIA's); Mesa's software
driver lavapipe (``vulkan-swrast`` on Arch, part of ``mesa-vulkan-drivers``
elsewhere) runs the GPU tests without a GPU. For this documentation, add
doxygen and Sphinx (see `Building this documentation`_).

Then fetch the binary files with Git LFS, if the clone was made without it,
and build:

.. code:: bash

   git lfs install
   git lfs pull
   meson setup build
   meson compile -C build
   meson test -C build

macOS
-----

*Shift* builds on macOS with Apple's compiler and the libraries from
`Homebrew <https://brew.sh>`_; it has been built on Apple silicon (macOS 26,
Apple clang 21, Qt 6.12). Install the command line tools (or Xcode) and
Homebrew first:

.. code:: bash

   xcode-select --install

Then the build tools and the libraries:

.. code:: bash

   brew install meson ninja cmake pkgconf git-lfs \
     jsoncpp yaml-cpp assimp graphviz hdf5 lapack qhull libtiff libomp \
     eigen qtbase qtshadertools ffmpeg openexr \
     vulkan-headers vulkan-loader molten-vk shaderc googletest

``vulkan-tools`` adds ``vulkaninfo``, which lists the GPU as Vulkan sees it,
and ``vulkan-validationlayers`` the layer that ``SHIFT_VULKAN_VALIDATION=1``
turns on; neither is needed to build. Homebrew has no STEPcode: meson builds
it from its subproject (``subprojects/stepcode.wrap``), which adds a few
minutes to the first build.

Fetch the binary files of the clone with Git LFS (see above):

.. code:: bash

   git lfs install
   git lfs pull

Build with the native file ``machine/macos-homebrew.ini``, which gives meson
what it cannot find by itself: the paths of Homebrew's LAPACK and OpenMP
(both keg-only, so not on the default search paths), ``cpu=apple-m1``, and
``fp_trap=false``:

.. code:: bash

   meson setup build --native-file machine/macos-homebrew.ini
   meson compile -C build
   meson test -C build

and the optimized build:

.. code:: bash

   meson setup build-release --native-file machine/macos-homebrew.ini --buildtype=release
   meson compile -C build-release

On an Intel Mac, Homebrew is in ``/usr/local``: copy the native file and set
``brew = '/usr/local'`` and ``cpu = 'native'`` in it.

What differs from Linux:

* The viewer and the GPU calculations use Vulkan through MoltenVK, which
  Homebrew's Qt is built with. *Shift* turns off MoltenVK's fast math
  (``MVK_CONFIG_FAST_MATH_ENABLED=0``, unless it is already set), which the
  ray tracer needs.
* The ray traced view traces rays in compute shaders: MoltenVK has no
  hardware ray tracing, and NVIDIA DLSS is not available.
* ``fp_trap`` does nothing: macOS has no ``feenableexcept``, so
  ``shift-cli`` does not stop on a floating point division by zero.
* The viewer draws its scene into a texture of its own and copies it to the
  window, as on any GPU whose memory is lazily allocated (Apple's, and
  mobile GPUs): there the window's depth buffer is not kept while MoltenVK
  splits a render pass for a tessellated draw. ``SHIFT_SCENE_TEXTURE=0`` or
  ``1`` turns this off or on on any device.

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

NVIDIA DLSS in the viewer
~~~~~~~~~~~~~~~~~~~~~~~~~

In the ray traced view, the viewer can trace fewer pixels than the window has
and let NVIDIA DLSS (Deep Learning Super Sampling) fill in the rest, which
makes the view roughly twice as fast to update in its Quality mode. Option
``dlss`` fetches the NVIDIA DLSS SDK as a meson subproject
(``subprojects/dlss.wrap``, about 1.6 GB, once) when the viewer is built.
The SDK is NVIDIA's, under the NVIDIA RTX SDKs license in its
``LICENSE.txt``, not Shift's license; ``-Ddlss=disabled`` builds without it.

DLSS needs an NVIDIA RTX GPU and a recent driver. The viewer checks at start;
anywhere else it says why in the log and keeps its own anti-aliasing.
**Edit → Settings** turns DLSS off or picks its mode, from DLAA (anti-aliasing
at full resolution) to Ultra Performance (the fewest pixels traced), and
shows the resolution each mode traces at. Saved images and videos are always
traced at full resolution, so they do not depend on the setting.

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
