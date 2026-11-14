Shift examples
==============

*Shift* is open to the public at https://github.com/shift-dynamics/shift. For more information, refer to https://shift-dynamics.github.io/shift/.

This directory contains models defined using the *Shift* `JSON <https://shift-dynamics.github.io/shift/model-file/model-file.html>`_ syntax that demonstrate some capabilities of the software. Refer to the *readme.rst* file within each directory for more information, and to the `examples page <https://shift-dynamics.github.io/shift/examples/examples.html>`_ of the documentation for pictures of them.

Run a model from the top of the repository with the command line program, or open it in the viewer::

  build/shift/main/shift-cli examples/pendulum/pendulum.json
  build/shift/main/shift examples/pendulum/pendulum.json

Every model is also a test: ``meson test -C build`` loads and simulates each one, except for a few large models that are kept as benchmarks (``meson test -C build --benchmark``).

Mechanisms and robots
---------------------

* `Pendulum <pendulum>`_
* `Slider crank <slider_crank>`_
* `Bricard's mechanism <bricard>`_
* `Klann linkage and walker <klann_linkage>`_
* `Sarrus linkage <sarrus>`_
* `Bicycle caliper brake <brake>`_
* `Screw and gear joints <joints>`_
* `Abiflexx D4 delta robot <abiflexx_d4_1300>`_
* `Kuka LBR IIWA 14 R820 7R industrial robot <kuka_lbr_iiwa_14_r820>`_

Vehicles and machines
---------------------

* `Vehicle chassis and tires <vehicle>`_
* `Ford F-150 Raptor R pickup truck <raptor>`_
* `Wulf Revo rally car <wulf_revo>`_
* `Hydraulic excavator <excavator>`_
* `Wheels on soft soil <terramechanics>`_

Forces, contact and geometry
----------------------------

* `Force modules <force_modules>`_
* `Collision modeling <collision>`_
* `CAD parts, cam and track <cad>`_
* `Terrain modeling <terrain>`_
* `Discrete element method <dem>`_

Building models
---------------

* `Generating models with Python <scripting>`_
