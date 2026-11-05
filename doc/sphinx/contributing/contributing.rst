.. _contributing:

.. title:: Contributing

Contributing
============

*Shift* is under active development, and contributions are welcome,
particularly from those who would like to use it in their research. Areas
of work include:

* force models: lead screws, stick–slip friction, journal bearings, gear
  mesh, line contact, a real-time tire for uneven terrain, powertrains, and
  tracked vehicles with soil under the track shoes;
* flexible bodies;
* contact detection and the contact of exact CAD surfaces;
* the 3D viewer;
* new models and test cases.

See the `issues page <https://github.com/shift-dynamics/shift/issues>`_ for
other tasks, or contact the author with ideas.

Coding style
------------

*Shift* follows the
`Google C++ Style Guide <https://google.github.io/styleguide/cppguide.html>`_:

* ``lower_snake_case`` for local variables, and with a trailing underscore
  (``lower_snake_case_``) for class members;
* ``lower_snake_case()`` for getters and setters;
* ``UpperCamelCase`` for classes and structs, and ``UpperCamelCase()`` for
  other methods and free functions.

Helpers private to a source file are static members or static functions.

The repository contains a
`clang-format <https://clang.llvm.org/docs/ClangFormat.html>`_
configuration for the *Shift* conventions, which most editors can apply.

Changes
-------

Every change comes with tests (see :doc:`../force-modules/writing-a-force-module`)
and keeps ``meson test -C build`` passing. A new kind of model also gets a
small example in ``examples/`` and a page in these documents.
