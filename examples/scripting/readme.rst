Scripting model generation with Python
======================================

This directory contains an example of building a *Shift* `JSON <https://shift-dynamics.github.io/shift/model-file/model-file.html>`_ model programmatically using Python.

The **shift.py** module contains *Shift* datastructures and utility functions. The module can be imported into another Python script as::

  import shift

A model starts with a *Shift* context, which holds everything written to the file, and the directories to search for its meshes::

  cube = shift.Context()
  cube.add_include_dir(".")


An assembly can be added to the model as::

  main_assembly = shift.Assembly("main_assembly")


Bodies are added to the assembly as::

  main_assembly.add_body(
      shift.Body("cube",
                 definition=["cube.obj", "cube.mtl"]))


If no inertial properties are specified, as in the above example, the body will have a mass of 1 and moments of inertia Ixx = Iyy = Izz = 1 and Ixy = Iyz = Izx = 0. Give a body its real properties with the ``mass``, ``center_of_mass`` and ``moment_of_inertia`` arguments of ``shift.Body``.

A joint is then added to reference the body to the fixed inertial frame::

  main_assembly.add_joint(
      shift.Joint("joint1",
                  body_frame_pairs=[
                      ["fixed", "origin"], ["cube", "origin"]]))


Now the assembly is added to the context created earlier::

  cube.add_assembly(main_assembly)


And the file is written to disk as::

  cube.write_to_file("cube.json")


Refer to **cube.py** for the complete source code. It writes ``cube.json`` to the current directory, so run it from this folder. Then, from the top of the repository, run the model with::

  build/shift/main/shift-cli examples/scripting/cube.json

.. image:: cube.png
