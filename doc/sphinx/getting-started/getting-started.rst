.. _getting-started:
.. title:: Getting Started

Getting Started
===============

This page builds a model from nothing, runs it, and reads its results. The
model is :source:`examples/pendulum/pendulum.json`: one link, hinged to the
fixed frame, swinging under gravity. :ref:`model-file` describes every part
of the file in full.

The model file
--------------

A model is one JSON file. *Shift* reads it with comments allowed: ``//``
starts a comment that runs to the end of the line. The file has two
sections:

``model_parameters``
    How to simulate: the units, gravity, the solver of the equations of
    motion, the integrator and its time span, and the loggers that write
    the results.

``assemblies``
    What to simulate: bodies, the joints between them, and the force
    modules, sensors and controllers that act on them. Assemblies can hold
    subassemblies, nested as deeply as needed.

Fixed frames, coordinate systems on the fixed body (the ground), go in a
``frames`` array at the top level; every body, the fixed one included, has
a frame named ``origin``.

.. literalinclude:: ../../../examples/pendulum/pendulum.json
   :language: javascript
   :caption: examples/pendulum/pendulum.json

Step by step:

#. ``units`` names the units of the numbers in the file. The example models
   all use SI units (kilograms, meters, seconds, radians), and the
   defaults of the force modules assume them.
#. ``gravity`` is the acceleration of gravity in the fixed frame. The
   models in ``examples/`` put the z axis down, as vehicle models
   conventionally do (SAE axes), so gravity is ``[0, 0, 9.81]``.
#. ``solvers`` chooses how the equations of motion are set up and solved.
   ``rnca`` is the solver; ``infinity_norm_multiplier`` sets how far the
   partition of the coordinates may degrade before it is chosen again (see
   :ref:`about`).
#. ``integrators`` chooses the time integration. ``ode113`` is a variable
   step, variable order Adams–Bashforth–Moulton method that keeps the error
   of each step within ``relative_error`` and ``absolute_error``.
   ``time_span`` is the simulated time and ``time_step`` the interval at
   which results are written; the steps the integrator takes are chosen by
   its error control.
#. ``loggers`` writes the results: here a CSV file of every joint's
   coordinates, velocities and accelerations at every ``time_step``.
#. The body ``link1`` has a frame ``distal`` at its far end, 3 m along its
   z axis, and its center of mass 1.5 m along y from its origin. ``inertia``
   holds the mass and the moments of inertia about the center of mass, in
   the body's axes.
#. The joint ``fixed_to_link1`` is a revolute joint (a hinge) about the x
   axis, from the fixed body's ``origin`` frame (the *parent*) to the
   origin of ``link1`` (the *child*). It starts at 1 rad, turning at
   -1 rad/s.

*Shift* finds the structure of the model from the joints: which bodies are
connected to which, the loops they close, and the order in which to
evaluate them. A model lists bodies and joints in any order, and a closed
loop needs no special treatment: list the joint that closes it like any
other.

Running it
----------

From the top of the repository:

.. code:: bash

   build/shift/main/shift-cli examples/pendulum/pendulum.json

You do not need to change to the folder of the model: *Shift* runs from any
directory, given the path to the model file.

``shift-cli`` simulates the 100 s time span (in a few hundredths of a
second), printing its progress, and, as the model declares a ``csv`` logger,
writes ``pendulum.csv`` to the current directory. ``shift-cli -d examples/pendulum/pendulum.json`` instead draws the graph of bodies and joints
that *Shift* found, and exits. Its first columns are the time and the joint's
coordinate, velocity and acceleration:

.. code:: text

   time, pendulum__fixed_to_link1__q_0, pendulum__fixed_to_link1__dq_0, ...

followed by the root mean square error of the loop closure equations and
the potential, kinetic and total energy. The total energy stays at
-10.7572 J from the first row to the last, as it should for a pendulum
without friction. Any force module with outputs adds its own columns (see
:ref:`logging`).

To watch the model, open it in the viewer:

.. code:: bash

   build/shift/main/shift examples/pendulum/pendulum.json

The viewer can also render a model to an image without user input, which
is how the pictures in these documents are made
(:source:`scripts/render_doc_images.sh`):

.. code:: bash

   # simulate to 1 s, then save a ray traced image, viewed from 30 degrees
   # around the z axis and 20 degrees above the floor
   build/shift/main/shift examples/pendulum/double_pendulum.json \
     --image pendulum.png --time 1 --size 1200x800 --view 30,20 --samples 256

``shift --help`` lists the options.

To keep a moving body in view, tick *follow* in the *Camera* box of the
*Simulation* tab and pick the body. The camera then keeps its place relative to
that body, moving and turning with it, and you can still orbit, pan and zoom.
With ``--image``, ``--follow`` does the same: the view is set at the start and
follows the body to ``--time``.

In *body* mode the body looks still, because the camera moves with all of its
motion. To see a vehicle bounce and tilt, choose *ground* mode
(``--follow-ground``): the camera stays level, at the height of the ground under
the body, and turns only with the body's heading.

To export part of a run, use the film strip in the *Video* box of the
*Simulation* tab. Drag its two handles, or type in *Start* and *End*, to choose
the part to keep. The view shows the frame under a handle as you drag, and
hovering over the strip shows a larger preview. *Refresh Preview* redraws the
strip from the current camera.

Adding to the model
-------------------

.. figure:: ../images/examples/double_pendulum.jpg
   :width: 50%
   :alt: double_pendulum.json at 1 s

   double_pendulum.json at 1 s

:source:`examples/pendulum/double_pendulum.json` adds a second link on a
joint at the ``distal`` frame of the first. Beyond bodies and joints, a
model can include:

* force modules, which apply forces between bodies or on joints:
  contact, tires, bushings and rubber mounts, beams, belts, hydraulic
  cylinders (:ref:`force-modules`);
* sensors, which measure signals of the model as it runs
  (:ref:`sensors`);
* controllers, which read sensors and drive joints (:ref:`controllers`);
* geometry for display and collision geometry for contact
  (:ref:`model-file`).

Each of these has a short runnable example in ``examples/``; the
:ref:`examples` page lists them all.
