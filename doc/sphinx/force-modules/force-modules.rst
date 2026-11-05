.. _force-modules:
.. title:: Force Modules

Force Modules
=============

A force module applies forces to the model as it runs: between bodies (a
bushing, a beam, a belt, contact), on a joint (a hydraulic cylinder), or
between a wheel and the ground (a tire). Force modules are listed in the
``force_modules`` array of an assembly, each with a ``type`` that selects
the model, a ``name``, and the keys of its type:

.. code:: javascript

   "force_modules": [
     {"type": "bushing", "name": "front_bushing",
      "frames": [["fixed", "front_mount"], ["left_control_arm", "front"]],
      "stiffness": [3e6, 3e6, 3e6, 1500, 3000, 3000],
      "damping": [5000, 5000, 5000, 58, 100, 100]}
   ]

Keys of every force module:

.. shift-parameters:: shift/core/force_module/base.cc

.. toctree::
   :maxdepth: 1

   contact
   tire
   soil
   track
   bushing
   rubber-mount
   beam
   belt
   hydraulic-cylinder

.. list-table::
   :header-rows: 1

   * - type
     - what it models
   * - :doc:`contact <contact>`
     - contact and friction between collision geometry
   * - :doc:`tire <tire>`
     - a wheel on the ground or terrain: Fiala and PAC2002 tire models
   * - :doc:`bushing <bushing>`
     - a compliant joint: 6×6 stiffness and damping between frames on bodies, as at the pivots of a control arm or a steering link
   * - :doc:`rubber_mount <rubber-mount>`
     - a bushing whose stiffness and damping depend on the frequency of the motion
   * - :doc:`beam <beam>`
     - an elastic beam: tension, bending and torsion
   * - :doc:`belt <belt>`
     - a belt around two or more pulleys, with slip
   * - :doc:`hydraulic_cylinder <hydraulic-cylinder>`
     - a valve-controlled hydraulic cylinder on a prismatic joint

How force modules work
----------------------

Each time the equations of motion are set up, after the kinematics are
updated and the sensors and controllers have run, every active force
module adds its forces: forces and moments on bodies, or generalized forces
on joints.

**Forces from the state only.** Every force module computes its forces from
the state of the model alone: the joint coordinates and velocities, and its
own *internal states*. A model that needs memory, such as the pressures in a
hydraulic cylinder, the stretch of a rubber mount's internal springs, the
turns of a belt's pulleys, or the slip of a tire, keeps it in internal
states whose rates it computes, and the integrator advances them with the
joint coordinates. Where a force changes abruptly (a span going slack, a
relief valve opening, first touch in a contact), the change is made smooth
over a small band. The variable step integrators then see no jumps, and the
results do not depend on the step size.

**Outputs.** A force module may name output values, such as the pressures
of a cylinder or the tensions of a belt. The loggers record them as
columns named ``<module>_<output>`` (see :ref:`logging`).

**Starting in equilibrium.** A hydraulic cylinder with ``"hold": true``
starts with the pressures that hold the model still; the forces of all such
cylinders are found together from the equations of motion before the
simulation starts (see :doc:`hydraulic-cylinder`).

To add a new kind of force module, see :doc:`writing-a-force-module`.
