.. _force-modules:
.. title:: Force Modules

Force Modules
=============

A force module applies forces to the model as it runs: between bodies (a
bushing, a beam, a belt, contact), on a joint (a hydraulic cylinder), or
between a wheel and the ground (a tire).

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

**Outputs.** A force module may publish output values, such as the
pressures of a cylinder or the tensions of a belt, for other components to
read and for the loggers to record. A logger records every output, or those
of the modules it names, as columns named ``<assembly>__<module>__<output>``
(see :ref:`logging`).

**Starting in equilibrium.** A hydraulic cylinder with ``"hold": true``
starts with the pressures that hold the model still; the forces of all such
cylinders are found together from the equations of motion before the
simulation starts (see :doc:`hydraulic-cylinder`).

In the model file
-----------------

Force modules are listed in the
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
   terrain-contact
   tire
   soil
   soil-tool
   track
   bushing
   spring-damper
   joint-forces
   applied-load
   rubber-mount
   beam
   belt
   hydraulic-cylinder
   electric-motor
   lead-screw
   gear-mesh
   journal-bearing
   thrust-collar
   spline
   powertrain

.. list-table::
   :header-rows: 1

   * - type
     - what it models
   * - :doc:`contact <contact>`
     - contact and friction between collision geometry
   * - :doc:`terrain_contact <terrain-contact>`
     - contact of a body with the terrain at a set of points on it
   * - :doc:`tire <tire>`
     - a wheel on the ground or terrain: Fiala and PAC2002 tire models
   * - :doc:`soil_tool <soil-tool>`
     - the force of the soil on a blade cutting it, and the pile it pushes
   * - :doc:`bushing <bushing>`
     - a compliant joint: 6×6 stiffness and damping between frames on bodies, as at the pivots of a control arm or a steering link
   * - :doc:`spring_damper <spring-damper>`
     - a spring and damper along the line between two frames or about an axis, with measured curves, stops and hysteresis
   * - :doc:`thrust_collar <thrust-collar>`
     - the axial contact of a collar and a shoulder: oil film, tapered pads, surfaces and friction
   * - :doc:`spline <spline>`
     - a sliding spline: the torque it carries, its backlash, and the friction that grows with that torque
   * - :doc:`joint_friction <joint-forces>`
     - Coulomb and viscous friction on a joint, which can hold it at rest
   * - :doc:`joint_map <joint-forces>`
     - a force on a joint from a measured map of two variables
   * - :doc:`applied_load <applied-load>`
     - a force and moment on a body or between two bodies, constant, over time, or as an impulse
   * - :doc:`rubber_mount <rubber-mount>`
     - a bushing whose stiffness and damping depend on the frequency of the motion
   * - :doc:`beam <beam>`
     - an elastic beam: tension, bending and torsion
   * - :doc:`belt <belt>`
     - a belt around two or more pulleys, with slip
   * - :doc:`hydraulic_cylinder <hydraulic-cylinder>`
     - a valve-controlled hydraulic cylinder on a prismatic joint
   * - :doc:`electric_motor <electric-motor>`
     - a DC or field-oriented brushless motor on a revolute joint, driven by
       its current or voltage input

To add a new kind of force module, see :doc:`writing-a-force-module`.
