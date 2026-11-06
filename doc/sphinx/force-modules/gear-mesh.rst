.. title:: Gear mesh

Gear mesh
=========

``"type": "gear_mesh"`` is the mesh of a pair of spur gears: the teeth in contact
as springs and dampers, with a stiffness that changes as the teeth roll through the
mesh, backlash, and the errors of the teeth. It gives the forces and torques that
the gears put on each other and on their shafts, and the vibration of the pair that
is heard as the whine of gears.

Each gear is a body with a frame on its axis, with the z axis along the axis. The
two axes are parallel and the z axes point the same way. The gears are free bodies
(on revolute joints, say, with their bearings), and the mesh is a force between
them and not a constraint, so the shafts and bearings keep their own dynamics.

The line of action
------------------

The teeth touch along the *line of action*, a line through the *pitch point* (on
the line between the centers, at the pitch radius of each gear) that makes the
*pressure angle* :math:`\phi` with the common tangent of the pitch circles. The
force between the teeth is along it. When the pair is driven one way, one flank of
each tooth is in contact; driven the other way, the other flank is, and the line of
action is the mirror image of the first in the line of centers. The module keeps the
two contacts apart, a "plus" and a "minus" flank, each of which pushes but never
pulls. With the gears apart by the *backlash*, neither is in contact, and the gears
turn freely through the gap.

Two quantities say how far each flank is compressed:

* the displacement :math:`\delta_t` of the pitch point of gear 1 relative to that of
  gear 2, along the tangent of the pitch circles, the integral of their relative
  velocity (a state of the module, zero at the start, with the teeth midway through
  the backlash);
* the approach of the centers :math:`d_0 - d`, from the distance :math:`d` between
  them and its value :math:`d_0` at the start.

The compression of each flank, along its line of action, is

.. math::

   e_\pm = \pm\cos\phi\, \delta_t + \sin\phi\,(d_0 - d) + p(x) - \tfrac12 b \cos\phi

with the circumferential backlash :math:`b` at the pitch circle and the error
:math:`p(x)` of the teeth, their extra thickness along the line of action at the
position :math:`x` in the mesh (below). The force along the line of action is

.. math::

   f_\pm = \max\bigl(0,\; k(x)\, \langle e_\pm \rangle + c\, s(e_\pm)\, \dot e_\pm \bigr)

with the stiffness :math:`k(x)`, the damping :math:`c`, :math:`\langle e \rangle`
the positive part of :math:`e` made smooth over the length ``smoothing``, and
:math:`s` its slope (0 before the flank touches and 1 once it does), so that the
force has no jump as a flank touches. The force on gear 2 at the pitch point is
:math:`(f_+ - f_-)\cos\phi` along the tangent and :math:`(f_+ + f_-)\sin\phi` along the
line of centers, apart from gear 1; gear 1 gets the opposite. The torque on a gear is
the tangential force times its pitch radius, so the torques are in the ratio of the
radii, and the speeds in the opposite ratio.

The stiffness of the mesh
-------------------------

Teeth in contact are springs, and the number of pairs in contact changes as the
gears turn. The *contact ratio* :math:`\varepsilon`, between 1 and 2, is the mean
number of pairs in contact: two pairs share the load for the fraction
:math:`\varepsilon - 1` of each mesh, and one pair for the rest
:cite:`kahraman1990`, with the load sharing of teeth in contact as in
:cite:`elkholy1985`. With the stiffness :math:`k_t` of one pair of teeth,

.. math::

   k(x) = k_t\,(1 + m(x))

where :math:`m` is 1 while two pairs are in contact and 0 while one is. It changes
over a smooth ramp, a fraction ``ramp`` of the mesh (default 0.1), so that the
force has no jump, which the variable step integrator would have to find. The
mean stiffness over the mesh is :math:`k_t \varepsilon`.

The position :math:`x` in the mesh is the fraction of a tooth pitch that gear 1 has
turned, :math:`z_1\theta_1/2\pi` modulo 1 for its :math:`z_1` teeth, plus
``initial_phase``; the module integrates the rotation of gear 1 for it. A
``stiffness_table`` of the stiffness against :math:`x` over [0, 1], such as one
computed for the actual teeth, replaces the two-pair model. A ``profile_error``
table gives :math:`p(x)` the same way.

Gear whine
----------

The varying stiffness and the errors of the teeth push the pair at the mesh
frequency :math:`z_1\omega_1/2\pi` and its harmonics, which show up as ripples of
the transmission error and of the mesh force, and as vibration of the shafts.
With a constant stiffness (a contact ratio of 1) and no error of the teeth, the
transmission error is steady in steady running. The natural frequency of the pair
with one pair of teeth in contact is

.. math::

   \omega_n^2 = k_t \cos^2\phi \left(\frac{r_1^2}{i_k} + \frac{r_2^2}{i_k}\right)

for the pitch radii :math:`r_k` and the moments of inertia :math:`i_k` of the
gears (and what is rigidly attached to them) about their axes; the tests check the
oscillation of the pair against it. Pitch the mesh frequency away from it and its
harmonics.

Not in the module: friction between the teeth, the oil squeeze film (use
``damping``), internal gears, helical gears and the axial and radial flexibility of
the teeth beyond ``tooth_stiffness``.

.. code:: javascript

   {"type": "gear_mesh", "name": "mesh",
    "gear_1": ["pinion", "origin"], "gear_2": ["wheel", "origin"],
    "pitch_radius_1": 0.05, "pitch_radius_2": 0.1, "teeth_1": 20,
    "pressure_angle": 0.349, "tooth_stiffness": 3e8, "contact_ratio": 1.6,
    "damping": 1.2e4, "backlash": 2e-5}

``tooth_stiffness`` and ``damping`` can be changed as the model runs (see
:ref:`schedule`).

Outputs, logged as ``<name>_<output>``: ``transmission_error`` (:math:`\delta_t`),
``mesh_force`` (the net tangential force at the pitch circles), ``force_plus`` and
``force_minus`` (along the lines of action), ``stiffness`` and ``phase`` (the
position :math:`x` in the mesh).

.. shift-parameters:: shift/core/force_module/gear_mesh.cc
   :exclude: name, type, active

Example
-------

:source:`examples/force_modules/gear_mesh.json`: a 20 tooth pinion of 50 mm pitch
radius drives a wheel of 100 mm, loaded by a viscous torque, with a PID controller
holding the pinion at 100 rad/s. The mean mesh force is 100 N, the load divided by
the pitch radius of the wheel, and it ripples between about 70 and 130 N at the mesh
frequency, 318 Hz, from the stiffness of the mesh (a contact ratio of 1.6) and
the profile error. Tests: :source:`test/test_gear_mesh.cc`.
