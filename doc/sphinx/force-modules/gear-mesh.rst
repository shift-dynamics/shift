.. title:: Gear mesh

Gear mesh
=========

``"type": "gear_mesh"`` is the mesh of a pair of gears, spur, helical or internal: the teeth in contact
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

Stiffness from the shape of the teeth
-------------------------------------

With ``tooth_geometry`` and ``face_width`` :math:`w`, the module works out the contact
ratio and :math:`k(x)` from the teeth, in place of ``tooth_stiffness``,
``contact_ratio`` and ``stiffness_table``. Contact runs along the line of action
between the tip circles. A point on it is the distance :math:`\rho_k` from where the
line touches the base circle of gear :math:`k`, of radius
:math:`r_{bk} = r_k\cos\phi`, and the contact ratio is the length of that path over
the base pitch.

A pair of teeth is a chain of springs (the potential energy method
:cite:`yang1987,tian2004`):

.. math::

   \frac{1}{k} = \frac{1}{k_h} + \sum_{i=1}^{2}\Bigl(\frac{1}{k_{bi}} + \frac{1}{k_{si}}
   + \frac{1}{k_{ai}} + \frac{1}{k_{fi}}\Bigr)

for the Hertzian contact, and for each tooth its bending, shear and axial compression
as a cantilever, and the give of the gear's body under it :cite:`sainsot2004`. With
Young's modulus :math:`e_m`, Poisson's ratio :math:`\nu`, the half angle of the tooth at
its base circle :math:`\alpha_2 = \pi/2z + \tan\phi - \phi`, the angle of the load
:math:`\alpha_1 = \rho/r_b - \alpha_2` and
:math:`g(\alpha) = \sin\alpha + (\alpha_2 - \alpha)\cos\alpha`:

.. math::

   \begin{aligned}
   \frac{1}{k_h} &= \frac{4(1 - \nu^2)}{\pi e_m w} \\
   \frac{1}{k_b} &= \int_{-\alpha_1}^{\alpha_r}
     \frac{3\bigl(1 + \cos\alpha_1((\alpha_2 - \alpha)\sin\alpha - \cos\alpha)\bigr)^2
     (\alpha_2 - \alpha)\cos\alpha}{2 e_m w\,g(\alpha)^3}\,d\alpha \\
   \frac{1}{k_s} &= \int_{-\alpha_1}^{\alpha_r}
     \frac{6(1 + \nu)(\alpha_2 - \alpha)\cos\alpha\cos^2\alpha_1}{5 e_m w\,g(\alpha)}\,d\alpha \\
   \frac{1}{k_a} &= \int_{-\alpha_1}^{\alpha_r}
     \frac{(\alpha_2 - \alpha)\cos\alpha\sin^2\alpha_1}{2 e_m w\,g(\alpha)}\,d\alpha \\
   \frac{1}{k_f} &= \frac{\cos^2\alpha_1}{e_m w}\Bigl(l^*\bigl(\tfrac{u_f}{s_f}\bigr)^2
     + m^*\tfrac{u_f}{s_f} + p^*(1 + q^*\tan^2\alpha_1)\Bigr)
   \end{aligned}

The cantilever starts at the base circle, :math:`\alpha_r = \alpha_2`, or at the root
circle if that is larger. :math:`u_f` is the distance along the tooth's center line
from the root circle to the line of the load, :math:`s_f` the thickness of the tooth
at the root, and :math:`l^*, m^*, p^*, q^*` are the published fits in the ratio of the
root radius to the hub radius and the half angle of the tooth at the root. The
stiffness of the mesh at each position is the sum over the pairs in contact.

The tests check the stiffness of a single pair against the single stiffness of ISO
6336-1 for solid steel gears: it is within 9 % for 20 to 40 teeth, with a ratio of 2.
The teeth are refused if they interfere or if the contact ratio is below 1. Full depth
teeth of 20 in a ring of 60, for example, interfere.

Not modelled: profile shift and tip relief, the growth of the contact under load, and
the coupling of neighboring teeth through the gear's body. A tooth of a ring gear is
taken as that of an external gear with the same number of teeth, loaded at the same
height.

Internal gears
--------------

With ``"internal": true``, gear 2 is a ring and gear 1 runs inside it. The centers are
:math:`r_2 - r_1` apart, the pitch point is on the far side of gear 1 from the ring's
center, and the gears turn the same way. In the compression of the flanks, the
approach of the centers changes sign: pushing the pinion into the ring's teeth takes
the centers apart.

Helical gears
-------------

With ``helix_angle`` :math:`\beta`, ``pressure_angle`` is :math:`\phi_n`, measured in the
section normal to the teeth, and the line of action leaves the plane of the gears. For
the tangent :math:`\underline{n}` of the pitch circles, the line of centers
:math:`\underline{c}` from gear 1 to gear 2 and the axis :math:`\underline{a}`, that of
the plus flank of external gears is

.. math::

   \underline{u} = \cos\phi_n\cos\beta\,\underline{n} + \sin\phi_n\,\underline{c}
     - \cos\phi_n\sin\beta\,\underline{a}

which gives the usual relations: an axial force of the tangential force times
:math:`\tan\beta`, and a radial force of that times :math:`\tan\phi_n/\cos\beta`. The
compression of a flank includes the motion of the gears along the axis, and the force
acts at the pitch point, so the axial force also tips the gears. A positive angle is a
right hand helix on gear 1; external gears have opposite hands, and a ring the same hand.

With ``face_width``, each section across the face lags the one before it, so a pair
comes into contact gradually. The module takes the mean of the stiffness over the
*overlap ratio* :math:`w\tan\beta\,z_1/2\pi r_1`, in pitches. With an overlap of a
whole pitch the stiffness no longer varies, which is why helical gears run quietly.

The squeeze film of the oil
---------------------------

With ``oil_viscosity`` :math:`\mu`, a flank closing on its partner squeezes the oil out
of the gap :math:`h = \mathrm{pos}(-e) + h_0`, with ``min_film`` :math:`h_0`. Two
cylinders of relative radius :math:`r_e` at the pitch point, along a line of length
:math:`w/\cos\beta_b` for the helix angle :math:`\beta_b` at the base circle, give from
Reynolds' equation

.. math::

   f_s = 3\sqrt{2}\,\pi\,\mu\,\frac{w}{\cos\beta_b}
     \Bigl(\frac{r_e}{h}\Bigr)^{3/2}\dot e

while the gap closes, and no force as it opens, when the film cavitates. The force grows
as the gap closes, so it cushions the teeth as they cross the backlash, on top of
``damping``. The tests check it against a direct integration of the pressure.

Not in the module: friction between the teeth.

.. code:: javascript

   {"type": "gear_mesh", "name": "mesh",
    "gear_1": ["pinion", "origin"], "gear_2": ["wheel", "origin"],
    "pitch_radius_1": 0.05, "pitch_radius_2": 0.1, "teeth_1": 20,
    "pressure_angle": 0.349, "tooth_stiffness": 3e8, "contact_ratio": 1.6,
    "damping": 1.2e4, "backlash": 2e-5}

``tooth_stiffness`` and ``damping`` can be changed as the model runs (see
:ref:`schedule`).

Outputs, logged as ``<assembly>__<name>__<output>``: ``transmission_error`` (:math:`\delta_t`),
``mesh_force`` (the net tangential force at the pitch circles), ``force_plus`` and
``force_minus`` (along the lines of action), ``stiffness``, ``phase`` (the
position :math:`x` in the mesh), ``radial_force``, ``axial_force`` and ``squeeze_force``.

.. shift-parameters:: shift/core/force_module/gear_mesh.cc
   :exclude: name, type, active

Example
-------

:source:`examples/force_modules/gear_mesh.json`: a 20 tooth pinion of 50 mm pitch
radius drives a wheel of 100 mm, loaded by a viscous torque, with a PID controller
holding the pinion at 100 rad/s. The mean mesh force is 100 N, the load divided by
the pitch radius of the wheel, and it ripples between about 70 and 130 N at the mesh
frequency, 318 Hz, from the stiffness of the mesh (a contact ratio of 1.6) and
the profile error.

:source:`examples/force_modules/gear_mesh_helical.json`: the same pair as helical gears
in oil, with a 15 degree helix on a 50 mm face, an overlap of 0.85 of a pitch, and the
stiffness computed from the teeth. The stiffness stays between 9.3e8 and 1.03e9 N/m
through the mesh, where the spur pair's doubles, and the mesh force ripples between 88
and 110 N. The teeth push the gears along their axis with 26.8 N, the 100 N of the mesh
times :math:`\tan\beta`.

Tests: :source:`test/test_gear_mesh.cc`.
