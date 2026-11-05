.. title:: Bushing

Bushing
=======

``"type": "bushing"`` joins frames on two bodies as a bushing does, for the
pivots of a linkage (a suspension control arm, a steering link, a tie rod),
where a rubber sleeve stands in for an ideal joint: it holds the part
against every motion but a small twist or sway. It has full 6×6 stiffness and damping matrices of their relative motion, so that
every direction of motion may load every other, as in a bushing whose
stiffness has been measured or computed by finite elements. A group of
:math:`n` frames joins each of its :math:`n (n - 1)/2` pairs, with one set
of matrices over all the pairs, as a simple model of a flexible structure.

The relative motion
-------------------

For each pair (:math:`i`, :math:`j`), the six coordinates are those of
frame :math:`j` relative to frame :math:`i`, measured from the pair's rest
pose and in the axes of the rest pose:

.. math::

   \underline{x} = [d_x, d_y, d_z, r_x, r_y, r_z]

with :math:`\underline{d}` the displacement of :math:`j`'s origin and
:math:`\underline{r}` the rotation vector (axis times angle, radians) of its
rotation. Their rates are the velocity of :math:`j`'s origin relative to
the point of :math:`i`'s body there, and the angular velocity of :math:`j`
relative to :math:`i`. The rest pose is the pose of the pair as the model
starts (``"rest_pose": "initial"``), or the pose where the two frames
coincide (``"frames"``).

The force and moment on frame :math:`j`, at its origin and in the rest
axes, are

.. math::

   \underline{f} = -K \underline{x} - C \dot{\underline{x}} +
   \underline{f}_0

and the opposite force and moment act on frame :math:`i`'s body at the
same point. For :math:`m` pairs, :math:`K` and :math:`C` are
:math:`6m \times 6m`, in the order of the pairs (0, 1), (0, 2), ...,
(1, 2), ..., so the forces of one pair may depend on the motion of every
pair.

JSON
----

``stiffness`` and ``damping`` take :math:`6m` numbers for a diagonal matrix
or :math:`(6m)^2` for the full matrix, row by row; a full matrix may also
come from a text file (``stiffness_file``, ``damping_file``).

.. literalinclude:: ../../../examples/force_modules/bushing.json
   :language: javascript
   :caption: examples/force_modules/bushing.json

.. shift-parameters:: shift/core/force_module/bushing.cc
   :exclude: name, type, active

Outputs
-------

``pair_<k>_fx``, ``_fy``, ``_fz``, ``_mx``, ``_my``, ``_mz``: the force and
moment :math:`\underline{f}` of each pair :math:`k` on frame :math:`j`, in
the rest axes.

Example
-------

.. figure:: ../images/examples/bushing.jpg
   :width: 60%
   :alt: bushing.json: the front left suspension of the vehicle model, with its control arm on two bushings

   bushing.json: the front left suspension of the vehicle model, with its control arm on two bushings

:source:`examples/force_modules/bushing.json`: the front left suspension of
the :doc:`vehicle model <../examples/examples>` (its lower control arm,
steering knuckle, McPherson strut, tie rod and wheel, with their meshes and
mass properties), on the fixed frame in place of the chassis. The control
arm does not pivot on a revolute joint but on two bushings, 0.12 m either
side of its pivot on the x axis. The bushings are stiff in every direction
but the twist about that axis, so the arm swings on them. The strut's
spring (58.5 kN/m, a prismatic joint with a spring and damper) carries the
weight of the wheel and suspension, so the bushings carry no load at the
start; its damper is 400 N s/m, a lighter damping than the vehicle's, which
has the body above it. The arm starts with a bump, 0.8 rad/s, and the wheel
rises 4 mm, then rings down at about 7.7 Hz and is still within 0.2 s. The
bushings and the spring act in parallel, so the twist of the bushings (10
mrad at the peak) adds to the stiffness of the strut at the wheel.
Tests: :source:`test/test_bushing.cc`.
