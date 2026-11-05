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
   :alt: bushing.json: a wheel and hub on a control arm that pivots on two bushings and hangs from a coil-over strut

   bushing.json: a wheel and hub on a control arm that pivots on two bushings and hangs from a coil-over strut

:source:`examples/force_modules/bushing.json`: a suspension corner, with a
wheel and hub (42 kg in all, 0.45 m out from the pivot axis) on an A-shaped
control arm that pivots on two bushings at its inboard end, on the x axis.
The bushings are stiff in every direction but the twist about that axis, so
the arm swings on them. A coil-over strut carries the weight, as in a
vehicle: a universal joint on the fixed frame, 0.5 m above the outboard end
of the arm, a prismatic joint with a spring (20 kN/m) and a damper
(600 N s/m), and a ball joint on the arm. Its spring is stretched to pull
with :math:`m g` = 412 N, so the bushings carry no load at the start. The
arm starts with a bump, 0.3 rad/s, and the wheel rises 2.6 mm, then rings
down at about 3.8 Hz and is still within 0.2 s. It settles 0.56 mm below its
start, with a twist of 1.25 mrad in the bushings, under the 20 N of the
weight of the strut that the arm carries: the spring and the bushings share
that load as springs in parallel, at the wheel
:math:`k_s + 2 k_{rx}/L^2` = 34.8 kN/m.
Tests: :source:`test/test_bushing.cc`.
