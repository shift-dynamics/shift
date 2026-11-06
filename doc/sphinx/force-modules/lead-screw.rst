.. title:: Lead screw

Lead screw
==========

``"type": "lead_screw"`` is the thread between a screw and its nut, which turns
the rotation of one into an axial push on the other: the screw of a jack, a
linear actuator or a press. It gives the torque that drives the screw, the axial
force that it delivers, and whether the screw holds its load when it is not
driven.

The screw and the nut are two bodies, each with a frame on the axis of the
thread, whose z axis is that axis. The bodies are free to turn and to slide along
the axis (a revolute joint and a prismatic joint in series, for example); the
module makes the thread an elastic contact between them, so that they move as a
screw does, to a small deflection.

The ideal screw, turned by the angle :math:`\theta` relative to the nut, advances
along the axis by :math:`p\,\theta`, with :math:`p` the lead divided by
:math:`2\pi`, and by :math:`-p\,\theta` for a left handed thread. The module
measures the advance :math:`s` of the screw (zero at the start, when the thread is
unloaded) and calls the difference the *deflection*

.. math::

   \delta = s - p\,\theta .

Where :math:`\delta` is zero the thread carries nothing. Elsewhere the flanks
of the thread are pressed together.

The flank of the thread has the *lead angle* :math:`\lambda`, with
:math:`\tan\lambda = \text{lead}/(2\pi r)` at the mean radius :math:`r` where
the flank carries the load, and the *thread angle* :math:`\alpha`, the half
angle of the thread in a plane through the axis (0 for a square thread, 14.5° for
an Acme thread). The deflection presses the flanks together along their normal by
:math:`\delta \cos\alpha\cos\lambda`, and the contact force along the normal is

.. math::

   n = \cos\alpha\cos\lambda\,\bigl(k\,\delta + c\,\dot\delta\bigr)

for the contact stiffness :math:`k` and damping :math:`c`. Its radial part
cancels around the thread. Its part along the axis, :math:`n \cos\alpha\cos\lambda`,
pushes the screw along the axis, and the circular part turns it, with :math:`p`
times the axial part as torque: the screw is driven by torque and delivers force
as the ideal screw does.

*Friction.* Friction opposes the sliding of the flank over its nut, which is along
the thread, at the speed :math:`v = \sqrt{(r\omega)^2 + \dot s^2}` for the
turning speed :math:`\omega` and the advance speed :math:`\dot s` of the screw.
The friction force is

.. math::

   \mu\,|n|\,\frac{v}{\sqrt{v^2 + v_s^2}}

along the sliding, with the friction coefficient :math:`\mu` and the slip speed
:math:`v_s`: Coulomb friction that goes smoothly to zero at low speed. This
friction is the loss of the screw: the torque to drive it exceeds that of the ideal
screw.

In steady motion the module gives the torque that power screw theory does
:cite:`budynas2015`. To raise a load :math:`f` (to drive the screw against it) the
torque is

.. math::

   \tau = f\,r\,\frac{\tan\lambda + \mu\sec\alpha}{1 - \mu\sec\alpha\tan\lambda},

and to lower it

.. math::

   \tau = f\,r\,\frac{\mu\sec\alpha - \tan\lambda}{1 + \mu\sec\alpha\tan\lambda},

which is negative if :math:`\mu\sec\alpha < \tan\lambda`: the load turns the screw,
and the screw is not *self-locking*. The tests compare the module's torque and
axial force with these formulas for square and Acme threads, raising and lowering.

*What the smoothing costs.* Because the friction is smoothed at low speed, a load
held by a self-locking screw creeps slowly, at a speed up to the slip speed times
the ratio of the force needed to hold it to the friction available, where a real
screw would stay. Make ``slip_speed`` small against the speeds of the motion, or use
the bristles below.

*Stick and slip.* With ``bristle_stiffness`` :math:`\sigma_0` the friction of the thread
holds at rest, by the bristle model of friction (the LuGre model,
see :doc:`contact`). The flanks slide along the thread at the speed
:math:`u = r\omega\cos\lambda + \dot s\sin\lambda` (with the sign of
:math:`\dot s` reversed for a left handed thread), and the deflection :math:`z` of the
bristles is a state of the module:

.. math::

   \dot z = u - \frac{\sigma_0|u|}{g(|u|)}\,z, \qquad
   g(|u|) = \mu + (\mu_s - \mu)\,e^{-(|u|/v_s)^2}, \qquad
   f_t = |n|\,(\sigma_0 z + \sigma_1\dot z + \sigma_2 u)

for the friction force :math:`f_t` along the thread, with ``bristle_damping``
:math:`\sigma_1`, ``viscous_friction`` :math:`\sigma_2`, ``static_friction``
:math:`\mu_s` (at least :math:`\mu`, default :math:`\mu`) and ``stribeck_velocity``
:math:`v_s`, which takes the place of ``slip_speed``. A load held on the thread moves
by a small :math:`z` and stops, as long as :math:`\mu_s\sec\alpha \ge \tan\lambda`, so a
thread can hold from rest and still run once it has started, if :math:`\mu\sec\alpha`
is below :math:`\tan\lambda`. In steady motion :math:`f_t = \mu|n|`, the friction of the
formulas above. The tests compare the travel of a held load with the creep of the
smoothed friction, and check that the bristles at rest carry :math:`\tan\lambda` of the
normal force.

The bristles are a stiff spring on the screw, of :math:`|n|\sigma_0` along the thread.
``bristle_damping`` settles them and defaults to 5 s/m. With 0 the screw rings on this
spring after a load comes on, with nothing to settle it. For a quick settle, choose
:math:`\sigma_1` so that the screw's inertia on this spring is about critically damped.

*Backlash.* A ``backlash`` (axial play in the thread) leaves the deflection free
for :math:`|\delta| < \text{backlash}/2`, and the thread loads smoothly beyond it.

The friction of the bearing that carries the load on the screw (the thrust collar)
is not in the module; put it on the joint that carries the screw.

.. code:: javascript

   {"type": "lead_screw", "name": "thread",
    "screw": ["screw", "origin"], "nut": ["fixed", "origin"],
    "lead": 0.008, "mean_radius": 0.015, "thread_angle": 0.2531,
    "friction": 0.12, "stiffness": 5e8, "damping": 5e4}

``friction``, ``stiffness``, ``damping`` and ``slip_speed`` can be changed as the
model runs (see :ref:`schedule`).

Outputs, logged as ``<assembly>__<name>__<output>``: ``axial_force`` (the force of the thread on
the screw along the axis), ``torque`` (on the screw about the axis),
``deflection``, ``normal_force``, ``friction_force``, ``turns`` (the rotation
of the screw relative to the nut, in turns) and ``stick``
(:math:`\sigma_0|z|/\mu_s`, 1 at the limit of holding).

.. shift-parameters:: shift/core/force_module/lead_screw.cc
   :exclude: name, type, active

Examples
--------

:source:`examples/force_modules/lead_screw.json`: a screw jack. A 500 kg load
hangs on an Acme screw in a fixed nut, and a PID controller turns the screw at
20 rad/s to raise it. In steady motion the torque is 15.6 N m for the 501 kg that
hang on it, as the formula for raising gives.

:source:`examples/force_modules/lead_screw_hold.json`: the same jack left to hold its
load, with no drive. The bristles of the thread hold it, 0.0001 of a turn from the
start, with ``stick`` at 0.548, which is
:math:`\tan\lambda/(\mu_s\sec\alpha)`.

Tests: :source:`test/test_lead_screw.cc`.
