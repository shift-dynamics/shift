.. title:: Rubber Mount

Rubber Mount
============

``"type": "rubber_mount"`` is a :doc:`bushing` whose stiffness and damping
depend on the frequency of the motion, as those of rubber do: it stiffens
and its damping changes as the motion grows faster. The mount has the
bushing's frames, relative coordinates, rest pose, static stiffness
:math:`K`, damping :math:`C` and preload, and in parallel with them a
hardening term and spring–damper pairs in series (Maxwell elements) on each
coordinate:

.. math::

   \underline{f} = -K \underline{x} - C \dot{\underline{x}} +
   \underline{f}_0 - k_3 \underline{x}^3 - \sum_e k_e \underline{s}_e

Each element is a spring of stiffness :math:`k_e` in series with a damper
of coefficient :math:`c_e`. The stretch :math:`s_e` of its spring is an
internal state, with

.. math::

   \dot s_e = \dot x - s_e / \tau_e, \qquad \tau_e = c_e / k_e

Slow motion relaxes the springs through their dampers, leaving the static
stiffness; motion much faster than :math:`1/\tau_e` loads the springs,
adding :math:`k_e` to the stiffness. In between, an element damps the most,
near the frequency :math:`1/(2\pi\tau_e)`. Several elements with time
constants spread over the frequencies of interest fit a measured dynamic
stiffness and loss angle (a Prony series). The springs start relaxed.

JSON
----

``maxwell_stiffness`` and ``maxwell_damping`` hold :math:`6m` numbers for
each element, element by element: the :math:`k_e` and :math:`c_e` of every
coordinate of the first element, then of the second, and so on. A zero
stiffness leaves a coordinate without that element.

.. literalinclude:: ../../../examples/force_modules/rubber_mount.json
   :language: javascript
   :caption: examples/force_modules/rubber_mount.json

.. shift-parameters:: shift/core/force_module/rubber_mount.cc

and the keys of :doc:`bushing`.

Outputs
-------

As for :doc:`bushing`: the force and moment of each pair, now including the
hardening term and the Maxwell elements.

Example
-------

.. figure:: ../images/examples/rubber_mount.jpg
   :width: 60%
   :alt: rubber_mount.json: a 150 kg V8 engine on two rubber mounts (V8 Engine by Tahminayeasminmony, CC-BY-4.0)

   rubber_mount.json: a 150 kg V8 engine on two rubber mounts (V8 Engine by Tahminayeasminmony, CC-BY-4.0)

:source:`examples/force_modules/rubber_mount.json`: an engine of 150 kg on
two rubber mounts, one in front and one behind. It starts with a knock, and
its Maxwell elements relax as it comes to
rest, so it settles where the static stiffness and the hardening term
carry its weight, :math:`2(k x + k_3 x^3) = mg`, 1.95 mm down. Tests:
:source:`test/test_rubber_mount.cc`.
