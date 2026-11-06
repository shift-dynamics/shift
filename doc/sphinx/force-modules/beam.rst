.. title:: Beam

Beam
====

``"type": "beam"`` is a straight, uniform, elastic beam between frames on
two bodies. It runs from the origin of frame A, along A's x axis, to the
origin of frame B, which must lie on that axis as the model starts; its
length :math:`l` is the distance between them, and the axes of its cross
section are A's y and z axes. The beam is straight and unloaded as the
model starts.

The forces are those of a linear Euler–Bernoulli beam element
:cite:`przemieniecki1968`: end B moves relative to end A as the tip of a
beam clamped at A, and the force and moment that hold it there are

.. math::

   \begin{aligned}
   f_x &= \frac{e a}{l} u & m_x &= \frac{g j}{l} r_x \\
   f_y &= \frac{12 e i_z}{l^3} v - \frac{6 e i_z}{l^2} r_z &
   m_z &= -\frac{6 e i_z}{l^2} v + \frac{4 e i_z}{l} r_z \\
   f_z &= \frac{12 e i_y}{l^3} w + \frac{6 e i_y}{l^2} r_y &
   m_y &= \frac{6 e i_y}{l^2} w + \frac{4 e i_y}{l} r_y
   \end{aligned}

with :math:`(u, v, w)` the displacement of B and :math:`(r_x, r_y, r_z)` its
rotation relative to A, in A's axes. The beam applies the opposite,
restoring forces to B and the equal and opposite load to A, so the two
bodies are in equilibrium with the beam. The relative motion is measured as
for :doc:`bushing`, so the element suits deflections that are small
relative to its length; a chain of elements on light bodies at the nodes
gives a flexible link that bends as far as needed.

Damping proportional to the stiffness, :math:`C = \beta K` (``damping``,
seconds), gives each mode of angular frequency :math:`\omega` the damping
ratio :math:`\beta\omega/2`: it damps the fast modes of a chain of elements
(stretching, the higher bending modes) strongly and the slow ones only a
little.

Sections
--------

======================= ======================================================
``section``             keys
======================= ======================================================
``circle``              ``diameter``
``tube``                ``diameter``, ``inner_diameter``
``rectangle``           ``width`` (along y), ``height`` (along z); the
                        torsion constant is Roark's :cite:`young2002roark`
``general`` (default)   ``area``, ``Iy``, ``Iz`` (second moments about y and
                        z), ``J`` (torsion constant)
======================= ======================================================

The shear modulus is ``shear_modulus``, or :math:`g = e / (2(1 + \nu))`
from ``poisson_ratio``. The stiffness follows from the material and the
section, so a beam takes no ``stiffness`` or ``damping`` matrices.

JSON
----

.. literalinclude:: ../../../examples/force_modules/beam.json
   :language: javascript
   :caption: examples/force_modules/beam.json

.. shift-parameters:: shift/core/force_module/beam.cc

and ``frames`` (two frames, A and B) of :doc:`bushing`.

Outputs
-------

As for :doc:`bushing`: ``pair_0_fx`` ... ``pair_0_mz``, the force and moment
on end B in the axes of B at rest.

Example
-------

.. figure:: ../images/examples/beam.jpg
   :width: 80%
   :alt: beam.json at 0.5 s: four beam elements, each drawn as a tube on the node at its end, with the 10 kg tip mass

   beam.json at 0.5 s: four beam elements, each drawn as a tube on the node at its end, with the 10 kg tip mass

:source:`examples/force_modules/beam.json`: a steel tube 2 m long,
clamped at one end with a 10 kg mass at the other, as a chain of four
elements. Released straight, it swings about its static deflection, 28 mm
at the tip (22 mm from the mass and 6 mm from its own weight), between zero
and twice that. Tests, including a cantilever that settles to
:math:`f l^3/(3 e i)` and :math:`f l^2/(2 e i)`: :source:`test/test_beam.cc`.
