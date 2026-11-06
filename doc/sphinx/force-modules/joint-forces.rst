.. title:: Joint forces

Joint forces
============

Revolute and prismatic joints have polynomial stiffness and damping of their
own (``stiffness_coefficients`` and ``damping_coefficients``). These modules add
what a polynomial cannot give: friction, and a force from a measured map. Both
act on a revolute or prismatic joint, where the force is a torque and the rate
an angular velocity on a revolute joint. For the spring and damper of a
spherical or float joint, put a :doc:`bushing <bushing>` across the joint.

Joint friction
--------------

``"type": "joint_friction"`` is Coulomb and viscous friction on a joint. With the
joint's rate :math:`v`, the force on the joint is

.. math::

   f = -\left( f_c \frac{v}{\sqrt{v^2 + v_s^2}} + c_v v \right)

with the ``friction`` :math:`f_c`, the ``slip_speed`` :math:`v_s` and the
``viscous_friction`` :math:`c_v`. This force is smooth, which is easy on the
integrator, but it is zero at zero rate, so the joint creeps under a steady
load at the rate where the friction balances it.

With ``bristle_stiffness`` :math:`\sigma_0` the friction holds at rest, by the
LuGre model :cite:`canudas1995`, as in :doc:`contact <contact>`. The friction
acts through bristles of deflection :math:`z`, a state:

.. math::

   \dot{z} = v - \frac{\sigma_0 |v|}{g(v)} z, \quad
   g(v) = f_c + (f_s - f_c) e^{-(v / v_b)^2}, \quad
   f = -\left( \sigma_0 z + \sigma_1 \dot{z} + c_v v \right)

with the ``static_friction`` :math:`f_s`, the ``stribeck_velocity`` :math:`v_b` and
the ``bristle_damping`` :math:`\sigma_1`. The joint holds against any steady load
up to :math:`f_s`. The bristles give a little first, and it moves further than
they bend, as they slip a little on the way.

A load that comes on all at once swings the joint on its bristles. If that swing
is faster than :math:`v_b`, the friction falls towards :math:`f_c` and the joint
breaks loose, though the load is below :math:`f_s`. Raise ``bristle_damping`` or
:math:`v_b` if the joint should hold such a load. With a ``bristle_damping`` of 0
the bristles ring, with little to settle them.

.. code:: javascript

   {"type": "joint_friction", "name": "hinge_friction", "joint": "hinge",
    "friction": 2.0, "static_friction": 3.0, "viscous_friction": 0.1,
    "bristle_stiffness": 5000}

.. shift-parameters:: shift/core/force_module/joint_force.cc
   :function: force_module::JointFriction::DeclareParameters

Outputs: ``friction``, the force on the joint, and with bristles ``stick``, the
share of the static friction in use.

Joint map
---------

``"type": "joint_map"`` is a force on a joint from a grid of measured values of
two variables, :math:`f(x, y)`. Each variable is the ``"coordinate"`` or the
``"rate"`` of a joint: ``x`` of ``joint``, and ``y`` of ``y_joint``, or of ``joint``
without it. The force is positive along the joint's coordinate. Between the
points the map is a smooth curve that does not overshoot them, along :math:`x`
and then along :math:`y`, and outside the grid it continues with the end slopes.

``forces`` holds a row of the forces along ``x_values`` for each of the
``y_values``. In ``map_file`` the same numbers are in a text file, separated by
blanks, commas or line breaks: the number of ``x`` values, the number of ``y``
values, the ``x`` values, the ``y`` values, and the forces. The file is found
next to the model file or in an include directory.

.. code:: javascript

   {"type": "joint_map", "name": "strut", "joint": "slide",
    "x": "coordinate", "y": "rate",
    "x_values": [-0.1, 0, 0.1], "y_values": [-1, 0, 1],
    "forces": [800, 900, 1000, 0, 0, 0, -1000, -1100, -1200]}

.. shift-parameters:: shift/core/force_module/joint_force.cc
   :function: force_module::JointMap::DeclareParameters

Output: ``force``.
