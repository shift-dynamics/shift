.. title:: Terrain contact

Terrain contact
===============

``"type": "terrain_contact"`` is the contact of a body with the terrain at a set
of points on it, for the skids, feet and corners of a machine, or a body that
falls on an elevation map. The :doc:`contact <contact>` module works on
collision geometry and knows no terrain. This module tests each point on its
own against the surface under it, as a tire does: the active terrains of the
model, or the ground :math:`z = 0` without any.

At a point below the surface by the depth :math:`d`, along the normal
:math:`\underline{n}` into the terrain, moving with the velocity
:math:`\underline{v}`, the normal force follows Hunt and Crossley:

.. math::

   f_n = \max\left(0, \; k d^e (1 + a \dot{d})\right), \quad
   \dot{d} = \underline{v} \cdot \underline{n}

with the ``stiffness`` :math:`k`, the ``exponent`` :math:`e` and the ``damping``
:math:`a`. It starts from zero as the contact starts, and it only pushes.
Friction opposes the sliding velocity :math:`\underline{v}_t` in the plane of the
surface, :math:`-\mu f_n \underline{v}_t / \sqrt{|\underline{v}_t|^2 + v_s^2}`, with
the ``friction`` :math:`\mu` and the ``slip_speed`` :math:`v_s`. It is smooth through
zero, so a body creeps slowly down a slope instead of holding on it.

A point only sees the surface under it. A body with a few points can sink a
bump between them, so space the points closer than the bumps of the terrain.

.. code:: javascript

   {"type": "terrain_contact", "name": "skids", "body": "sled",
    "points": [1, 0.5, 0.2, 1, -0.5, 0.2, -1, 0.5, 0.2, -1, -0.5, 0.2],
    "stiffness": 2e5, "damping": 1.0, "friction": 0.6}

``points`` holds x, y, z of each point, in the body's coordinates.

.. shift-parameters:: shift/core/force_module/terrain_contact.cc
   :function: force_module::TerrainContact::DeclareParameters

Outputs: ``normal_force`` (the total), ``contacts`` (the points in contact) and
``depth`` (the deepest).
