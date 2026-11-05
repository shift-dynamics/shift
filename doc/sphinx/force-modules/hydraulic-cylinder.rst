.. title:: Hydraulic Cylinder

Hydraulic Cylinder
==================

``"type": "hydraulic_cylinder"`` is a double-acting hydraulic cylinder on a
prismatic joint between its barrel and its rod, fed by a four-way
proportional valve.

Pressures
---------

The pressures in the head side (the full bore) and the rod side (the
annulus) are two internal states. Each changes with the flow into its side
less the rate at which its volume grows, over the side's compliance
:cite:`merritt1967`:

.. math::

   \dot p_h = \frac{\beta}{V_h}\left(Q_h - A_h \dot x - Q_l + Q_{rh}\right),
   \qquad
   \dot p_r = \frac{\beta}{V_r}\left(Q_r + A_r \dot x + Q_l + Q_{rr}\right)

where :math:`x` is the extension from fully retracted, :math:`A_h` and
:math:`A_r` the head and annulus areas, :math:`V_h = V_{h0} + A_h x` and
:math:`V_r = V_{r0} + A_r(s - x)` the volumes of oil on each side for the
stroke :math:`s`, :math:`\beta` the bulk modulus of the oil, and
:math:`Q_l` the leakage past the piston.

Valve
-----

The valve opening :math:`u`, from -1 to 1, connects the supply to the head
side and the rod side to the tank for :math:`u > 0` (extending), and the
other way for :math:`u < 0`. The flow through each open edge is that of an
orifice,

.. math::

   Q = |u|\, Q_\text{rated} \sqrt{\Delta p / \Delta p_\text{rated}}

with the square root made smooth near zero pressure drop, where the flow
becomes laminar. A relief valve on each side lets oil out to the tank above
the relief pressure, and a makeup (anti-cavitation) valve lets oil in from
the tank below the tank pressure; both open smoothly over a small band of
pressure. ``command`` gives the opening against time as a table
:math:`[t_0, u_0, t_1, u_1, \dots]`, a monotone cubic through the points
:cite:`fritsch1980` that holds the end values.

Force
-----

The force of the cylinder along its extension,

.. math::

   F = p_h A_h - p_r A_r - c\,\dot x - F_\text{stop}

is applied to the joint. :math:`F_\text{stop}` holds the piston at the ends
of its stroke: :math:`k\,\delta (1 + e\,\dot\delta)` for a penetration
:math:`\delta` past an end, never pulling (Hunt and Crossley's contact force
:cite:`hunt1975`).

Starting in equilibrium
-----------------------

With ``"hold": true`` the cylinder starts with the pressures that hold the
model still, in place of ``initial_pressures``. Before the simulation
starts, the forces of all holding cylinders that give their joints zero
acceleration are found together from the equations of motion; each
cylinder's force is then made by the pressure on one side, with the other
at the tank pressure. Cylinders that share a load, such as a pair side by
side, share it equally.

JSON
----

.. code:: javascript

   {
     // boom cylinder: 140 mm bore, 100 mm rod
     "type": "hydraulic_cylinder", "name": "boom_cylinder_1",
     "joint": "L_boom_cylinder_1_to_boom_rod_1",
     "retracted_position": 1.95, "stroke": 1.3,
     "bore_diameter": 0.14, "rod_diameter": 0.1,
     "supply_pressure": 30e6, "tank_pressure": 0.5e6,
     "relief_pressure": 34e6, "rated_pressure_drop": 10e6,
     "rated_flow": 0.0044, "friction": 2e4,
     "end_stop_stiffness": 5e7, "end_stop_damping": 5,
     "hold": true,
     // valve opening against time: [t0, u0, t1, u1, ...]
     "command": [0, 0, 0.2, 0, 0.5, -0.4, 1.8, -0.4, 2.1, 0]
   }

.. shift-parameters:: shift/core/force_module/hydraulic_cylinder.cc
   :exclude: name, type, active

Pressures, flows and lengths are in the model's units: pascals, cubic
meters per second and meters for SI.

Outputs
-------

``head_pressure``, ``rod_pressure``, ``force`` (along the extension),
``extension``, ``opening`` (of the valve), ``head_flow`` and ``rod_flow``
(through the valve into each side).

Example
-------

.. figure:: ../images/examples/excavator.jpg
   :width: 80%
   :alt: excavator.json at the start: held by its cylinders, with the bucket above the terrain

   excavator.json at the start: held by its cylinders, with the bucket above the terrain

:source:`examples/excavator/excavator.json`: a 30 t class excavator (the
geometry and weights of a Cat 330) with two boom cylinders, a stick
cylinder and a bucket cylinder, all starting held. Their valve commands
lower the boom, dig with the stick and bucket, and raise the load; the CSV
and HDF5 logs record each cylinder's pressures, force and flows.
:source:`examples/excavator/excavator_simple.json` is the linkage alone,
without cylinders and with placeholder masses, moving freely under
gravity. Tests: :source:`test/test_hydraulic_cylinder.cc`.
