.. title:: Soil tool

Soil tool
=========

``"type": "soil_tool"`` is the force of the soil on a wide blade cutting it, such
as a dozer blade or a grader's moldboard, and the pile the blade pushes. The
soil is that of :doc:`soil <soil>`, whose ``wall_friction`` is the friction
between soil and blade. The blade's lift, pitch and tilt are the model's own
joints and controllers.

The blade is a flat rectangle on a body: its cutting edge through ``tip`` along
``edge``, ``width`` long, rising along ``face`` for its ``height``, and working
towards ``front``, all in the body's coordinates. The ground is the level plane
:math:`z` = ``ground``, with the soil below it (z down).

The cutting force
-----------------

The force per width on a blade cutting to the depth :math:`d` follows Reece's
fundamental equation of earth moving :cite:`reece1964`:

.. math::

   f = w d^2 n_w + c d n_c + c_a d n_a + q n_q

with the unit weight :math:`w`, the cohesion :math:`c`, the ``adhesion``
:math:`c_a` of the soil to the blade, and the weight :math:`q` per width of the
pile. A wedge of soil fails along a plane at the angle :math:`\beta` to the
ground. The blade is at the angle :math:`\rho` to the ground: 90° is vertical,
and below that its top trails its edge. With the friction angle :math:`\phi` of
the soil, :math:`\delta` between soil and blade, and
:math:`s = \sin(\delta + \rho + \phi + \beta)`, the factors are those of McKyes
:cite:`mckyes1985`, in the form of :cite:`wagner2025`:

.. math::

   n_w = \frac{(\cot\rho + \cot\beta)\sin(\phi + \beta)}{2 s}, \quad
   n_c = \frac{\cos\phi}{s \sin\beta}, \quad
   n_a = \frac{-\cos(\rho + \phi + \beta)}{s \sin\rho}, \quad
   n_q = \frac{\sin(\phi + \beta)}{s}

:math:`\beta` is the angle of least :math:`n_w`, found at each evaluation. For a
vertical blade this is the passive earth pressure of :doc:`soil <soil>`:
:math:`2 n_w` is Coulomb's coefficient along the line of the thrust, Rankine's
without friction, and then :math:`n_c = 2\sqrt{\kappa_p}`.

The force on the blade, of the width :math:`b`, is

.. math::

   m b f \left( -\sin(\rho + \delta) \, \underline{e} -
     \cos(\rho + \delta) \, \underline{u} \right)

with :math:`\underline{e}` the level direction of ``front`` and :math:`\underline{u}`
up: the draft against the motion, and a pull into the soil for a blade below
:math:`90^\circ - \delta`. It acts on the face, a third of the depth above the
edge. It builds up with the speed :math:`v` of the edge along
:math:`\underline{e}`, :math:`m = \max(0, v)/\sqrt{v^2 + v_s^2}`, with the
``slip_speed`` :math:`v_s`, so a blade backing up or at rest has none.

The pile
--------

The pile has the area :math:`a` per width, a state. It grows with the soil cut.
Past the area of a heap of the blade's height at the angle of repose
:math:`\phi`, :math:`a_m = h^2 / (2 \tan\phi)`, it spills over the blade and past
its ends, and it is left behind when the blade backs up or lifts:

.. math::

   \dot{a} = m v d - \max(0, a - a_m)/t_s - (1 - m) a / t_s

with the ``spill_time`` :math:`t_s`, and :math:`q = w a`.

Limits
------

The ground stays level: the blade leaves no cut behind it. The model does not
include the inertia of the soil, the soil moving along an angled blade, or the
resistance to a blade pressed down at rest. It is for wide blades: under a
narrow tine or a ripper the soil also fails to the sides, which adds to the
force.

.. code:: javascript

   {"type": "soil_tool", "name": "blade",
    "soil": {"name": "sandy_loam", "unit_weight": 16000, "wall_friction": 0.25},
    "body": "dozer", "tip": [2.5, 0, 1.15], "face": [-0.17, 0, -0.985],
    "edge": [0, 1, 0], "front": [1, 0, 0], "width": 3.2, "height": 1.1}

.. shift-parameters:: shift/core/force_module/soil_tool.cc
   :function: force_module::SoilTool::DeclareParameters

Outputs: ``depth``, ``rake``, ``failure_angle``, ``pile``, ``draft`` and ``lift``.
``examples/force_modules/dozer_blade.json`` pushes a blade through sandy loam at
1.5 m/s until its pile is full.
