.. title:: Spline

Spline
======

``"type": "spline"`` is a sliding spline, as in the slip spline of a driveshaft: it
carries torque between two halves that are free to slide along their axis, and the
more torque it carries, the harder it is to slide.

The two halves are two bodies, each with a frame on the axis, and their joints leave
them free to turn and slide on it: a prismatic and a revolute joint in series. The z
axis of the first frame is the axis :math:`\underline{a}`. The teeth are lumped, so
the module is not a contact of each tooth. For the teeth themselves, use the
:doc:`contact <contact>` of their geometry.

The module carries the torque itself rather than leaving it to a rigid joint, because
the friction depends on that torque and a force module cannot read the reaction of a
joint.

The torque
----------

With :math:`\phi` the angle of the second half relative to the first about the axis,
from the start, which counts past a full turn, and the ``backlash``
:math:`2\beta`, the whole angle from one flank to the other, the torque on the second
half is

.. math::

   \tau = -w\,(k\,d + c\,\dot{\phi}), \qquad
   d = \rho(\phi - \beta) - \rho(-\phi - \beta)

with the ``torsional_stiffness`` :math:`k` and the ``torsional_damping`` :math:`c`.
:math:`\rho` is a ramp: zero until the flanks meet and then the angle past them, with
its corner rounded over ``smoothing``. :math:`w` is its slope, so that the damper
comes in with the spring. The first half has the opposite torque.

The load on the teeth
---------------------

The torque, at the ``pitch_radius`` :math:`r` and ``pressure_angle`` :math:`\alpha`,
presses the flanks together with the normal force

.. math::

   n = \frac{|\tau|}{r\cos\alpha} + n_0

with the ``preload`` :math:`n_0`, of a tight fit or of the weight of the shaft.
Inside the backlash the torque is zero, so only the preload remains.

The friction
------------

The friction along the axis follows the LuGre model, as in
:doc:`contact <contact>`. The teeth are taken to touch through bristles of deflection
:math:`z`. With :math:`v` the rate of the second half relative to the first along the
axis, the friction force on the second half is

.. math::

   \dot{z} = v - \frac{\sigma_0 |v|}{g(v)} z, \qquad
   g(v) = \mu + (\mu_s - \mu)\,e^{-(v/v_b)^2}, \qquad
   f = -n\,(\sigma_0 z + \sigma_1 \dot{z}) - c_v v

with the ``friction`` :math:`\mu`, the ``static_friction`` :math:`\mu_s`, the
``stribeck_velocity`` :math:`v_b`, the ``bristle_stiffness`` :math:`\sigma_0` and
``bristle_damping`` :math:`\sigma_1`, both per unit of :math:`n`, and the
``viscous_friction`` :math:`c_v`. The first half has the opposite force.

So a spline under torque holds against any steady axial force up to
:math:`\mu_s n`, giving only by the bristles, and slides against :math:`\mu n`. With
no torque and no preload it slides freely.

The rate :math:`\sigma_0 |v| / g(v)` at which :math:`z` settles is large at speed, and
would force the integrator to small steps for as long as the spline slides. Above
:math:`1/t_b`, with the ``bristle_time`` :math:`t_b`, the deflection settles at that
rate instead, to the same sliding value:

.. math::

   \dot{z} = \frac{\mathrm{sgn}(v)\,g(v)/\sigma_0 - z}{t_b}

The two equations agree where they join, and the sliding friction is unchanged. Only
the moment of breaking loose takes a little longer.

Parameters
----------

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Key
     - Default
     - Meaning
   * - ``frames``
     - required
     - ``[[body, frame], [body, frame]]``: a frame on each half, on the axis; the
       first frame's z axis is the axis
   * - ``pitch_radius``
     - required
     - of the teeth
   * - ``torsional_stiffness``
     - required
     - torque per angle, past the backlash
   * - ``pressure_angle``
     - 0.5236
     - of the teeth, radians (30 degrees)
   * - ``torsional_damping``
     - 10
     - torque per rate of the angle, past the backlash. With 0 the halves ring on the
       teeth, with little to settle them
   * - ``backlash``
     - 0
     - the whole angle the halves turn freely
   * - ``smoothing``
     - 1e-4
     - width in angle of the edge of the backlash
   * - ``preload``
     - 0
     - normal force on the teeth without torque
   * - ``friction``
     - 0.1
     - coefficient of sliding friction of the teeth
   * - ``static_friction``
     - ``friction``
     - coefficient that holds the spline at rest
   * - ``viscous_friction``
     - 0
     - force per axial speed
   * - ``bristle_stiffness``
     - 2e4
     - per unit of the normal force, 1/m. The coefficient of friction over this is
       about how far the spline gives before it slides
   * - ``bristle_damping``
     - 5
     - per unit of the normal force, s/m. With 0 the bristles ring, with little to
       settle them
   * - ``bristle_time``
     - 1e-4
     - the shortest time the bristles take to reach their sliding deflection. With 0
       there is no limit: the plain LuGre model, whose time step falls with the
       sliding speed
   * - ``stribeck_velocity``
     - 0.01
     - speed over which static friction falls to sliding friction

.. code-block:: json

   {"type": "spline", "name": "slip_spline",
    "frames": [["shaft", "spline"], ["yoke", "spline"]],
    "pitch_radius": 0.02, "pressure_angle": 0.5236, "torsional_stiffness": 5e4,
    "backlash": 0.002, "friction": 0.1, "static_friction": 0.12}

The module acts along the axis and about it only. The side load and tilt of a spline
running at an angle are not modelled.

The outputs are ``angle``, ``torque``, ``normal_force``, ``friction_force`` and ``stick``,
the share of the static friction in use. The model
``examples/force_modules/slip_spline.json`` pushes a yoke along its spline while the
torque rises: it slides until the teeth load up, and then the friction stops it.
