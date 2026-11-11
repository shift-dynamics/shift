.. title:: Tire

Tire
====

``"type": "tire"`` puts a tire on a wheel body. Each time the forces are
evaluated, the module finds where the wheel disc touches the ground, builds
the contact frame and the slip velocities of the contact point, evaluates
the tire model chosen by ``tire_type``, and applies its force and moment to
the wheel. Each tire model declares its own parameters, which are read from a block
named after it: ``"fiala"``, ``"pac2002"``, ``"bekker_wong"`` or ``"iowa"``.

The wheel spins about the y axis of the wheel body's origin frame. It rolls
on the active terrains of the model (see :ref:`model-file`, Terrain),
taken as the plane tangent to the terrain under the wheel; without terrain,
the ground is the plane z = 0. Forces are reported in SAE axes: x along the
heading, y to the right, z down.

Fiala
-----

``"tire_type": "fiala"``: the normal force is a spring and damper on the
penetration of the wheel disc into the ground. The longitudinal slip
:math:`\kappa` and the slip angle :math:`\alpha` are internal states that
follow the slip velocities with first order relaxation,

.. math::

   \sigma_x \dot\kappa = -(|v_x| \kappa + v_{sx}), \qquad
   \sigma_y \dot\alpha = v_{sy} - |v_x| \tan\alpha

so the forces build up over a rolling distance (the relaxation lengths
``x_relaxation_length`` and ``y_relaxation_length``) and stay smooth as the
wheel comes to rest. The friction coefficient falls from ``u_max`` to
``u_min`` with the combined slip; the longitudinal force is linear in
:math:`\kappa` up to the critical slip and saturates beyond it, and the
lateral force and aligning moment follow Fiala's expressions up to the
critical slip angle. All parameters have defaults for a passenger car tire.

.. code:: javascript

   {"type": "tire", "name": "left_front", "tire_type": "fiala",
    "wheel_body": "left_front_wheel",
    "fiala": {"unloaded_radius": 0.344, "vertical_stiffness": 310000}}

PAC2002
-------

``"tire_type": "pac2002"``: the Magic Formula 5.2 of Pacejka
:cite:`pacejka2012`, with its coefficients read from a ``.tir`` property
file (TNO/Adams format; its values are converted from the units of its
``[UNITS]`` section). Below 1 m/s the forces blend into a Dahl friction
model, whose bristle deflections are internal states, so that a vehicle
can stand still on a slope. With ``use_transient_slip`` the slips relax
over the relaxation lengths of the property file (two more states). The
slip velocities are those of the contact center moving with the wheel
carrier, as MF-Tyre defines them. ``mounted_side`` mirrors a tire mounted
on the side opposite to the property file's ``TYRESIDE``.

.. code:: javascript

   {"type": "tire", "name": "left_front", "tire_type": "pac2002",
    "wheel_body": "left_front_wheel",
    "pac2002": {"definition": "pac2002_235_60R15.tir",
                "use_transient_slip": true}}

Bekker-Wong
-----------

``"tire_type": "bekker_wong"``: a rigid wheel on soft soil such as sand, clay
or snow, from the pressure and shear of the soil under the tread. The soil is
given by ``soil``, either a name from the table of :ref:`soil` or its
properties. The wheel's radius and width are ``unloaded_radius`` and
``width``, as for Fiala.

The wheel presses into the soil by the depth of the lowest point of its rim
below the terrain. The tread touches the soil from the *entry angle*
:math:`\theta_1`, where it first meets it in front of the lowest point,
:math:`\cos\theta_1 = 1 - h/r` for the sinkage :math:`h`, to the exit
angle behind it (the soil springs back by the fraction ``rebound`` of the
sinkage; none by default). Bekker's pressure-sinkage law gives the pressure
at a point of the tread that is :math:`z` below the terrain, with the wheel's
width :math:`b` and the soil's moduli :math:`k_c`, :math:`k_\phi` and
exponent :math:`n` :cite:`wong1967,wong2001`,

.. math::

   \sigma = \left(\frac{k_c}{b} + k_\phi\right) z^n

It is greatest at the angle :math:`\theta_m = (c_1 + c_2 s)\,\theta_1`
(``max_stress_angle`` and ``max_stress_slip``; the slip :math:`s`) and falls
off to the exit angle behind it. The pressure acts along the radius.

The soil also shears the tread. Mohr and Coulomb's law gives the greatest
shear stress, :math:`c + \sigma\tan\phi`, with the cohesion :math:`c` and
the friction angle :math:`\phi`, and Janosi and Hanamoto's curve how much of
it the soil gives after the tread has slid by :math:`j` over it,

.. math::

   \tau = (c + \sigma\tan\phi)\left(1 - e^{-j/k_j}\right)

with the shear modulus :math:`k_j` of the soil. The shear acts opposite to the
sliding velocity of the tread over the soil, and :math:`j` is the distance the
tread has slid since it entered the soil. Both come from the full velocity of
the hub and the angular velocity of the wheel, so the model covers any slip,
braking, skidding and sliding sideways (the slip angle). At rest the shear
displacement goes to zero with the velocities, so that the force stays smooth.
Lugs of height ``grouser_height`` start the shear at the entry angle of their
tips.

*Camber.* If the axle is tilted out of the terrain plane, one side of the
wheel is deeper in the soil than the other, so the sinkage changes across the
width. The model divides the width into ``strips``, each with its own
sinkage, entry angle and shear, and adds up the forces. The pressure still
acts along the radius, which now leans out of the vertical: the soil pushes
the wheel towards the side its top leans to, with a force of the load times
:math:`\tan\gamma` for the camber :math:`\gamma`. A wheel on its edge,
with the middle of its rim above the terrain, is in contact by the lower edge
of its tread.

*Bulldozing.* A wheel that slips sideways, or is cambered, pushes the soil
with the sides of its tread. Where a side moves into the soil, the model adds
the earth pressure of the soil on it, at depth :math:`d` below the terrain
:cite:`jia2012`,

.. math::

   p = 2c\sqrt{\kappa_p} + w\,d\,\kappa_p, \qquad \kappa_p = \tan^2(45^\circ + \phi/2)

with the soil's unit weight :math:`w`, over the part of the side that is below
the terrain. It is fully applied above a sliding speed of ``slip_speed``, and
grows smoothly from zero below it, so there is no jump as a side starts to
push. ``"bulldozing": false`` leaves it out. The soil piled up in front of the
wheel, the shear on the sides, and the rut left for the next wheel are not
modelled.

The force is integrated over the tread with Gauss-Legendre quadrature of
``stations`` points on each side of the angle :math:`\theta_m`.

.. code:: javascript

   {"type": "tire", "name": "wheel", "tire_type": "bekker_wong",
    "wheel_body": "wheel",
    "bekker_wong": {"unloaded_radius": 0.08, "width": 0.032,
                    "soil": {"name": "tri1_lunar_simulant"}}}

Outputs, logged as ``<assembly>__<name>__<output>``: ``sinkage``, ``slip``,
``drawbar_pull`` (the force along the heading), ``load``, ``shear_torque``
(the torque the soil's shear puts on the wheel, which its driving torque has
to overcome), ``camber`` and ``side_force``.

Iowa
----

``"tire_type": "iowa"`` is a simple, fast tire for driving simulators and long runs over
rough ground, after the real-time tire of the National Advanced Driving Simulator of the
University of Iowa. It takes a few parameters, and what it adds is its contact with uneven
ground: Fiala and PAC2002 describe the forces of a tire in more detail, this one the
ground under it.

*The contact.* The wheel is a disc in its plane, and the module samples the terrain under
the lower half of it at ``segments`` + 1 points equally spaced along the heading. Each
slice between two points adds the area of the disc below the terrain, a trapezoid or, where
the rim crosses the terrain, a triangle. From the area :math:`a_s` follow the point of action
(the centroid of the area), the terrain normal (the mean of the normals over the area) and
the deflection :math:`\delta`, that of a wheel on flat ground with the same area:

.. math::

   r^2\bigl(\theta - \tfrac12\sin 2\theta\bigr) = a_s, \qquad \delta = r(1 - \cos\theta)

so the force, its point of action and the normal follow bumps, steps and slopes smaller
than the contact patch. Use an even number of ``segments``, so that a point is under the
hub; ``"segments": 0`` is the plane tangent to the terrain, as for the other models.

*The forces.* In SAE axes, with the hub's velocity :math:`(v_x, v_y, v_z)` in the contact
frame and the spin :math:`\omega` about the hub's y axis (negative rolling forward), the
load is a spring and a damper that do not pull, :math:`f_n = \max(k\delta + c\,v_z, 0)`.
The tread moves over the ground at :math:`v_p = v_x + \omega(r - \delta)`, which gives
the slip ratio and the slip angle, both fading out below ``low_speed`` :math:`v_\epsilon`,
so that the forces stay smooth as the vehicle comes to rest:

.. math::

   \kappa = -\frac{v_p}{\max(|\omega|(r - \delta), |v_x|, v_\epsilon)}\,
            \frac{|v_p|}{v_\epsilon + |v_p|}, \qquad
   \alpha = \arctan\frac{|v_y|}{\max(|v_x|, v_\epsilon)}\,
            \frac{v_y}{v_\epsilon + |v_y|}

The longitudinal force is :math:`\mathrm{sign}(\kappa)\,\mu_x(|\kappa|)\,f_n`, where the
friction rises in proportion to the slip to a peak of :math:`1.05\,\mu` at a slip of 0.2,
holds to 0.25, and falls to ``friction_coefficient`` :math:`\mu` at 0.5. The lateral force,
with the limit :math:`f_m = \mu f_n` and :math:`x = c_\alpha|\alpha|/f_m` for the cornering stiffness
:math:`c_\alpha` (``cornering_stiffness``), is

.. math::

   f_y = -\mathrm{sign}(\alpha)\,f_m\,(x - 0.32x^2 + 0.032x^3)

up to :math:`x = 2.5`, where it reaches :math:`f_m` with no slope, and
:math:`-\mathrm{sign}(\alpha)f_m` beyond. If the two together exceed :math:`f_m`, both are scaled
down to it (the friction ellipse). The rolling resistance is the force
:math:`-c_r f_n v_x/(v_\epsilon + |v_x|)` at the hub. The lateral force acts behind the
point of action by the trail ``aligning_coefficient``, which gives the aligning moment,
and the component of the load across a cambered wheel acts behind it likewise.

It has no relaxation lengths, camber thrust or soft soil (see Bekker-Wong above). Outputs:
``deflection``, ``slip``, ``slip_angle``, ``normal_force``, ``longitudinal_force``,
``lateral_force`` and ``aligning_moment``.

.. code:: javascript

   {"type": "tire", "name": "left_front", "tire_type": "iowa",
    "wheel_body": "left_front_wheel",
    "iowa": {"friction_coefficient": 0.8, "aligning_coefficient": 0.03,
             "segments": 20}}

Its parameters, in ``"iowa"``:

.. shift-parameters:: shift/core/force_module/tire/iowa.cc
   :function: Iowa::DeclareParameters

Parameters
----------

.. shift-parameters:: shift/core/force_module/tire/base.cc
   :function: tire::Base::DeclareParameters
   :exclude: name, type, active

``fiala``:

.. shift-parameters:: shift/core/force_module/tire/fiala.cc
   :function: Fiala::DeclareParameters

``pac2002``:

.. shift-parameters:: shift/core/force_module/tire/pac2002.cc
   :function: Pac2002::DeclareParameters

``bekker_wong``:

.. shift-parameters:: shift/core/force_module/tire/bekker_wong.cc
   :function: BekkerWong::DeclareParameters

Examples
--------

.. figure:: ../images/examples/tire_fiala.jpg
   :width: 50%
   :alt: tire_fiala.json: a single wheel with the Fiala tire

   tire_fiala.json: a single wheel with the Fiala tire

.. figure:: ../images/examples/vehicle_terrain.jpg
   :width: 80%
   :alt: vehicle_terrain.json at 3 s: the passenger car on an elevation map

   vehicle_terrain.json at 3 s: the passenger car on an elevation map

* :source:`examples/vehicle/tire_fiala.json`,
  :source:`examples/vehicle/tire_pac2002.json` and
  :source:`examples/vehicle/tire_iowa.json`: a single wheel with each
  tire model; :source:`examples/vehicle/tire_fiala_rig.json` holds the
  wheel on a test rig.
* :source:`examples/vehicle/vehicle_control.json`: a passenger car with
  Fiala tires, driven by PID speed control; ``vehicle_control_5mps.json``,
  ``vehicle_control_15mps.json`` and ``vehicle_control_magic.json`` (PAC2002
  tires) vary it, and :source:`examples/vehicle/vehicle_terrain.json` drives
  it over a rally stage elevation map.
* :tree:`examples/vehicle/fiala`: steady state slip sweeps of the Fiala
  tire, with scripts to run and plot them.
* :source:`examples/terramechanics/wheel_test_bed.json`: a single-wheel test
  bed with a Bekker-Wong wheel on soil, as in the tests of rovers' wheels: the
  carriage is held at a constant speed and the wheel at a constant spin, by
  PID controllers, so the wheel slips by 20%, and the drawbar pull, load and
  torque are logged. The wheel and soil are those of Kumar and
  Jayalekshmi :cite:`kumar2019`.
  :source:`examples/terramechanics/wheel_test_bed_camber.json` tilts the
  wheel by 10 degrees, and the side force is the camber thrust.
* Tests: :source:`test/test_tire_pac2002.cc` and
  :source:`test/terramechanics/test_terramechanics.cc`.
