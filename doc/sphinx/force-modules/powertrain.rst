.. title:: Powertrain

Powertrain
==========

``"type": "powertrain"`` is the drive of a vehicle from its engine to its wheels:
an engine with its governor, a torque converter or a clutch, a transmission with gear
shifts, driveshafts, final drives and differentials, and brakes. It turns the
driver's inputs (throttle, gear, lockup, transfer case, brake pressure) into torques
on the wheels, and the speeds of the wheels back into the load on the engine.

The powertrain is a chain of rotating inertias joined by elements that carry torque.
Its states are the speed of the engine, the speed of the transmission's input
shaft (with the turbine of the converter), the speed of its output shaft, the twist
of the driveshaft of each axle, and the engagement of the clutch of each gear. The
wheels are the revolute joints of the model, whose speeds the module reads and to
which it applies the torques of the differentials: so the powertrain couples to the
wheels, tires and chassis of a vehicle, as the tire models do. The inputs are numbers
that can be set as the model runs, by a schedule controller (see :ref:`schedule`).

.. code:: javascript

   {"type": "powertrain", "name": "drive",
    "engine": {"inertia": 1.5, "idle_speed": 80, "high_idle_speed": 230,
               "torque_curve": [80, 500, 150, 900, 200, 800, 230, 0],
               "friction_curve": [80, -60, 230, -150]},
    "coupling": {"type": "converter",
                 "capacity_curve": [0, 10, 0.8, 13, 0.95, 30, 1.0, 300],
                 "torque_ratio_curve": [0, 2.2, 0.85, 1.0, 1.0, 1.0],
                 "capacity": 1200},
    "transmission": {"ratios": [3.5, 2.0, 1.0], "reverse_ratio": -3.5,
                     "gear": 1, "clutch_capacity": 4000},
    "axles": [{"name": "rear", "left": "left_wheel", "right": "right_wheel",
               "direction": -1, "final_drive": 4.0, "shaft_stiffness": 2e4,
               "shaft_damping": 100}],
    "brakes": [{"name": "left", "joint": "left_wheel", "direction": -1,
                "capacity": 4000}]}

The engine
----------

The torque of the engine at the speed :math:`\omega` is the net torque at full fuel,
:math:`\tau_\mathrm{full}(\omega)`, given by ``torque_curve`` as pairs of a speed and a
torque, blended with the torque of the closed throttle (the friction and pumping of
the engine, ``friction_curve``, negative) by the *fuel fraction* :math:`f`:

.. math::

   \tau_e = f\,\tau_\mathrm{full}(\omega) + (1 - f)\,\tau_\mathrm{friction}(\omega)

The fuel fraction is the throttle (0 to 1) or what the governor asks for, whichever is
more, and it is cut off above the *high idle speed*:

.. math::

   f = \max\bigl(\text{throttle},\; g\,(\omega_\mathrm{idle} - \omega)\bigr)\,\chi(\omega)

The governor, with the gain :math:`g` (``governor_gain``), opens the throttle as the
engine slows below ``idle_speed``, so that it holds near idle, a little below it (the
*droop*: with a gain of 0.1 per rad/s and a friction torque of 60 N m against a full
torque of 500 N m, the engine idles 1.07 rad/s below ``idle_speed``). The factor
:math:`\chi` falls smoothly from 1 to 0 over ``cut_range`` below ``high_idle_speed``. The
engine's inertia accelerates it under the difference between its torque and the
torque of the coupling.

The coupling
------------

A *torque converter* (``"type": "converter"``) joins the engine, as the pump, to the
transmission's input shaft, as the turbine, through oil. With the speed ratio
:math:`s = \omega_t/\omega_e` of turbine to pump, the *capacity curve*
:math:`k_c(s)` (pairs of :math:`s` and :math:`k_c`, in rad/s per square root of N m)
gives the torque that the pump takes, and the *torque ratio curve*
:math:`g_t(s)` that the turbine gives:

.. math::

   \tau_p = \frac{\omega_e|\omega_e|}{k_c(s)^2}, \qquad \tau_t = g_t(s)\,\tau_p

A stalled turbine (:math:`s = 0`) has the greatest torque ratio, which multiplies the
torque of the engine as a vehicle starts, and :math:`g_t` falls to 1 at the coupling
point, where the converter acts as a fluid coupling. A converter cannot give out more
power than it takes in, :math:`g_t(s)\,s \le 1`, which the module checks when it loads;
and the capacity :math:`k_c` of a real converter rises steeply as :math:`s \to 1`, so that
the pump's torque vanishes when the turbine reaches its speed.

A *lockup clutch* in parallel, of capacity ``capacity`` and engagement ``engagement``
(0 to 1), adds the torque

.. math::

   \tau_\mathrm{lockup} = \text{capacity}\cdot\text{engagement}\cdot\tanh\frac{\omega_e - \omega_t}{\omega_s}

between engine and turbine, which locks them together as it is applied. A *clutch*
(``"type": "clutch"``) has only this second part and no oil coupling, as the starting
clutch of a manual transmission.

The transmission
----------------

Each gear :math:`i` has a clutch between the input shaft and the output shaft, which
transmits the torque

.. math::

   \tau_i = e_i\,c_\mathrm{max}\,\tanh\frac{\omega_t - r_i\,\omega_o}{\omega_s}

for the gear's ratio :math:`r_i` (input speed over output speed), the capacity
:math:`c_\mathrm{max}` (``clutch_capacity``), the speeds of the input and output shafts, and the
*engagement* :math:`e_i` of the clutch. The input shaft takes the torque :math:`-\tau_i`,
and the output shaft gets :math:`r_i \tau_i`. The gear selected by ``gear`` (1 to the
number of ratios, -1 for ``reverse_ratio``, 0 for neutral) is engaged and the others
are released: each :math:`e_i` rises or falls as a first order lag over
``shift_time``. In a shift the old gear's clutch releases while the new one applies,
as in a power-shift transmission, so the torque is handed over smoothly, and the
clutches slip while they do.

A gear holds its ratio as closely as its clutch has torque to spare: the smoothed
friction of the clutch carries a torque :math:`\tau` with a slip of
:math:`\omega_s\,\mathrm{atanh}(\tau/c_\mathrm{max})`, which is small when the capacity :math:`c_\mathrm{max}` is
generous.

The losses
~~~~~~~~~~

The losses of a gearbox are of two kinds: those that do not depend on the load, the
churning of the oil and the drag of seals and open clutches, and those of the teeth,
which lose a fraction of the power they carry. ``spin_loss`` is a viscous loss and
``drag_torque`` a constant one on the output shaft. ``efficiencies`` gives the
efficiency :math:`\eta_i` of the teeth of each gear, one for each with the reverse last
(1 by default), and the gear takes from the output shaft the torque

.. math::

   \tau_{\mathrm{loss},i} = (1 - \eta_i)\,|r_i\tau_i|\,\tanh\frac{\omega_o}{\omega_s}

which opposes the shaft's speed whichever way the power flows: driving, or with the
wheels driving the engine. An axle has the same pair, ``efficiency`` and
``drag_torque``, for its final drive, and takes
:math:`\bigl((1 - \eta_f)\,|r_f\tau_s| + \tau_d\bigr)\tanh(\omega_c/\omega_s)` from the
torque of the carrier. The output ``gear_loss`` is the torque that the transmission's
teeth and drag take.

Clutch hydraulics
~~~~~~~~~~~~~~~~~

Without ``clutch_hydraulics``, the engagement :math:`e_i` of a clutch is the first order
lag above. With it, :math:`e_i` is the opening of the gear's valve, and the clutch has
to be filled with oil before it carries torque, so that a shift has a delay and a
clutch is applied at the rate the oil can reach it (:cite:`merritt1967` for the flow of
an orifice). The valve feeds the clutch from the pressure :math:`e_i p_s`, the
``supply_pressure`` :math:`p_s`, through an orifice of area :math:`a_o`
(``orifice_area``, or ``drain_orifice_area`` as the oil leaves). The state of a clutch is
the volume :math:`v` of oil that has entered it,

.. math::

   \dot v = c_d a_o \sqrt{\frac{2}{\rho}}\,
     \frac{e_i p_s - p}{\bigl((e_i p_s - p)^2 + p_t^2\bigr)^{1/4}}

the orifice equation, which goes smoothly through zero below the
``transition_pressure`` :math:`p_t`, with the ``discharge_coefficient`` :math:`c_d` and
``density`` :math:`\rho`. The piston is light and quickly balances the oil against its
return spring, so the pressure is a function of the volume:

.. math::

   p(v) = \begin{cases}
     f_0/a + v/c_o & v < 0 \quad\text{(on its stop, the oil decompressing)} \\
     (f_0 + k_r v/a)/a & 0 \le v \le a l \quad\text{(stroking the piston)} \\
     p_k + (v - a l)/c_o & v > a l \quad\text{(against the plates)}
   \end{cases}

with the ``piston_area`` :math:`a`, ``piston_stroke`` :math:`l` (the travel until the
plates touch), the return spring's ``spring_preload`` :math:`f_0` and ``spring_rate``
:math:`k_r`, the *kiss pressure* :math:`p_k = (f_0 + k_r l)/a`, and the compliance of the
oil :math:`c_o` = ``chamber_volume`` / ``bulk_modulus``. Taking the pressure from the
volume, in place of a state for the pressure and one for the piston, leaves out the
piston's own fast motion, which a variable step integrator would have to follow.

The plates are pressed with the force :math:`a(p - p_k)`, so the engagement that takes
the place of :math:`e_i` in the clutch's torque is
:math:`(p - p_k)/(p_s - p_k)`, between 0 and 1, and ``clutch_capacity`` is the capacity at
the supply pressure. The fill takes about :math:`a l / q` for the flow :math:`q` of the
orifice at the drop :math:`p_s - p_k`. The outputs ``clutch_1_pressure``,
``clutch_2_pressure``, ... give the pressures, the reverse's last.

The axles
---------

Each entry of ``axles`` is a propshaft from the transmission's output to the axle,
the axle's final drive and differential, and its two wheels, the revolute joints
``left`` and ``right``, which turn positively going forward when ``direction`` is 1,
and negatively when it is -1 (as the wheels of the vehicle examples). The propshaft
has the torsional stiffness and damping ``shaft_stiffness`` and ``shaft_damping``,
and the ratio ``transfer_ratio`` of the transfer case (its speed over the output
shaft's); the ratio of the final drive is ``final_drive``. The twist :math:`\theta` of
the propshaft changes at the rate

.. math::

   \dot\theta = r_t\,\omega_o - r_f\,\omega_c, \qquad \omega_c = \tfrac12(\omega_l + \omega_r)

with the speeds :math:`\omega_l`, :math:`\omega_r` of the wheels, and its torque is
:math:`\tau_s = \text{engaged}\,(k\theta + c\,\dot\theta)`. It acts on the output shaft
as :math:`-r_t \tau_s`, and on the differential's carrier as :math:`r_f \tau_s`, which the
differential divides between the wheels:

``open``
   half to each wheel, so a wheel that spins freely takes no more torque than the
   other can give.

``locked``
   half to each wheel, and a viscous coupling ``lock_damping`` that holds the wheels
   to one speed, so that the torque goes to the wheel that needs it.

``limited_slip``
   half to each wheel, and a torque :math:`(b + \beta|\tau_\mathrm{carrier}|)
   \tanh((\omega_l - \omega_r)/\omega_s)` taken from the faster wheel and given to the
   slower, from the preload :math:`b` (``bias_torque``) of the clutches and in
   proportion :math:`\beta` (``bias_ratio``) to the torque of the carrier.

``engaged`` (0 to 1, an input) connects the axle to the transfer case, or disconnects
it, as in the shift of a transfer case from two wheel drive to four: the propshaft
carries nothing at 0. Several axles share the transmission's output through their
propshafts: the transfer case is locked, and the sharing of the torque follows from
the stiffnesses and the speeds of the propshafts, so that the axles wind up against
each other when their wheels turn at different speeds.

The center differential
-----------------------

``center_differential`` puts a differential between the transmission's output and the
axles, as in permanent all wheel drive, in place of the locked transfer case. It divides
the torque in a fixed ratio between two outputs, and each axle says which one drives it
with its ``center_output``, 1 or 2. With ``split`` :math:`s`, the share of output 1, and
the speed :math:`\omega_d` of the differential's gears (a state of the module, with their
``inertia`` :math:`i_d`), the outputs turn at

.. math::

   \omega_1 = \omega_o + (1 - s)\,\omega_d, \qquad \omega_2 = \omega_o - s\,\omega_d

which take the place of :math:`\omega_o` in the twist of the propshafts, so the output
shaft turns at the mean of the two weighted by the split. With :math:`\tau_1` and
:math:`\tau_2` the torques of the propshafts on each output, the output shaft takes
:math:`\tau_1 + \tau_2`, and

.. math::

   i_d\,\dot\omega_d = s\,\tau_2 - (1 - s)\,\tau_1 - \tau_\mathrm{lock}

which brings the torques to :math:`\tau_1/\tau_2 = s/(1 - s)`. The small inertia is what
lets the module find that balance without solving for it; it must be small next to the
other inertias of the drive, but the smaller it is, the faster the differential's own
vibration on the propshafts, and the smaller the integrator's steps.

``type`` is that of an axle's differential: ``open`` (:math:`\tau_\mathrm{lock} = 0`),
``locked`` (``lock_damping`` :math:`\omega_d`) or ``limited_slip``
(:math:`(b + \beta|\tau_1 + \tau_2|)\tanh(\omega_d/\omega_s)`, with ``bias_torque``,
``bias_ratio`` and ``slip_speed``). The output ``center_speed`` is :math:`\omega_d`.

The brakes
----------

A brake of capacity ``capacity`` on a joint, in ``brakes``, has the torque of smoothed
Coulomb friction against the joint's speed :math:`\omega`,

.. math::

   \tau = -\text{capacity}\cdot a\cdot\tanh\frac{\omega}{\omega_s}

where :math:`a` is the pressure (``"applied_by": "pressure"``, the default) or 1 minus
the pressure (``"applied_by": "spring"``: the spring applies the brake, and the
pressure releases it, as a parking brake). The pressure is an input between 0 and 1.

The equations of motion
-----------------------

Putting the parts together, the module integrates these equations, with the torques
defined above. The engine, with the inertia :math:`i_e` of ``engine``, is slowed by
the coupling:

.. math::

   i_e\,\dot\omega_e = \tau_e - \tau_p - \tau_\mathrm{lockup}

The input shaft, with the turbine, has the inertia :math:`i_t` (``input_inertia``),
and is driven by the turbine and loaded by the clutches of the gears:

.. math::

   i_t\,\dot\omega_t = \tau_t + \tau_\mathrm{lockup} - \sum_i \tau_i

The output shaft, with the inertia :math:`i_o` (``output_inertia``), is driven by the
clutches through the ratios, loaded by the propshafts of the axles, and slowed by the
loss ``spin_loss`` :math:`c_o`:

.. math::

   i_o\,\dot\omega_o = \sum_i r_i \tau_i - \sum_a r_{t,a}\,\tau_{s,a} - c_o\,\omega_o

and each propshaft twist :math:`\theta_a` and each clutch engagement :math:`e_i` follow
their own equations (:math:`\dot\theta_a` above, and
:math:`\dot e_i = (\text{target}_i - e_i)\,/\,(\text{shift\_time}/3)`, the target being 1 for
the selected gear and 0 for the others). The wheels' own equations are those of the
model: the module adds to each wheel joint the torque of its differential, and to the
joints of the brakes the torque of the brakes, as joint forces, and reads the wheels'
speeds from the joints' velocities. Torques are positive when they speed up the shaft
that they act on, and the signs of the speeds follow ``direction``.

The module starts with the engine at ``idle_speed``, the input and output shafts at
rest, the propshafts untwisted, and the clutch of the selected gear (``gear`` at the
start) engaged, so that a vehicle that starts at rest is in gear with the engine
idling and the converter stalled, and creeps. A vehicle that is moving at the start
settles over a fraction of a second as the clutches and propshafts take up the
difference of speeds.

The power follows from the equations: the engine's power less its inertia's goes to the
coupling, the converter gives out at most what it takes in (:math:`g_t(s)\,s \le 1`), the
clutches dissipate the product of their torque and slip, and what reaches the output
shaft goes through the propshafts to the wheels, less the loss and the work of the
differentials' clutches.

Friction that holds
-------------------

The brakes, the lockup clutch or clutch of the coupling, the clutches of the gears
and the clutches of a limited slip differential use smoothed Coulomb friction by
default. It gives the torque of its load only at a slip, :math:`\omega_s\,
\mathrm{atanh}(\tau/c_\mathrm{max})` for a torque :math:`\tau` of a capacity :math:`c_\mathrm{max}`, so a brake lets
a vehicle on a slope creep, and a locked clutch always slips a little.

Each of these elements has the option of the bristle model of friction
(:cite:`canudas1995`, the LuGre model, as the ``stick_slip`` of the :doc:`contact`
module), switched on by its ``bristle_stiffness`` :math:`\sigma_0` (in 1/rad; zero, the
default, keeps the smoothed friction). The friction torque of an element of capacity
:math:`c_e` (the capacity times its engagement) between parts with the relative speed
:math:`u` is then

.. math::

   \tau = c_e\,(\sigma_0 z + \sigma_1\dot z), \qquad
   \dot z = u - \sigma_0\,\frac{|u|}{g(|u|)}\,z, \qquad
   g(|u|) = 1 + (r_s - 1)\,e^{-(|u|/v_s)^2}

with the bristle deflection :math:`z` (an angle, a state of the module for each such
element), the damping :math:`\sigma_1` (``bristle_damping``, in s/rad), the static
friction over the Coulomb friction :math:`r_s` (``static_ratio``, at least 1) and the
speed :math:`v_s` (``stribeck_speed``) over which it falls from the first to the
second. At rest the deflection settles where :math:`\sigma_0 z` carries the load, so
the element holds: a brake with the load a fraction :math:`\eta` of its capacity turns
by :math:`-\ln(1-\eta)/\sigma_0` as the bristles deflect, and no more (the *pre-sliding
displacement* of the model), up to :math:`r_s c_e`, and slides beyond that with the
Coulomb friction :math:`c_e`. A locked clutch carries its torque without slip, and a
limited slip differential locks while the torque it has to carry is within its
capacity.

The bristles are elastic, and without damping they ring. Give a ``bristle_damping``
:math:`\sigma_1` that makes the damping torque :math:`c_e\sigma_1` the critical damping
:math:`2\sqrt{c_e\sigma_0 i}` of the bristles' spring, for the inertia :math:`i` that they
act on, that is :math:`\sigma_1 = 2\sqrt{\sigma_0 i/c_e}`. They are also stiff, relaxing
within :math:`1/(\sigma_0|u|)` at the slip speed :math:`|u|`, which the variable step
integrator resolves, so a stiffer :math:`\sigma_0` holds more closely and takes smaller
steps. A body that is loaded suddenly moves :math:`-\ln(1-\eta)/\sigma_0` before the
friction carries the load: if it gains more speed in that than ``stribeck_speed``, the
friction has fallen from the static value, and holds no more than the Coulomb value.

Limits
------

The module does not have: the hydraulics of the lockup clutch and the brakes, whose
engagement and pressure are inputs; the limit on the flow of a pump shared by the
clutches; the bristle model on the center differential; and the inertia of the oil in the
converter.

Inputs and outputs
------------------

The inputs that can be changed as the model runs are ``engine/throttle``,
``coupling/engagement``, ``transmission/gear``, an axle's ``engaged`` (and
``bias_torque`` and ``bias_ratio``) and a brake's ``pressure``. A schedule's target is
the path to it, such as ``"force_modules/drive/transmission/gear"`` or
``"force_modules/drive/axles/front/engaged"``.

Outputs, logged as ``<assembly>__<name>__<output>``: ``engine_speed``, ``engine_torque``, ``fuel``,
``turbine_speed``, ``output_speed``, ``converter_torque_ratio``, ``gear_ratio`` (the
ratios of the gears weighted by their engagement), ``gear_loss``, for each axle
``<axle>_shaft_torque``, ``<axle>_left_torque`` and ``<axle>_right_torque``, and
``center_speed`` and ``clutch_<i>_pressure`` where those parts are given.

Each of the parts of the module:

.. shift-parameters:: shift/core/force_module/powertrain.cc
   :function: force_module::Powertrain::DeclareParameters
   :exclude: name, type, active

``engine``:

.. shift-parameters:: shift/core/force_module/powertrain.cc
   :function: PowertrainEngine::DeclareParameters

``coupling``:

.. shift-parameters:: shift/core/force_module/powertrain.cc
   :function: PowertrainCoupling::DeclareParameters

``transmission``:

.. shift-parameters:: shift/core/force_module/powertrain.cc
   :function: PowertrainTransmission::DeclareParameters

``center_differential``:

.. shift-parameters:: shift/core/force_module/powertrain.cc
   :function: PowertrainCenterDifferential::DeclareParameters

``clutch_hydraulics``:

.. shift-parameters:: shift/core/force_module/powertrain.cc
   :function: PowertrainClutchHydraulics::DeclareParameters

Each entry of ``axles``:

.. shift-parameters:: shift/core/force_module/powertrain.cc
   :function: PowertrainAxle::DeclareParameters

Each entry of ``brakes``:

.. shift-parameters:: shift/core/force_module/powertrain.cc
   :function: PowertrainBrake::DeclareParameters

Example
-------

:source:`examples/force_modules/powertrain.json`: a vehicle of four flywheel wheels
with a drag torque in proportion to speed, on two axles (the rear with a limited slip
differential), driven by an engine and converter through a three speed transmission.
Schedule controllers give the throttle, the gear changes at 3 s and 6 s, the
engagement of the front axle, the lockup clutch and the brakes. The converter
multiplies the engine's torque at the start, the engine speed falls at each upshift as
the ratio drops, and the engine returns to idle as the brakes stop the wheels.
:source:`examples/force_modules/powertrain_awd.json` is the same drive with a center
differential, the clutch hydraulics and the losses of the gears. Tests:
:source:`test/test_powertrain.cc`, which check the engine and governor, the converter's
torques and efficiency, the ratios and torques of a locked drive and after a shift, the
three differentials, the brakes, reverse, the losses, the center differential's split
and the fill of a clutch.
