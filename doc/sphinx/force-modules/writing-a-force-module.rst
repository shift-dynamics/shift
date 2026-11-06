.. _writing-a-force-module:
.. title:: Writing a Force Module

Writing a Force Module
======================

A force module is a class derived from
:cpp:class:`shift::core::force_module::Base`, registered with the factory
under the ``type`` that model files use. The modules in
:tree:`shift/core/force_module` are complete examples: ``bushing.cc`` and
``belt.cc`` apply forces to bodies, ``hydraulic_cylinder.cc`` to a joint,
and ``rubber_mount.cc`` and ``belt.cc`` keep internal states.

The life of a module
--------------------

``DeclareParameters()``
    Called from the constructor: declares each JSON key, the member it is
    read into, its description and whether it is required. These
    declarations also generate the parameter tables of this documentation.

``Connect()``
    Called once the model is loaded: look up the bodies, frames and joints
    the module names, and fail with a :cpp:class:`shift::core::Exception`
    that names the module if one is missing.

``Load()``
    Called once the kinematics of the initial state are known: check the
    parameters, compute what depends on the initial pose (rest lengths,
    tangent angles, ...), and declare the internal states: set
    ``equation_count_`` and their initial values ``q_0_``.

``Service(y, dy)``
    Called each time the equations of motion are set up, with the state
    ``y = [q, dq, internal states]``. Compute the forces from ``y`` and the
    kinematics in the context, which are up to date with it; add them to the
    context; and write the rates of the internal states to
    ``dy[state_offset_ + k]``. ``state_offset_`` is set before the first
    call (:cpp:func:`shift::core::Context::InitializeAuxiliaryStates`).

``get_output_names()``, ``UpdateOutputs(t, y)``, ``AppendOutputs(values)``
    Optional: values for the loggers. ``UpdateOutputs`` evaluates the
    module at ``y`` without applying forces, before each logged row.

``holds()``, ``get_held_joint()``, ``set_hold_force()``, ``Hold()``
    Optional: for a module that drives a joint and can start holding it
    still (see :doc:`hydraulic-cylinder`).

Applying forces
---------------

A force on a body is a spatial force (a coscrew),
:math:`\mathbf{\underline{f}}^*`, in the fixed frame, referred to the fixed
origin: the force :math:`\underline{f}` and the moment
:math:`\underline{p} \times \underline{f}` of a force applied at the point
:math:`\underline{p}`. The context accumulates them by the body's *load
index*:

.. code:: cpp

   int index = context_->GetBodyLoadIndex(body);   // once, after loading
   ksl_SE3_t pose = context_->get_body_origin_pose(body, context_->get_dhva());
   ksl_screw_t velocity = context_->GetBodyVelocity(index);
   //   velocity of a point p of the body: velocity.lin + velocity.ang x p

   ksl_coscrew_t load;
   load.lin = force;                 // ksl_vec3_t, fixed axes
   load.ang = cross(point, force);   // its moment about the fixed origin
   context_->AddBodyLoad(index, load);

A generalized force on a joint is added to its entry of the working forces,
``context_->get_joint_working_forces()[joint->get_dq_index()]``.

Rules
-----

**Forces from the state only.** The forces and rates must be functions of
``y`` alone, with no history kept between calls: the integrators evaluate
a step several times and may reject it, so a value remembered from the
last call is from a state that may never be accepted. Memory belongs in
internal states. A quantity that would otherwise jump (contact starting, a
valve opening, a span going slack) is made smooth over a small band.

**Registration.** Register the type in the ``.cc`` file:

.. code:: cpp

   bool shift::core::force_module::Belt::registered_ =
     shift::core::Factory<shift::core::force_module::Base>::get()
       ->RegisterModule("belt", shift::core::force_module::Belt::Create);

and add a check of ``is_registered()`` to
``force_module::Modules::is_initialized()`` in
:source:`shift/core/force_module/base.cc`, which keeps the linker from
dropping the module from the static library, and the source file to
:source:`shift/core/force_module/meson.build`.

**Tests.** Add a GoogleTest file under ``test/`` and to
:source:`test/meson.build`; tests link ``shift_core_dep`` rather than
compiling sources again. ``test_belt.cc`` and ``test_beam.cc`` show the
pattern: write a small model to a temporary file, load it, and check the
forces against a closed form result, the rates against finite differences,
and a short simulation against its known end state. Randomized tests use
fixed seeds.

**Example and documentation.** Add a small model to ``examples/`` (it
becomes a test of its own) and a page to ``doc/sphinx/force-modules`` with
a ``shift-parameters`` table of its keys.
