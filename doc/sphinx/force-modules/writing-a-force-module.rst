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

Force module lifecycle
----------------------

``DeclareParameters()``
    Called from the constructor: declares each JSON key, the member it is
    read into, its description and whether it is required. These
    declarations also generate the parameter tables of this documentation.
    It also declares the module's inputs and outputs (see below).

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

``UpdateOutputs(t, y)``
    Optional: evaluates the module's outputs (below) at ``y`` without
    applying forces, before each logged row, so that the loggers record the
    outputs of the logged state.

``holds()``, ``get_held_joint()``, ``set_hold_force()``, ``Hold()``
    Optional: for a module that drives a joint and can start holding it
    still (see :doc:`hydraulic-cylinder`).

Inputs and outputs
------------------

A number that a module reads at every evaluation, and that a controller
sets as the model runs, is an *input*; a number that the module computes,
for other components to read, is an *output*. Both are declared in
``DeclareParameters()``, next to the parameters:

.. code:: cpp

   DeclareInput("current", &current_, "winding current, set by a controller");
   DeclareOutput("speed", &speed_, "rotor speed relative to the stator");

An input may be given in the model file, which sets its value until a
controller does; without it, the input starts at the member's initial value.
It can be changed while the model runs, so a :ref:`schedule <schedule>` can
set it too. An output is not a key of the model file, and giving it there is
an error; nor can it share a key with a parameter. The parameter tables of
this documentation list both, marked "Input." and "Output.".

Outputs are what the loggers record: each logger connects to every output,
or to those of the components it names (see :ref:`logging`), once the model is
loaded, and reads them at each logged row, in the order they are declared.
A module whose outputs depend on the model, one per pulley of a belt say,
declares them at the end of ``Load()``, once it knows them, pointing into
storage that is not resized after:

.. code:: cpp

   // in Load(), after tension_.assign(pulley_count, 0.0)
   for(size_t k = 0; k < tension_.size(); k++) {
     DeclareOutput("span_" + std::to_string(k) + "_tension", &tension_[k],
                   "tension of the span");
   }

A controller finds the component by name when it connects, claims the input
(only one component may set an input), and sets it at each evaluation:

.. code:: cpp

   // in Connect()
   motor_ = parent_assembly_ptr_->FindComponent(motor_name_);
   motor_->ConnectInput("current", FullyQualifiedName(get_name()));
   // in Service(y, dy)
   motor_->SetInput("current", effort);

:cpp:func:`shift::core::Assembly::FindComponent` takes a bare name, or the
kind and name (``"force_modules/motor"``) where two kinds of component share
a name. ``GetOutput("speed")`` reads an output. The controllers are evaluated
before the force modules, so a module reads the input set in the same
evaluation. The ``pid`` controller sets an input with ``"component"`` and
``"input"`` (see :ref:`controllers`); the ``command`` of a hydraulic
cylinder is an input. Tests: :source:`test/test_component_io.cc`.

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
