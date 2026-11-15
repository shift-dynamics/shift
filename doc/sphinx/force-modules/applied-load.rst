.. title:: Applied load

Applied load
============

``"type": "applied_load"`` applies a force and moment to a body: a weight, a wind
load, a test load or an impulse. The load acts at the origin of ``frame`` on
``body``. Its coordinates are those of the frame (``"frame"``, the default, so that
it turns with the body) or of the fixed frame (``"fixed"``). With ``reaction_body``
the opposite load acts on that body at the same point, as an actuator between
two bodies would. Without it the load comes from outside the model.

The load is ``force`` and ``moment`` times a scale :math:`s(t)`, the product of:

- ``scale``, which a :doc:`schedule </controllers/controllers>` can change as the
  model runs;
- ``scale_curve``, pairs ``[t0, s0, t1, s1, ...]`` of time and scale, or the same
  numbers in ``scale_file``, if given. Between the points it is a smooth curve
  that does not overshoot them, and it holds the end values before and after
  them;
- a pulse, if ``pulse_duration`` :math:`t_p` is given, from ``pulse_start``
  :math:`t_0`: :math:`1 - \cos(2 \pi (t - t_0) / t_p)` while it lasts, zero
  outside. It is smooth so that the integrator sees no jump, and its integral
  over time is :math:`t_p`: a force of the impulse over :math:`t_p` gives that
  impulse.

A variable step integrator knows nothing of a pulse ahead of it and can step
right over it, with no effect at all. Keep the integrator's ``hmax`` (0.01 s by default) below
``pulse_duration``.

.. code:: javascript

   {"type": "applied_load", "name": "wind", "body": "cab", "frame": "roof",
    "coordinates": "fixed", "force": [0, 800, 0]}

   {"type": "applied_load", "name": "hammer", "body": "arm",
    "force": [0, 0, 5000], "pulse_start": 1.0, "pulse_duration": 0.01}

.. shift-parameters:: shift/core/force_module/applied_load.cc
   :function: force_module::AppliedLoad::DeclareParameters

Output: ``load_scale``, the scale :math:`s(t)`.
