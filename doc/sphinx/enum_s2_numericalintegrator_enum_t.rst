.. index:: pair: enum; s2_numericalintegrator_enum_t
.. _doxid-context_8h_1af852b2c473abbd9d2bf3556aab33755f:
.. _cid-s2_numericalintegrator_enum_t:

enum s2_numericalintegrator_enum_t
==================================



Overview
~~~~~~~~

Current supported numerical integration types. :ref:`More...<details-doxid-context_8h_1af852b2c473abbd9d2bf3556aab33755f>`

.. ref-code-block:: cpp
	:class: overview-code-block

	// enum values

	:ref:`ODE113<doxid-context_8h_1af852b2c473abbd9d2bf3556aab33755face1ad6271b673b7e42d7a995230d4c78>` 
	:ref:`ODE113B<doxid-context_8h_1af852b2c473abbd9d2bf3556aab33755fa61c805df4d3adb8a77417b7ded1ff232>` 

.. _details-doxid-context_8h_1af852b2c473abbd9d2bf3556aab33755f:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Current supported numerical integration types.

Currently only the Adams-Bashforth-Molton variable time step predictor correction numerical integrator is supported

Enum Values
-----------

.. _doxid-context_8h_1af852b2c473abbd9d2bf3556aab33755face1ad6271b673b7e42d7a995230d4c78:
.. _cid-s2_numericalintegrator_enum_t::ode113:
.. ref-code-block:: cpp
	:class: title-code-block

	ODE113

variable time step Adams-Bashforth-Molton, output is interpolated at user specified time intervals

.. _doxid-context_8h_1af852b2c473abbd9d2bf3556aab33755fa61c805df4d3adb8a77417b7ded1ff232:
.. _cid-s2_numericalintegrator_enum_t::ode113b:
.. ref-code-block:: cpp
	:class: title-code-block

	ODE113B

variable time step Adams-Bashforth-Molton, output is reported at time interval closest to requested time interval.

