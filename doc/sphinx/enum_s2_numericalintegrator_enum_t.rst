.. index:: pair: enum; s2_numericalintegrator_enum_t
.. _doxid-numericalintegrator_8h_1af852b2c473abbd9d2bf3556aab33755f:
.. _cid-s2_numericalintegrator_enum_t:

enum s2_numericalintegrator_enum_t
==================================



Overview
~~~~~~~~

Current supported numerical integration types. :ref:`More...<details-doxid-numericalintegrator_8h_1af852b2c473abbd9d2bf3556aab33755f>`

.. ref-code-block:: cpp
	:class: overview-code-block

	// enum values

	:ref:`S2_NUMERICALINTEGRATOR_ODE113<doxid-numericalintegrator_8h_1af852b2c473abbd9d2bf3556aab33755fa8998dafe0a3d8fd2d83235bebc278d6a>` 
	:ref:`S2_NUMERICALINTEGRATOR_ODE113B<doxid-numericalintegrator_8h_1af852b2c473abbd9d2bf3556aab33755fa5e37869292748edc4a4d34e5f2126848>` 

.. _details-doxid-numericalintegrator_8h_1af852b2c473abbd9d2bf3556aab33755f:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Current supported numerical integration types.

Currently only the Adams-Bashforth-Molton variable time step predictor correction numerical integrator is supported

Enum Values
-----------

.. _doxid-numericalintegrator_8h_1af852b2c473abbd9d2bf3556aab33755fa8998dafe0a3d8fd2d83235bebc278d6a:
.. _cid-s2_numericalintegrator_enum_t::s2_numericalintegrator_ode113:
.. ref-code-block:: cpp
	:class: title-code-block

	S2_NUMERICALINTEGRATOR_ODE113

variable time step Adams-Bashforth-Molton, output is interpolated at user specified time intervals

.. _doxid-numericalintegrator_8h_1af852b2c473abbd9d2bf3556aab33755fa5e37869292748edc4a4d34e5f2126848:
.. _cid-s2_numericalintegrator_enum_t::s2_numericalintegrator_ode113b:
.. ref-code-block:: cpp
	:class: title-code-block

	S2_NUMERICALINTEGRATOR_ODE113B

variable time step Adams-Bashforth-Molton, output is reported at time interval closest to requested time interval.

