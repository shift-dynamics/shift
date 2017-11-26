.. index:: pair: struct; s2_context_t::modelParameters
.. _doxid-structs2__context__t_1_1model_parameters:
.. _cid-s2_context_t::modelparameters-2:

struct s2_context_t::modelParameters
====================================

.. toctree::
	:hidden:

	struct_s2_context_t_modelParameters_units.rst



Overview
~~~~~~~~



.. _doxid-structs2__context__t_1_1model_parameters_1afc362f9d7277a55c924c6aa2d9e2a50a:
.. _cid-s2_context_t::modelparameters::units:
.. _doxid-structs2__context__t_1_1model_parameters_1a1ebd20385e9fc1f3c4ad0c7bf5d7acc3:
.. _cid-s2_context_t::modelparameters::sumconstrainterror:
.. _doxid-structs2__context__t_1_1model_parameters_1a36c018e702bbcb83c26e6461b88b68de:
.. _cid-s2_context_t::modelparameters::maxconstrainterror:
.. _doxid-structs2__context__t_1_1model_parameters_1ad82d188e3b93e43b0b871ef0a6f0068b:
.. _cid-s2_context_t::modelparameters::maxconstrainterrorstep:
.. _doxid-structs2__context__t_1_1model_parameters_1aaeef45ea8b370926745259b7dea6ac47:
.. _cid-s2_context_t::modelparameters::sumiterationcount:
.. _doxid-structs2__context__t_1_1model_parameters_1a05e37dec6a462fe9dad67024fcd91317:
.. _cid-s2_context_t::modelparameters::maxiterationcount:
.. _doxid-structs2__context__t_1_1model_parameters_1abafff4e59d29b6174fea9f8f4c81edf2:
.. _cid-s2_context_t::modelparameters::maxiterationcountstep:
.. _doxid-structs2__context__t_1_1model_parameters_1a9fbc21b4c37d3ca7505189f962bb2a23:
.. _cid-s2_context_t::modelparameters::pivotratiomultiple:
.. ref-code-block:: cpp
	:class: overview-code-block

	// structs

	struct :ref:`units<doxid-structs2__context__t_1_1model_parameters_1_1units>`

	// fields

	char* :ref:`version<doxid-structs2__context__t_1_1model_parameters_1af9ca9d99f6c8da758f25e0aa542f6676>`
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`includeDirectory<doxid-structs2__context__t_1_1model_parameters_1a45d1c73f14991589405feaa8fd455e75>`
	:ref:`s2_vector_t<doxid-structs2__vector__t>`* :ref:`gravity<doxid-structs2__context__t_1_1model_parameters_1afdd5e3caa35ed2cad186dd9a416f77ca>`
	struct :ref:`s2_context_t::modelParameters::units<doxid-structs2__context__t_1_1model_parameters_1_1units>` units
	:ref:`s2_numericalintegrator_enum_t<doxid-numericalintegrator_8h_1af852b2c473abbd9d2bf3556aab33755f>` :ref:`integrationType<doxid-structs2__context__t_1_1model_parameters_1a9aa03ce30398f86b0cadd34ddfe4f224>`
	int :ref:`iflag<doxid-structs2__context__t_1_1model_parameters_1a756356cbb5d6e282d4630e3b172d1262>`
	double :ref:`timeSpan<doxid-structs2__context__t_1_1model_parameters_1a219b01a742c1b7e00c554bb3fbfac3b6>`
	double :ref:`timeStep<doxid-structs2__context__t_1_1model_parameters_1a1b152fe313ce3952382bc31078822f3f>`
	double :ref:`relativeError<doxid-structs2__context__t_1_1model_parameters_1a36269f996b86df83671f4fa9f3d1cbd5>`
	double :ref:`absoluteError<doxid-structs2__context__t_1_1model_parameters_1a8086b9e84bb6f2b3e23484f998b61d49>`
	char* :ref:`outFile<doxid-structs2__context__t_1_1model_parameters_1ae95ab753cd08ac0362a5fb9909972ec5>`
	FILE* :ref:`outFilePointer<doxid-structs2__context__t_1_1model_parameters_1a14103763fe928c2d88f06d5214e168bc>`
	char* :ref:`debugFile<doxid-structs2__context__t_1_1model_parameters_1a0017d545116223a8a775a9a4a248955b>`
	FILE* :ref:`debugFilePointer<doxid-structs2__context__t_1_1model_parameters_1a1c34083737d625452a070788984c27f4>`
	FILE* :ref:`dhvaFilePointer<doxid-structs2__context__t_1_1model_parameters_1a1475274d58b9226c36df43be4b45209f>`
	char* :ref:`delimiter<doxid-structs2__context__t_1_1model_parameters_1a57789dc3bfa0042617286b353a24ba76>`
	double :ref:`baumgarteOmegaN<doxid-structs2__context__t_1_1model_parameters_1a475a689593bb75b30587471964a61bb5>`
	double :ref:`baumgarte2OmegaN<doxid-structs2__context__t_1_1model_parameters_1a0c954f4e8d2bd47d81c8aa1ce0355116>`
	double :ref:`baumgarteOmegaN2<doxid-structs2__context__t_1_1model_parameters_1a200f7f464399f51b41d1091860a2f88b>`
	bool :ref:`monitorIterations<doxid-structs2__context__t_1_1model_parameters_1a847c767bc31186bfa7733e95df4b982d>`
	bool :ref:`monitorConstraintError<doxid-structs2__context__t_1_1model_parameters_1a4113556aee50d911c78aab16053027ff>`
	double sumConstraintError
	double maxConstraintError
	int maxConstraintErrorStep
	int sumIterationCount
	int maxIterationCount
	int maxIterationCountStep
	double pivotRatioMultiple

.. _details-doxid-structs2__context__t_1_1model_parameters:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. _doxid-structs2__context__t_1_1model_parameters_1af9ca9d99f6c8da758f25e0aa542f6676:
.. _cid-s2_context_t::modelparameters::version:
.. ref-code-block:: cpp
	:class: title-code-block

	char* version

specify star2 file version number

.. _doxid-structs2__context__t_1_1model_parameters_1a45d1c73f14991589405feaa8fd455e75:
.. _cid-s2_context_t::modelparameters::includedirectory:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* includeDirectory

specify directories to search for external files

.. _doxid-structs2__context__t_1_1model_parameters_1afdd5e3caa35ed2cad186dd9a416f77ca:
.. _cid-s2_context_t::modelparameters::gravity:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vector_t<doxid-structs2__vector__t>`* gravity

specify gravity force field, a 3x1 vector with units as specified in the Units datastructure

.. _doxid-structs2__context__t_1_1model_parameters_1a9aa03ce30398f86b0cadd34ddfe4f224:
.. _cid-s2_context_t::modelparameters::integrationtype:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_numericalintegrator_enum_t<doxid-numericalintegrator_8h_1af852b2c473abbd9d2bf3556aab33755f>` integrationType

type of numerical integrator to use, currently only ODE113 Adams-Bashforth-Molton is supported

.. _doxid-structs2__context__t_1_1model_parameters_1a756356cbb5d6e282d4630e3b172d1262:
.. _cid-s2_context_t::modelparameters::iflag:
.. ref-code-block:: cpp
	:class: title-code-block

	int iflag

interpolation flag for numerical integration, used by ODE113 integrator. The default value is set to -1, which reports simulation results at possibly irregular time intervals (whatever the largest time step ode113 can take). To interpolate simulation results at fixed time intervals (slower), set to 1

.. _doxid-structs2__context__t_1_1model_parameters_1a219b01a742c1b7e00c554bb3fbfac3b6:
.. _cid-s2_context_t::modelparameters::timespan:
.. ref-code-block:: cpp
	:class: title-code-block

	double timeSpan

span of simulation time used for numerical integration. if not specified in JSON file, the default is 1 second

.. _doxid-structs2__context__t_1_1model_parameters_1a1b152fe313ce3952382bc31078822f3f:
.. _cid-s2_context_t::modelparameters::timestep:
.. ref-code-block:: cpp
	:class: title-code-block

	double timeStep

nominal time step used for simulation, if not specified in JSON file, the default is 0.001 seconds

.. _doxid-structs2__context__t_1_1model_parameters_1a36269f996b86df83671f4fa9f3d1cbd5:
.. _cid-s2_context_t::modelparameters::relativeerror:
.. ref-code-block:: cpp
	:class: title-code-block

	double relativeError

relative error convergence tolerance term, used by ode113. default value if not provided in JSON file is 0.0

.. _doxid-structs2__context__t_1_1model_parameters_1a8086b9e84bb6f2b3e23484f998b61d49:
.. _cid-s2_context_t::modelparameters::absoluteerror:
.. ref-code-block:: cpp
	:class: title-code-block

	double absoluteError

absolute error convergence tolerance term, used by ode113. default value if not provided in JSON fle is 1e-4

.. _doxid-structs2__context__t_1_1model_parameters_1ae95ab753cd08ac0362a5fb9909972ec5:
.. _cid-s2_context_t::modelparameters::outfile:
.. ref-code-block:: cpp
	:class: title-code-block

	char* outFile

filename to write resuts to

.. _doxid-structs2__context__t_1_1model_parameters_1a14103763fe928c2d88f06d5214e168bc:
.. _cid-s2_context_t::modelparameters::outfilepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	FILE* outFilePointer

file pointer to write results to

.. _doxid-structs2__context__t_1_1model_parameters_1a0017d545116223a8a775a9a4a248955b:
.. _cid-s2_context_t::modelparameters::debugfile:
.. ref-code-block:: cpp
	:class: title-code-block

	char* debugFile

filename to write debug information to

.. _doxid-structs2__context__t_1_1model_parameters_1a1c34083737d625452a070788984c27f4:
.. _cid-s2_context_t::modelparameters::debugfilepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	FILE* debugFilePointer

file pointer to write debug information to

.. _doxid-structs2__context__t_1_1model_parameters_1a1475274d58b9226c36df43be4b45209f:
.. _cid-s2_context_t::modelparameters::dhvafilepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	FILE* dhvaFilePointer

file pointer to write dhva debug information to

.. _doxid-structs2__context__t_1_1model_parameters_1a57789dc3bfa0042617286b353a24ba76:
.. _cid-s2_context_t::modelparameters::delimiter:
.. ref-code-block:: cpp
	:class: title-code-block

	char* delimiter

delimiter used in writing to file

.. _doxid-structs2__context__t_1_1model_parameters_1a475a689593bb75b30587471964a61bb5:
.. _cid-s2_context_t::modelparameters::baumgarteomegan:
.. ref-code-block:: cpp
	:class: title-code-block

	double baumgarteOmegaN

natural frequency for baumgarte stabilization

.. _doxid-structs2__context__t_1_1model_parameters_1a0c954f4e8d2bd47d81c8aa1ce0355116:
.. _cid-s2_context_t::modelparameters::baumgarte2omegan:
.. ref-code-block:: cpp
	:class: title-code-block

	double baumgarte2OmegaN

2 \* natural freq for baumgarte constraint stabilization

.. _doxid-structs2__context__t_1_1model_parameters_1a200f7f464399f51b41d1091860a2f88b:
.. _cid-s2_context_t::modelparameters::baumgarteomegan2:
.. ref-code-block:: cpp
	:class: title-code-block

	double baumgarteOmegaN2

omega_N ^2 for baumgarte constraint stabilization

.. _doxid-structs2__context__t_1_1model_parameters_1a847c767bc31186bfa7733e95df4b982d:
.. _cid-s2_context_t::modelparameters::monitoriterations:
.. ref-code-block:: cpp
	:class: title-code-block

	bool monitorIterations

monitor iterations in de

.. _doxid-structs2__context__t_1_1model_parameters_1a4113556aee50d911c78aab16053027ff:
.. _cid-s2_context_t::modelparameters::monitorconstrainterror:
.. ref-code-block:: cpp
	:class: title-code-block

	bool monitorConstraintError

monitor constraint error

