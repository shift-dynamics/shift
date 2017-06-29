.. index:: pair: struct; s2_modelparameters_t
.. _doxid-structs2__modelparameters__t:
.. _cid-s2_modelparameters_t:

struct s2_modelparameters_t
===========================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <context.h>


Overview
~~~~~~~~

Attributes associated with force fields, file version, numerical integration and include directories. :ref:`More...<details-doxid-structs2__modelparameters__t>`

.. _doxid-structs2__modelparameters__t_1ac71feadcc91856dc6e7865e5e64898d4:
.. _cid-s2_modelparameters_t::sumconstrainterror:
.. _doxid-structs2__modelparameters__t_1af0adb33402c21411c197f215446fbf96:
.. _cid-s2_modelparameters_t::maxconstrainterror:
.. _doxid-structs2__modelparameters__t_1a55796362c56c94bf6ed212359f3ee244:
.. _cid-s2_modelparameters_t::maxconstrainterrorstep:
.. _doxid-structs2__modelparameters__t_1ace69c60945aca469c4e6d229463d111e:
.. _cid-s2_modelparameters_t::sumiterationcount:
.. _doxid-structs2__modelparameters__t_1af1394590ae670e76f1766e23ff8f39e0:
.. _cid-s2_modelparameters_t::maxiterationcount:
.. _doxid-structs2__modelparameters__t_1a51e30a3fec8e087ac42ed71b5f2f0569:
.. _cid-s2_modelparameters_t::maxiterationcountstep:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	char* :ref:`version<doxid-structs2__modelparameters__t_1aeb313024f323bbd12d90f6ed03b2b36d>`
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`includeDirectory<doxid-structs2__modelparameters__t_1ac3b0393fee2ae2c1609fa4b24f6ba899>`
	:ref:`s2_vector_t<doxid-structs2__vector__t>`* :ref:`gravity<doxid-structs2__modelparameters__t_1a49eff45eef350476099245d7a4405e08>`
	:ref:`s2_units_t<doxid-structs2__units__t>`* :ref:`units<doxid-structs2__modelparameters__t_1a246179fe8dfc45e29716c18a2b77f913>`
	:ref:`s2_numericalintegrator_enum_t<doxid-context_8h_1af852b2c473abbd9d2bf3556aab33755f>` :ref:`integrationType<doxid-structs2__modelparameters__t_1aed1f6db3ab96b248714b858ecc83a746>`
	int :ref:`iflag<doxid-structs2__modelparameters__t_1acd5d4f87ad003cb7cdd00dd4b9bfaefc>`
	double :ref:`timeSpan<doxid-structs2__modelparameters__t_1aa9b8b54f86637dfa6a35cae72c32d1f6>`
	double :ref:`timeStep<doxid-structs2__modelparameters__t_1a72757c8e0664818236bb71ab46eaeb6f>`
	double :ref:`relativeError<doxid-structs2__modelparameters__t_1a3379add6d12389b71bee776054638465>`
	double :ref:`absoluteError<doxid-structs2__modelparameters__t_1aae109fd77c8097f2fe0dd434176d5213>`
	char* :ref:`outFile<doxid-structs2__modelparameters__t_1afb9f318ca8c092570953876139892f70>`
	FILE* :ref:`outFilePointer<doxid-structs2__modelparameters__t_1a3a2b40ed2716281ad348dd5799043529>`
	char* :ref:`debugFile<doxid-structs2__modelparameters__t_1a6f2cb52ef6a32e30e95f7ad981fe0537>`
	FILE* :ref:`debugFilePointer<doxid-structs2__modelparameters__t_1a15e8eeaae8d10eb62bf26de88cb44318>`
	FILE* :ref:`dhvaFilePointer<doxid-structs2__modelparameters__t_1a1a938f95efca5c08b4f0220a7a66d37f>`
	char* :ref:`delimiter<doxid-structs2__modelparameters__t_1a63fdffbc99d0d58b1160e80c36edbfe3>`
	double :ref:`baumgarteOmegaN<doxid-structs2__modelparameters__t_1ae4bc60a29f4a2a16a921d98da6015f0d>`
	double :ref:`baumgarte2OmegaN<doxid-structs2__modelparameters__t_1ab3d2991c01d90e30619ae9c71b93a122>`
	double :ref:`baumgarteOmegaN2<doxid-structs2__modelparameters__t_1a7a6d0957f5801f95d641ec1ec39eb8af>`
	bool :ref:`monitorIterations<doxid-structs2__modelparameters__t_1aaff4bcbf2125a4d06a77dd7e031bff95>`
	bool :ref:`monitorConstraintError<doxid-structs2__modelparameters__t_1a906654eb38dd09e0dec54b576154f7f1>`
	double sumConstraintError
	double maxConstraintError
	int maxConstraintErrorStep
	int sumIterationCount
	int maxIterationCount
	int maxIterationCountStep

.. _details-doxid-structs2__modelparameters__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Attributes associated with force fields, file version, numerical integration and include directories.

Fields
------

.. _doxid-structs2__modelparameters__t_1aeb313024f323bbd12d90f6ed03b2b36d:
.. _cid-s2_modelparameters_t::version:
.. ref-code-block:: cpp
	:class: title-code-block

	char* version

specify mbdl file version number

.. _doxid-structs2__modelparameters__t_1ac3b0393fee2ae2c1609fa4b24f6ba899:
.. _cid-s2_modelparameters_t::includedirectory:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* includeDirectory

specify directories to search for external files

.. _doxid-structs2__modelparameters__t_1a49eff45eef350476099245d7a4405e08:
.. _cid-s2_modelparameters_t::gravity:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vector_t<doxid-structs2__vector__t>`* gravity

specify gravity force field, a 3x1 vector with units as specified in the Units datastructure

.. _doxid-structs2__modelparameters__t_1a246179fe8dfc45e29716c18a2b77f913:
.. _cid-s2_modelparameters_t::units:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_units_t<doxid-structs2__units__t>`* units

Refer to the Units data structure definition for more information

.. _doxid-structs2__modelparameters__t_1aed1f6db3ab96b248714b858ecc83a746:
.. _cid-s2_modelparameters_t::integrationtype:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_numericalintegrator_enum_t<doxid-context_8h_1af852b2c473abbd9d2bf3556aab33755f>` integrationType

type of numerical integrator to use, currently only ODE113 Adams-Bashforth-Molton is supported

.. _doxid-structs2__modelparameters__t_1acd5d4f87ad003cb7cdd00dd4b9bfaefc:
.. _cid-s2_modelparameters_t::iflag:
.. ref-code-block:: cpp
	:class: title-code-block

	int iflag

interpolation flag for numerical integration, used by ODE113 integrator. The default value is set to -1, which reports simulation results at possibly irregular time intervals (whatever the largest time step ode113 can take). To interpolate simulation results at fixed time intervals (slower), set to 1

.. _doxid-structs2__modelparameters__t_1aa9b8b54f86637dfa6a35cae72c32d1f6:
.. _cid-s2_modelparameters_t::timespan:
.. ref-code-block:: cpp
	:class: title-code-block

	double timeSpan

span of simulation time used for numerical integration. if not specified in JSON file, the default is 1 second

.. _doxid-structs2__modelparameters__t_1a72757c8e0664818236bb71ab46eaeb6f:
.. _cid-s2_modelparameters_t::timestep:
.. ref-code-block:: cpp
	:class: title-code-block

	double timeStep

nominal time step used for simulation, if not specified in JSON file, the default is 0.001 seconds

.. _doxid-structs2__modelparameters__t_1a3379add6d12389b71bee776054638465:
.. _cid-s2_modelparameters_t::relativeerror:
.. ref-code-block:: cpp
	:class: title-code-block

	double relativeError

relative error convergence tolerance term, used by ode113. default value if not provided in JSON file is 0.0

.. _doxid-structs2__modelparameters__t_1aae109fd77c8097f2fe0dd434176d5213:
.. _cid-s2_modelparameters_t::absoluteerror:
.. ref-code-block:: cpp
	:class: title-code-block

	double absoluteError

absolute error convergence tolerance term, used by ode113. default value if not provided in JSON fle is 1e-4

.. _doxid-structs2__modelparameters__t_1afb9f318ca8c092570953876139892f70:
.. _cid-s2_modelparameters_t::outfile:
.. ref-code-block:: cpp
	:class: title-code-block

	char* outFile

filename to write resuts to

.. _doxid-structs2__modelparameters__t_1a3a2b40ed2716281ad348dd5799043529:
.. _cid-s2_modelparameters_t::outfilepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	FILE* outFilePointer

file pointer to write results to

.. _doxid-structs2__modelparameters__t_1a6f2cb52ef6a32e30e95f7ad981fe0537:
.. _cid-s2_modelparameters_t::debugfile:
.. ref-code-block:: cpp
	:class: title-code-block

	char* debugFile

filename to write debug information to

.. _doxid-structs2__modelparameters__t_1a15e8eeaae8d10eb62bf26de88cb44318:
.. _cid-s2_modelparameters_t::debugfilepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	FILE* debugFilePointer

file pointer to write debug information to

.. _doxid-structs2__modelparameters__t_1a1a938f95efca5c08b4f0220a7a66d37f:
.. _cid-s2_modelparameters_t::dhvafilepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	FILE* dhvaFilePointer

file pointer to write dhva debug information to

.. _doxid-structs2__modelparameters__t_1a63fdffbc99d0d58b1160e80c36edbfe3:
.. _cid-s2_modelparameters_t::delimiter:
.. ref-code-block:: cpp
	:class: title-code-block

	char* delimiter

delimiter used in writing to file

.. _doxid-structs2__modelparameters__t_1ae4bc60a29f4a2a16a921d98da6015f0d:
.. _cid-s2_modelparameters_t::baumgarteomegan:
.. ref-code-block:: cpp
	:class: title-code-block

	double baumgarteOmegaN

natural frequency for baumgarte stabilization

.. _doxid-structs2__modelparameters__t_1ab3d2991c01d90e30619ae9c71b93a122:
.. _cid-s2_modelparameters_t::baumgarte2omegan:
.. ref-code-block:: cpp
	:class: title-code-block

	double baumgarte2OmegaN

2 * natural freq for baumgarte constraint stabilization

.. _doxid-structs2__modelparameters__t_1a7a6d0957f5801f95d641ec1ec39eb8af:
.. _cid-s2_modelparameters_t::baumgarteomegan2:
.. ref-code-block:: cpp
	:class: title-code-block

	double baumgarteOmegaN2

omega_N ^2 for baumgarte constraint stabilization

.. _doxid-structs2__modelparameters__t_1aaff4bcbf2125a4d06a77dd7e031bff95:
.. _cid-s2_modelparameters_t::monitoriterations:
.. ref-code-block:: cpp
	:class: title-code-block

	bool monitorIterations

monitor iterations in de

.. _doxid-structs2__modelparameters__t_1a906654eb38dd09e0dec54b576154f7f1:
.. _cid-s2_modelparameters_t::monitorconstrainterror:
.. ref-code-block:: cpp
	:class: title-code-block

	bool monitorConstraintError

monitor constraint error

