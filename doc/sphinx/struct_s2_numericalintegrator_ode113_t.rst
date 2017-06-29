.. index:: pair: struct; s2_numericalintegrator_ode113_t
.. _doxid-structs2__numericalintegrator__ode113__t:
.. _cid-s2_numericalintegrator_ode113_t:

struct s2_numericalintegrator_ode113_t
======================================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <ode113.h>


Overview
~~~~~~~~

Attributes associated with numerical integrator. :ref:`More...<details-doxid-structs2__numericalintegrator__ode113__t>`

.. _doxid-structs2__numericalintegrator__ode113__t_1aa4d6f9eae6ddbd0d9d113474bc1db35f:
.. _cid-s2_numericalintegrator_ode113_t::stepcount:
.. _doxid-structs2__numericalintegrator__ode113__t_1abe1e278a4b6513722ce9000386b68bac:
.. _cid-s2_numericalintegrator_ode113_t::iterationcount:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	double* :ref:`y<doxid-structs2__numericalintegrator__ode113__t_1a91ad1ab10815034e59e6bc2f98d48a09>`
	double* :ref:`dy<doxid-structs2__numericalintegrator__ode113__t_1a150611eb771e3d070b51fd6f26a66c51>`
	double* :ref:`ydata<doxid-structs2__numericalintegrator__ode113__t_1ade308206907ce3fe0a680707dd36248a>`
	int stepCount
	char* :ref:`outFile<doxid-structs2__numericalintegrator__ode113__t_1abc4024c64efb81b6d6264f4476dcb525>`
	int :ref:`neqn<doxid-structs2__numericalintegrator__ode113__t_1ad3c209cf13baf8431b69b6f431247147>`
	int :ref:`neqn2<doxid-structs2__numericalintegrator__ode113__t_1a4e3cdbb43614e81ff660de356246bd34>`
	double :ref:`t<doxid-structs2__numericalintegrator__ode113__t_1a4614e8fcf146fe082bca77f1077b7758>`
	double :ref:`tout<doxid-structs2__numericalintegrator__ode113__t_1ab9c75168aa7b44c4933254b0c670e6c0>`
	int :ref:`iflag<doxid-structs2__numericalintegrator__ode113__t_1a0ca662106471204f95a69a5157517462>`
	double* :ref:`work<doxid-structs2__numericalintegrator__ode113__t_1a00c688b309270863b29a8b9e738aa426>`
	int* :ref:`iwork<doxid-structs2__numericalintegrator__ode113__t_1aefad9cea2bd2fc3b3d5d80d9f1b7bd9c>`
	int iterationCount
	:ref:`s2_stack_t<doxid-structs2__stack__t>`* :ref:`reconfigurations<doxid-structs2__numericalintegrator__ode113__t_1a7049134484fad4b46a58738f3ebbf874>`

.. _details-doxid-structs2__numericalintegrator__ode113__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Attributes associated with numerical integrator.

Fields
------

.. _doxid-structs2__numericalintegrator__ode113__t_1a91ad1ab10815034e59e6bc2f98d48a09:
.. _cid-s2_numericalintegrator_ode113_t::y:
.. ref-code-block:: cpp
	:class: title-code-block

	double* y

the current solution

.. _doxid-structs2__numericalintegrator__ode113__t_1a150611eb771e3d070b51fd6f26a66c51:
.. _cid-s2_numericalintegrator_ode113_t::dy:
.. ref-code-block:: cpp
	:class: title-code-block

	double* dy

first time derivative of generalized coordinates

.. _doxid-structs2__numericalintegrator__ode113__t_1ade308206907ce3fe0a680707dd36248a:
.. _cid-s2_numericalintegrator_ode113_t::ydata:
.. ref-code-block:: cpp
	:class: title-code-block

	double* ydata

array to store results from numerical integration

.. _doxid-structs2__numericalintegrator__ode113__t_1abc4024c64efb81b6d6264f4476dcb525:
.. _cid-s2_numericalintegrator_ode113_t::outfile:
.. ref-code-block:: cpp
	:class: title-code-block

	char* outFile

number of steps in ydata array, found by tspan / tstep filename to write results to

.. _doxid-structs2__numericalintegrator__ode113__t_1ad3c209cf13baf8431b69b6f431247147:
.. _cid-s2_numericalintegrator_ode113_t::neqn:
.. ref-code-block:: cpp
	:class: title-code-block

	int neqn

number of equations for dynamics

.. _doxid-structs2__numericalintegrator__ode113__t_1a4e3cdbb43614e81ff660de356246bd34:
.. _cid-s2_numericalintegrator_ode113_t::neqn2:
.. ref-code-block:: cpp
	:class: title-code-block

	int neqn2

total number of equations including additional state variables that must be integrated for force modules

.. _doxid-structs2__numericalintegrator__ode113__t_1a4614e8fcf146fe082bca77f1077b7758:
.. _cid-s2_numericalintegrator_ode113_t::t:
.. ref-code-block:: cpp
	:class: title-code-block

	double t

current value of the independent variable

.. _doxid-structs2__numericalintegrator__ode113__t_1ab9c75168aa7b44c4933254b0c670e6c0:
.. _cid-s2_numericalintegrator_ode113_t::tout:
.. ref-code-block:: cpp
	:class: title-code-block

	double tout

the desired value of t of simulation output

.. _doxid-structs2__numericalintegrator__ode113__t_1a0ca662106471204f95a69a5157517462:
.. _cid-s2_numericalintegrator_ode113_t::iflag:
.. ref-code-block:: cpp
	:class: title-code-block

	int iflag

indicates the status of integration. On input, IFLAG is normally 1 (or -1 in the special case where TOUT is not to be exceeded.) On normal output, IFLAG is 2. Other output values are: 3, integration did not reach TOUT because the error tolerances were too small. But RELERR and ABSERR were increased appropriately for continuing; 4, integration did not reach TOUT because more than 500 steps were taken; 5, integration did not reach TOUT because the equations appear to be stiff; 6, invalid input parameters (fatal error). The value of IFLAG is returned negative when the input value is negative and the integration does not reach TOUT.

.. _doxid-structs2__numericalintegrator__ode113__t_1a00c688b309270863b29a8b9e738aa426:
.. _cid-s2_numericalintegrator_ode113_t::work:
.. ref-code-block:: cpp
	:class: title-code-block

	double* work

work[100 + 21 * neqn]

.. _doxid-structs2__numericalintegrator__ode113__t_1aefad9cea2bd2fc3b3d5d80d9f1b7bd9c:
.. _cid-s2_numericalintegrator_ode113_t::iwork:
.. ref-code-block:: cpp
	:class: title-code-block

	int* iwork

iwork[5]

.. _doxid-structs2__numericalintegrator__ode113__t_1a7049134484fad4b46a58738f3ebbf874:
.. _cid-s2_numericalintegrator_ode113_t::reconfigurations:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stack_t<doxid-structs2__stack__t>`* reconfigurations

records time step indices of a full factor event during a running simulation

