.. index:: pair: struct; s2_numericalintegrator_ode113b_t
.. _doxid-structs2__numericalintegrator__ode113b__t:
.. _cid-s2_numericalintegrator_ode113b_t:

struct s2_numericalintegrator_ode113b_t
=======================================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <ode113b.h>


Overview
~~~~~~~~

Attributes associated with numerical integrator. :ref:`More...<details-doxid-structs2__numericalintegrator__ode113b__t>`

.. _doxid-structs2__numericalintegrator__ode113b__t_1ac66c9ebef99a02ba210e252e2baf237d:
.. _cid-s2_numericalintegrator_ode113b_t::yy:
.. _doxid-structs2__numericalintegrator__ode113b__t_1a0dd9ea0ea3f18cd908f74a8a9a00ddb6:
.. _cid-s2_numericalintegrator_ode113b_t::dyout:
.. _doxid-structs2__numericalintegrator__ode113b__t_1aad953bbac01f867ecce73816300935a5:
.. _cid-s2_numericalintegrator_ode113b_t::wt:
.. _doxid-structs2__numericalintegrator__ode113b__t_1a71b955f1144a0befe7ba379c56a9267d:
.. _cid-s2_numericalintegrator_ode113b_t::p:
.. _doxid-structs2__numericalintegrator__ode113b__t_1adb0727dfa06ec27e38ca1617f73984cf:
.. _cid-s2_numericalintegrator_ode113b_t::yerr:
.. _doxid-structs2__numericalintegrator__ode113b__t_1aedd35e2d49c80ac61f60ea5221d3bd24:
.. _cid-s2_numericalintegrator_ode113b_t::phi:
.. _doxid-structs2__numericalintegrator__ode113b__t_1a4ed4fac2033f2f12f9fc82391daf8774:
.. _cid-s2_numericalintegrator_ode113b_t::stepcount:
.. _doxid-structs2__numericalintegrator__ode113b__t_1afa9a7982be2589ba8bee8514ad3b5a94:
.. _cid-s2_numericalintegrator_ode113b_t::newtonraphsonfailure:
.. _doxid-structs2__numericalintegrator__ode113b__t_1a2c5b49e19a760f7619bc4517ed4ebcab:
.. _cid-s2_numericalintegrator_ode113b_t::iterationcount:
.. _doxid-structs2__numericalintegrator__ode113b__t_1a4bae5ad1b927c1688f2e0d3c53b5345e:
.. _cid-s2_numericalintegrator_ode113b_t::told:
.. _doxid-structs2__numericalintegrator__ode113b__t_1ae4c5235002bb4fd26276268da30ba51d:
.. _cid-s2_numericalintegrator_ode113b_t::hh:
.. _doxid-structs2__numericalintegrator__ode113b__t_1ad2486ce6d1d4279acb25bec2f2c4604c:
.. _cid-s2_numericalintegrator_ode113b_t::safetyfactor:
.. _doxid-structs2__numericalintegrator__ode113b__t_1a8a2186c115aeafae77eeefbbde1d0d65:
.. _cid-s2_numericalintegrator_ode113b_t::ns:
.. _doxid-structs2__numericalintegrator__ode113b__t_1a36076ce9e74e53dbc540ba883b8af5bf:
.. _cid-s2_numericalintegrator_ode113b_t::ifail:
.. _doxid-structs2__numericalintegrator__ode113b__t_1a292c636a5d208d716b372451ca78ca52:
.. _cid-s2_numericalintegrator_ode113b_t::phase1:
.. _doxid-structs2__numericalintegrator__ode113b__t_1ab8c660c563479533d95276a334afb827:
.. _cid-s2_numericalintegrator_ode113b_t::nornd:
.. _doxid-structs2__numericalintegrator__ode113b__t_1a0d492a38d1b1d9da6f6b2fb0fd944a46:
.. _cid-s2_numericalintegrator_ode113b_t::start:
.. _doxid-structs2__numericalintegrator__ode113b__t_1a350fff683d7340992799c5a0a2160ebe:
.. _cid-s2_numericalintegrator_ode113b_t::baumgarteomegan2:
.. _doxid-structs2__numericalintegrator__ode113b__t_1a3040f01a73a4bc4e6de1cdd238ed2f8a:
.. _cid-s2_numericalintegrator_ode113b_t::baumgarte2omegan:
.. _doxid-structs2__numericalintegrator__ode113b__t_1ae9ff65fa3e88fa11c1691733cb2578bd:
.. _cid-s2_numericalintegrator_ode113b_t::smoothingfilter:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	double* :ref:`y<doxid-structs2__numericalintegrator__ode113b__t_1afae834302e664e9e693c920674e50c99>`
	double* :ref:`dy<doxid-structs2__numericalintegrator__ode113b__t_1a54875c0e1b1c87f58340872741b71423>`
	double* :ref:`ydata<doxid-structs2__numericalintegrator__ode113b__t_1a8ed1f4e3a2cee1ba15fc614bddc3e1ae>`
	double* yy
	double* dyout
	double* wt
	double* p
	double* yerr
	double** phi
	int stepCount
	char* :ref:`outFile<doxid-structs2__numericalintegrator__ode113b__t_1a09b0aca0fa20296138f8ab637e9bd30b>`
	int :ref:`neqn<doxid-structs2__numericalintegrator__ode113b__t_1ac7a2b7656972362f0f1b32d9af224634>`
	int :ref:`neqn2<doxid-structs2__numericalintegrator__ode113b__t_1aec50a57817a69b9fe22405152b8da8cc>`
	double :ref:`t<doxid-structs2__numericalintegrator__ode113b__t_1a1fc6738295a1a878dbb46c005d139b4f>`
	double* :ref:`tdata<doxid-structs2__numericalintegrator__ode113b__t_1a5072db5d422eeff325be81a033f61a33>`
	double :ref:`tout<doxid-structs2__numericalintegrator__ode113b__t_1a929142ee1cc88a48b4f69a1b48379e90>`
	int :ref:`iflag<doxid-structs2__numericalintegrator__ode113b__t_1a8635b20f6f74be45d0ee0a6cc6d9de86>`
	bool newtonRaphsonFailure
	int iterationCount
	:ref:`s2_stack_t<doxid-structs2__stack__t>`* :ref:`reconfigurations<doxid-structs2__numericalintegrator__ode113b__t_1a0f31c4b07cbfb6917522b805ec515aaa>`
	double :ref:`twou<doxid-structs2__numericalintegrator__ode113b__t_1a9b2321cac4474e21121999a23c6f121b>`
	double :ref:`fouru<doxid-structs2__numericalintegrator__ode113b__t_1a0ec5b998f9ccd548e86a94a58517e388>`
	double told
	double :ref:`hold<doxid-structs2__numericalintegrator__ode113b__t_1a42422580ac8b4e3bf4b3e85d81ff2b26>`
	double :ref:`h<doxid-structs2__numericalintegrator__ode113b__t_1a1d70581ee15f7f2570d6738c2f5dd960>`
	double hh
	double :ref:`eps<doxid-structs2__numericalintegrator__ode113b__t_1ac3717b067d25ee6318dff3289af0169c>`
	double :ref:`hmax<doxid-structs2__numericalintegrator__ode113b__t_1ae04862c5bc2c8ae8b3c3d2f2e504da38>`
	double :ref:`x<doxid-structs2__numericalintegrator__ode113b__t_1af5efa9ec4d18174722fb61534f24c05e>`
	double :ref:`relerr<doxid-structs2__numericalintegrator__ode113b__t_1a36ca1db3dbc0f5769a95af1838864aa3>`
	double :ref:`abserr<doxid-structs2__numericalintegrator__ode113b__t_1a38095f96ffcc82b90d73a8c928483009>`
	double :ref:`alpha<doxid-structs2__numericalintegrator__ode113b__t_1a14618342f5ac3d9e11243e378888aee4>` [13]
	double :ref:`beta<doxid-structs2__numericalintegrator__ode113b__t_1a8ac12d8c978afb7f7b880ff5a4048d37>` [13]
	double :ref:`psi<doxid-structs2__numericalintegrator__ode113b__t_1a5c25fb4a77f566a4d39dc4d27eea1455>` [13]
	double :ref:`sig<doxid-structs2__numericalintegrator__ode113b__t_1a18461ac859fe600441a287cb9ad26f8d>` [14]
	double :ref:`w<doxid-structs2__numericalintegrator__ode113b__t_1a43b52b119144b4669fd9e8c5f63092a2>` [14]
	double :ref:`v<doxid-structs2__numericalintegrator__ode113b__t_1a724e368f18f791eac418bf1b42fbf4f5>` [13]
	double :ref:`g<doxid-structs2__numericalintegrator__ode113b__t_1af9fb2ee9ae47b4b12bf4389252799ca8>` [14]
	double :ref:`gstr<doxid-structs2__numericalintegrator__ode113b__t_1a3aad9d8b3df3a87e149104bbc57f4a33>` [14]
	double :ref:`two<doxid-structs2__numericalintegrator__ode113b__t_1ab3d3fd00469e46ad93bd8bc1f593210d>` [14]
	double :ref:`rho<doxid-structs2__numericalintegrator__ode113b__t_1a1505744fa233fc376bdf178d97dc2ae6>` [14]
	double :ref:`stabilityLimits<doxid-structs2__numericalintegrator__ode113b__t_1a44e7015dab4b10cb0914f62f10b93b63>` [17]
	double safetyFactor
	int ns
	int ifail
	bool phase1
	bool nornd
	bool :ref:`crash<doxid-structs2__numericalintegrator__ode113b__t_1ac44935d067d4db238eb8f37c2d11dc79>`
	bool start
	int :ref:`k<doxid-structs2__numericalintegrator__ode113b__t_1a092b8221aefbc92fce2222eef0d7544b>`
	int :ref:`kold<doxid-structs2__numericalintegrator__ode113b__t_1a8dfafaafc7c8dea7644a8557bbb35111>`
	int :ref:`iconv<doxid-structs2__numericalintegrator__ode113b__t_1a9cc9b6e92685ac0d22105ecbeb78cd0f>`
	int :ref:`step<doxid-structs2__numericalintegrator__ode113b__t_1a908b5de0dcd3faa99088977d05b31b36>`
	double baumgarteOmegaN2
	double baumgarte2OmegaN
	double smoothingFilter
	double* :ref:`integralError<doxid-structs2__numericalintegrator__ode113b__t_1a42964737a15ad73d75c8397234a4b231>`
	double :ref:`integralGain<doxid-structs2__numericalintegrator__ode113b__t_1a2c12a18c1f267e825895d26b0ce65259>`

.. _details-doxid-structs2__numericalintegrator__ode113b__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Attributes associated with numerical integrator.

Fields
------

.. _doxid-structs2__numericalintegrator__ode113b__t_1afae834302e664e9e693c920674e50c99:
.. _cid-s2_numericalintegrator_ode113b_t::y:
.. ref-code-block:: cpp
	:class: title-code-block

	double* y

y is a de copy of the states communicated to the calling program. Calling program provides y as the the initial conditions. de copies y into yy and sends this to step. If step fails to advance solution, de copies latest yy from step back to y and returns this result to calling program for continuation.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a54875c0e1b1c87f58340872741b71423:
.. _cid-s2_numericalintegrator_ode113b_t::dy:
.. ref-code-block:: cpp
	:class: title-code-block

	double* dy

This is the array de passes to step to use for predicted and corrected state derivatives. If successful, step computes dy and returns it to de as dy. If step is unsuccessful, de just ignores dy.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a8ed1f4e3a2cee1ba15fc614bddc3e1ae:
.. _cid-s2_numericalintegrator_ode113b_t::ydata:
.. ref-code-block:: cpp
	:class: title-code-block

	double* ydata

array to store results from numerical integration

.. _doxid-structs2__numericalintegrator__ode113b__t_1a09b0aca0fa20296138f8ab637e9bd30b:
.. _cid-s2_numericalintegrator_ode113b_t::outfile:
.. ref-code-block:: cpp
	:class: title-code-block

	char* outFile

approximate number of steps in ydata array, found by tspan / tstep filename to write results to

.. _doxid-structs2__numericalintegrator__ode113b__t_1ac7a2b7656972362f0f1b32d9af224634:
.. _cid-s2_numericalintegrator_ode113b_t::neqn:
.. ref-code-block:: cpp
	:class: title-code-block

	int neqn

number of equations for dynamics

.. _doxid-structs2__numericalintegrator__ode113b__t_1aec50a57817a69b9fe22405152b8da8cc:
.. _cid-s2_numericalintegrator_ode113b_t::neqn2:
.. ref-code-block:: cpp
	:class: title-code-block

	int neqn2

total number of equations including additional state variables that must be integrated for force modules

.. _doxid-structs2__numericalintegrator__ode113b__t_1a1fc6738295a1a878dbb46c005d139b4f:
.. _cid-s2_numericalintegrator_ode113b_t::t:
.. ref-code-block:: cpp
	:class: title-code-block

	double t

current value of the independent variable

.. _doxid-structs2__numericalintegrator__ode113b__t_1a5072db5d422eeff325be81a033f61a33:
.. _cid-s2_numericalintegrator_ode113b_t::tdata:
.. ref-code-block:: cpp
	:class: title-code-block

	double* tdata

saved reported values of t, equal to number of entries in ydata / neqn

.. _doxid-structs2__numericalintegrator__ode113b__t_1a929142ee1cc88a48b4f69a1b48379e90:
.. _cid-s2_numericalintegrator_ode113b_t::tout:
.. ref-code-block:: cpp
	:class: title-code-block

	double tout

the desired value of t of simulation output

.. _doxid-structs2__numericalintegrator__ode113b__t_1a8635b20f6f74be45d0ee0a6cc6d9de86:
.. _cid-s2_numericalintegrator_ode113b_t::iflag:
.. ref-code-block:: cpp
	:class: title-code-block

	int iflag

indicates the status of integration. On input, IFLAG is normally 1 (or -1 in the special case where TOUT is not to be exceeded.) On normal output, IFLAG is 2. Other output values are: 3, integration did not reach TOUT because the error tolerances were too small. But RELERR and ABSERR were increased appropriately for continuing; 4, integration did not reach TOUT because more than 500 steps were taken; 5, integration did not reach TOUT because the equations appear to be stiff; 6, invalid input parameters (fatal error). The value of IFLAG is returned negative when the input value is negative and the integration does not reach TOUT.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a0f31c4b07cbfb6917522b805ec515aaa:
.. _cid-s2_numericalintegrator_ode113b_t::reconfigurations:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stack_t<doxid-structs2__stack__t>`* reconfigurations

records time step indices of a full factor event during a running simulation

.. _doxid-structs2__numericalintegrator__ode113b__t_1a9b2321cac4474e21121999a23c6f121b:
.. _cid-s2_numericalintegrator_ode113b_t::twou:
.. ref-code-block:: cpp
	:class: title-code-block

	double twou

u is based on the machine unit roundoff error. it is the smallest positive number such that 1.0+u .gt. 1.0. the user must calculate u and assign twou = 2.0*u and fouru=4.0*u in the following statements before calling the code.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a0ec5b998f9ccd548e86a94a58517e388:
.. _cid-s2_numericalintegrator_ode113b_t::fouru:
.. ref-code-block:: cpp
	:class: title-code-block

	double fouru

u is based on the machine unit roundoff error. it is the smallest positive number such that 1.0+u .gt. 1.0. the user must calculate u and assign twou = 2.0*u and fouru=4.0*u in the following statements before calling the code.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a42422580ac8b4e3bf4b3e85d81ff2b26:
.. _cid-s2_numericalintegrator_ode113b_t::hold:
.. ref-code-block:: cpp
	:class: title-code-block

	double hold

Previous time stepsize used by de and step

.. _doxid-structs2__numericalintegrator__ode113b__t_1a1d70581ee15f7f2570d6738c2f5dd960:
.. _cid-s2_numericalintegrator_ode113b_t::h:
.. ref-code-block:: cpp
	:class: title-code-block

	double h

the nominal step size indicating direction of integration and maximum size of step. H must be a variable, not a constant

.. _doxid-structs2__numericalintegrator__ode113b__t_1ac3717b067d25ee6318dff3289af0169c:
.. _cid-s2_numericalintegrator_ode113b_t::eps:
.. ref-code-block:: cpp
	:class: title-code-block

	double eps

local error tolerance per step, must be variable, not a constant

.. _doxid-structs2__numericalintegrator__ode113b__t_1ae04862c5bc2c8ae8b3c3d2f2e504da38:
.. _cid-s2_numericalintegrator_ode113b_t::hmax:
.. ref-code-block:: cpp
	:class: title-code-block

	double hmax

Maximum allowed integration stepsize.

.. _doxid-structs2__numericalintegrator__ode113b__t_1af5efa9ec4d18174722fb61534f24c05e:
.. _cid-s2_numericalintegrator_ode113b_t::x:
.. ref-code-block:: cpp
	:class: title-code-block

	double x

Time variable passed by de to and changed by step

.. _doxid-structs2__numericalintegrator__ode113b__t_1a36ca1db3dbc0f5769a95af1838864aa3:
.. _cid-s2_numericalintegrator_ode113b_t::relerr:
.. ref-code-block:: cpp
	:class: title-code-block

	double relerr

relative error

.. _doxid-structs2__numericalintegrator__ode113b__t_1a38095f96ffcc82b90d73a8c928483009:
.. _cid-s2_numericalintegrator_ode113b_t::abserr:
.. ref-code-block:: cpp
	:class: title-code-block

	double abserr

absolute error

.. _doxid-structs2__numericalintegrator__ode113b__t_1a14618342f5ac3d9e11243e378888aee4:
.. _cid-s2_numericalintegrator_ode113b_t::alpha:
.. ref-code-block:: cpp
	:class: title-code-block

	double alpha [13]

Internal array used to set up polynomials.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a8ac12d8c978afb7f7b880ff5a4048d37:
.. _cid-s2_numericalintegrator_ode113b_t::beta:
.. ref-code-block:: cpp
	:class: title-code-block

	double beta [13]

Internal array used to set up polynomials.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a5c25fb4a77f566a4d39dc4d27eea1455:
.. _cid-s2_numericalintegrator_ode113b_t::psi:
.. ref-code-block:: cpp
	:class: title-code-block

	double psi [13]

Internal array used to set up polynomials.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a18461ac859fe600441a287cb9ad26f8d:
.. _cid-s2_numericalintegrator_ode113b_t::sig:
.. ref-code-block:: cpp
	:class: title-code-block

	double sig [14]

Internal array used to set up polynomials.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a43b52b119144b4669fd9e8c5f63092a2:
.. _cid-s2_numericalintegrator_ode113b_t::w:
.. ref-code-block:: cpp
	:class: title-code-block

	double w [14]

Internal array used to set up polynomials.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a724e368f18f791eac418bf1b42fbf4f5:
.. _cid-s2_numericalintegrator_ode113b_t::v:
.. ref-code-block:: cpp
	:class: title-code-block

	double v [13]

Internal array used to set up polynomials.

.. _doxid-structs2__numericalintegrator__ode113b__t_1af9fb2ee9ae47b4b12bf4389252799ca8:
.. _cid-s2_numericalintegrator_ode113b_t::g:
.. ref-code-block:: cpp
	:class: title-code-block

	double g [14]

Internal array used to set up polynomials.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a3aad9d8b3df3a87e149104bbc57f4a33:
.. _cid-s2_numericalintegrator_ode113b_t::gstr:
.. ref-code-block:: cpp
	:class: title-code-block

	double gstr [14]

Internal array used to set up polynomials.

.. _doxid-structs2__numericalintegrator__ode113b__t_1ab3d3fd00469e46ad93bd8bc1f593210d:
.. _cid-s2_numericalintegrator_ode113b_t::two:
.. ref-code-block:: cpp
	:class: title-code-block

	double two [14]

Internal array used to set up polynomials.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a1505744fa233fc376bdf178d97dc2ae6:
.. _cid-s2_numericalintegrator_ode113b_t::rho:
.. ref-code-block:: cpp
	:class: title-code-block

	double rho [14]

Internal array used to set up polynomials.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a44e7015dab4b10cb0914f62f10b93b63:
.. _cid-s2_numericalintegrator_ode113b_t::stabilitylimits:
.. ref-code-block:: cpp
	:class: title-code-block

	double stabilityLimits [17]

stability limits, used to set adaptive omega_n

.. _doxid-structs2__numericalintegrator__ode113b__t_1ac44935d067d4db238eb8f37c2d11dc79:
.. _cid-s2_numericalintegrator_ode113b_t::crash:
.. ref-code-block:: cpp
	:class: title-code-block

	bool crash

set to true if no step can be taken

.. _doxid-structs2__numericalintegrator__ode113b__t_1a092b8221aefbc92fce2222eef0d7544b:
.. _cid-s2_numericalintegrator_ode113b_t::k:
.. ref-code-block:: cpp
	:class: title-code-block

	int k

the appropriate order for the next step

.. _doxid-structs2__numericalintegrator__ode113b__t_1a8dfafaafc7c8dea7644a8557bbb35111:
.. _cid-s2_numericalintegrator_ode113b_t::kold:
.. ref-code-block:: cpp
	:class: title-code-block

	int kold

the order used on the last successful step

.. _doxid-structs2__numericalintegrator__ode113b__t_1a9cc9b6e92685ac0d22105ecbeb78cd0f:
.. _cid-s2_numericalintegrator_ode113b_t::iconv:
.. ref-code-block:: cpp
	:class: title-code-block

	int iconv

Integrator status flag provided to fcn. iconv = 1 when fcn is called after a successful correction. In this case the states are accurate. iconv = 0 when fcn is called after a prediction following a successful correction. iconv = -1 when fcn is called after a prediction following an unsuccessful correction.

.. _doxid-structs2__numericalintegrator__ode113b__t_1a908b5de0dcd3faa99088977d05b31b36:
.. _cid-s2_numericalintegrator_ode113b_t::step:
.. ref-code-block:: cpp
	:class: title-code-block

	int step

current time step of numerical integration

.. _doxid-structs2__numericalintegrator__ode113b__t_1a42964737a15ad73d75c8397234a4b231:
.. _cid-s2_numericalintegrator_ode113b_t::integralerror:
.. ref-code-block:: cpp
	:class: title-code-block

	double* integralError

integral of error, equal to number of constraints

.. _doxid-structs2__numericalintegrator__ode113b__t_1a2c12a18c1f267e825895d26b0ce65259:
.. _cid-s2_numericalintegrator_ode113b_t::integralgain:
.. ref-code-block:: cpp
	:class: title-code-block

	double integralGain

integral gain

