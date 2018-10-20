.. index:: pair: struct; s2_pid_t
.. _doxid-structs2__pid__t:
.. _cid-s2_pid_t:

struct s2_pid_t
===============

.. toctree::
	:hidden:

	struct_s2_pid_t_gains.rst
	struct_s2_pid_t_input.rst
	struct_s2_pid_t_output.rst

.. code-block:: cpp
	:class: overview-code-block

	#include <pid.h>


initially set up to apply a force in a joint

.. _doxid-structs2__pid__t_1a35db9bdc9ccd5ba96f9dcabcee114852:
.. _cid-s2_pid_t::gains:
.. _doxid-structs2__pid__t_1adebfa285ac42736a61cd08ab5df15982:
.. _cid-s2_pid_t::target:
.. _doxid-structs2__pid__t_1a2f5f63275618de04ac673fb22a397574:
.. _cid-s2_pid_t::error:
.. _doxid-structs2__pid__t_1a061d58809f0e7a201bd0c40780c4d67f:
.. _cid-s2_pid_t::lasterror:
.. _doxid-structs2__pid__t_1a17bde9b1d16424bc5db2c3db4e201b5b:
.. _cid-s2_pid_t::input:
.. _doxid-structs2__pid__t_1a8a905ac0c0bfbf3e4bd82cb4c29aeaa1:
.. _cid-s2_pid_t::output:
.. _doxid-structs2__pid__t_1a1cce6ca9ce8651f8a53a399abd3559a7:
.. _cid-s2_pid_t::dy:
.. _doxid-structs2__pid__t_1a3048886558480c3fefd3905dfa08d992:
.. _cid-s2_pid_t::y:
.. ref-code-block:: cpp
	:class: overview-code-block

	// structs

	struct :ref:`gains<doxid-structs2__pid__t_1_1gains>`
	struct :ref:`input<doxid-structs2__pid__t_1_1input>`
	struct :ref:`output<doxid-structs2__pid__t_1_1output>`

	// fields

	struct :ref:`s2_pid_t::gains<doxid-structs2__pid__t_1_1gains>` gains
	double target
	double error
	double lastError
	struct :ref:`s2_pid_t::input<doxid-structs2__pid__t_1_1input>` input
	struct :ref:`s2_pid_t::output<doxid-structs2__pid__t_1_1output>` output
	double* dy
	double y [1]

