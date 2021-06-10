.. index:: pair: struct; s2_damper_t
.. _doxid-structs2__damper__t:
.. _cid-s2_damper_t:

struct s2_damper_t
==================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <damping.h>


Overview
~~~~~~~~

damper data structure :ref:`More...<details-doxid-structs2__damper__t>`

.. _doxid-structs2__damper__t_1a171f40de4006d8db760be48c6a06a0d6:
.. _cid-s2_damper_t::order:
.. _doxid-structs2__damper__t_1a33be7c22e2482054b29d3790bafb01d4:
.. _cid-s2_damper_t::coefficients:
.. _doxid-structs2__damper__t_1a6a507784c46a110464f4af75cffcf0de:
.. _cid-s2_damper_t::parentframe:
.. _doxid-structs2__damper__t_1af8031793d49af45daccf23928c8b3dae:
.. _cid-s2_damper_t::childframe:
.. _doxid-structs2__damper__t_1ab89d27921240ef92cf01cef72fde50c5:
.. _cid-s2_damper_t::parentforce:
.. _doxid-structs2__damper__t_1a4ee7288c45d2e1cb1eabf7aa2ab5f3eb:
.. _cid-s2_damper_t::childforce:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	int :ref:`id<doxid-structs2__damper__t_1a02cf717fe43dbc427b84ad57996228a9>`
	int order
	double* coefficients
	s2_displacement_t* parentFrame
	s2_displacement_t* childFrame
	s2_screw_t* parentForce
	s2_screw_t* childForce

.. _details-doxid-structs2__damper__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

damper data structure

piecewise curves (i.e. splines) are not currently supported

Fields
------

.. _doxid-structs2__damper__t_1a02cf717fe43dbc427b84ad57996228a9:
.. _cid-s2_damper_t::id:
.. ref-code-block:: cpp
	:class: title-code-block

	int id

inputs to model

