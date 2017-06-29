.. index:: pair: struct; s2_restraint_t
.. _doxid-structs2__restraint__t:
.. _cid-s2_restraint_t:

struct s2_restraint_t
=====================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <context.h>


defines a Restraint

.. _doxid-structs2__restraint__t_1a2d99d761ca71a564bd39c79afad47c4c:
.. _cid-s2_restraint_t::name:
.. _doxid-structs2__restraint__t_1a8c8be09fa35fbb64117e49f298ff5735:
.. _cid-s2_restraint_t::knots:
.. _doxid-structs2__restraint__t_1a7011462f4c2bdf4a3a21ac86655eca08:
.. _cid-s2_restraint_t::coefficients:
.. _doxid-structs2__restraint__t_1aadbca46edffc8609b37428d22a63c45c:
.. _cid-s2_restraint_t::index:
.. _doxid-structs2__restraint__t_1ad179f6c917282cafd6010eb52103396a:
.. _cid-s2_restraint_t::restrainttype:
.. _doxid-structs2__restraint__t_1a18c38399e7505c2a9b02d00c8becde7c:
.. _cid-s2_restraint_t::coordinatetype:
.. _doxid-structs2__restraint__t_1a75f2a9f976876a054f9655fae8f0892b:
.. _cid-s2_restraint_t::distancetype:
.. _doxid-structs2__restraint__t_1a494452dc6f5a95d73c5ccb6af51a86b1:
.. _cid-s2_restraint_t::parentframe:
.. _doxid-structs2__restraint__t_1acbb5371f18e43e91a169268ac5eb030a:
.. _cid-s2_restraint_t::childframe:
.. _doxid-structs2__restraint__t_1a5d4dd9ff94f80324c1d3a45cfde466eb:
.. _cid-s2_restraint_t::next:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name
	double* knots
	double** coefficients
	int index
	:ref:`s2_restraint_enum_t<doxid-context_8h_1a76313e0949644f4f5d871ed6e881fd1d>` restraintType
	:ref:`s2_coordinate_enum_t<doxid-context_8h_1a57ade64fd6279202ad85f6ea7232dd63>` coordinateType
	:ref:`s2_distance_enum_t<doxid-context_8h_1aacd69c927ddfaa300fb4e07c00e14c0f>` distanceType
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* parentFrame
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* childFrame
	:ref:`s2_restraint_t<doxid-structs2__restraint__t>`* next

