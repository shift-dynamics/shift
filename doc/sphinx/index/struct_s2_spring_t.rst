.. index:: pair: struct; s2_spring_t
.. _doxid-structs2__spring__t:
.. _cid-s2_spring_t:

struct s2_spring_t
==================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <stiffness.h>


Overview
~~~~~~~~

spring data structure :ref:`More...<details-doxid-structs2__spring__t>`

.. _doxid-structs2__spring__t_1ac8714a931ab8b150298dd1d0a65a4879:
.. _cid-s2_spring_t::order:
.. _doxid-structs2__spring__t_1a2e11b4c770be7b0a6dbe8e81eff53c11:
.. _cid-s2_spring_t::coefficients:
.. _doxid-structs2__spring__t_1af1340cf924afc5e0723fb178033e38b2:
.. _cid-s2_spring_t::parentframe:
.. _doxid-structs2__spring__t_1aeba8408501abe4aabdf15081a136c7ac:
.. _cid-s2_spring_t::childframe:
.. _doxid-structs2__spring__t_1a3f720ecc5f7584e0738d0313565797ad:
.. _cid-s2_spring_t::parentforce:
.. _doxid-structs2__spring__t_1adafb0564be52846ecaf2e5d939479b12:
.. _cid-s2_spring_t::childforce:
.. _doxid-structs2__spring__t_1ac0a6c6767e1f5d7fe9271d4c602e8180:
.. _cid-s2_spring_t::jointpointer:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	int :ref:`id<doxid-structs2__spring__t_1ab54b1671716a62537e35c4786f8d1238>`
	int order
	double* coefficients
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* parentFrame
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* childFrame
	:ref:`s2_screw_t<doxid-structs2__screw__t>`* parentForce
	:ref:`s2_screw_t<doxid-structs2__screw__t>`* childForce
	:ref:`s2_joint_t<doxid-structs2__joint__t>`* jointPointer

.. _details-doxid-structs2__spring__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

spring data structure

piecewise curves (i.e. splines) are not currently supported

Fields
------

.. _doxid-structs2__spring__t_1ab54b1671716a62537e35c4786f8d1238:
.. _cid-s2_spring_t::id:
.. ref-code-block:: cpp
	:class: title-code-block

	int id

inputs to model

