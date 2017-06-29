.. index:: pair: struct; s2_inertia_t
.. _doxid-structs2__inertia__t:
.. _cid-s2_inertia_t:

struct s2_inertia_t
===================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <context.h>


Overview
~~~~~~~~

mass and inertia properties :ref:`More...<details-doxid-structs2__inertia__t>`

.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	double :ref:`m<doxid-structs2__inertia__t_1ab76fb0d609caee29669ee03015644c04>`
	:ref:`s2_vector_t<doxid-structs2__vector__t>` :ref:`mt<doxid-structs2__inertia__t_1abfc5cb2360cbde9691b5614e03699a49>`
	double :ref:`Ixx<doxid-structs2__inertia__t_1a423d8bc9fd9b3c43e733a68f95092abd>`
	double :ref:`Iyy<doxid-structs2__inertia__t_1a2191644c198a9d520a41545cd4f881bd>`
	double :ref:`Izz<doxid-structs2__inertia__t_1a45937092461d4cf4dab60e952109394a>`
	double :ref:`Ixy<doxid-structs2__inertia__t_1a5305bc51bc69d36f46331aa53be41c1d>`
	double :ref:`Iyz<doxid-structs2__inertia__t_1a550d5bf7af4b8f8b5b429f3ca4294253>`
	double :ref:`Izx<doxid-structs2__inertia__t_1acd18026f13df4d4f526daeafbf221603>`

.. _details-doxid-structs2__inertia__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

mass and inertia properties

Fields
------

.. _doxid-structs2__inertia__t_1ab76fb0d609caee29669ee03015644c04:
.. _cid-s2_inertia_t::m:
.. ref-code-block:: cpp
	:class: title-code-block

	double m

mass

.. _doxid-structs2__inertia__t_1abfc5cb2360cbde9691b5614e03699a49:
.. _cid-s2_inertia_t::mt:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vector_t<doxid-structs2__vector__t>` mt

mass times a vector, t, from centroid to reference frame

.. _doxid-structs2__inertia__t_1a423d8bc9fd9b3c43e733a68f95092abd:
.. _cid-s2_inertia_t::ixx:
.. ref-code-block:: cpp
	:class: title-code-block

	double Ixx

Ixx centroidal inertia term, expressed in reference frame coordinates

.. _doxid-structs2__inertia__t_1a2191644c198a9d520a41545cd4f881bd:
.. _cid-s2_inertia_t::iyy:
.. ref-code-block:: cpp
	:class: title-code-block

	double Iyy

Ixy centroidal inertia term, expressed in reference frame coordinates

.. _doxid-structs2__inertia__t_1a45937092461d4cf4dab60e952109394a:
.. _cid-s2_inertia_t::izz:
.. ref-code-block:: cpp
	:class: title-code-block

	double Izz

Izz centroidal inertia term, expressed in reference frame coordinates

.. _doxid-structs2__inertia__t_1a5305bc51bc69d36f46331aa53be41c1d:
.. _cid-s2_inertia_t::ixy:
.. ref-code-block:: cpp
	:class: title-code-block

	double Ixy

Ixy centroidal inertia term, expressed in reference frame coordinates

.. _doxid-structs2__inertia__t_1a550d5bf7af4b8f8b5b429f3ca4294253:
.. _cid-s2_inertia_t::iyz:
.. ref-code-block:: cpp
	:class: title-code-block

	double Iyz

Iyz centroidal inertia term, expressed in reference frame coordinates

.. _doxid-structs2__inertia__t_1acd18026f13df4d4f526daeafbf221603:
.. _cid-s2_inertia_t::izx:
.. ref-code-block:: cpp
	:class: title-code-block

	double Izx

Izx centroidal inertia term, expressed in reference frame coordinates

