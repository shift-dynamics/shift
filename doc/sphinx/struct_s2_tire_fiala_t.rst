.. index:: pair: struct; s2_tire_fiala_t
.. _doxid-structs2__tire__fiala__t:
.. _cid-s2_tire_fiala_t:

struct s2_tire_fiala_t
======================

.. toctree::
	:hidden:



Overview
~~~~~~~~



.. _doxid-structs2__tire__fiala__t_1a10b51c60534b9a63acee7d24cc0142eb:
.. _cid-s2_tire_fiala_t::verticalstiffness:
.. _doxid-structs2__tire__fiala__t_1ab526c4b473ea7c1dbe3bb92ddfee650d:
.. _cid-s2_tire_fiala_t::verticaldamping:
.. _doxid-structs2__tire__fiala__t_1a0036d9eeee088968ca7c9245502025f8:
.. _cid-s2_tire_fiala_t::rollingresistance:
.. _doxid-structs2__tire__fiala__t_1a600bc7559233f85111d104e5392f85e5:
.. _cid-s2_tire_fiala_t::width:
.. _doxid-structs2__tire__fiala__t_1ace14dea026dff2650aea215d2232c585:
.. _cid-s2_tire_fiala_t::cslip:
.. _doxid-structs2__tire__fiala__t_1a89ff30de83a72a9b736d33a2b39e8bd0:
.. _cid-s2_tire_fiala_t::calpha:
.. _doxid-structs2__tire__fiala__t_1a9a204d5e6d2f8bb7e8625b737b5e8aeb:
.. _cid-s2_tire_fiala_t::umax:
.. _doxid-structs2__tire__fiala__t_1ab825a5ae0f22e6fd745a7f8a35cd88fc:
.. _cid-s2_tire_fiala_t::umin:
.. _doxid-structs2__tire__fiala__t_1a1c0e9547098ab0bcfb940091db19199c:
.. _cid-s2_tire_fiala_t::xrelaxationlength:
.. _doxid-structs2__tire__fiala__t_1a9a18606954cabea34d1c896dc95c060c:
.. _cid-s2_tire_fiala_t::yrelaxationlength:
.. _doxid-structs2__tire__fiala__t_1a86df5ff1d9d2a4f9c122e1245063dcec:
.. _cid-s2_tire_fiala_t::omega:
.. _doxid-structs2__tire__fiala__t_1a15f557ccd270939877c867ca9ecbb151:
.. _cid-s2_tire_fiala_t::slipangle:
.. _doxid-structs2__tire__fiala__t_1aab4a19bfbd17c8270d4881351129c6cb:
.. _cid-s2_tire_fiala_t::longitudinalslip:
.. _doxid-structs2__tire__fiala__t_1ad2eef3cec887323b0f93712a07837564:
.. _cid-s2_tire_fiala_t::camberangle:
.. _doxid-structs2__tire__fiala__t_1a7a68800f3092aa72d2cdc84539eecd2d:
.. _cid-s2_tire_fiala_t::wframe:
.. _doxid-structs2__tire__fiala__t_1a0d9d7c869242634e7f08969783214695:
.. _cid-s2_tire_fiala_t::incontact:
.. _doxid-structs2__tire__fiala__t_1ac7f938e2dd4bdd86bb65b977f240b582:
.. _cid-s2_tire_fiala_t::flocal:
.. _doxid-structs2__tire__fiala__t_1aacf016278e3ad6a26df7ade49145e2b8:
.. _cid-s2_tire_fiala_t::fglobal:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	double verticalStiffness
	double verticalDamping
	double :ref:`r0<doxid-structs2__tire__fiala__t_1a39e50ade21d259c4d3fc06d4d4cb37b6>`
	double rollingResistance
	double width
	double cslip
	double calpha
	double uMax
	double uMin
	double xRelaxationLength
	double yRelaxationLength
	:ref:`s2_screw_t<doxid-structs2__screw__t>` :ref:`force<doxid-structs2__tire__fiala__t_1ad8e69ee259791100d76fb162d3ac15b2>`
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`wheelBody<doxid-structs2__tire__fiala__t_1a869402408288e514f5ee6039746f04a2>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`wheelBodyPointer<doxid-structs2__tire__fiala__t_1a0b6e1429d58c65471501f0fe0862dfe8>`
	:ref:`s2_screw_t<doxid-structs2__screw__t>` :ref:`velocityLocal<doxid-structs2__tire__fiala__t_1a80b8b313c2d649ddd3c9936628e4a030>`
	:ref:`s2_vector_t<doxid-structs2__vector__t>` :ref:`wheelNormal<doxid-structs2__tire__fiala__t_1a0b9fad55335a562ccdfe0b09dc17d14c>`
	:ref:`s2_vector_t<doxid-structs2__vector__t>` :ref:`terrainNormal<doxid-structs2__tire__fiala__t_1aedee4aaa200b2f996aa0e0a5e393d584>`
	:ref:`s2_vector_t<doxid-structs2__vector__t>` :ref:`xDirection<doxid-structs2__tire__fiala__t_1a0f0a78359f8001c497b79750351dd359>`
	:ref:`s2_vector_t<doxid-structs2__vector__t>` :ref:`yDirection<doxid-structs2__tire__fiala__t_1a2fac292d98bcc90f25289575bee7cdd0>`
	double omega
	double slipAngle
	double longitudinalSlip
	double camberAngle
	double :ref:`depth<doxid-structs2__tire__fiala__t_1aad00707d9e73b0d03742611ce7274b4e>`
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>` wFrame
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>` :ref:`contactFrame<doxid-structs2__tire__fiala__t_1a430c5b206ccef5cec10c383104311e10>`
	bool inContact
	:ref:`s2_screw_t<doxid-structs2__screw__t>` fLocal
	:ref:`s2_screw_t<doxid-structs2__screw__t>` fGlobal
	double :ref:`y<doxid-structs2__tire__fiala__t_1a4ce4dbc1c2ca9a7ad745ae17349df4fa>` [2]
	double* :ref:`dy<doxid-structs2__tire__fiala__t_1ab4921f8c59a81e1384cf7ee4ff505a95>`

.. _details-doxid-structs2__tire__fiala__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. _doxid-structs2__tire__fiala__t_1a39e50ade21d259c4d3fc06d4d4cb37b6:
.. _cid-s2_tire_fiala_t::r0:
.. ref-code-block:: cpp
	:class: title-code-block

	double r0

unloaded radius

.. _doxid-structs2__tire__fiala__t_1ad8e69ee259791100d76fb162d3ac15b2:
.. _cid-s2_tire_fiala_t::force:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_screw_t<doxid-structs2__screw__t>` force

reaction force in w frame

.. _doxid-structs2__tire__fiala__t_1a869402408288e514f5ee6039746f04a2:
.. _cid-s2_tire_fiala_t::wheelbody:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* wheelBody

linked list of char arrays representing path to wheel body in assembly hierarchy

.. _doxid-structs2__tire__fiala__t_1a0b6e1429d58c65471501f0fe0862dfe8:
.. _cid-s2_tire_fiala_t::wheelbodypointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* wheelBodyPointer

pointer to wheel body

.. _doxid-structs2__tire__fiala__t_1a80b8b313c2d649ddd3c9936628e4a030:
.. _cid-s2_tire_fiala_t::velocitylocal:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_screw_t<doxid-structs2__screw__t>` velocityLocal

velocity screw expressed in frame w

.. _doxid-structs2__tire__fiala__t_1a0b9fad55335a562ccdfe0b09dc17d14c:
.. _cid-s2_tire_fiala_t::wheelnormal:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vector_t<doxid-structs2__vector__t>` wheelNormal

wheel normal expressed in global frame

.. _doxid-structs2__tire__fiala__t_1aedee4aaa200b2f996aa0e0a5e393d584:
.. _cid-s2_tire_fiala_t::terrainnormal:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vector_t<doxid-structs2__vector__t>` terrainNormal

terrain normal at wheel location

.. _doxid-structs2__tire__fiala__t_1a0f0a78359f8001c497b79750351dd359:
.. _cid-s2_tire_fiala_t::xdirection:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vector_t<doxid-structs2__vector__t>` xDirection

longitudinal heading in terrain plane

.. _doxid-structs2__tire__fiala__t_1a2fac292d98bcc90f25289575bee7cdd0:
.. _cid-s2_tire_fiala_t::ydirection:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vector_t<doxid-structs2__vector__t>` yDirection

lateral direction in terrain plane

.. _doxid-structs2__tire__fiala__t_1aad00707d9e73b0d03742611ce7274b4e:
.. _cid-s2_tire_fiala_t::depth:
.. ref-code-block:: cpp
	:class: title-code-block

	double depth

penetration depth

.. _doxid-structs2__tire__fiala__t_1a430c5b206ccef5cec10c383104311e10:
.. _cid-s2_tire_fiala_t::contactframe:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_displacement_t<doxid-structs2__displacement__t>` contactFrame

frame of contact patch following ISO conventions, forces are computed in w frame

.. _doxid-structs2__tire__fiala__t_1a4ce4dbc1c2ca9a7ad745ae17349df4fa:
.. _cid-s2_tire_fiala_t::y:
.. ref-code-block:: cpp
	:class: title-code-block

	double y [2]

store a copy of initial conditions here

.. _doxid-structs2__tire__fiala__t_1ab4921f8c59a81e1384cf7ee4ff505a95:
.. _cid-s2_tire_fiala_t::dy:
.. ref-code-block:: cpp
	:class: title-code-block

	double* dy

pointer to yp array, to be set in :ref:`s2_forcemodule_connect() <doxid-forcemodule_8c_1a65c554aaea385eb8ec611f9609469d7e>` function

