.. index:: pair: struct; s2_frame_t
.. _doxid-structs2__frame__t:
.. _cid-s2_frame_t:

struct s2_frame_t
=================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <context.h>


Overview
~~~~~~~~

Frame or coordinate system. :ref:`More...<details-doxid-structs2__frame__t>`

.. _doxid-structs2__frame__t_1ac0281533866d9af34e466ae540be4861:
.. _cid-s2_frame_t::name:
.. _doxid-structs2__frame__t_1a6e4cf15a709c3df5500d7f537f75b274:
.. _cid-s2_frame_t::childframe:
.. _doxid-structs2__frame__t_1ad30622d3d090937da4f387660283858b:
.. _cid-s2_frame_t::accelerationquadraticabsolute:
.. _doxid-structs2__frame__t_1a15b6caabdce5054db627a364a850cb26:
.. _cid-s2_frame_t::index:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`referenceFramePointer<doxid-structs2__frame__t_1ad5ddbc87d8b5bc34363c7b324fe838db>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`next<doxid-structs2__frame__t_1a32975a079bf25e6e9bb5cb66ff8d7ad6>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* childFrame
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* :ref:`dWRTOrigin<doxid-structs2__frame__t_1ac5b7d84bd5eecd2e57e4685fdb1da8de>`
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* :ref:`dWRTOriginInverse<doxid-structs2__frame__t_1a008a4127a1b44c6770e0c0f8ccf1206f>`
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* :ref:`dWRTReference<doxid-structs2__frame__t_1aa09e0c902d269b34af1018255b326681>`
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* :ref:`dWRTReferenceInverse<doxid-structs2__frame__t_1aea9ad5d1b138eb17bf5e887f47455605>`
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* :ref:`dAbsolute<doxid-structs2__frame__t_1a4e212ecef16e35a3798393e3f9a33ecd>`
	:ref:`s2_screw_t<doxid-structs2__screw__t>`* :ref:`velocityAbsolute<doxid-structs2__frame__t_1a28e1b0a15d3b1c06256578c498f19d82>`
	:ref:`s2_screw_t<doxid-structs2__screw__t>`* accelerationQuadraticAbsolute
	bool :ref:`isReference<doxid-structs2__frame__t_1a9d5d41194f7abe12d2d373c60eae5acc>`
	bool :ref:`isOrigin<doxid-structs2__frame__t_1ad0f2628d7143df3e2c974f70a805ac09>`
	bool :ref:`isParent<doxid-structs2__frame__t_1a753ea07bfff5cd3c4d43ce39dbccd8c3>`
	bool :ref:`inTree<doxid-structs2__frame__t_1a3c57a7d23ce9df204a5737745e7fc70f>`
	int index
	int :ref:`dIndex<doxid-structs2__frame__t_1aaf398f65d822d694dca61cf098997338>`
	int :ref:`vaIndex<doxid-structs2__frame__t_1a3ae2dadba7da60e3a53cba6f68ab9302>`

.. _details-doxid-structs2__frame__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Frame or coordinate system.

Fields
------

.. _doxid-structs2__frame__t_1ad5ddbc87d8b5bc34363c7b324fe838db:
.. _cid-s2_frame_t::referenceframepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* referenceFramePointer

reference frame of the body set to null if bool isParent = true, i.e. this frame is the reference frame

.. _doxid-structs2__frame__t_1a32975a079bf25e6e9bb5cb66ff8d7ad6:
.. _cid-s2_frame_t::next:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* next

next frame in linked list

.. _doxid-structs2__frame__t_1ac5b7d84bd5eecd2e57e4685fdb1da8de:
.. _cid-s2_frame_t::dwrtorigin:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* dWRTOrigin

rotation with respect to origin

.. _doxid-structs2__frame__t_1a008a4127a1b44c6770e0c0f8ccf1206f:
.. _cid-s2_frame_t::dwrtorigininverse:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* dWRTOriginInverse

inverse of rotation with respect

.. _doxid-structs2__frame__t_1aa09e0c902d269b34af1018255b326681:
.. _cid-s2_frame_t::dwrtreference:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* dWRTReference

displacement with respect to reference frame

.. _doxid-structs2__frame__t_1aea9ad5d1b138eb17bf5e887f47455605:
.. _cid-s2_frame_t::dwrtreferenceinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* dWRTReferenceInverse

rotation with respect to reference frame inverse

.. _doxid-structs2__frame__t_1a4e212ecef16e35a3798393e3f9a33ecd:
.. _cid-s2_frame_t::dabsolute:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* dAbsolute

absolute rotation in inertial reference frame

.. _doxid-structs2__frame__t_1a28e1b0a15d3b1c06256578c498f19d82:
.. _cid-s2_frame_t::velocityabsolute:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_screw_t<doxid-structs2__screw__t>`* velocityAbsolute

for a rigid body, the spatial velocity is constant for all frames on the body, for flexible bodies, the spatial velocity is not constant

.. _doxid-structs2__frame__t_1a9d5d41194f7abe12d2d373c60eae5acc:
.. _cid-s2_frame_t::isreference:
.. ref-code-block:: cpp
	:class: title-code-block

	bool isReference

for a rigid body, the spatial acceleration is constant for all frames on the body, for flexible bodies, the spatial acceleration is not constant designates if the current frame is the reference frame - or is the child of a Joint

.. _doxid-structs2__frame__t_1ad0f2628d7143df3e2c974f70a805ac09:
.. _cid-s2_frame_t::isorigin:
.. ref-code-block:: cpp
	:class: title-code-block

	bool isOrigin

designates whether the current frame is the origin

.. _doxid-structs2__frame__t_1a753ea07bfff5cd3c4d43ce39dbccd8c3:
.. _cid-s2_frame_t::isparent:
.. ref-code-block:: cpp
	:class: title-code-block

	bool isParent

designates if the current frame is the parent of a joint

.. _doxid-structs2__frame__t_1a3c57a7d23ce9df204a5737745e7fc70f:
.. _cid-s2_frame_t::intree:
.. ref-code-block:: cpp
	:class: title-code-block

	bool inTree

designates if the current frame is either the parent or child of a joint

.. _doxid-structs2__frame__t_1aaf398f65d822d694dca61cf098997338:
.. _cid-s2_frame_t::dindex:
.. ref-code-block:: cpp
	:class: title-code-block

	int dIndex

index into d and dWRTReference arrays

.. _doxid-structs2__frame__t_1a3ae2dadba7da60e3a53cba6f68ab9302:
.. _cid-s2_frame_t::vaindex:
.. ref-code-block:: cpp
	:class: title-code-block

	int vaIndex

index into velocity and acceleration arrays

