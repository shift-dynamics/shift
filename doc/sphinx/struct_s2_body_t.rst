.. index:: pair: struct; s2_body_t
.. _doxid-structs2__body__t:
.. _cid-s2_body_t:

struct s2_body_t
================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <body.h>


Overview
~~~~~~~~

Describes properties related to a rigid body. :ref:`More...<details-doxid-structs2__body__t>`

.. _doxid-structs2__body__t_1a46bef141ee14cfe2263d82bb08e4a933:
.. _cid-s2_body_t::referenceframe:
.. _doxid-structs2__body__t_1a78805bf552bef1be2dbe2e2e5b961643:
.. _cid-s2_body_t::glreferenceframe:
.. _doxid-structs2__body__t_1aac69ce8daf64a70cfdec5e39207d1763:
.. _cid-s2_body_t::glreferenceframef:
.. _doxid-structs2__body__t_1a391718fbcda0b5152c705488e2c1ea7a:
.. _cid-s2_body_t::glmodelview:
.. _doxid-structs2__body__t_1aa452748af73c3a83e6710d32aafe8d09:
.. _cid-s2_body_t::glmodelviewprojection:
.. _doxid-structs2__body__t_1a955f4b33321b840a4d5ccfe9178dddab:
.. _cid-s2_body_t::convexhulls:
.. _doxid-structs2__body__t_1aa2d630aa11f0d0cbe370b18a7e1f4cfb:
.. _cid-s2_body_t::convexhullcolor:
.. _doxid-structs2__body__t_1a946836040ff86268eed4ab0ccba84d68:
.. _cid-s2_body_t::convexhullcount:
.. _doxid-structs2__body__t_1a5d56769a3618e73d66db804a0049198e:
.. _cid-s2_body_t::drawenabled:
.. _doxid-structs2__body__t_1a96c03ba8d88a7177521754768d7c8ea4:
.. _cid-s2_body_t::isvisible:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`name<doxid-structs2__body__t_1ae8ac6de404e9a85d780983db95f1bc02>`
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`definition<doxid-structs2__body__t_1a14ea5132768a8e51f9d11c5d13f67bd1>`
	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* :ref:`assembly<doxid-structs2__body__t_1ab24b21093b400ae94f974d7a9a5ac0f7>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`frameHead<doxid-structs2__body__t_1aff530f254547dbdcdfe1d8acb2cb920c>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`frameTail<doxid-structs2__body__t_1ae5dbebf73d15cc3a49314d87fd811a64>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* referenceFrame
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`origin<doxid-structs2__body__t_1a9d3fc6b2d50dba52389a61b51def85a2>`
	:ref:`s2_joint_t<doxid-structs2__joint__t>`* :ref:`parentJointPointer<doxid-structs2__body__t_1a6f55d13f40c6c3570707901c2396646a>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`parentBodyPointer<doxid-structs2__body__t_1a7dcff85dc48498c545c0fc18c1a8bafc>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`parentFramePointer<doxid-structs2__body__t_1a76619f388355012382ba992ac8e16024>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`next<doxid-structs2__body__t_1a875c6b3512a910ee9916e4a01788c873>`
	:ref:`s2_material_t<doxid-structs2__material__t>`* :ref:`materials<doxid-structs2__body__t_1ae3fc51a5c72e4706c7f76888c12e6046>`
	int :ref:`frameCount<doxid-structs2__body__t_1ae2c67568691309f9d33ad87450ea78af>`
	int :ref:`index<doxid-structs2__body__t_1aa5f929a9b17496aa5af30067ebc6350f>`
	int :ref:`level<doxid-structs2__body__t_1a88160894b5b5cfa1fe60ffc16399c13d>`
	int :ref:`materialCount<doxid-structs2__body__t_1a4c5ee449c0c36d2145dfc71eff97df7f>`
	bool :ref:`isVirtual<doxid-structs2__body__t_1a2a4e22a0afc05cf9e36cf0f9a5373092>`
	:ref:`s2_inertia_t<doxid-structs2__inertia__t>`* :ref:`inertia<doxid-structs2__body__t_1a81aec537e8ed4bdd5e401e57b1c6bb9b>`
	:ref:`s2_vector_t<doxid-structs2__vector__t>`* :ref:`centerOfMassWRTOrigin<doxid-structs2__body__t_1a6bbfa00987c1a3baee2b8477283747d8>`
	:ref:`s2_vector_t<doxid-structs2__vector__t>`* :ref:`centerOfMassWRTReference<doxid-structs2__body__t_1af23881d6218c4470af08fc182c323bfd>`
	:ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`* glReferenceFrame
	:ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* glReferenceFramef
	:ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* glModelView
	:ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* glModelViewProjection
	:ref:`s2_convexhull_t<doxid-structs2__convexhull__t>`* convexHulls
	GLfloat convexHullColor [3]
	int convexHullCount
	bool drawEnabled
	bool isVisible

.. _details-doxid-structs2__body__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Describes properties related to a rigid body.

A Body datastructure contains:

* A descriptive name

* A Frame linked list

* Pointer to Frame defined as origin

The following attributes are assigned by the s2_ProcessTopology() function:

* An index, which is used to identify position in minimal *run-time* arrays

* A level, which indicates level in the spanning tree

* The unique reference frame, which may not be the origin

Fields
------

.. _doxid-structs2__body__t_1ae8ac6de404e9a85d780983db95f1bc02:
.. _cid-s2_body_t::name:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name

Descriptive name of body

.. _doxid-structs2__body__t_1a14ea5132768a8e51f9d11c5d13f67bd1:
.. _cid-s2_body_t::definition:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* definition

Path to files that define body definitions, i.e. obj and mtl files

.. _doxid-structs2__body__t_1ab24b21093b400ae94f974d7a9a5ac0f7:
.. _cid-s2_body_t::assembly:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly

Pointer to assembly that body is a member of.

.. _doxid-structs2__body__t_1aff530f254547dbdcdfe1d8acb2cb920c:
.. _cid-s2_body_t::framehead:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* frameHead

head of Frame linked list

.. _doxid-structs2__body__t_1ae5dbebf73d15cc3a49314d87fd811a64:
.. _cid-s2_body_t::frametail:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* frameTail

tail of Frame linked list

.. _doxid-structs2__body__t_1a9d3fc6b2d50dba52389a61b51def85a2:
.. _cid-s2_body_t::origin:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* origin

pointer to Body's unique reference frame, which may not be the Frame's origin. Assigned automatically by s2_ProcessTopology A pointer to a body's unique *origin* coordinate system, which may not be the reference frame

.. _doxid-structs2__body__t_1a6f55d13f40c6c3570707901c2396646a:
.. _cid-s2_body_t::parentjointpointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_joint_t<doxid-structs2__joint__t>`* parentJointPointer

Pointer to Body's unique *parent* Joint datastructure

.. _doxid-structs2__body__t_1a7dcff85dc48498c545c0fc18c1a8bafc:
.. _cid-s2_body_t::parentbodypointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* parentBodyPointer

A pointer to the unique *parent* Body, set by s2_ProcessTopology

.. _doxid-structs2__body__t_1a76619f388355012382ba992ac8e16024:
.. _cid-s2_body_t::parentframepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* parentFramePointer

parent frame of body reference frame

.. _doxid-structs2__body__t_1a875c6b3512a910ee9916e4a01788c873:
.. _cid-s2_body_t::next:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* next

Pointer to next Body in linked list datastructure

.. _doxid-structs2__body__t_1ae3fc51a5c72e4706c7f76888c12e6046:
.. _cid-s2_body_t::materials:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_material_t<doxid-structs2__material__t>`* materials

:ref:`s2_material_t <doxid-structs2__material__t>` datastructure

.. _doxid-structs2__body__t_1ae2c67568691309f9d33ad87450ea78af:
.. _cid-s2_body_t::framecount:
.. ref-code-block:: cpp
	:class: title-code-block

	int frameCount

count of total number of frames

.. _doxid-structs2__body__t_1aa5f929a9b17496aa5af30067ebc6350f:
.. _cid-s2_body_t::index:
.. ref-code-block:: cpp
	:class: title-code-block

	int index

index in contiguous *run-time* array of Bodies, set by s2_ProcessTopology

.. _doxid-structs2__body__t_1a88160894b5b5cfa1fe60ffc16399c13d:
.. _cid-s2_body_t::level:
.. ref-code-block:: cpp
	:class: title-code-block

	int level

level in the spanning tree, set by s2_ProcessTopology

.. _doxid-structs2__body__t_1a4c5ee449c0c36d2145dfc71eff97df7f:
.. _cid-s2_body_t::materialcount:
.. ref-code-block:: cpp
	:class: title-code-block

	int materialCount

number of materials assigned to a body

.. _doxid-structs2__body__t_1a2a4e22a0afc05cf9e36cf0f9a5373092:
.. _cid-s2_body_t::isvirtual:
.. ref-code-block:: cpp
	:class: title-code-block

	bool isVirtual

indicates whether the Body is *virtual* . By default, all user-defined bodies are not virtual. Massless virtual bodies are added to simplify processing kinematics if multiple loops are present in the mechanism

.. _doxid-structs2__body__t_1a81aec537e8ed4bdd5e401e57b1c6bb9b:
.. _cid-s2_body_t::inertia:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia

centroidal Mass and moment of Inertia properties in origin coordinate system

.. _doxid-structs2__body__t_1a6bbfa00987c1a3baee2b8477283747d8:
.. _cid-s2_body_t::centerofmasswrtorigin:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vector_t<doxid-structs2__vector__t>`* centerOfMassWRTOrigin

vector from body origin to center of mass

.. _doxid-structs2__body__t_1af23881d6218c4470af08fc182c323bfd:
.. _cid-s2_body_t::centerofmasswrtreference:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vector_t<doxid-structs2__vector__t>`* centerOfMassWRTReference

vector from body reference frame to center of mass

