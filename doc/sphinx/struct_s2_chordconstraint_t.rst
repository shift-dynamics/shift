.. index:: pair: struct; s2_chordconstraint_t
.. _doxid-structs2__chordconstraint__t:
.. _cid-s2_chordconstraint_t:

struct s2_chordconstraint_t
===========================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <context.h>


Overview
~~~~~~~~

chordConstraintNode properties :ref:`More...<details-doxid-structs2__chordconstraint__t>`

.. _doxid-structs2__chordconstraint__t_1a1ecb5fa8cd32a054d0ce06acc58607a3:
.. _cid-s2_chordconstraint_t::level:
.. _doxid-structs2__chordconstraint__t_1a103cc8f17b2019bf62949e80ef99e883:
.. _cid-s2_chordconstraint_t::index:
.. _doxid-structs2__chordconstraint__t_1a96f6c9ccd67431e3c40e7039d3b439c4:
.. _cid-s2_chordconstraint_t::substructure:
.. _doxid-structs2__chordconstraint__t_1a340fa4ad10f9323f2fc51e04cb92f941:
.. _cid-s2_chordconstraint_t::name:
.. _doxid-structs2__chordconstraint__t_1a24ed053c8cc97444aba03a8bd72398f5:
.. _cid-s2_chordconstraint_t::parentjointpointer:
.. _doxid-structs2__chordconstraint__t_1acc333dcf2969ed5c9cbd73b77245d137:
.. _cid-s2_chordconstraint_t::childjointpointer:
.. _doxid-structs2__chordconstraint__t_1ab1d64b49f78769ad9b17f4834e7cca77:
.. _cid-s2_chordconstraint_t::next:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	int level
	int index
	int substructure
	char* name
	char* :ref:`parentBody<doxid-structs2__chordconstraint__t_1a50f1311ba5bdbde0695c319aff0b4827>`
	char* :ref:`originalParentBody<doxid-structs2__chordconstraint__t_1a39a722bd220f71f755ffc90a4bdc0197>`
	char* :ref:`parentFrame<doxid-structs2__chordconstraint__t_1a4fb2adc72939678645e0b70894e2d6e7>`
	char* :ref:`originalParentFrame<doxid-structs2__chordconstraint__t_1ab2c8a12ab7b4d43b2edb92166f1c2d18>`
	char* :ref:`childBody<doxid-structs2__chordconstraint__t_1aac5c1ce73fd2c65ca8c8c7eca9544b87>`
	char* :ref:`childFrame<doxid-structs2__chordconstraint__t_1a91134170da1b8270623f4b93a9f4b966>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`parentBodyPointer<doxid-structs2__chordconstraint__t_1aa0ba0c1463541b0cab68df0843b7ca05>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`originalParentBodyPointer<doxid-structs2__chordconstraint__t_1a12db7f3bdf9cb97b71604baa38d11992>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`parentFramePointer<doxid-structs2__chordconstraint__t_1ad7aff35b8bdf890d5d15328993a4aeb5>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`originalParentFramePointer<doxid-structs2__chordconstraint__t_1ab74818c79e8c1344159eec9708ebb8bc>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`childBodyPointer<doxid-structs2__chordconstraint__t_1a6be7fe8f18b9ab873ea009fd2c271fb4>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`childFramePointer<doxid-structs2__chordconstraint__t_1aef57e8698b8324ca70e92d371082a4b6>`
	:ref:`s2_joint_t<doxid-structs2__joint__t>`* parentJointPointer
	:ref:`s2_joint_t<doxid-structs2__joint__t>`* childJointPointer
	:ref:`s2_chordconstraint_t<doxid-structs2__chordconstraint__t>`* next

.. _details-doxid-structs2__chordconstraint__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

chordConstraintNode properties

If chord loops are detected, a virtual massless body is inserted and a chordConstraintNode data structure is created for each chord loop

Fields
------

.. _doxid-structs2__chordconstraint__t_1a50f1311ba5bdbde0695c319aff0b4827:
.. _cid-s2_chordconstraint_t::parentbody:
.. ref-code-block:: cpp
	:class: title-code-block

	char* parentBody

name of parentBody (a virtual body)

.. _doxid-structs2__chordconstraint__t_1a39a722bd220f71f755ffc90a4bdc0197:
.. _cid-s2_chordconstraint_t::originalparentbody:
.. ref-code-block:: cpp
	:class: title-code-block

	char* originalParentBody

name of original bodyFrom

.. _doxid-structs2__chordconstraint__t_1a4fb2adc72939678645e0b70894e2d6e7:
.. _cid-s2_chordconstraint_t::parentframe:
.. ref-code-block:: cpp
	:class: title-code-block

	char* parentFrame

name of frameFrom

.. _doxid-structs2__chordconstraint__t_1ab2c8a12ab7b4d43b2edb92166f1c2d18:
.. _cid-s2_chordconstraint_t::originalparentframe:
.. ref-code-block:: cpp
	:class: title-code-block

	char* originalParentFrame

name of original frameFrom

.. _doxid-structs2__chordconstraint__t_1aac5c1ce73fd2c65ca8c8c7eca9544b87:
.. _cid-s2_chordconstraint_t::childbody:
.. ref-code-block:: cpp
	:class: title-code-block

	char* childBody

name of child body

.. _doxid-structs2__chordconstraint__t_1a91134170da1b8270623f4b93a9f4b966:
.. _cid-s2_chordconstraint_t::childframe:
.. ref-code-block:: cpp
	:class: title-code-block

	char* childFrame

name of child frame

.. _doxid-structs2__chordconstraint__t_1aa0ba0c1463541b0cab68df0843b7ca05:
.. _cid-s2_chordconstraint_t::parentbodypointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* parentBodyPointer

pointer to virtual body

.. _doxid-structs2__chordconstraint__t_1a12db7f3bdf9cb97b71604baa38d11992:
.. _cid-s2_chordconstraint_t::originalparentbodypointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* originalParentBodyPointer

pointer to original bodyFrom (on the nonvirtual body)

.. _doxid-structs2__chordconstraint__t_1ad7aff35b8bdf890d5d15328993a4aeb5:
.. _cid-s2_chordconstraint_t::parentframepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* parentFramePointer

pointer to parentFrame (on the virtual body)

.. _doxid-structs2__chordconstraint__t_1ab74818c79e8c1344159eec9708ebb8bc:
.. _cid-s2_chordconstraint_t::originalparentframepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* originalParentFramePointer

pointer to original frameFrom (on the nonvirtual body)

.. _doxid-structs2__chordconstraint__t_1a6be7fe8f18b9ab873ea009fd2c271fb4:
.. _cid-s2_chordconstraint_t::childbodypointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* childBodyPointer

pointer to body data structure

.. _doxid-structs2__chordconstraint__t_1aef57e8698b8324ca70e92d371082a4b6:
.. _cid-s2_chordconstraint_t::childframepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* childFramePointer

pointer to frame data structure

