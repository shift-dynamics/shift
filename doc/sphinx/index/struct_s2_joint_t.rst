.. index:: pair: struct; s2_joint_t
.. _doxid-structs2__joint__t:
.. _cid-s2_joint_t:

struct s2_joint_t
=================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <context.h>


Overview
~~~~~~~~

properties related to Joints :ref:`More...<details-doxid-structs2__joint__t>`

.. _doxid-structs2__joint__t_1aa41f193c891919aa2626a2fc5bbec3e8:
.. _cid-s2_joint_t::velocityrelative:
.. _doxid-structs2__joint__t_1a27b298a2072139e868ea9f38e3c52dfd:
.. _cid-s2_joint_t::accelerationquadraticrelative:
.. _doxid-structs2__joint__t_1a82ff8def8fb427a3c15a2e8b3f559d48:
.. _cid-s2_joint_t::h:
.. _doxid-structs2__joint__t_1ab4dc89876b6523b60a05ac350357fe36:
.. _cid-s2_joint_t::mobilitynumber:
.. _doxid-structs2__joint__t_1a0a1d4f083419da8b700c24b30d94d5ef:
.. _cid-s2_joint_t::dampingcoefficientcount:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`name<doxid-structs2__joint__t_1a7f1cb8247ad92b703ca1de45929fa313>`
	int :ref:`dof<doxid-structs2__joint__t_1a35eaf902a5b27bd4e7c7eaea7b3bf66d>`
	int :ref:`index<doxid-structs2__joint__t_1af30cd8e0c8b1d3ad57eaf5161df5efb0>`
	int :ref:`qIndex<doxid-structs2__joint__t_1a6f40185f9d8f6f022557008ae83b0397>`
	int :ref:`dqIndex<doxid-structs2__joint__t_1accba60b96bf06696da236dc225afbf86>`
	int :ref:`dcIndex<doxid-structs2__joint__t_1a4a22a7dc328596b69834a948b655ebf4>`
	int :ref:`indexSubstructure<doxid-structs2__joint__t_1a0de6193d9b1912632a0644550e7b41f3>`
	int :ref:`sense<doxid-structs2__joint__t_1a576cdf86bb737b43085b4cb466f37f07>`
	int :ref:`level<doxid-structs2__joint__t_1a7dbd26edc9168c7c7540df802ee7f546>`
	:ref:`s2_joint_enum_t<doxid-context_8h_1a7a43c175bb516fb74df6eb8503a0475c>` :ref:`jointType<doxid-structs2__joint__t_1ac3226af45ae3aa539c1a37864a231cd4>`
	char* :ref:`jointTypeStr<doxid-structs2__joint__t_1a4f3dd16b2eaeb9d586e79002483dfb55>`
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`parentBody<doxid-structs2__joint__t_1ac07e824ed6cde2043cd58ed6e9ecb224>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`parentBodyPointer<doxid-structs2__joint__t_1a7f50f5a5c3d8cf4d2b4bb3a417b71112>`
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`parentFrame<doxid-structs2__joint__t_1a79b1afc0c4fee73785faad0b6e6b11a9>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`parentFramePointer<doxid-structs2__joint__t_1ac4779cc5dc06f6ae10d2989d0b5172e1>`
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`childBody<doxid-structs2__joint__t_1a13cd90d226c5dfc594e2dc10eecdd297>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`childBodyPointer<doxid-structs2__joint__t_1a8afb6929b36ed52a19f574841e01d881>`
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`childFrame<doxid-structs2__joint__t_1a499952b1499f6fb29308e634ddcb86c9>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`childFramePointer<doxid-structs2__joint__t_1abe8484b3adfb154ebb735b360d0fbb70>`
	double* :ref:`initialPrimitiveDisplacement<doxid-structs2__joint__t_1af5ca6d96cef2ac2cdd87143f1ea72067>`
	double* :ref:`initialPrimitiveVelocity<doxid-structs2__joint__t_1a4af7cd6a0d28140d061477e12cc235b8>`
	double* :ref:`primitiveDisplacement<doxid-structs2__joint__t_1a34e677e721196bde245cbe718189c710>`
	double* :ref:`primitiveVelocity<doxid-structs2__joint__t_1a2afd39da7ff3cdeaeafdc2fea3bfa0cf>`
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* :ref:`generalDisplacement<doxid-structs2__joint__t_1af5507dc5bbdc91b78b2fb14c4a390816>`
	double* :ref:`directionCosine<doxid-structs2__joint__t_1a95078344238755b897f0532644923fdb>`
	:ref:`s2_screw_t<doxid-structs2__screw__t>`* velocityRelative
	:ref:`s2_screw_t<doxid-structs2__screw__t>`* accelerationQuadraticRelative
	:ref:`s2_screw_t<doxid-structs2__screw__t>`* h
	double* mobilityNumber
	bool :ref:`independent<doxid-structs2__joint__t_1af7c713b0f80199e22c5af987a1bcdc6e>`
	:ref:`s2_joint_t<doxid-structs2__joint__t>`* :ref:`parentJoint<doxid-structs2__joint__t_1a174cd3b7e12c17ad5bbfc9a3990079b5>`
	:ref:`s2_joint_t<doxid-structs2__joint__t>`* :ref:`next<doxid-structs2__joint__t_1ac69cc6034e1b9b29a65f8196a1fe6e52>`
	:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* :ref:`substructurePointer<doxid-structs2__joint__t_1a34154035aaa835d6890780abdf77b47d>`
	double* :ref:`dampingCoefficients<doxid-structs2__joint__t_1a152201a04cab14b75062f645d5325e44>`
	int dampingCoefficientCount
	double* :ref:`stiffnessCoefficients<doxid-structs2__joint__t_1abf5211d78404e284d65ddfa0f111042f>`
	int :ref:`stiffnessCoefficientCount<doxid-structs2__joint__t_1a804efaed65532e34eeaaec28e73fd5ba>`

.. _details-doxid-structs2__joint__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

properties related to Joints

Fields
------

.. _doxid-structs2__joint__t_1a7f1cb8247ad92b703ca1de45929fa313:
.. _cid-s2_joint_t::name:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name

descriptive name

.. _doxid-structs2__joint__t_1a35eaf902a5b27bd4e7c7eaea7b3bf66d:
.. _cid-s2_joint_t::dof:
.. ref-code-block:: cpp
	:class: title-code-block

	int dof

number of degrees of freedom

.. _doxid-structs2__joint__t_1af30cd8e0c8b1d3ad57eaf5161df5efb0:
.. _cid-s2_joint_t::index:
.. ref-code-block:: cpp
	:class: title-code-block

	int index

index in joint array, set by s2_ProcessTopology

.. _doxid-structs2__joint__t_1a6f40185f9d8f6f022557008ae83b0397:
.. _cid-s2_joint_t::qindex:
.. ref-code-block:: cpp
	:class: title-code-block

	int qIndex

index into q array

.. _doxid-structs2__joint__t_1accba60b96bf06696da236dc225afbf86:
.. _cid-s2_joint_t::dqindex:
.. ref-code-block:: cpp
	:class: title-code-block

	int dqIndex

index into dq array

.. _doxid-structs2__joint__t_1a4a22a7dc328596b69834a948b655ebf4:
.. _cid-s2_joint_t::dcindex:
.. ref-code-block:: cpp
	:class: title-code-block

	int dcIndex

index into dc array

.. _doxid-structs2__joint__t_1a0de6193d9b1912632a0644550e7b41f3:
.. _cid-s2_joint_t::indexsubstructure:
.. ref-code-block:: cpp
	:class: title-code-block

	int indexSubstructure

local index into substructure

.. _doxid-structs2__joint__t_1a576cdf86bb737b43085b4cb466f37f07:
.. _cid-s2_joint_t::sense:
.. ref-code-block:: cpp
	:class: title-code-block

	int sense

indicates whether joint is positively aligned with spanning tree (+1), or negatively aligned with spanning tree (-1). Automatically set by s2_ProcessTopology

.. _doxid-structs2__joint__t_1a7dbd26edc9168c7c7540df802ee7f546:
.. _cid-s2_joint_t::level:
.. ref-code-block:: cpp
	:class: title-code-block

	int level

Indicates Joint's level in the spanning tree, automatically set by s2_ProcessTopology

.. _doxid-structs2__joint__t_1ac3226af45ae3aa539c1a37864a231cd4:
.. _cid-s2_joint_t::jointtype:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_joint_enum_t<doxid-context_8h_1a7a43c175bb516fb74df6eb8503a0475c>` jointType

specifies type of joint

.. _doxid-structs2__joint__t_1a4f3dd16b2eaeb9d586e79002483dfb55:
.. _cid-s2_joint_t::jointtypestr:
.. ref-code-block:: cpp
	:class: title-code-block

	char* jointTypeStr

descriptive name of joint type

.. _doxid-structs2__joint__t_1ac07e824ed6cde2043cd58ed6e9ecb224:
.. _cid-s2_joint_t::parentbody:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* parentBody

descriptive name of parent Body

.. _doxid-structs2__joint__t_1a7f50f5a5c3d8cf4d2b4bb3a417b71112:
.. _cid-s2_joint_t::parentbodypointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* parentBodyPointer

pointer to parent Body

.. _doxid-structs2__joint__t_1a79b1afc0c4fee73785faad0b6e6b11a9:
.. _cid-s2_joint_t::parentframe:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* parentFrame

descriptive name of parent Frame

.. _doxid-structs2__joint__t_1ac4779cc5dc06f6ae10d2989d0b5172e1:
.. _cid-s2_joint_t::parentframepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* parentFramePointer

pointer to parent Frame

.. _doxid-structs2__joint__t_1a13cd90d226c5dfc594e2dc10eecdd297:
.. _cid-s2_joint_t::childbody:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* childBody

descriptive name of child Body

.. _doxid-structs2__joint__t_1a8afb6929b36ed52a19f574841e01d881:
.. _cid-s2_joint_t::childbodypointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* childBodyPointer

pointer to child Body

.. _doxid-structs2__joint__t_1a499952b1499f6fb29308e634ddcb86c9:
.. _cid-s2_joint_t::childframe:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* childFrame

descriptive name of child frame

.. _doxid-structs2__joint__t_1abe8484b3adfb154ebb735b360d0fbb70:
.. _cid-s2_joint_t::childframepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* childFramePointer

pointer to child Frame

.. _doxid-structs2__joint__t_1af5ca6d96cef2ac2cdd87143f1ea72067:
.. _cid-s2_joint_t::initialprimitivedisplacement:
.. ref-code-block:: cpp
	:class: title-code-block

	double* initialPrimitiveDisplacement

initial primitive displacement specified by user

.. _doxid-structs2__joint__t_1a4af7cd6a0d28140d061477e12cc235b8:
.. _cid-s2_joint_t::initialprimitivevelocity:
.. ref-code-block:: cpp
	:class: title-code-block

	double* initialPrimitiveVelocity

initial primitive velocity specified by user

.. _doxid-structs2__joint__t_1a34e677e721196bde245cbe718189c710:
.. _cid-s2_joint_t::primitivedisplacement:
.. ref-code-block:: cpp
	:class: title-code-block

	double* primitiveDisplacement

current primitive displacement in model

.. _doxid-structs2__joint__t_1a2afd39da7ff3cdeaeafdc2fea3bfa0cf:
.. _cid-s2_joint_t::primitivevelocity:
.. ref-code-block:: cpp
	:class: title-code-block

	double* primitiveVelocity

current primitive velocity in model

.. _doxid-structs2__joint__t_1af5507dc5bbdc91b78b2fb14c4a390816:
.. _cid-s2_joint_t::generaldisplacement:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* generalDisplacement

used for rigid joints and to set initial conditions for float and spherical joints

.. _doxid-structs2__joint__t_1a95078344238755b897f0532644923fdb:
.. _cid-s2_joint_t::directioncosine:
.. ref-code-block:: cpp
	:class: title-code-block

	double* directionCosine

array of direction cosines

.. _doxid-structs2__joint__t_1af7c713b0f80199e22c5af987a1bcdc6e:
.. _cid-s2_joint_t::independent:
.. ref-code-block:: cpp
	:class: title-code-block

	bool independent

boolean to indicate whether joint is independent (true) or dependent (false) as determined by fullFactor_LU algorithm

.. _doxid-structs2__joint__t_1a174cd3b7e12c17ad5bbfc9a3990079b5:
.. _cid-s2_joint_t::parentjoint:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_joint_t<doxid-structs2__joint__t>`* parentJoint

pointer to unique parentJoint

.. _doxid-structs2__joint__t_1ac69cc6034e1b9b29a65f8196a1fe6e52:
.. _cid-s2_joint_t::next:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_joint_t<doxid-structs2__joint__t>`* next

pointer to next Joint in linked-list data structure

.. _doxid-structs2__joint__t_1a34154035aaa835d6890780abdf77b47d:
.. _cid-s2_joint_t::substructurepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructurePointer

pointer to substructure assignment

.. _doxid-structs2__joint__t_1a152201a04cab14b75062f645d5325e44:
.. _cid-s2_joint_t::dampingcoefficients:
.. ref-code-block:: cpp
	:class: title-code-block

	double* dampingCoefficients

damping coefficients, c0 + c1*x + c2 * x**2

* ... + cn * x**n

.. _doxid-structs2__joint__t_1abf5211d78404e284d65ddfa0f111042f:
.. _cid-s2_joint_t::stiffnesscoefficients:
.. ref-code-block:: cpp
	:class: title-code-block

	double* stiffnessCoefficients

number of damping coefficients spring coefficients, k0 + k1 * x + k2 * x**2 + ... + kn * x**n

.. _doxid-structs2__joint__t_1a804efaed65532e34eeaaec28e73fd5ba:
.. _cid-s2_joint_t::stiffnesscoefficientcount:
.. ref-code-block:: cpp
	:class: title-code-block

	int stiffnessCoefficientCount

number of spring coefficients

