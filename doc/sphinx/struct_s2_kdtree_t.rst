.. index:: pair: struct; s2_kdtree_t
.. _doxid-structs2__kdtree__t:
.. _cid-s2_kdtree_t:

struct s2_kdtree_t
==================

.. toctree::
	:hidden:



Overview
~~~~~~~~



.. _doxid-structs2__kdtree__t_1a38bd40b2fb82c09c8abdc67d37d2a318:
.. _cid-s2_kdtree_t::precision:
.. _doxid-structs2__kdtree__t_1a515ee06706c42a6138ea6d26e1562533:
.. _cid-s2_kdtree_t::vertexcount:
.. _doxid-structs2__kdtree__t_1a20851f93e8e4b6a89357dffd09b37693:
.. _cid-s2_kdtree_t::facecount:
.. _doxid-structs2__kdtree__t_1aef7e63b06516f646f3ac72964986451d:
.. _cid-s2_kdtree_t::nodecount:
.. _doxid-structs2__kdtree__t_1a4367ba1d474337dc3a6387b37b6dfac1:
.. _cid-s2_kdtree_t::kdnodes:
.. _doxid-structs2__kdtree__t_1a8483992e0e51a88206b50b37a803b983:
.. _cid-s2_kdtree_t::vertexlocations:
.. _doxid-structs2__kdtree__t_1ac893f97aa39e703f5b0a048ef0cfa964:
.. _cid-s2_kdtree_t::facelocations:
.. _doxid-structs2__kdtree__t_1abec185ded7d5b2f634e0cb2cf695bf4e:
.. _cid-s2_kdtree_t::visited:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	s2_kdtree_enum_t precision
	int vertexCount
	int faceCount
	int nodeCount
	:ref:`s2_kdnode_t<doxid-structs2__kdnode__t>`* kdnodes
	double* :ref:`vertices<doxid-structs2__kdtree__t_1ad785bf97aa025bff0d435b602aef8d0f>`
	float* :ref:`verticesf<doxid-structs2__kdtree__t_1ab5cecbd73e5aaab064ea93c9cde22115>`
	int* :ref:`faces<doxid-structs2__kdtree__t_1acd0d8965e93d65819966040698aac054>`
	int* vertexLocations
	int* faceLocations
	bool* visited

.. _details-doxid-structs2__kdtree__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. _doxid-structs2__kdtree__t_1ad785bf97aa025bff0d435b602aef8d0f:
.. _cid-s2_kdtree_t::vertices:
.. ref-code-block:: cpp
	:class: title-code-block

	double* vertices

pointer to double precision vertices

.. _doxid-structs2__kdtree__t_1ab5cecbd73e5aaab064ea93c9cde22115:
.. _cid-s2_kdtree_t::verticesf:
.. ref-code-block:: cpp
	:class: title-code-block

	float* verticesf

pointer to single precision vertices

.. _doxid-structs2__kdtree__t_1acd0d8965e93d65819966040698aac054:
.. _cid-s2_kdtree_t::faces:
.. ref-code-block:: cpp
	:class: title-code-block

	int* faces

pointer to faces

