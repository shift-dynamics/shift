.. index:: pair: struct; s2_kdnode_t
.. _doxid-structs2__kdnode__t:
.. _cid-s2_kdnode_t:

struct s2_kdnode_t
==================

.. toctree::
	:hidden:





.. _doxid-structs2__kdnode__t_1a690cac720030712e2e65fd9c8ec4c63a:
.. _cid-s2_kdnode_t::splitvalue:
.. _doxid-structs2__kdnode__t_1aca8dfc236071e1c1ac46ac40f4cdf012:
.. _cid-s2_kdnode_t::splitaxis:
.. _doxid-structs2__kdnode__t_1ab23c436d16215df5d5171bd561280814:
.. _cid-s2_kdnode_t::children:
.. _doxid-structs2__kdnode__t_1acc4183dc840f21ab4373e54435ccce77:
.. _cid-s2_kdnode_t::vertexcount:
.. _doxid-structs2__kdnode__t_1a909a914ef60aab2cfa8a65af1734bea7:
.. _cid-s2_kdnode_t::facecount:
.. _doxid-structs2__kdnode__t_1a3004bf765a8a3fc2ed1bbb4ccd2176b6:
.. _cid-s2_kdnode_t::vertexlocations:
.. _doxid-structs2__kdnode__t_1ac48ca190a8e7e048a24c42d59d0f55c1:
.. _cid-s2_kdnode_t::facelocations:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	double splitValue
	int8_t splitAxis
	uint32_t children [2]
	uint32_t vertexCount
	uint32_t faceCount
	int* vertexLocations
	int* faceLocations

