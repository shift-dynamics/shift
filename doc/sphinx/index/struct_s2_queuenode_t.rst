.. index:: pair: struct; s2_queuenode_t
.. _doxid-structs2__queuenode__t:
.. _cid-s2_queuenode_t:

struct s2_queuenode_t
=====================

.. toctree::
	:hidden:





.. _doxid-structs2__queuenode__t_1a43f626eea0a60d49fb4e55d122416eed:
.. _cid-s2_queuenode_t::type:
.. _doxid-structs2__queuenode__t_1afb9bd5504aeadaaf2cf3ba309f805274:
.. _cid-s2_queuenode_t::integer:
.. _doxid-structs2__queuenode__t_1acc988db58c41c671984a6bea41655eae:
.. _cid-s2_queuenode_t::body:
.. _doxid-structs2__queuenode__t_1ad42eddd854774da15ea1fd8a9518f269:
.. _cid-s2_queuenode_t::joint:
.. _doxid-structs2__queuenode__t_1a0d1ef6cde2c0724a2b322be3363f6720:
.. _cid-s2_queuenode_t::frame:
.. _doxid-structs2__queuenode__t_1aae97820f549f686d7d172996224d0aeb:
.. _cid-s2_queuenode_t::stack:
.. _doxid-structs2__queuenode__t_1a14aace5f0c835ede75ad745178b1a658:
.. _cid-s2_queuenode_t::queue:
.. _doxid-structs2__queuenode__t_1a6fac8de2cbacc8c21613ce225eed644e:
.. _cid-s2_queuenode_t::value:
.. _doxid-structs2__queuenode__t_1ab044504032056d9a2eacaf16cc4ae251:
.. _cid-s2_queuenode_t::next:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	s2_queue_enum_t type
	int* integer
	:ref:`s2_body_t<doxid-structs2__body__t>`* body
	:ref:`s2_joint_t<doxid-structs2__joint__t>`* joint
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* frame
	:ref:`s2_stack_t<doxid-structs2__stack__t>`* stack
	:ref:`s2_queue_t<doxid-structs2__queue__t>`* queue
	union s2_queuenode_t::@0 value
	struct :ref:`s2_queuenode_t<doxid-structs2__queuenode__t>`* next

