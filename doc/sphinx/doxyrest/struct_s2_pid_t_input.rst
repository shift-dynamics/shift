.. index:: pair: struct; s2_pid_t::input
.. _doxid-structs2__pid__t_1_1input:
.. _cid-s2_pid_t::input-2:

struct s2_pid_t::input
======================

.. toctree::
	:hidden:





.. _doxid-structs2__pid__t_1_1input_1a3718d8b0b3053221719cb50bd1a103d3:
.. _cid-s2_pid_t::input::coordinatesystem:
.. _doxid-structs2__pid__t_1_1input_1afa10c54cb5308ec163dc26b9a6963c6f:
.. _cid-s2_pid_t::input::dof:
.. _doxid-structs2__pid__t_1_1input_1a8a717408152bf633a6baadd34604e5f3:
.. _cid-s2_pid_t::input::framepointer:
.. _doxid-structs2__pid__t_1_1input_1a1c7b2a87d8752be0d3ad63e0cf167e41:
.. _cid-s2_pid_t::input::jointpointer:
.. _doxid-structs2__pid__t_1_1input_1ae3fdf0752412b54abd3181da11625cc4:
.. _cid-s2_pid_t::input::frame:
.. _doxid-structs2__pid__t_1_1input_1a5e0c74d32d6753dd94ca871bacfea051:
.. _cid-s2_pid_t::input::joint:
.. _doxid-structs2__pid__t_1_1input_1a9dbdff78d2a86f56279a4291f32e557c:
.. _cid-s2_pid_t::input::iotype:
.. _doxid-structs2__pid__t_1_1input_1a88d187f8143fc4a701a12c85a3a1e402:
.. _cid-s2_pid_t::input::targettype:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	s2_coordinate_enum_t coordinateSystem
	s2_joint_enum_t dof
	s2_frame_t* framePointer
	s2_joint_t* jointPointer
	s2_stackstr_t* frame
	s2_stackstr_t* joint
	s2_pid_io_enum_t ioType
	s2_pid_target_enum_t targetType

