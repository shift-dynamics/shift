.. index:: pair: struct; s2_camera_t
.. _doxid-structs2__camera__t:
.. _cid-s2_camera_t:

struct s2_camera_t
==================

.. toctree::
	:hidden:





.. _doxid-structs2__camera__t_1afd0477969c0aa1fb9fe8e6435a2905d4:
.. _cid-s2_camera_t::projectiontype:
.. _doxid-structs2__camera__t_1ad85a1d5232cae032787b38e387de8941:
.. _cid-s2_camera_t::fieldofview:
.. _doxid-structs2__camera__t_1abc9409a011eadfd9a12d44345793e63f:
.. _cid-s2_camera_t::aspectratio:
.. _doxid-structs2__camera__t_1a5295c8ec59a8b2fe7048b29b6e3fb1af:
.. _cid-s2_camera_t::nearclippingplane:
.. _doxid-structs2__camera__t_1a28043c3c34c07a4c89bcbbef5cfaee9b:
.. _cid-s2_camera_t::farclippingplane:
.. _doxid-structs2__camera__t_1ac32c52f625f75aa3f9f3f99b9b093843:
.. _cid-s2_camera_t::view:
.. _doxid-structs2__camera__t_1af8a9aadd6e63a91c5bb0b9635ea65f6f:
.. _cid-s2_camera_t::projection:
.. _doxid-structs2__camera__t_1a26d3300f186d9395d0a982ba7609529d:
.. _cid-s2_camera_t::isvisible:
.. _doxid-structs2__camera__t_1a8cbb6a346f94902fb61cf06caddd9afb:
.. _cid-s2_camera_t::next:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	s2_projection_enum_t projectionType
	float fieldOfView
	float aspectRatio
	float nearClippingPlane
	float farClippingPlane
	:ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>` view
	:ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>` projection
	bool isVisible
	:ref:`s2_camera_t<doxid-structs2__camera__t>`* next

