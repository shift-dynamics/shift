.. index:: pair: struct; s2_pointcloud_t
.. _doxid-structs2__pointcloud__t:
.. _cid-s2_pointcloud_t:

struct s2_pointcloud_t
======================

.. toctree::
	:hidden:



Overview
~~~~~~~~



.. _doxid-structs2__pointcloud__t_1ac9a43fcb74410c759f55526f06b18000:
.. _cid-s2_pointcloud_t::vertices:
.. _doxid-structs2__pointcloud__t_1a850f29bb40756ff8e41d8b468c1215c7:
.. _cid-s2_pointcloud_t::verticesf:
.. _doxid-structs2__pointcloud__t_1ac1f6d661d4d68b2a927b4f1e61ce11e4:
.. _cid-s2_pointcloud_t::colors:
.. _doxid-structs2__pointcloud__t_1a1d255b2411ec7c7f7322b7980fe2f886:
.. _cid-s2_pointcloud_t::vertexcount:
.. _doxid-structs2__pointcloud__t_1a3c49274d098880eab942a7e5e4dcc2ab:
.. _cid-s2_pointcloud_t::frame:
.. _doxid-structs2__pointcloud__t_1ac9016c90801ee1cf28c817a83b965e22:
.. _cid-s2_pointcloud_t::transform:
.. _doxid-structs2__pointcloud__t_1a0d290ae6a4f9ffa09d6852aff5134df2:
.. _cid-s2_pointcloud_t::name:
.. _doxid-structs2__pointcloud__t_1aad34f363467d054d8617243234f20711:
.. _cid-s2_pointcloud_t::next:
.. _doxid-structs2__pointcloud__t_1ab48042bcfdc47c47ac1c05c227cb9545:
.. _cid-s2_pointcloud_t::isvisible:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	double* vertices
	float* verticesf
	float* colors
	size_t vertexCount
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`definition<doxid-structs2__pointcloud__t_1ac9adc5cb1bea8c0d19d62eba0cc71f03>`
	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* :ref:`assembly<doxid-structs2__pointcloud__t_1a27cacc202da285d7de8c398b5521e023>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`body<doxid-structs2__pointcloud__t_1a7e4c50fadc4b3862d1a05a1d8a419dc6>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* frame
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* transform
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name
	:ref:`s2_pointcloud_t<doxid-structs2__pointcloud__t>`* next
	bool isVisible

.. _details-doxid-structs2__pointcloud__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. _doxid-structs2__pointcloud__t_1ac9adc5cb1bea8c0d19d62eba0cc71f03:
.. _cid-s2_pointcloud_t::definition:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* definition

Path to files that define body definitions, i.e. ply files

.. _doxid-structs2__pointcloud__t_1a27cacc202da285d7de8c398b5521e023:
.. _cid-s2_pointcloud_t::assembly:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly

Pointer to assembly that pointcloud is a member of.

.. _doxid-structs2__pointcloud__t_1a7e4c50fadc4b3862d1a05a1d8a419dc6:
.. _cid-s2_pointcloud_t::body:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* body

pointer to body that point cloud belongs to

