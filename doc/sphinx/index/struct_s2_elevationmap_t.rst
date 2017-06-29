.. index:: pair: struct; s2_elevationmap_t
.. _doxid-structs2__elevationmap__t:
.. _cid-s2_elevationmap_t:

struct s2_elevationmap_t
========================

.. toctree::
	:hidden:



Overview
~~~~~~~~



.. _doxid-structs2__elevationmap__t_1abd73c9256eb478f1dfc039f515f6f377:
.. _cid-s2_elevationmap_t::name:
.. _doxid-structs2__elevationmap__t_1a7428e87942f77ae8c6a126a531b2de7f:
.. _cid-s2_elevationmap_t::parentbody:
.. _doxid-structs2__elevationmap__t_1ab61d41612aae9b8976715ed832241446:
.. _cid-s2_elevationmap_t::parentbodypointer:
.. _doxid-structs2__elevationmap__t_1a6c1a6a0e150dadaf1c3ad2b553551018:
.. _cid-s2_elevationmap_t::parentframe:
.. _doxid-structs2__elevationmap__t_1a58f90a92b58aca8ee81e6c3b90d1c217:
.. _cid-s2_elevationmap_t::parentframepointer:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	:ref:`s2_interpolation_enum_t<doxid-context_8h_1aecaa7b02bc380403679f716f60ae49c5>` :ref:`interpolationType<doxid-structs2__elevationmap__t_1acce8ca8989a23f12ee9d9778f29f9bc4>`
	:ref:`s2_coordinate_convention_enum_t<doxid-context_8h_1a03501e9507988449e75a997381910836>` :ref:`coordinateConvention<doxid-structs2__elevationmap__t_1a7daf6e6686b3d567657bdc2babca9b14>`
	size_t :ref:`columns<doxid-structs2__elevationmap__t_1abe0db3f1ba0dc7cb777bd356ea272769>`
	size_t :ref:`rows<doxid-structs2__elevationmap__t_1a2e6079722809b7decd3da8ed491da284>`
	double :ref:`xOffset<doxid-structs2__elevationmap__t_1a2a5a8bfdf1a8f7c5784802ddddc4eaaa>`
	double :ref:`yOffset<doxid-structs2__elevationmap__t_1a72f4cf71261ebe892f48aaa9583cd2d0>`
	double :ref:`xScale<doxid-structs2__elevationmap__t_1aedf84e7e3a9b244bde22827f2825861c>`
	double :ref:`yScale<doxid-structs2__elevationmap__t_1ae382c2d74dd783c41e3637cc0b802cad>`
	double :ref:`zMin<doxid-structs2__elevationmap__t_1ae6318981c4f5759932c23252af3160e0>`
	double :ref:`zMax<doxid-structs2__elevationmap__t_1ac6cd052bb442b4a32c50d6a0bcf79a11>`
	double* :ref:`controlPoints<doxid-structs2__elevationmap__t_1a3db1176cccfd6bb4b1431bcf4c81c681>`
	unsigned char* :ref:`texture<doxid-structs2__elevationmap__t_1ac42d2ea79be9c818af81fb986aa77a1c>`
	size_t :ref:`textureRows<doxid-structs2__elevationmap__t_1adaa8be3c52ee466f8a655bedcdb75670>`
	size_t :ref:`textureColumns<doxid-structs2__elevationmap__t_1aa176425131bef887bcedd51c4f4a276d>`
	char* :ref:`propertyMap<doxid-structs2__elevationmap__t_1a96c9bd7a81ed2818c9cf8dada22626b6>`
	size_t :ref:`propertyMapRows<doxid-structs2__elevationmap__t_1a289aa8a0db271aa71d7a1d30f0e1b5f1>`
	size_t :ref:`propertyMapColumns<doxid-structs2__elevationmap__t_1a21baef2770f537f023fbfe5ce15d0ee5>`
	double (* :ref:`getZ<doxid-structs2__elevationmap__t_1a9c5dce0862e9646ae87281959fe52791>` )(struct s2_elevationmap_t *elevationMap, double x, double y, s2_vector_t *normal)
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* parentBody
	:ref:`s2_body_t<doxid-structs2__body__t>`* parentBodyPointer
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* parentFrame
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* parentFramePointer

.. _details-doxid-structs2__elevationmap__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. _doxid-structs2__elevationmap__t_1acce8ca8989a23f12ee9d9778f29f9bc4:
.. _cid-s2_elevationmap_t::interpolationtype:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_interpolation_enum_t<doxid-context_8h_1aecaa7b02bc380403679f716f60ae49c5>` interpolationType

technique used for 2D interpolation

.. _doxid-structs2__elevationmap__t_1a7daf6e6686b3d567657bdc2babca9b14:
.. _cid-s2_elevationmap_t::coordinateconvention:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_coordinate_convention_enum_t<doxid-context_8h_1a03501e9507988449e75a997381910836>` coordinateConvention

ISO or SAE coordinate system

.. _doxid-structs2__elevationmap__t_1abe0db3f1ba0dc7cb777bd356ea272769:
.. _cid-s2_elevationmap_t::columns:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t columns

number of columns of elevation map

.. _doxid-structs2__elevationmap__t_1a2e6079722809b7decd3da8ed491da284:
.. _cid-s2_elevationmap_t::rows:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t rows

number of rows in elevation map

.. _doxid-structs2__elevationmap__t_1a2a5a8bfdf1a8f7c5784802ddddc4eaaa:
.. _cid-s2_elevationmap_t::xoffset:
.. ref-code-block:: cpp
	:class: title-code-block

	double xOffset

x offset of origin of elevation map

.. _doxid-structs2__elevationmap__t_1a72f4cf71261ebe892f48aaa9583cd2d0:
.. _cid-s2_elevationmap_t::yoffset:
.. ref-code-block:: cpp
	:class: title-code-block

	double yOffset

y offset of origin of elevation map

.. _doxid-structs2__elevationmap__t_1aedf84e7e3a9b244bde22827f2825861c:
.. _cid-s2_elevationmap_t::xscale:
.. ref-code-block:: cpp
	:class: title-code-block

	double xScale

uniform scale factor for x dimension

.. _doxid-structs2__elevationmap__t_1ae382c2d74dd783c41e3637cc0b802cad:
.. _cid-s2_elevationmap_t::yscale:
.. ref-code-block:: cpp
	:class: title-code-block

	double yScale

uniform scale factor for y dimension

.. _doxid-structs2__elevationmap__t_1ae6318981c4f5759932c23252af3160e0:
.. _cid-s2_elevationmap_t::zmin:
.. ref-code-block:: cpp
	:class: title-code-block

	double zMin

minimum z height in surface

.. _doxid-structs2__elevationmap__t_1ac6cd052bb442b4a32c50d6a0bcf79a11:
.. _cid-s2_elevationmap_t::zmax:
.. ref-code-block:: cpp
	:class: title-code-block

	double zMax

maximum z height in surface

.. _doxid-structs2__elevationmap__t_1a3db1176cccfd6bb4b1431bcf4c81c681:
.. _cid-s2_elevationmap_t::controlpoints:
.. ref-code-block:: cpp
	:class: title-code-block

	double* controlPoints

control points loaded from image

.. _doxid-structs2__elevationmap__t_1ac42d2ea79be9c818af81fb986aa77a1c:
.. _cid-s2_elevationmap_t::texture:
.. ref-code-block:: cpp
	:class: title-code-block

	unsigned char* texture

rgba texture

.. _doxid-structs2__elevationmap__t_1adaa8be3c52ee466f8a655bedcdb75670:
.. _cid-s2_elevationmap_t::texturerows:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t textureRows

number of rows in texture

.. _doxid-structs2__elevationmap__t_1aa176425131bef887bcedd51c4f4a276d:
.. _cid-s2_elevationmap_t::texturecolumns:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t textureColumns

number of columns in texture

.. _doxid-structs2__elevationmap__t_1a96c9bd7a81ed2818c9cf8dada22626b6:
.. _cid-s2_elevationmap_t::propertymap:
.. ref-code-block:: cpp
	:class: title-code-block

	char* propertyMap

rgba texture

.. _doxid-structs2__elevationmap__t_1a289aa8a0db271aa71d7a1d30f0e1b5f1:
.. _cid-s2_elevationmap_t::propertymaprows:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t propertyMapRows

number of rows in texture

.. _doxid-structs2__elevationmap__t_1a21baef2770f537f023fbfe5ce15d0ee5:
.. _cid-s2_elevationmap_t::propertymapcolumns:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t propertyMapColumns

number of columns in texture

.. _doxid-structs2__elevationmap__t_1a9c5dce0862e9646ae87281959fe52791:
.. _cid-s2_elevationmap_t::getz:
.. ref-code-block:: cpp
	:class: title-code-block

	double (* getZ )(struct s2_elevationmap_t *elevationMap, double x, double y, s2_vector_t *normal)

function used to obtain elevation at specified x, y location. This function pointer is set based on type of interpolation specified when elevationmap is created

