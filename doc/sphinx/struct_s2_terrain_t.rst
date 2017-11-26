.. index:: pair: struct; s2_terrain_t
.. _doxid-structs2__terrain__t:
.. _cid-s2_terrain_t:

struct s2_terrain_t
===================

.. toctree::
	:hidden:



Overview
~~~~~~~~



.. _doxid-structs2__terrain__t_1ac6e8aebd0013238394114b6aa3f3e8d7:
.. _cid-s2_terrain_t::kd:
.. _doxid-structs2__terrain__t_1aa2cf6522630bc779401fc4a6f6b0dc87:
.. _cid-s2_terrain_t::kaoverkd:
.. _doxid-structs2__terrain__t_1aaf7222afb98831328ce1949e7a957172:
.. _cid-s2_terrain_t::ns:
.. _doxid-structs2__terrain__t_1abe9ff0971426f9a9a464f65db02ae0ea:
.. _cid-s2_terrain_t::name:
.. _doxid-structs2__terrain__t_1a1cf26e6928912eb2798605559939bfee:
.. _cid-s2_terrain_t::parentbody:
.. _doxid-structs2__terrain__t_1a75cb6f6c445bd9cd52a441acf1883a44:
.. _cid-s2_terrain_t::parentbodypointer:
.. _doxid-structs2__terrain__t_1a7370d5dc6b7c3d560e974742feda757f:
.. _cid-s2_terrain_t::parentframe:
.. _doxid-structs2__terrain__t_1a4acc03bc2fb9799c7bea63d01dc3bdfe:
.. _cid-s2_terrain_t::parentframepointer:
.. _doxid-structs2__terrain__t_1a4e5c69fd0c29d0fb1a157eae746abc61:
.. _cid-s2_terrain_t::isvisible:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	s2_interpolation_enum_t :ref:`interpolationType<doxid-structs2__terrain__t_1a8cdf5fc8ed7942ed82e317f2e0dd11f3>`
	s2_coordinate_convention_enum_t :ref:`coordinateConvention<doxid-structs2__terrain__t_1ad081f7a16f003474c1cdffddc13280db>`
	size_t :ref:`columns<doxid-structs2__terrain__t_1a433a84660b0637a4ac3f445a7caac45a>`
	size_t :ref:`rows<doxid-structs2__terrain__t_1ab405b1169207f88f99219aa94b3d036f>`
	double :ref:`xOffset<doxid-structs2__terrain__t_1a0e5a85f98583edc35d56170ac9eb47a7>`
	double :ref:`yOffset<doxid-structs2__terrain__t_1a2d56e0a120eb5fe989d6dd108cfc1fee>`
	double :ref:`xScale<doxid-structs2__terrain__t_1ae0a8aa9c4da433266c73f2ecb9924e3c>`
	double :ref:`yScale<doxid-structs2__terrain__t_1a45cbdbb912c69929f2891484027d35d1>`
	double :ref:`zMin<doxid-structs2__terrain__t_1af5e73b87bc7ad0e3960147b15a3c3f27>`
	double :ref:`zMax<doxid-structs2__terrain__t_1a4ac7534095ebf990c660e18409aca724>`
	double* :ref:`controlPoints<doxid-structs2__terrain__t_1a494897deb461de5cdb77fbbd3ffad828>`
	unsigned char* :ref:`texture<doxid-structs2__terrain__t_1a9f4730f871f3e19e29d29d1627d9dca2>`
	size_t :ref:`textureRows<doxid-structs2__terrain__t_1a9c9f35bd6163cd828c6bff6114752771>`
	size_t :ref:`textureColumns<doxid-structs2__terrain__t_1a5743f0e00818d7df91b1760a0ab46589>`
	char* :ref:`propertyMap<doxid-structs2__terrain__t_1a3bf762fe213135125e3b790da1dc0554>`
	size_t :ref:`propertyMapRows<doxid-structs2__terrain__t_1a6ce5c1c0620466c77e8432b31ca23fa1>`
	size_t :ref:`propertyMapColumns<doxid-structs2__terrain__t_1a27a942a9782f9e71dddf3edd25d6b19e>`
	GLfloat Kd [4]
	GLfloat :ref:`KsOverKd<doxid-structs2__terrain__t_1a831478628ee316e0518c5605909c90c3>`
	GLfloat KaOverKd
	GLint Ns
	double (* :ref:`getZ<doxid-structs2__terrain__t_1ab010f735a1a20e3f8d0cbcf2d6a3f5fc>` )(struct s2_terrain_t *terrain, double x, double y, s2_vector_t *normal)
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* parentBody
	:ref:`s2_body_t<doxid-structs2__body__t>`* parentBodyPointer
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* parentFrame
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* parentFramePointer
	bool isVisible

.. _details-doxid-structs2__terrain__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. _doxid-structs2__terrain__t_1a8cdf5fc8ed7942ed82e317f2e0dd11f3:
.. _cid-s2_terrain_t::interpolationtype:
.. ref-code-block:: cpp
	:class: title-code-block

	s2_interpolation_enum_t interpolationType

technique used for 2D interpolation

.. _doxid-structs2__terrain__t_1ad081f7a16f003474c1cdffddc13280db:
.. _cid-s2_terrain_t::coordinateconvention:
.. ref-code-block:: cpp
	:class: title-code-block

	s2_coordinate_convention_enum_t coordinateConvention

ISO or SAE coordinate system

.. _doxid-structs2__terrain__t_1a433a84660b0637a4ac3f445a7caac45a:
.. _cid-s2_terrain_t::columns:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t columns

number of columns of elevation map

.. _doxid-structs2__terrain__t_1ab405b1169207f88f99219aa94b3d036f:
.. _cid-s2_terrain_t::rows:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t rows

number of rows in elevation map

.. _doxid-structs2__terrain__t_1a0e5a85f98583edc35d56170ac9eb47a7:
.. _cid-s2_terrain_t::xoffset:
.. ref-code-block:: cpp
	:class: title-code-block

	double xOffset

x offset of origin of elevation map

.. _doxid-structs2__terrain__t_1a2d56e0a120eb5fe989d6dd108cfc1fee:
.. _cid-s2_terrain_t::yoffset:
.. ref-code-block:: cpp
	:class: title-code-block

	double yOffset

y offset of origin of elevation map

.. _doxid-structs2__terrain__t_1ae0a8aa9c4da433266c73f2ecb9924e3c:
.. _cid-s2_terrain_t::xscale:
.. ref-code-block:: cpp
	:class: title-code-block

	double xScale

uniform scale factor for x dimension

.. _doxid-structs2__terrain__t_1a45cbdbb912c69929f2891484027d35d1:
.. _cid-s2_terrain_t::yscale:
.. ref-code-block:: cpp
	:class: title-code-block

	double yScale

uniform scale factor for y dimension

.. _doxid-structs2__terrain__t_1af5e73b87bc7ad0e3960147b15a3c3f27:
.. _cid-s2_terrain_t::zmin:
.. ref-code-block:: cpp
	:class: title-code-block

	double zMin

minimum z height in surface

.. _doxid-structs2__terrain__t_1a4ac7534095ebf990c660e18409aca724:
.. _cid-s2_terrain_t::zmax:
.. ref-code-block:: cpp
	:class: title-code-block

	double zMax

maximum z height in surface

.. _doxid-structs2__terrain__t_1a494897deb461de5cdb77fbbd3ffad828:
.. _cid-s2_terrain_t::controlpoints:
.. ref-code-block:: cpp
	:class: title-code-block

	double* controlPoints

control points loaded from image

.. _doxid-structs2__terrain__t_1a9f4730f871f3e19e29d29d1627d9dca2:
.. _cid-s2_terrain_t::texture:
.. ref-code-block:: cpp
	:class: title-code-block

	unsigned char* texture

rgba texture

.. _doxid-structs2__terrain__t_1a9c9f35bd6163cd828c6bff6114752771:
.. _cid-s2_terrain_t::texturerows:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t textureRows

number of rows in texture

.. _doxid-structs2__terrain__t_1a5743f0e00818d7df91b1760a0ab46589:
.. _cid-s2_terrain_t::texturecolumns:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t textureColumns

number of columns in texture

.. _doxid-structs2__terrain__t_1a3bf762fe213135125e3b790da1dc0554:
.. _cid-s2_terrain_t::propertymap:
.. ref-code-block:: cpp
	:class: title-code-block

	char* propertyMap

rgba texture

.. _doxid-structs2__terrain__t_1a6ce5c1c0620466c77e8432b31ca23fa1:
.. _cid-s2_terrain_t::propertymaprows:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t propertyMapRows

number of rows in texture

.. _doxid-structs2__terrain__t_1a27a942a9782f9e71dddf3edd25d6b19e:
.. _cid-s2_terrain_t::propertymapcolumns:
.. ref-code-block:: cpp
	:class: title-code-block

	size_t propertyMapColumns

number of columns in texture

.. _doxid-structs2__terrain__t_1a831478628ee316e0518c5605909c90c3:
.. _cid-s2_terrain_t::ksoverkd:
.. ref-code-block:: cpp
	:class: title-code-block

	GLfloat KsOverKd

diffuse color value

.. _doxid-structs2__terrain__t_1ab010f735a1a20e3f8d0cbcf2d6a3f5fc:
.. _cid-s2_terrain_t::getz:
.. ref-code-block:: cpp
	:class: title-code-block

	double (* getZ )(struct s2_terrain_t *terrain, double x, double y, s2_vector_t *normal)

function used to obtain elevation at specified x, y location. This function pointer is set based on type of interpolation specified when terrain is created

