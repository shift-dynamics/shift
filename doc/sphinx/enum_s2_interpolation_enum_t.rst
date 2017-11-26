.. index:: pair: enum; s2_interpolation_enum_t
.. _doxid-terrain_8h_1aecaa7b02bc380403679f716f60ae49c5:
.. _cid-s2_interpolation_enum_t:

enum s2_interpolation_enum_t
============================



Overview
~~~~~~~~



.. ref-code-block:: cpp
	:class: overview-code-block

	// enum values

	:ref:`S2_INTERPOLATION_BILINEAR<doxid-terrain_8h_1aecaa7b02bc380403679f716f60ae49c5a0c8c5a62cebb9b2b25c6c60f9cd50c0e>` 
	:ref:`S2_INTERPOLATION_BICUBIC<doxid-terrain_8h_1aecaa7b02bc380403679f716f60ae49c5aa24d13d31df6123c61d4e3dd4e3c9f9b>` 
	:ref:`S2_INTERPOLATION_CATMULROM<doxid-terrain_8h_1aecaa7b02bc380403679f716f60ae49c5a025673effb849cf1e718e39c8c8ef70a>` 

.. _details-doxid-terrain_8h_1aecaa7b02bc380403679f716f60ae49c5:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Enum Values
-----------

.. _doxid-terrain_8h_1aecaa7b02bc380403679f716f60ae49c5a0c8c5a62cebb9b2b25c6c60f9cd50c0e:
.. _cid-s2_interpolation_enum_t::s2_interpolation_bilinear:
.. ref-code-block:: cpp
	:class: title-code-block

	S2_INTERPOLATION_BILINEAR

bilinear interpolation is the simplest technique for 2D interpolation. The resulting surface is C0 continuous (position continuity is maintained between surface patches, but not tangency and curvature). The surface intersects the control points.

.. _doxid-terrain_8h_1aecaa7b02bc380403679f716f60ae49c5aa24d13d31df6123c61d4e3dd4e3c9f9b:
.. _cid-s2_interpolation_enum_t::s2_interpolation_bicubic:
.. ref-code-block:: cpp
	:class: title-code-block

	S2_INTERPOLATION_BICUBIC

bicubic interpolation uses knot points as control points, surface passes near control point, but may not intersect control point, surface is C0, C1 and C2 continuous (position, tangency and curvature continuity are maintained between surface patches)

.. _doxid-terrain_8h_1aecaa7b02bc380403679f716f60ae49c5a025673effb849cf1e718e39c8c8ef70a:
.. _cid-s2_interpolation_enum_t::s2_interpolation_catmulrom:
.. ref-code-block:: cpp
	:class: title-code-block

	S2_INTERPOLATION_CATMULROM

catmul-rom interpolation, surface passes through control points, but is only C0 and C1 continuous (position and tangency continuity are maintained between surface patches, but not curvature)

