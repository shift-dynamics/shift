.. index:: pair: enum; s2_interpolation_enum_t
.. _doxid-context_8h_1aecaa7b02bc380403679f716f60ae49c5:
.. _cid-s2_interpolation_enum_t:

enum s2_interpolation_enum_t
============================



Overview
~~~~~~~~



.. ref-code-block:: cpp
	:class: overview-code-block

	// enum values

	:ref:`BILINEAR<doxid-context_8h_1aecaa7b02bc380403679f716f60ae49c5a712792aa7e7c16d741c63da90e3c94dc>` 
	:ref:`BICUBIC<doxid-context_8h_1aecaa7b02bc380403679f716f60ae49c5a8310e3ff71c506fdaaf6b5e4fcb8eb85>` 
	:ref:`CATMULROM<doxid-context_8h_1aecaa7b02bc380403679f716f60ae49c5a2d2851e8a1ddd7b5301ccf21c166a7ef>` 

.. _details-doxid-context_8h_1aecaa7b02bc380403679f716f60ae49c5:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Enum Values
-----------

.. _doxid-context_8h_1aecaa7b02bc380403679f716f60ae49c5a712792aa7e7c16d741c63da90e3c94dc:
.. _cid-s2_interpolation_enum_t::bilinear:
.. ref-code-block:: cpp
	:class: title-code-block

	BILINEAR

bilinear interpolation is the simplest technique for 2D interpolation. The resulting surface is C0 continuous (position continuity is maintained between surface patches, but not tangency and curvature). The surface intersects the control points.

.. _doxid-context_8h_1aecaa7b02bc380403679f716f60ae49c5a8310e3ff71c506fdaaf6b5e4fcb8eb85:
.. _cid-s2_interpolation_enum_t::bicubic:
.. ref-code-block:: cpp
	:class: title-code-block

	BICUBIC

bicubic interpolation uses knot points as control points, surface passes near control point, but may not intersect control point, surface is C0, C1 and C2 continuous (position, tangency and curvature continuity are maintained between surface patches)

.. _doxid-context_8h_1aecaa7b02bc380403679f716f60ae49c5a2d2851e8a1ddd7b5301ccf21c166a7ef:
.. _cid-s2_interpolation_enum_t::catmulrom:
.. ref-code-block:: cpp
	:class: title-code-block

	CATMULROM

catmul-rom interpolation, surface passes through control points, but is only C0 and C1 continuous (position and tangency continuity are maintained between surface patches, but not curvature)

