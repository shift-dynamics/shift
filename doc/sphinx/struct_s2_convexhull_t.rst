.. index:: pair: struct; s2_convexhull_t
.. _doxid-structs2__convexhull__t:
.. _cid-s2_convexhull_t:

struct s2_convexhull_t
======================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <convexhull.h>


Overview
~~~~~~~~

convex hull data structure contains vertices and faces in contiguous blocks of memory :ref:`More...<details-doxid-structs2__convexhull__t>`

.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	GLfloat* :ref:`vertices<doxid-structs2__convexhull__t_1a1805a5c2051c53a90e3a85c39cf0b7e3>`
	int :ref:`vertexCount<doxid-structs2__convexhull__t_1ac728c0433323e09f279a1b06f4896b6a>`
	GLuint* :ref:`faces<doxid-structs2__convexhull__t_1a08f7d0228adc3c990b5f450793c55786>`
	int :ref:`faceCount<doxid-structs2__convexhull__t_1a1462be0cd52fd64c5885bc5dcc8b9a27>`
	bool :ref:`drawEnabled<doxid-structs2__convexhull__t_1a9c46d3fcdc0e0a872c18ff564f342720>`

.. _details-doxid-structs2__convexhull__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

convex hull data structure contains vertices and faces in contiguous blocks of memory

Fields
------

.. _doxid-structs2__convexhull__t_1a1805a5c2051c53a90e3a85c39cf0b7e3:
.. _cid-s2_convexhull_t::vertices:
.. ref-code-block:: cpp
	:class: title-code-block

	GLfloat* vertices

vertices in contiguous memory

.. _doxid-structs2__convexhull__t_1ac728c0433323e09f279a1b06f4896b6a:
.. _cid-s2_convexhull_t::vertexcount:
.. ref-code-block:: cpp
	:class: title-code-block

	int vertexCount

number of vertices

.. _doxid-structs2__convexhull__t_1a08f7d0228adc3c990b5f450793c55786:
.. _cid-s2_convexhull_t::faces:
.. ref-code-block:: cpp
	:class: title-code-block

	GLuint* faces

faces in contiguous memory

.. _doxid-structs2__convexhull__t_1a1462be0cd52fd64c5885bc5dcc8b9a27:
.. _cid-s2_convexhull_t::facecount:
.. ref-code-block:: cpp
	:class: title-code-block

	int faceCount

number of faces

.. _doxid-structs2__convexhull__t_1a9c46d3fcdc0e0a872c18ff564f342720:
.. _cid-s2_convexhull_t::drawenabled:
.. ref-code-block:: cpp
	:class: title-code-block

	bool drawEnabled

indicates whether the convexhull object should be drawin in gui

