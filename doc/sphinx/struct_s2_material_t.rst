.. index:: pair: struct; s2_material_t
.. _doxid-structs2__material__t:
.. _cid-s2_material_t:

struct s2_material_t
====================

.. toctree::
	:hidden:

	struct_s2_material_t_GLuniforms.rst

.. code-block:: cpp
	:class: overview-code-block

	#include <material.h>


Overview
~~~~~~~~

data structure defining a material :ref:`More...<details-doxid-structs2__material__t>`

.. _doxid-structs2__material__t_1a61c2a7eabd55fba6474dcd91e8619521:
.. _cid-s2_material_t::name:
.. _doxid-structs2__material__t_1aa9947184c05d075d6035f86ca61743d5:
.. _cid-s2_material_t::index:
.. _doxid-structs2__material__t_1a0774a5ed731702fc9abf1deb82154070:
.. _cid-s2_material_t::filepath:
.. _doxid-structs2__material__t_1a0d110dc2c378d1a79cd99406dde8db32:
.. _cid-s2_material_t::shadingtype:
.. _doxid-structs2__material__t_1acda99854b3d0e0175be9b51b592f2cc9:
.. _cid-s2_material_t::gluniforms:
.. _doxid-structs2__material__t_1acb27e6f1b5cf5e8fdf3dc261faec69a4:
.. _cid-s2_material_t::vertexcount:
.. _doxid-structs2__material__t_1a4951c6737429e44488fff9ea6472bed0:
.. _cid-s2_material_t::facecount:
.. _doxid-structs2__material__t_1a6bbca2d1ee4e00e82a0e3ef66f4836a6:
.. _cid-s2_material_t::vertices:
.. _doxid-structs2__material__t_1aad4f4f953b32c933be85a8ff30b194fc:
.. _cid-s2_material_t::vertexnormals:
.. _doxid-structs2__material__t_1a1b8874b16f8e42ff893cdc0423afecfb:
.. _cid-s2_material_t::faces:
.. _doxid-structs2__material__t_1a1b7e44d9e9333c5d9714d0a13542e135:
.. _cid-s2_material_t::vertexnormalbuffer:
.. ref-code-block:: cpp
	:class: overview-code-block

	// structs

	struct :ref:`GLuniforms<doxid-structs2__material__t_1_1_g_luniforms>`

	// fields

	char* name
	int index
	char* filePath
	s2_shading_enum_t shadingType
	struct :ref:`s2_material_t::GLuniforms<doxid-structs2__material__t_1_1_g_luniforms>` GLuniforms
	int vertexCount
	int faceCount
	GLfloat* vertices
	GLfloat* vertexNormals
	GLuint* faces
	GLuint :ref:`vertexBuffer<doxid-structs2__material__t_1af4fa885749e7f6273d4f56172883ad24>`
	GLuint vertexNormalBuffer
	GLuint :ref:`vao<doxid-structs2__material__t_1ae78e76b202febfa89a935931e0255ddd>`
	GLuint :ref:`ibo<doxid-structs2__material__t_1a3609985ef907cccb196088223ccb9c41>`

.. _details-doxid-structs2__material__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

data structure defining a material

Fields
------

.. _doxid-structs2__material__t_1af4fa885749e7f6273d4f56172883ad24:
.. _cid-s2_material_t::vertexbuffer:
.. ref-code-block:: cpp
	:class: title-code-block

	GLuint vertexBuffer

vertex buffer objects

.. _doxid-structs2__material__t_1ae78e76b202febfa89a935931e0255ddd:
.. _cid-s2_material_t::vao:
.. ref-code-block:: cpp
	:class: title-code-block

	GLuint vao

vertex array objects

.. _doxid-structs2__material__t_1a3609985ef907cccb196088223ccb9c41:
.. _cid-s2_material_t::ibo:
.. ref-code-block:: cpp
	:class: title-code-block

	GLuint ibo

index buffer objects

