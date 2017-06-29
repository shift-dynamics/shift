.. index:: pair: struct; s2_light_t
.. _doxid-structs2__light__t:
.. _cid-s2_light_t:

struct s2_light_t
=================

.. toctree::
	:hidden:

	struct_s2_light_t_glLight.rst

.. code-block:: cpp
	:class: overview-code-block

	#include <context.h>


Overview
~~~~~~~~

Defines a Light. :ref:`More...<details-doxid-structs2__light__t>`

.. _doxid-structs2__light__t_1a0d0ad11060e2ff6ff18d815b497468a5:
.. _cid-s2_light_t::index:
.. _doxid-structs2__light__t_1a408dca1e00277c6c64fc2fa96ac4087f:
.. _cid-s2_light_t::lightuniform:
.. _doxid-structs2__light__t_1a2ee4483b1c210f25800b481dbf95aa0e:
.. _cid-s2_light_t::lighttype:
.. _doxid-structs2__light__t_1ab9637a8dd0d0feccfcad3324ba95763c:
.. _cid-s2_light_t::gllight:
.. _doxid-structs2__light__t_1a8c2a0ed4bc5e3489ad6c63e44c277fab:
.. _cid-s2_light_t::next:
.. ref-code-block:: cpp
	:class: overview-code-block

	// structs

	struct :ref:`glLight<doxid-structs2__light__t_1_1gl_light>`

	// fields

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`name<doxid-structs2__light__t_1a630787013d81b26deab30c817e005103>`
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`parentFrame<doxid-structs2__light__t_1ae5d1a6946b0282e3a4835d67c32f827e>`
	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`parentFramePointer<doxid-structs2__light__t_1a81027745043f043d8a451876a96fff6e>`
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`parentBody<doxid-structs2__light__t_1afe561c0b2a0085760f9db3db367a4dc0>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`parentBodyPointer<doxid-structs2__light__t_1a94daf5ec83bf9c5881156ba3ed417d45>`
	int index
	GLuint lightUniform
	:ref:`s2_light_enum_t<doxid-context_8h_1a19bdf894dc0735ff78ec2d5d08821bef>` lightType
	struct :ref:`s2_light_t::glLight<doxid-structs2__light__t_1_1gl_light>` glLight
	struct :ref:`s2_light_t<doxid-structs2__light__t>`* next

.. _details-doxid-structs2__light__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Defines a Light.

Fields
------

.. _doxid-structs2__light__t_1a630787013d81b26deab30c817e005103:
.. _cid-s2_light_t::name:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name

descriptive name of Light

.. _doxid-structs2__light__t_1ae5d1a6946b0282e3a4835d67c32f827e:
.. _cid-s2_light_t::parentframe:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* parentFrame

frame that Light is attached to

.. _doxid-structs2__light__t_1a81027745043f043d8a451876a96fff6e:
.. _cid-s2_light_t::parentframepointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* parentFramePointer

pointer to frame that Light is attached to

.. _doxid-structs2__light__t_1afe561c0b2a0085760f9db3db367a4dc0:
.. _cid-s2_light_t::parentbody:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* parentBody

descriptive name of body that Light is attached to note, set to "fixed" for inertial reference frame

.. _doxid-structs2__light__t_1a94daf5ec83bf9c5881156ba3ed417d45:
.. _cid-s2_light_t::parentbodypointer:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* parentBodyPointer

pointer to parent body that Light is light is attached to

