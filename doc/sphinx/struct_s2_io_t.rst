.. index:: pair: struct; s2_io_t
.. _doxid-structs2__io__t:
.. _cid-s2_io_t:

struct s2_io_t
==============

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <io.h>


Overview
~~~~~~~~

input/output data structure :ref:`More...<details-doxid-structs2__io__t>`

.. _doxid-structs2__io__t_1a989989d997069011333e6816382cbd37:
.. _cid-s2_io_t::name:
.. _doxid-structs2__io__t_1a31edefaccab41dfccb5aafe66861ecd5:
.. _cid-s2_io_t::stepcount:
.. _doxid-structs2__io__t_1ae3ae75a21e627bf461b5a21311c4bf4e:
.. _cid-s2_io_t::index:
.. _doxid-structs2__io__t_1a9c00d43ddce87405e19325fb372b0299:
.. _cid-s2_io_t::deltadisplacement:
.. _doxid-structs2__io__t_1afa16b93bb781159838d6992e117dbd73:
.. _cid-s2_io_t::displacement:
.. _doxid-structs2__io__t_1a8276930026288795e42e0f2017bc1274:
.. _cid-s2_io_t::sensorhead:
.. _doxid-structs2__io__t_1a147d1de7fbb4dd2f7a30f8ca8d896669:
.. _cid-s2_io_t::sensortail:
.. _doxid-structs2__io__t_1a89acfaf02a7a70fc2a2698631eeb757e:
.. _cid-s2_io_t::camerahead:
.. _doxid-structs2__io__t_1a222e343b4dcc4774e9f040c5d021d120:
.. _cid-s2_io_t::cameratail:
.. _doxid-structs2__io__t_1aca1620572d919ca5dfb327fcbcbd70ae:
.. _cid-s2_io_t::next:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name
	int stepCount
	int index
	double deltaDisplacement
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* displacement
	:ref:`s2_sensor_t<doxid-structs2__sensor__t>`* sensorHead
	:ref:`s2_sensor_t<doxid-structs2__sensor__t>`* sensorTail
	:ref:`s2_camera_t<doxid-structs2__camera__t>`* cameraHead
	:ref:`s2_camera_t<doxid-structs2__camera__t>`* cameraTail
	:ref:`s2_io_t<doxid-structs2__io__t>`* next

.. _details-doxid-structs2__io__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

input/output data structure

This structure stores data related to sensors, i.e. cameras, lidar, radar, touch

