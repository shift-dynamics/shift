.. index:: pair: struct; s2_units_t
.. _doxid-structs2__units__t:
.. _cid-s2_units_t:

struct s2_units_t
=================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <context.h>


Overview
~~~~~~~~

attributes associated with units :ref:`More...<details-doxid-structs2__units__t>`

.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	char* :ref:`angle<doxid-structs2__units__t_1a66d020183fce23a1be921a80de49ce54>`
	char* :ref:`length<doxid-structs2__units__t_1add71dc9dfea1412404c3cb83422cee25>`
	char* :ref:`mass<doxid-structs2__units__t_1a4c3a8732c753219736f082d65316dd7e>`
	char* :ref:`time<doxid-structs2__units__t_1a8f91331f78cb7e4f554859efc94e4325>`

.. _details-doxid-structs2__units__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

attributes associated with units

Fields
------

.. _doxid-structs2__units__t_1a66d020183fce23a1be921a80de49ce54:
.. _cid-s2_units_t::angle:
.. ref-code-block:: cpp
	:class: title-code-block

	char* angle

internally, radians will always be used, valid values are "radians" or "degrees".

.. _doxid-structs2__units__t_1add71dc9dfea1412404c3cb83422cee25:
.. _cid-s2_units_t::length:
.. ref-code-block:: cpp
	:class: title-code-block

	char* length

specify distance units to report to user (default is "meters")

.. _doxid-structs2__units__t_1a4c3a8732c753219736f082d65316dd7e:
.. _cid-s2_units_t::mass:
.. ref-code-block:: cpp
	:class: title-code-block

	char* mass

specify mass units to report to user (default is "kilograms")

.. _doxid-structs2__units__t_1a8f91331f78cb7e4f554859efc94e4325:
.. _cid-s2_units_t::time:
.. ref-code-block:: cpp
	:class: title-code-block

	char* time

specify time units to report to user (default is "seconds")

