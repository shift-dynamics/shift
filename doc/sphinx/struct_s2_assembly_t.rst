.. index:: pair: struct; s2_assembly_t
.. _doxid-structs2__assembly__t:
.. _cid-s2_assembly_t:

struct s2_assembly_t
====================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <assembly.h>


Overview
~~~~~~~~

Describes properties related to an Assembly. :ref:`More...<details-doxid-structs2__assembly__t>`

.. _doxid-structs2__assembly__t_1a2bdf8a32d7d64237e7069fb656e32f26:
.. _cid-s2_assembly_t::pointcloudhead:
.. _doxid-structs2__assembly__t_1a68d840a2fe232a8f7ca79f479a0b8efc:
.. _cid-s2_assembly_t::pointcloudtail:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* :ref:`next<doxid-structs2__assembly__t_1ac3bdf6578aaa0c1bc80f319e2d2a4c88>`
	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* :ref:`subAssemblyHead<doxid-structs2__assembly__t_1a57d99420d2914143dff453eef7d43c89>`
	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* :ref:`subAssemblyTail<doxid-structs2__assembly__t_1a6fb575d9d0a0f01fe3e1f93b5f65f206>`
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`name<doxid-structs2__assembly__t_1ad26103b24333dff365d23bf1683b3b4a>`
	bool :ref:`active<doxid-structs2__assembly__t_1af20176169b3312b9f08b13d2278f0132>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`bodyHead<doxid-structs2__assembly__t_1a2146155467ab684d2dbd26290461103e>`
	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`bodyTail<doxid-structs2__assembly__t_1aa91c2295fa4bd86e6df9b0e05c7c1caf>`
	:ref:`s2_joint_t<doxid-structs2__joint__t>`* :ref:`jointHead<doxid-structs2__assembly__t_1a040cd5dc49845080cacdf47290eb38a1>`
	:ref:`s2_joint_t<doxid-structs2__joint__t>`* :ref:`jointTail<doxid-structs2__assembly__t_1aa046ac0c2e2da3025f5e8d03aa77a938>`
	:ref:`s2_light_t<doxid-structs2__light__t>`* :ref:`lightHead<doxid-structs2__assembly__t_1a053c01832be030e8c57495572ac2c8ca>`
	:ref:`s2_light_t<doxid-structs2__light__t>`* :ref:`lightTail<doxid-structs2__assembly__t_1aa9888734c2173144fc40cbf63d4d77ef>`
	:ref:`s2_restraint_t<doxid-structs2__restraint__t>`* :ref:`restraintHead<doxid-structs2__assembly__t_1a45eac317f20490d2ce541c0e7656a4ae>`
	:ref:`s2_restraint_t<doxid-structs2__restraint__t>`* :ref:`restraintTail<doxid-structs2__assembly__t_1ad0d905631709191e910f5b231db507ca>`
	:ref:`s2_io_t<doxid-structs2__io__t>`* :ref:`ioHead<doxid-structs2__assembly__t_1a0a6e9740532ee8d87d01a9deac264a6b>`
	:ref:`s2_io_t<doxid-structs2__io__t>`* :ref:`ioTail<doxid-structs2__assembly__t_1aecdae67f5939dfa4d3a21446ba9b429d>`
	:ref:`s2_pointcloud_t<doxid-structs2__pointcloud__t>`* pointCloudHead
	:ref:`s2_pointcloud_t<doxid-structs2__pointcloud__t>`* pointCloudTail
	:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* :ref:`forceModuleHead<doxid-structs2__assembly__t_1ae90986003260af71f72eeae58385f62f>`
	:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* :ref:`forceModuleTail<doxid-structs2__assembly__t_1ae1fb32ba961f32443d8d2a32b6b76694>`

.. _details-doxid-structs2__assembly__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Describes properties related to an Assembly.

Each assembly contains a descriptive name, a flag to specify whether the Assembly is active/inactive and a linked list of:

* Subassemblies

* Bodies

* Joints

* Lights

* Restraints

Fields
------

.. _doxid-structs2__assembly__t_1ac3bdf6578aaa0c1bc80f319e2d2a4c88:
.. _cid-s2_assembly_t::next:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* next

pointer to next assembly in linked list

.. _doxid-structs2__assembly__t_1a57d99420d2914143dff453eef7d43c89:
.. _cid-s2_assembly_t::subassemblyhead:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* subAssemblyHead

pointer to head of subAssembly linked list datastructure

.. _doxid-structs2__assembly__t_1a6fb575d9d0a0f01fe3e1f93b5f65f206:
.. _cid-s2_assembly_t::subassemblytail:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* subAssemblyTail

pointer to tail of subAssembly linked list datastructure

.. _doxid-structs2__assembly__t_1ad26103b24333dff365d23bf1683b3b4a:
.. _cid-s2_assembly_t::name:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name

Assembly descriptive name

.. _doxid-structs2__assembly__t_1af20176169b3312b9f08b13d2278f0132:
.. _cid-s2_assembly_t::active:
.. ref-code-block:: cpp
	:class: title-code-block

	bool active

specifies whether current assembly is active, i.e. true for "switched on", false for "switched off"

.. _doxid-structs2__assembly__t_1a2146155467ab684d2dbd26290461103e:
.. _cid-s2_assembly_t::bodyhead:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* bodyHead

head of Body linked list

.. _doxid-structs2__assembly__t_1aa91c2295fa4bd86e6df9b0e05c7c1caf:
.. _cid-s2_assembly_t::bodytail:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* bodyTail

tail of Body linked list

.. _doxid-structs2__assembly__t_1a040cd5dc49845080cacdf47290eb38a1:
.. _cid-s2_assembly_t::jointhead:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_joint_t<doxid-structs2__joint__t>`* jointHead

head of Joint linked list

.. _doxid-structs2__assembly__t_1aa046ac0c2e2da3025f5e8d03aa77a938:
.. _cid-s2_assembly_t::jointtail:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_joint_t<doxid-structs2__joint__t>`* jointTail

tail of Joint linked list

.. _doxid-structs2__assembly__t_1a053c01832be030e8c57495572ac2c8ca:
.. _cid-s2_assembly_t::lighthead:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_light_t<doxid-structs2__light__t>`* lightHead

head of Light linked list

.. _doxid-structs2__assembly__t_1aa9888734c2173144fc40cbf63d4d77ef:
.. _cid-s2_assembly_t::lighttail:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_light_t<doxid-structs2__light__t>`* lightTail

tail of Light linked list

.. _doxid-structs2__assembly__t_1a45eac317f20490d2ce541c0e7656a4ae:
.. _cid-s2_assembly_t::restrainthead:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_restraint_t<doxid-structs2__restraint__t>`* restraintHead

head of Restraint linked list

.. _doxid-structs2__assembly__t_1ad0d905631709191e910f5b231db507ca:
.. _cid-s2_assembly_t::restrainttail:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_restraint_t<doxid-structs2__restraint__t>`* restraintTail

tail of Restraint linked list

.. _doxid-structs2__assembly__t_1a0a6e9740532ee8d87d01a9deac264a6b:
.. _cid-s2_assembly_t::iohead:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_io_t<doxid-structs2__io__t>`* ioHead

head of Device linked list

.. _doxid-structs2__assembly__t_1aecdae67f5939dfa4d3a21446ba9b429d:
.. _cid-s2_assembly_t::iotail:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_io_t<doxid-structs2__io__t>`* ioTail

tail of Device linked list

.. _doxid-structs2__assembly__t_1ae90986003260af71f72eeae58385f62f:
.. _cid-s2_assembly_t::forcemodulehead:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* forceModuleHead

head of forcemodule linked list

.. _doxid-structs2__assembly__t_1ae1fb32ba961f32443d8d2a32b6b76694:
.. _cid-s2_assembly_t::forcemoduletail:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* forceModuleTail

tail of forcemodule linked list

