.. index:: pair: struct; s2_forcemodule_t
.. _doxid-structs2__forcemodule__t:
.. _cid-s2_forcemodule_t:

struct s2_forcemodule_t
=======================

.. toctree::
	:hidden:



Overview
~~~~~~~~



.. _doxid-structs2__forcemodule__t_1a95898956a9297fceade17c11cf8345e9:
.. _cid-s2_forcemodule_t::forcemoduletype:
.. _doxid-structs2__forcemodule__t_1a1eadbd04ca7109c3cc1348b8f4afa9a0:
.. _cid-s2_forcemodule_t::data:
.. _doxid-structs2__forcemodule__t_1aabce94387c9efccd420513a29dad0bd2:
.. _cid-s2_forcemodule_t::update:
.. _doxid-structs2__forcemodule__t_1afc12b7b5e5140e69b1c7b6ba35c924be:
.. _cid-s2_forcemodule_t::savehistory:
.. _doxid-structs2__forcemodule__t_1ab2e061cebecb70fa509e285a23b47d7f:
.. _cid-s2_forcemodule_t::parameters:
.. _doxid-structs2__forcemodule__t_1af5883413b20d840b7fc5266df88a24f3:
.. _cid-s2_forcemodule_t::yoffset:
.. _doxid-structs2__forcemodule__t_1a457c507e73dc9c46a94afd622f507baa:
.. _cid-s2_forcemodule_t::y:
.. _doxid-structs2__forcemodule__t_1ad8be615c28b20d6f918d24ed50e22cbf:
.. _cid-s2_forcemodule_t::usetransient:
.. _doxid-structs2__forcemodule__t_1a57f653441ac46b480063609ede56accc:
.. _cid-s2_forcemodule_t::name:
.. _doxid-structs2__forcemodule__t_1ac53ee9e87e1b73dd8033a0730a3642fb:
.. _cid-s2_forcemodule_t::next:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	:ref:`s2_forcemodule_enum_t<doxid-context_8h_1aef216ac0fc3d71c6419b69df97ab6091>` forceModuleType
	void* data
	void (* update )(s2_context_t *context, void **parameters, double *y)
	void (* saveHistory )(void *parameters)
	void** parameters
	int yOffset
	int :ref:`neqn<doxid-structs2__forcemodule__t_1a962dca97725645c5efeeeb81b1fe4e7f>`
	double* y
	bool useTransient
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name
	:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* next

.. _details-doxid-structs2__forcemodule__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. _doxid-structs2__forcemodule__t_1a962dca97725645c5efeeeb81b1fe4e7f:
.. _cid-s2_forcemodule_t::neqn:
.. ref-code-block:: cpp
	:class: title-code-block

	int neqn

number of states that need to be numerically integrated

