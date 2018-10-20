.. _global-namespace:

Global Namespace
================

.. index:: pair: namespace; global

.. toctree::
	:hidden:

	enum_s2_forcemodule_enum_t.rst
	enum_s2_numericalintegrator_enum_t.rst
	enum_s2_pid_io_enum_t.rst
	enum_s2_pid_target_enum_t.rst
	enum_s2_spring_enum_t.rst
	struct_aligning_coefficients.rst
	struct_bessel.rst
	struct_combinedLatCoefs.rst
	struct_combinedLongCoefs.rst
	struct_combinedTorqueCoefs.rst
	struct_dimension.rst
	struct_inclination_angle_range.rst
	struct_lateral_coefficients.rst
	struct_long_slip_range.rst
	struct_longitudinal_coefficients.rst
	struct_model.rst
	struct_overturning_coefficients.rst
	struct_pureLatCoefs.rst
	struct_pureLongCoefs.rst
	struct_pureTorqueCoefs.rst
	struct_relaxationL.rst
	struct_rolling_coefficients.rst
	struct_s2_damper_t.rst
	struct_s2_forcemodule_t.rst
	struct_s2_numericalintegrator_ode113_t.rst
	struct_s2_numericalintegrator_ode113b_t.rst
	struct_s2_pid_t.rst
	struct_s2_spring_t.rst
	struct_s2_tire_fiala_t.rst
	struct_s2_tire_pac2002_t.rst
	struct_scaling_coefficients.rst
	struct_shape.rst
	struct_slip_angle_range.rst
	struct_slips.rst
	struct_thresholds.rst
	struct_vertical.rst
	struct_vertical_force_range.rst
	struct_zetaCoefs.rst



Overview
~~~~~~~~



.. _doxid-damping_8h_1a873aba81c508a23704e626d0e0e601fd:
.. _cid-s2_damping_joint:
.. _doxid-forcemodule_8h_1a41e9009a1a740467d85e1f288c8b29a2:
.. _cid-s2_context_updatetransientforcemodules:
.. _doxid-pac2002_8h_1a2b23b28368c29fdd8a6e6722aafbd971:
.. _cid-s2_tire_pac2002_getslipangle:
.. _doxid-pac2002_8h_1ad14ad96c35ae30de74ca5229755b8ed1:
.. _cid-s2_tire_pac2002_evaluateslips:
.. _doxid-pac2002_8h_1a638d583618f1ac7c30f49fd502052b8a:
.. _cid-s2_tire_pac2002_updatetransient:
.. _doxid-tire_2pac2002_8h_1a2b23b28368c29fdd8a6e6722aafbd971:
.. _cid-s2_tire_pac2002_getslipangle-2:
.. _doxid-tire_2pac2002_8h_1ad14ad96c35ae30de74ca5229755b8ed1:
.. _cid-s2_tire_pac2002_evaluateslips-2:
.. _doxid-tire_2pac2002_8h_1a638d583618f1ac7c30f49fd502052b8a:
.. _cid-s2_tire_pac2002_updatetransient-2:
.. _doxid-pid_8h_1ac3fcf5ae5ad71668057d00f0ae90c88e:
.. _cid-s2_pid:
.. _doxid-pid_8h_1a185d820d6dcffa61f7e366c2c29de821:
.. _cid-s2_pid_connectforcemodulepointer:
.. _doxid-pid_8h_1a85e32c1d56debf7f65c542282af661d2:
.. _cid-s2_pid_savehistory:
.. _doxid-ode113_8h_1adcd75d7f65e131700874564af7d23950:
.. _cid-s2_ode113:
.. _doxid-ode113_8h_1ae7315a7bfae54317d282dafb04db8b4a:
.. _cid-s2_ode113_de:
.. _doxid-ode113_8h_1a76596de5ad03eec0ee220c97d1ed3094:
.. _cid-s2_ode113_step:
.. _doxid-ode113_8h_1a02d8f81e512334c1e2e5be025c41afa8:
.. _cid-timestamp:
.. _doxid-ode113_8h_1a89a71b03e989e682ed4502045c59e529:
.. _cid-s2_context_allocode113:
.. _doxid-ode113b_8h_1a0771d9f38f3152af927c16eb5400a972:
.. _cid-s2_ode113b_step:
.. ref-code-block:: cpp
	:class: overview-code-block

	// typedefs

	typedef struct :ref:`s2_damper_t<doxid-structs2__damper__t>` :ref:`s2_damper_t<doxid-damping_8h_1ad827256522fc82b2ad4001167238a775>`
	typedef struct :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>` :ref:`s2_forcemodule_t<doxid-forcemodule_8h_1a6cfa09e2454a092d1d37c32a584f809a>`
	typedef struct :ref:`s2_pid_t<doxid-structs2__pid__t>` :ref:`s2_pid_t<doxid-pid_8h_1ab0e710f017bf8a44db747a1ada3e6bfc>`
	typedef struct :ref:`s2_spring_t<doxid-structs2__spring__t>` :ref:`s2_spring_t<doxid-stiffness_8h_1a810c566c41ba7f20abc258759f13d3b0>`
	typedef struct :ref:`s2_numericalintegrator_ode113_t<doxid-structs2__numericalintegrator__ode113__t>` :ref:`s2_numericalintegrator_ode113_t<doxid-ode113_8h_1a204cc0f0ebd6f99f35ec1a8fdf2816b5>`
	typedef struct :ref:`s2_numericalintegrator_ode113b_t<doxid-structs2__numericalintegrator__ode113b__t>` :ref:`s2_numericalintegrator_ode113b_t<doxid-ode113b_8h_1a749caa804014e62fb4ab043f25398d97>`

	// enums

	enum :ref:`s2_forcemodule_enum_t<doxid-forcemodule_8h_1aef216ac0fc3d71c6419b69df97ab6091>`
	enum :ref:`s2_numericalintegrator_enum_t<doxid-numericalintegrator_8h_1af852b2c473abbd9d2bf3556aab33755f>`
	enum :ref:`s2_pid_io_enum_t<doxid-pid_8h_1a81311461f9fd6d8fa2460aeb85cfcd8f>`
	enum :ref:`s2_pid_target_enum_t<doxid-pid_8h_1aa2bf678870fa220e7f7e78cad0d64a87>`
	enum :ref:`s2_spring_enum_t<doxid-stiffness_8h_1ae87c61c350fb6f47f850b1edac2f43f6>`

	// structs

	struct :ref:`aligning_coefficients<doxid-structaligning__coefficients>`
	struct :ref:`bessel<doxid-structbessel>`
	struct :ref:`combinedLatCoefs<doxid-structcombined_lat_coefs>`
	struct :ref:`combinedLongCoefs<doxid-structcombined_long_coefs>`
	struct :ref:`combinedTorqueCoefs<doxid-structcombined_torque_coefs>`
	struct :ref:`dimension<doxid-structdimension>`
	struct :ref:`inclination_angle_range<doxid-structinclination__angle__range>`
	struct :ref:`lateral_coefficients<doxid-structlateral__coefficients>`
	struct :ref:`long_slip_range<doxid-structlong__slip__range>`
	struct :ref:`longitudinal_coefficients<doxid-structlongitudinal__coefficients>`
	struct :ref:`model<doxid-structmodel>`
	struct :ref:`overturning_coefficients<doxid-structoverturning__coefficients>`
	struct :ref:`pureLatCoefs<doxid-structpure_lat_coefs>`
	struct :ref:`pureLongCoefs<doxid-structpure_long_coefs>`
	struct :ref:`pureTorqueCoefs<doxid-structpure_torque_coefs>`
	struct :ref:`relaxationL<doxid-structrelaxation_l>`
	struct :ref:`rolling_coefficients<doxid-structrolling__coefficients>`
	struct :ref:`s2_damper_t<doxid-structs2__damper__t>`
	struct :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`
	struct :ref:`s2_numericalintegrator_ode113_t<doxid-structs2__numericalintegrator__ode113__t>`
	struct :ref:`s2_numericalintegrator_ode113b_t<doxid-structs2__numericalintegrator__ode113b__t>`
	struct :ref:`s2_pid_t<doxid-structs2__pid__t>`
	struct :ref:`s2_spring_t<doxid-structs2__spring__t>`
	struct :ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`
	struct :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`
	struct :ref:`scaling_coefficients<doxid-structscaling__coefficients>`
	struct :ref:`shape<doxid-structshape>`
	struct :ref:`slip_angle_range<doxid-structslip__angle__range>`
	struct :ref:`slips<doxid-structslips>`
	struct :ref:`thresholds<doxid-structthresholds>`
	struct :ref:`vertical<doxid-structvertical>`
	struct :ref:`vertical_force_range<doxid-structvertical__force__range>`
	struct :ref:`zetaCoefs<doxid-structzeta_coefs>`

	// global functions

	void
	:ref:`s2_context_basicSweep<doxid-contact_8h_1aea526a1aa339c059baf450365e358798>` (s2_context_t*)

	void
	:ref:`s2_context_sweepAndPrune<doxid-contact_8h_1a09906598ec946d4dcb6bfb9f995a2aba>` (s2_context_t*)

	void
	:ref:`s2_context_contactDetection<doxid-contact_8h_1ad296820a87cce60f4e47af0af0a20fb8>` (s2_context_t*)

	void
	s2_damping_joint (
	    s2_context_t* context,
	    s2_joint_t* joint
	    )

	:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`*
	:ref:`s2_tire_fiala<doxid-fiala_8h_1a8658302eeb1bec4475877e8367172394>` (
	    s2_context_t* context,
	    const char* filePath,
	    s2_stackstr_t* wheelBody
	    )

	void
	:ref:`s2_tire_fiala_free<doxid-fiala_8h_1a3e1a2e637efc322442633553baba498c>` (:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire)

	void
	:ref:`s2_tire_fiala_update<doxid-fiala_8h_1a322fce69ed2f93ae89452a861d8303ca>` (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

	void
	:ref:`s2_tire_fiala_loadTireDataFile<doxid-fiala_8h_1aef0605aba2e6664ad511704f3adc29fc>` (
	    s2_context_t* context,
	    :ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire,
	    const char* filePath
	    )

	:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`*
	:ref:`s2_tire_fiala<doxid-tire_2fiala_8h_1a8658302eeb1bec4475877e8367172394>` (
	    s2_context_t* context,
	    const char* filePath,
	    s2_stackstr_t* wheelBody
	    )

	void
	:ref:`s2_tire_fiala_free<doxid-tire_2fiala_8h_1a3e1a2e637efc322442633553baba498c>` (:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire)

	void
	:ref:`s2_tire_fiala_update<doxid-tire_2fiala_8h_1a322fce69ed2f93ae89452a861d8303ca>` (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

	void
	:ref:`s2_tire_fiala_loadTireDataFile<doxid-tire_2fiala_8h_1aef0605aba2e6664ad511704f3adc29fc>` (
	    s2_context_t* context,
	    :ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire,
	    const char* filePath
	    )

	:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`*
	:ref:`s2_forcemodule<doxid-forcemodule_8h_1a10ba39388d465932d035489090731365>` (:ref:`s2_forcemodule_enum_t<doxid-forcemodule_8h_1aef216ac0fc3d71c6419b69df97ab6091>`)

	void
	:ref:`s2_context_flattenForceModules<doxid-forcemodule_8h_1a1462d12cabcbd4c2af3cd01be718db08>` (s2_context_t*)

	void
	:ref:`s2_context_connectForceModules<doxid-forcemodule_8h_1a4a9554e0c66689426792d17240ffe55b>` (s2_context_t*)

	void
	s2_context_updateTransientForceModules (
	    s2_context_t*,
	    double*
	    )

	void
	:ref:`s2_context_saveForceModuleHistory<doxid-forcemodule_8h_1a7a29be11cf9afd14c11f80670658753e>` (s2_context_t* context)

	void
	:ref:`s2_forcemodule_append<doxid-forcemodule_8h_1aa53d7be6608c3b229cbc7c2dd1b5bd56>` (
	    s2_assembly_t* assembly,
	    :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* forceModule
	    )

	void
	:ref:`s2_forcemodule_free<doxid-forcemodule_8h_1a5bd61c1828561862ae4357cf37cbc2bb>` (:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`*)

	void
	:ref:`s2_forcemodule_load<doxid-forcemodule_8h_1ac28262f7c2ba16e662ff311b36ba2c09>` (
	    s2_context_t* context,
	    s2_assembly_t* assembly,
	    s2_stackstr_t* prefix,
	    json_t* forceModuleObject
	    )

	:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`*
	:ref:`s2_tire_pac2002<doxid-pac2002_8h_1a52c64dbbbd2c064b8032d7dcce7a10d2>` (
	    s2_context_t* context,
	    const char* filePath,
	    s2_stackstr_t* wheelBody,
	    bool useTransientSlip
	    )

	void
	:ref:`s2_tire_pac2002_free<doxid-pac2002_8h_1a75e3fa6c00463a48be7638e921500143>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_update<doxid-pac2002_8h_1a73243b38626d6cb76ca38ee1d00b83f6>` (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

	void
	s2_tire_pac2002_getSlipAngle (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_loadTireDataFile<doxid-pac2002_8h_1a3392f865e9daaa82bbf47793f3a5b8ab>` (
	    s2_context_t* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    const char* filePath
	    )

	void
	:ref:`s2_tire_pac2002_updateWFrame<doxid-pac2002_8h_1a5075ecf586909626c01f2df725bc3af4>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	double
	:ref:`s2_tire_pac2002_calc_Fz<doxid-pac2002_8h_1aeaaea7981cb1d6b432d05f587fd66ec9>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_updateRelaxationLengths<doxid-pac2002_8h_1a215220f8e29bf377a51244a6ca6cced0>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_updateSlipKinematic<doxid-pac2002_8h_1a80e0c5ab777c6793a5e1a74a83dfa5c7>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_zeroSlips<doxid-pac2002_8h_1a3bd2800b93b071b82df4ef3986561110>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	s2_tire_pac2002_evaluateSlips (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_evaluateReactions<doxid-pac2002_8h_1ac20ea3476c1d1cbf4dc6f6cfcf52e168>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    bool write_violations,
	    bool enforce_threshold
	    )

	void
	:ref:`s2_tire_pac2002_slip_from_uv<doxid-pac2002_8h_1ae0239415e4200b821dec893a8bf183b8>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    bool use_besselink,
	    double bessel_Cx,
	    double bessel_Cy,
	    double V_low
	    )

	void
	:ref:`s2_tire_pac2002_pureSlipReactions<doxid-pac2002_8h_1ae9766ff827292da718194567c723b4eb>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_combinedSlipReactions<doxid-pac2002_8h_1a2e900f40155187ffbb850e80964d17bd>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	double
	:ref:`s2_tire_pac2002_Fx_pureLong<doxid-pac2002_8h_1a0c87e22d7746b5f7630c920217b6a3aa>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double kappa
	    )

	double
	:ref:`s2_tire_pac2002_Fy_pureLat<doxid-pac2002_8h_1aab053dca94043c0115b8c86b755608f3>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma
	    )

	double
	:ref:`s2_tire_pac2002_Mz_pureLat<doxid-pac2002_8h_1aa1f7798f862f1b1af24b410938970577>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double Fy_pureSlip
	    )

	double
	:ref:`s2_tire_pac2002_Fx_combined<doxid-pac2002_8h_1a75967c7dc5b18d94b56e03beb9d7cbb4>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double kappa,
	    double Fx_pureSlip
	    )

	double
	:ref:`s2_tire_pac2002_Fy_combined<doxid-pac2002_8h_1a5873fda415a72e239ea243bed79405c7>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double kappa,
	    double Fy_pureSlip
	    )

	double
	:ref:`s2_tire_pac2002_Mz_combined<doxid-pac2002_8h_1ad36b82bb284940522554a8675cbf9e2f>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha_r,
	    double alpha_t,
	    double gamma,
	    double kappa,
	    double Fx_combined,
	    double Fy_combined
	    )

	double
	:ref:`s2_tire_pac2002_calc_Mx<doxid-pac2002_8h_1a095bb61ec83dc5a79153f55e4d66f56d>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double Fy_combined
	    )

	double
	:ref:`s2_tire_pac2002_calc_My<doxid-pac2002_8h_1ae8f86738eda52bbc1d5ea709659c07d7>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double Fx_combined
	    )

	bool
	:ref:`s2_tire_pac2002_discTerrainContact<doxid-pac2002_8h_1ab7a1630c7e0a24a451236c2cb143dc15>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	s2_tire_pac2002_updateTransient (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

	void
	:ref:`s2_tire_pac2002_updateVerticalLoad<doxid-pac2002_8h_1aad631fb210d174a3be0f80a058b42197>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_transferForces<doxid-pac2002_8h_1ac1c6d29395ad7cec3dd2d9127b6fdbd9>` (
	    s2_context_t* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire
	    )

	:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`*
	:ref:`s2_tire_pac2002<doxid-tire_2pac2002_8h_1a52c64dbbbd2c064b8032d7dcce7a10d2>` (
	    s2_context_t* context,
	    const char* filePath,
	    s2_stackstr_t* wheelBody,
	    bool useTransientSlip
	    )

	void
	:ref:`s2_tire_pac2002_free<doxid-tire_2pac2002_8h_1a75e3fa6c00463a48be7638e921500143>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_update<doxid-tire_2pac2002_8h_1a73243b38626d6cb76ca38ee1d00b83f6>` (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

	void
	s2_tire_pac2002_getSlipAngle (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_loadTireDataFile<doxid-tire_2pac2002_8h_1a3392f865e9daaa82bbf47793f3a5b8ab>` (
	    s2_context_t* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    const char* filePath
	    )

	void
	:ref:`s2_tire_pac2002_updateWFrame<doxid-tire_2pac2002_8h_1a5075ecf586909626c01f2df725bc3af4>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	double
	:ref:`s2_tire_pac2002_calc_Fz<doxid-tire_2pac2002_8h_1aeaaea7981cb1d6b432d05f587fd66ec9>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_updateRelaxationLengths<doxid-tire_2pac2002_8h_1a215220f8e29bf377a51244a6ca6cced0>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_updateSlipKinematic<doxid-tire_2pac2002_8h_1a80e0c5ab777c6793a5e1a74a83dfa5c7>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_zeroSlips<doxid-tire_2pac2002_8h_1a3bd2800b93b071b82df4ef3986561110>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	s2_tire_pac2002_evaluateSlips (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_evaluateReactions<doxid-tire_2pac2002_8h_1ac20ea3476c1d1cbf4dc6f6cfcf52e168>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    bool write_violations,
	    bool enforce_threshold
	    )

	void
	:ref:`s2_tire_pac2002_slip_from_uv<doxid-tire_2pac2002_8h_1ae0239415e4200b821dec893a8bf183b8>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    bool use_besselink,
	    double bessel_Cx,
	    double bessel_Cy,
	    double V_low
	    )

	void
	:ref:`s2_tire_pac2002_pureSlipReactions<doxid-tire_2pac2002_8h_1ae9766ff827292da718194567c723b4eb>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_combinedSlipReactions<doxid-tire_2pac2002_8h_1a2e900f40155187ffbb850e80964d17bd>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	double
	:ref:`s2_tire_pac2002_Fx_pureLong<doxid-tire_2pac2002_8h_1a0c87e22d7746b5f7630c920217b6a3aa>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double kappa
	    )

	double
	:ref:`s2_tire_pac2002_Fy_pureLat<doxid-tire_2pac2002_8h_1aab053dca94043c0115b8c86b755608f3>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma
	    )

	double
	:ref:`s2_tire_pac2002_Mz_pureLat<doxid-tire_2pac2002_8h_1aa1f7798f862f1b1af24b410938970577>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double Fy_pureSlip
	    )

	double
	:ref:`s2_tire_pac2002_Fx_combined<doxid-tire_2pac2002_8h_1a75967c7dc5b18d94b56e03beb9d7cbb4>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double kappa,
	    double Fx_pureSlip
	    )

	double
	:ref:`s2_tire_pac2002_Fy_combined<doxid-tire_2pac2002_8h_1a5873fda415a72e239ea243bed79405c7>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double kappa,
	    double Fy_pureSlip
	    )

	double
	:ref:`s2_tire_pac2002_Mz_combined<doxid-tire_2pac2002_8h_1ad36b82bb284940522554a8675cbf9e2f>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha_r,
	    double alpha_t,
	    double gamma,
	    double kappa,
	    double Fx_combined,
	    double Fy_combined
	    )

	double
	:ref:`s2_tire_pac2002_calc_Mx<doxid-tire_2pac2002_8h_1a095bb61ec83dc5a79153f55e4d66f56d>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double Fy_combined
	    )

	double
	:ref:`s2_tire_pac2002_calc_My<doxid-tire_2pac2002_8h_1ae8f86738eda52bbc1d5ea709659c07d7>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double Fx_combined
	    )

	bool
	:ref:`s2_tire_pac2002_discTerrainContact<doxid-tire_2pac2002_8h_1ab7a1630c7e0a24a451236c2cb143dc15>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	s2_tire_pac2002_updateTransient (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

	void
	:ref:`s2_tire_pac2002_updateVerticalLoad<doxid-tire_2pac2002_8h_1aad631fb210d174a3be0f80a058b42197>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void
	:ref:`s2_tire_pac2002_transferForces<doxid-tire_2pac2002_8h_1ac1c6d29395ad7cec3dd2d9127b6fdbd9>` (
	    s2_context_t* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire
	    )

	:ref:`s2_pid_t<doxid-structs2__pid__t>`*
	s2_pid ()

	void
	:ref:`s2_pid_updateFromJointToJoint<doxid-pid_8h_1adfefdc1c61c0e5157a09686e27e4d7ab>` (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

	void
	:ref:`s2_pid_updateFromFrameToJoint<doxid-pid_8h_1ac8c49e0c6bef94a9f38dde8182c6e8a0>` (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

	void
	s2_pid_connectForceModulePointer (
	    :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* forceModule,
	    :ref:`s2_pid_t<doxid-structs2__pid__t>`* pid
	    )

	void
	s2_pid_saveHistory (void* parameters)

	void
	:ref:`s2_context_setForces<doxid-service_8h_1a4649ffbdd2d342c04e7593e7117e324d>` (
	    s2_context_t*,
	    double*
	    )

	void
	:ref:`s2_stiffness_joint<doxid-stiffness_8h_1ad6dc04dd6bda9c49495fa1ef5f47a210>` (
	    s2_context_t* context,
	    s2_joint_t* joint
	    )

	void
	:ref:`s2_context_allocNumericalIntegrator<doxid-numericalintegrator_8h_1a29df5ebe204181e4306e96016f270d40>` (s2_context_t* context)

	void
	:ref:`s2_context_freeNumericalIntegrator<doxid-numericalintegrator_8h_1a1620d0638a9d85bf09042bec4d331474>` (s2_context_t* context)

	void
	s2_ode113 (
	    void  f double t, double y[], double dy[], s2_context_t *context,
	    s2_context_t* context
	    )

	void
	s2_ode113_de (
	    void  f double t, double y[], double dy[], s2_context_t *context,
	    s2_context_t* context,
	    int neqn,
	    double y [],
	    double* t,
	    double tout,
	    double relerr,
	    double abserr,
	    int* iflag,
	    double yy [],
	    double wt [],
	    double p [],
	    double yp [],
	    double ypout [],
	    double phi [],
	    double alpha [],
	    double beta [],
	    double sig [],
	    double v [],
	    double w [],
	    double g [],
	    int* phase1,
	    double psi [],
	    double* x,
	    double* h,
	    double* hold,
	    int* start,
	    double* told,
	    double* delsgn,
	    int* ns,
	    int* nornd,
	    int* k,
	    int* kold,
	    int* isnold
	    )

	int
	:ref:`i4_sign<doxid-ode113_8h_1a535d66c4385dd7e8c98f78cd4c8b3a2f>` (int i)

	void
	:ref:`s2_ode113_intrp<doxid-ode113_8h_1ade9963857d52e84c92408c920578c26d>` (
	    double x,
	    double y [],
	    double xout,
	    double yout [],
	    double ypout [],
	    int neqn,
	    int kold,
	    double phi [],
	    double psi []
	    )

	double
	:ref:`r8_epsilon<doxid-ode113_8h_1ab21d05128ddb24988dfbca84c955c6f8>` (void)

	void
	s2_ode113_step (
	    double* x,
	    double y [],
	    void  f double t, double y[], double dy[], s2_context_t *context,
	    s2_context_t* context,
	    int neqn,
	    double* h,
	    double* eps,
	    double wt [],
	    int* start,
	    double* hold,
	    int* k,
	    int* kold,
	    int* crash,
	    double phi [],
	    double p [],
	    double yp [],
	    double psi [],
	    double alpha [],
	    double beta [],
	    double sig [],
	    double v [],
	    double w [],
	    double g [],
	    int* phase1,
	    int* ns,
	    int* nornd
	    )

	void
	timestamp (void)

	void
	s2_context_allocOde113 (s2_context_t* context)

	void
	:ref:`s2_context_freeOde113<doxid-ode113_8h_1a4441054803552b0a25e8aa1d3b787774>` (s2_context_t* context)

	void
	:ref:`s2_context_integrateEquationsOde113<doxid-ode113_8h_1a67844d34b61458283d1060c72a1d7bcd>` (s2_context_t* context)

	void
	:ref:`s2_context_allocOde113b<doxid-ode113b_8h_1a3ec1c16e4555d46ae1a1a4a46e1f33f6>` (s2_context_t*)

	void
	:ref:`s2_context_freeOde113b<doxid-ode113b_8h_1a01f571e7d3e58a4018d465e143d72277>` (s2_context_t*)

	void
	:ref:`s2_ode113b_fcn<doxid-ode113b_8h_1a55e0e1b1dcf8b0236794e961336740b3>` (
	    double y [],
	    s2_context_t*
	    )

	void
	s2_ode113b_step (
	    void  f double y[], s2_context_t *context,
	    s2_context_t* context
	    )

	void
	:ref:`s2_ode113b_de<doxid-ode113b_8h_1a8cc5cb1907a1b5de62e501a7a9ac0096>` (
	    void  f double y[], s2_context_t *context,
	    s2_context_t* context
	    )

	void
	:ref:`s2_context_integrateEquationsOde113b<doxid-ode113b_8h_1aafd0cf36ccbf352667a9892ee5310eb7>` (s2_context_t* context)

	// macros

	#define :ref:`_SHIFT_H_<doxid-shift_8h_1a7d3083ae70159e2452e54760228608ee>`

.. _details-doxid-global:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Typedefs
--------

.. _doxid-damping_8h_1ad827256522fc82b2ad4001167238a775:
.. _cid-s2_damper_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_damper_t<doxid-structs2__damper__t>` s2_damper_t

damper data structure

piecewise curves (i.e. splines) are not currently supported

.. _doxid-forcemodule_8h_1a6cfa09e2454a092d1d37c32a584f809a:
.. _cid-s2_forcemodule_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>` s2_forcemodule_t

utility to feed back forces into a running simulation

.. _doxid-pid_8h_1ab0e710f017bf8a44db747a1ada3e6bfc:
.. _cid-s2_pid_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_pid_t<doxid-structs2__pid__t>` s2_pid_t

initially set up to apply a force in a joint

.. _doxid-stiffness_8h_1a810c566c41ba7f20abc258759f13d3b0:
.. _cid-s2_spring_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_spring_t<doxid-structs2__spring__t>` s2_spring_t

spring data structure

piecewise curves (i.e. splines) are not currently supported

.. _doxid-ode113_8h_1a204cc0f0ebd6f99f35ec1a8fdf2816b5:
.. _cid-s2_numericalintegrator_ode113_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_numericalintegrator_ode113_t<doxid-structs2__numericalintegrator__ode113__t>` s2_numericalintegrator_ode113_t

Attributes associated with numerical integrator.

.. _doxid-ode113b_8h_1a749caa804014e62fb4ab043f25398d97:
.. _cid-s2_numericalintegrator_ode113b_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_numericalintegrator_ode113b_t<doxid-structs2__numericalintegrator__ode113b__t>` s2_numericalintegrator_ode113b_t

Attributes associated with numerical integrator.

Global Functions
----------------

.. _doxid-contact_8h_1aea526a1aa339c059baf450365e358798:
.. _cid-s2_context_basicsweep:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_basicSweep (s2_context_t*)

sets up a basic "sweep" operation to identify 1/2 n^2 potential collisions. This basic sweep function need only be called once at initial setup and will set up simplices to test each collision object with every other collision object. No pruning is performed here which would reduce unnecessary computation.

.. _doxid-contact_8h_1a09906598ec946d4dcb6bfb9f995a2aba:
.. _cid-s2_context_sweepandprune:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_sweepAndPrune (s2_context_t*)

performs initial sweep and prune pass to identify potentially contacting bodies in scene

.. _doxid-contact_8h_1ad296820a87cce60f4e47af0af0a20fb8:
.. _cid-s2_context_contactdetection:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_contactDetection (s2_context_t*)

perform contact detection for potentially contacting bodies identified in sweep and prune pass

.. _doxid-fiala_8h_1a8658302eeb1bec4475877e8367172394:
.. _cid-s2_tire_fiala:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`*
	s2_tire_fiala (
	    s2_context_t* context,
	    const char* filePath,
	    s2_stackstr_t* wheelBody
	    )

allocate memory for tire with default parameters

.. _doxid-fiala_8h_1a3e1a2e637efc322442633553baba498c:
.. _cid-s2_tire_fiala_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_fiala_free (:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire)

free memory for :ref:`s2_tire_fiala_t <doxid-structs2__tire__fiala__t>`

.. _doxid-fiala_8h_1a322fce69ed2f93ae89452a861d8303ca:
.. _cid-s2_tire_fiala_update:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_fiala_update (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

placeholder function to update transient properties associated with fiala tire

.. _doxid-fiala_8h_1aef0605aba2e6664ad511704f3adc29fc:
.. _cid-s2_tire_fiala_loadtiredatafile:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_fiala_loadTireDataFile (
	    s2_context_t* context,
	    :ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire,
	    const char* filePath
	    )

Load a .tir tire specification file.

.. _doxid-tire_2fiala_8h_1a8658302eeb1bec4475877e8367172394:
.. _cid-s2_tire_fiala-2:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`*
	s2_tire_fiala (
	    s2_context_t* context,
	    const char* filePath,
	    s2_stackstr_t* wheelBody
	    )

allocate memory for tire with default parameters

.. _doxid-tire_2fiala_8h_1a3e1a2e637efc322442633553baba498c:
.. _cid-s2_tire_fiala_free-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_fiala_free (:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire)

free memory for :ref:`s2_tire_fiala_t <doxid-structs2__tire__fiala__t>`

.. _doxid-tire_2fiala_8h_1a322fce69ed2f93ae89452a861d8303ca:
.. _cid-s2_tire_fiala_update-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_fiala_update (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

placeholder function to update transient properties associated with fiala tire

.. _doxid-tire_2fiala_8h_1aef0605aba2e6664ad511704f3adc29fc:
.. _cid-s2_tire_fiala_loadtiredatafile-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_fiala_loadTireDataFile (
	    s2_context_t* context,
	    :ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire,
	    const char* filePath
	    )

Load a .tir tire specification file.

.. _doxid-forcemodule_8h_1a10ba39388d465932d035489090731365:
.. _cid-s2_forcemodule:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`*
	s2_forcemodule (:ref:`s2_forcemodule_enum_t<doxid-forcemodule_8h_1aef216ac0fc3d71c6419b69df97ab6091>`)

initialize a forcemodule data structure

.. _doxid-forcemodule_8h_1a1462d12cabcbd4c2af3cd01be718db08:
.. _cid-s2_context_flattenforcemodules:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_flattenForceModules (s2_context_t*)

flatten force modules into array of array of pointers to force modules with static and transient properties

.. _doxid-forcemodule_8h_1a4a9554e0c66689426792d17240ffe55b:
.. _cid-s2_context_connectforcemodules:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_connectForceModules (s2_context_t*)

connect force module dy array pointer and set yOffset, called after numerical integrator set up

.. _doxid-forcemodule_8h_1a7a29be11cf9afd14c11f80670658753e:
.. _cid-s2_context_saveforcemodulehistory:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_saveForceModuleHistory (s2_context_t* context)

save time histories of force modules

ode113 does not integrate equations in monotonically time increasing order, any values that need to be saved are set in this function and called in a different location when integrating the equations only when convergence is true

.. _doxid-forcemodule_8h_1aa53d7be6608c3b229cbc7c2dd1b5bd56:
.. _cid-s2_forcemodule_append:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_forcemodule_append (
	    s2_assembly_t* assembly,
	    :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* forceModule
	    )

Append a force module to an Assembly datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - an Assembly datastructure to append to

    *
        - forcemodule

        - a force module datastructure to append

.. _doxid-forcemodule_8h_1a5bd61c1828561862ae4357cf37cbc2bb:
.. _cid-s2_forcemodule_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_forcemodule_free (:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`*)

initialize a forcemodule data structure

.. _doxid-forcemodule_8h_1ac28262f7c2ba16e662ff311b36ba2c09:
.. _cid-s2_forcemodule_load:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_forcemodule_load (
	    s2_context_t* context,
	    s2_assembly_t* assembly,
	    s2_stackstr_t* prefix,
	    json_t* forceModuleObject
	    )

load forcemodule properties from json file

.. _doxid-pac2002_8h_1a52c64dbbbd2c064b8032d7dcce7a10d2:
.. _cid-s2_tire_pac2002:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`*
	s2_tire_pac2002 (
	    s2_context_t* context,
	    const char* filePath,
	    s2_stackstr_t* wheelBody,
	    bool useTransientSlip
	    )

allocate memory for tire with default parameters

.. _doxid-pac2002_8h_1a75e3fa6c00463a48be7638e921500143:
.. _cid-s2_tire_pac2002_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_free (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

free memory for :ref:`s2_tire_pac2002_t <doxid-structs2__tire__pac2002__t>`

.. _doxid-pac2002_8h_1a73243b38626d6cb76ca38ee1d00b83f6:
.. _cid-s2_tire_pac2002_update:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_update (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

update slip angle, longtitudinal slip and camber angle, update forces to model

.. _doxid-pac2002_8h_1a3392f865e9daaa82bbf47793f3a5b8ab:
.. _cid-s2_tire_pac2002_loadtiredatafile:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_loadTireDataFile (
	    s2_context_t* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    const char* filePath
	    )

Load a .tir tire specification file.

.. _doxid-pac2002_8h_1a5075ecf586909626c01f2df725bc3af4:
.. _cid-s2_tire_pac2002_updatewframe:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_updateWFrame (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

updates wFrame, wheel normal (in iso convention), terrain normal,

.. _doxid-pac2002_8h_1aeaaea7981cb1d6b432d05f587fd66ec9:
.. _cid-s2_tire_pac2002_calc_fz:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_calc_Fz (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

calculate Fz using spring - damper model

.. _doxid-pac2002_8h_1a215220f8e29bf377a51244a6ca6cced0:
.. _cid-s2_tire_pac2002_updaterelaxationlengths:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_updateRelaxationLengths (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

compute relaxation lengths

.. _doxid-pac2002_8h_1a80e0c5ab777c6793a5e1a74a83dfa5c7:
.. _cid-s2_tire_pac2002_updateslipkinematic:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_updateSlipKinematic (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

Calculate kinematic slip quantities from the current wheel state.

Note: when not in contact, all slips are assumed zero, but velocities still set

.. _doxid-pac2002_8h_1a3bd2800b93b071b82df4ef3986561110:
.. _cid-s2_tire_pac2002_zeroslips:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_zeroSlips (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

set the tire m_slip vector to all zeros

.. _doxid-pac2002_8h_1ac20ea3476c1d1cbf4dc6f6cfcf52e168:
.. _cid-s2_tire_pac2002_evaluatereactions:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_evaluateReactions (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    bool write_violations,
	    bool enforce_threshold
	    )

after calculating reactions, enforce limits when forces/moments exceed specified thresholds (except on Fz)

.. _doxid-pac2002_8h_1ae0239415e4200b821dec893a8bf183b8:
.. _cid-s2_tire_pac2002_slip_from_uv:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_slip_from_uv (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    bool use_besselink,
	    double bessel_Cx,
	    double bessel_Cy,
	    double V_low
	    )

calculate and set the transient slip values (kappaP, alphaP, gammaP) from u, v deflections. optionally turn on/off besselink low velocity damping

.. _doxid-pac2002_8h_1ae9766ff827292da718194567c723b4eb:
.. _cid-s2_tire_pac2002_pureslipreactions:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_pureSlipReactions (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

Calculate tire reactions.

pure slip reactions, if the tire is in contact with the ground, in the TYDEX W-Axis system. calcFx alphaP ~= 0 calcFy and calcMz kappaP ~= 0 NOTE: alphaP, gammaP and kappaP defined with respect to tire side specified from .tir input file. e.g., positive alpha = turn wheel toward vehicle centerline e.g., positive gamma = wheel vertical axis top pointing toward vehicle center

.. _doxid-pac2002_8h_1a2e900f40155187ffbb850e80964d17bd:
.. _cid-s2_tire_pac2002_combinedslipreactions:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_combinedSlipReactions (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

calculate combined slip reactions combined slip reactions, if the tire is in contact with the ground MUST call pureSlipReactions() first, as these functions rely on intermediate values

NOTE: alphaP and gammaP have already been modified for being on the L/R side e.g., positive alpha = turn wheel toward vehicle centerline e.g., positive gamma = wheel vertical axis top pointing toward vehicle center

.. _doxid-pac2002_8h_1a0c87e22d7746b5f7630c920217b6a3aa:
.. _cid-s2_tire_pac2002_fx_purelong:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Fx_pureLong (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double kappa
	    )

longitudinal force, alpha ~= 0 assign to m_FM.force.x assign m_pureLong, trionometric function calculated constants

.. _doxid-pac2002_8h_1aab053dca94043c0115b8c86b755608f3:
.. _cid-s2_tire_pac2002_fy_purelat:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Fy_pureLat (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma
	    )

lateral force, kappa ~= 0 assign to m_FM.force.y assign m_pureLong, trionometric function calculated constants

.. _doxid-pac2002_8h_1aa1f7798f862f1b1af24b410938970577:
.. _cid-s2_tire_pac2002_mz_purelat:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Mz_pureLat (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double Fy_pureSlip
	    )

aligning moment, kappa ~= 0 assign to m_FM.moment.z assign m_pureLong, trigonometric function calculated constants

.. _doxid-pac2002_8h_1a75967c7dc5b18d94b56e03beb9d7cbb4:
.. _cid-s2_tire_pac2002_fx_combined:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Fx_combined (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double kappa,
	    double Fx_pureSlip
	    )

longitudinal force, combined slip (general case) assign m_FM_combined.force.x assign m_combinedLong

.. _doxid-pac2002_8h_1a5873fda415a72e239ea243bed79405c7:
.. _cid-s2_tire_pac2002_fy_combined:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Fy_combined (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double kappa,
	    double Fy_pureSlip
	    )

calculate lateral force, combined slip (general case) assign m_FM_combined.force.y assign m_combinedLat

.. _doxid-pac2002_8h_1ad36b82bb284940522554a8675cbf9e2f:
.. _cid-s2_tire_pac2002_mz_combined:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Mz_combined (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha_r,
	    double alpha_t,
	    double gamma,
	    double kappa,
	    double Fx_combined,
	    double Fy_combined
	    )

calculate aligning torque, combined slip (gernal case) assign m_FM_combined.moment.z assign m_combinedTorque

.. _doxid-pac2002_8h_1a095bb61ec83dc5a79153f55e4d66f56d:
.. _cid-s2_tire_pac2002_calc_mx:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_calc_Mx (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double Fy_combined
	    )

calculate the overturning couple moment assign m_FM.moment.x and m_FM_combined.moment.x

.. _doxid-pac2002_8h_1ae8f86738eda52bbc1d5ea709659c07d7:
.. _cid-s2_tire_pac2002_calc_my:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_calc_My (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double Fx_combined
	    )

calculate the rolling resistance moment, assign m_FM.moment.y and m_FM_combined.moment.y

.. _doxid-pac2002_8h_1ab7a1630c7e0a24a451236c2cb143dc15:
.. _cid-s2_tire_pac2002_discterraincontact:
.. ref-code-block:: cpp
	:class: title-code-block

	bool
	s2_tire_pac2002_discTerrainContact (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

check for contact, set tire->inContact flag and set penetration depth

.. _doxid-pac2002_8h_1aad631fb210d174a3be0f80a058b42197:
.. _cid-s2_tire_pac2002_updateverticalload:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_updateVerticalLoad (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

Calculate the tire vertical load at the current configuration and update the tire deflection and tire rolling radius.

.. _doxid-pac2002_8h_1ac1c6d29395ad7cec3dd2d9127b6fdbd9:
.. _cid-s2_tire_pac2002_transferforces:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_transferForces (
	    s2_context_t* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire
	    )

wrapper function to update for use with force module data structure

.. _doxid-tire_2pac2002_8h_1a52c64dbbbd2c064b8032d7dcce7a10d2:
.. _cid-s2_tire_pac2002-2:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`*
	s2_tire_pac2002 (
	    s2_context_t* context,
	    const char* filePath,
	    s2_stackstr_t* wheelBody,
	    bool useTransientSlip
	    )

allocate memory for tire with default parameters

.. _doxid-tire_2pac2002_8h_1a75e3fa6c00463a48be7638e921500143:
.. _cid-s2_tire_pac2002_free-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_free (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

free memory for :ref:`s2_tire_pac2002_t <doxid-structs2__tire__pac2002__t>`

.. _doxid-tire_2pac2002_8h_1a73243b38626d6cb76ca38ee1d00b83f6:
.. _cid-s2_tire_pac2002_update-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_update (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

update slip angle, longtitudinal slip and camber angle, update forces to model

.. _doxid-tire_2pac2002_8h_1a3392f865e9daaa82bbf47793f3a5b8ab:
.. _cid-s2_tire_pac2002_loadtiredatafile-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_loadTireDataFile (
	    s2_context_t* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    const char* filePath
	    )

Load a .tir tire specification file.

.. _doxid-tire_2pac2002_8h_1a5075ecf586909626c01f2df725bc3af4:
.. _cid-s2_tire_pac2002_updatewframe-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_updateWFrame (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

updates wFrame, wheel normal (in iso convention), terrain normal,

.. _doxid-tire_2pac2002_8h_1aeaaea7981cb1d6b432d05f587fd66ec9:
.. _cid-s2_tire_pac2002_calc_fz-2:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_calc_Fz (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

calculate Fz using spring - damper model

.. _doxid-tire_2pac2002_8h_1a215220f8e29bf377a51244a6ca6cced0:
.. _cid-s2_tire_pac2002_updaterelaxationlengths-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_updateRelaxationLengths (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

compute relaxation lengths

.. _doxid-tire_2pac2002_8h_1a80e0c5ab777c6793a5e1a74a83dfa5c7:
.. _cid-s2_tire_pac2002_updateslipkinematic-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_updateSlipKinematic (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

Calculate kinematic slip quantities from the current wheel state.

Note: when not in contact, all slips are assumed zero, but velocities still set

.. _doxid-tire_2pac2002_8h_1a3bd2800b93b071b82df4ef3986561110:
.. _cid-s2_tire_pac2002_zeroslips-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_zeroSlips (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

set the tire m_slip vector to all zeros

.. _doxid-tire_2pac2002_8h_1ac20ea3476c1d1cbf4dc6f6cfcf52e168:
.. _cid-s2_tire_pac2002_evaluatereactions-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_evaluateReactions (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    bool write_violations,
	    bool enforce_threshold
	    )

after calculating reactions, enforce limits when forces/moments exceed specified thresholds (except on Fz)

.. _doxid-tire_2pac2002_8h_1ae0239415e4200b821dec893a8bf183b8:
.. _cid-s2_tire_pac2002_slip_from_uv-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_slip_from_uv (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    bool use_besselink,
	    double bessel_Cx,
	    double bessel_Cy,
	    double V_low
	    )

calculate and set the transient slip values (kappaP, alphaP, gammaP) from u, v deflections. optionally turn on/off besselink low velocity damping

.. _doxid-tire_2pac2002_8h_1ae9766ff827292da718194567c723b4eb:
.. _cid-s2_tire_pac2002_pureslipreactions-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_pureSlipReactions (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

Calculate tire reactions.

pure slip reactions, if the tire is in contact with the ground, in the TYDEX W-Axis system. calcFx alphaP ~= 0 calcFy and calcMz kappaP ~= 0 NOTE: alphaP, gammaP and kappaP defined with respect to tire side specified from .tir input file. e.g., positive alpha = turn wheel toward vehicle centerline e.g., positive gamma = wheel vertical axis top pointing toward vehicle center

.. _doxid-tire_2pac2002_8h_1a2e900f40155187ffbb850e80964d17bd:
.. _cid-s2_tire_pac2002_combinedslipreactions-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_combinedSlipReactions (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

calculate combined slip reactions combined slip reactions, if the tire is in contact with the ground MUST call pureSlipReactions() first, as these functions rely on intermediate values

NOTE: alphaP and gammaP have already been modified for being on the L/R side e.g., positive alpha = turn wheel toward vehicle centerline e.g., positive gamma = wheel vertical axis top pointing toward vehicle center

.. _doxid-tire_2pac2002_8h_1a0c87e22d7746b5f7630c920217b6a3aa:
.. _cid-s2_tire_pac2002_fx_purelong-2:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Fx_pureLong (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double kappa
	    )

longitudinal force, alpha ~= 0 assign to m_FM.force.x assign m_pureLong, trionometric function calculated constants

.. _doxid-tire_2pac2002_8h_1aab053dca94043c0115b8c86b755608f3:
.. _cid-s2_tire_pac2002_fy_purelat-2:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Fy_pureLat (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma
	    )

lateral force, kappa ~= 0 assign to m_FM.force.y assign m_pureLong, trionometric function calculated constants

.. _doxid-tire_2pac2002_8h_1aa1f7798f862f1b1af24b410938970577:
.. _cid-s2_tire_pac2002_mz_purelat-2:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Mz_pureLat (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double Fy_pureSlip
	    )

aligning moment, kappa ~= 0 assign to m_FM.moment.z assign m_pureLong, trigonometric function calculated constants

.. _doxid-tire_2pac2002_8h_1a75967c7dc5b18d94b56e03beb9d7cbb4:
.. _cid-s2_tire_pac2002_fx_combined-2:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Fx_combined (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double kappa,
	    double Fx_pureSlip
	    )

longitudinal force, combined slip (general case) assign m_FM_combined.force.x assign m_combinedLong

.. _doxid-tire_2pac2002_8h_1a5873fda415a72e239ea243bed79405c7:
.. _cid-s2_tire_pac2002_fy_combined-2:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Fy_combined (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double kappa,
	    double Fy_pureSlip
	    )

calculate lateral force, combined slip (general case) assign m_FM_combined.force.y assign m_combinedLat

.. _doxid-tire_2pac2002_8h_1ad36b82bb284940522554a8675cbf9e2f:
.. _cid-s2_tire_pac2002_mz_combined-2:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_Mz_combined (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha_r,
	    double alpha_t,
	    double gamma,
	    double kappa,
	    double Fx_combined,
	    double Fy_combined
	    )

calculate aligning torque, combined slip (gernal case) assign m_FM_combined.moment.z assign m_combinedTorque

.. _doxid-tire_2pac2002_8h_1a095bb61ec83dc5a79153f55e4d66f56d:
.. _cid-s2_tire_pac2002_calc_mx-2:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_calc_Mx (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double Fy_combined
	    )

calculate the overturning couple moment assign m_FM.moment.x and m_FM_combined.moment.x

.. _doxid-tire_2pac2002_8h_1ae8f86738eda52bbc1d5ea709659c07d7:
.. _cid-s2_tire_pac2002_calc_my-2:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	s2_tire_pac2002_calc_My (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double Fx_combined
	    )

calculate the rolling resistance moment, assign m_FM.moment.y and m_FM_combined.moment.y

.. _doxid-tire_2pac2002_8h_1ab7a1630c7e0a24a451236c2cb143dc15:
.. _cid-s2_tire_pac2002_discterraincontact-2:
.. ref-code-block:: cpp
	:class: title-code-block

	bool
	s2_tire_pac2002_discTerrainContact (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

check for contact, set tire->inContact flag and set penetration depth

.. _doxid-tire_2pac2002_8h_1aad631fb210d174a3be0f80a058b42197:
.. _cid-s2_tire_pac2002_updateverticalload-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_updateVerticalLoad (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

Calculate the tire vertical load at the current configuration and update the tire deflection and tire rolling radius.

.. _doxid-tire_2pac2002_8h_1ac1c6d29395ad7cec3dd2d9127b6fdbd9:
.. _cid-s2_tire_pac2002_transferforces-2:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_tire_pac2002_transferForces (
	    s2_context_t* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire
	    )

wrapper function to update for use with force module data structure

.. _doxid-pid_8h_1adfefdc1c61c0e5157a09686e27e4d7ab:
.. _cid-s2_pid_updatefromjointtojoint:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_pid_updateFromJointToJoint (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

update a force in a joint as a function of velocity in a given frame

.. _doxid-pid_8h_1ac8c49e0c6bef94a9f38dde8182c6e8a0:
.. _cid-s2_pid_updatefromframetojoint:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_pid_updateFromFrameToJoint (
	    s2_context_t* context,
	    void** parameters,
	    double* y
	    )

update a force in a joint as a function of velocity in a given frame

.. _doxid-service_8h_1a4649ffbdd2d342c04e7593e7117e324d:
.. _cid-s2_context_setforces:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_setForces (
	    s2_context_t*,
	    double*
	    )

clear and set externally applied forces to model

.. _doxid-stiffness_8h_1ad6dc04dd6bda9c49495fa1ef5f47a210:
.. _cid-s2_stiffness_joint:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_stiffness_joint (
	    s2_context_t* context,
	    s2_joint_t* joint
	    )

sets stiffness within a joint

.. _doxid-numericalintegrator_8h_1a29df5ebe204181e4306e96016f270d40:
.. _cid-s2_context_allocnumericalintegrator:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_allocNumericalIntegrator (s2_context_t* context)

Initialize and allocate memory for numericalIntegrator datastructure.



.. rubric:: Returns:

a numericalIntegrator datastructure

.. _doxid-numericalintegrator_8h_1a1620d0638a9d85bf09042bec4d331474:
.. _cid-s2_context_freenumericalintegrator:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_freeNumericalIntegrator (s2_context_t* context)

Free memory for numericalintegrator datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - numericalIntegrator

        - a numerical integrator datastructure

.. _doxid-ode113_8h_1a535d66c4385dd7e8c98f78cd4c8b3a2f:
.. _cid-i4_sign:
.. ref-code-block:: cpp
	:class: title-code-block

	int
	i4_sign (int i)

i4 returns the sign of an I4.

Licensing:

This code is distributed under the GNU LGPL license.

Modified:

23 October 2007

Author:

John Burkardt

Parameters:

Input, int I, the integer whose sign is desired.

Output, int I4_SIGN, the sign of I.

.. _doxid-ode113_8h_1ade9963857d52e84c92408c920578c26d:
.. _cid-s2_ode113_intrp:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_ode113_intrp (
	    double x,
	    double y [],
	    double xout,
	    double yout [],
	    double ypout [],
	    int neqn,
	    int kold,
	    double phi [],
	    double psi []
	    )

INTRP approximates the solution at XOUT by polynomial interpolation.

Discussion:

The methods in STEP approximate the solution near X by a polynomial. This routine approximates the solution at XOUT by evaluating the polynomial there. Information defining this polynomial is passed from STEP, so INTRP cannot be used alone.

Licensing:

This code is distributed under the GNU LGPL license.

Modified:

01 February 2012

Author:

Original FORTRAN77 version by Lawrence Shampine, Marilyn Gordon. C version by John Burkardt.

Reference:

Lawrence Shampine, Marilyn Gordon, Computer Solution of Ordinary Differential Equations: The Initial Value Problem, Freeman, 1975, ISBN: 0716704617, LC: QA372.S416.

Parameters:

Input, double X, the point where the solution has been computed.

Input, double Y[NEQN], the computed solution at X.

Input, double XOUT, the point at which the solution is desired.

Output, double YOUT[NEQN], the solution at XOUT.

Output, double YPOUT[NEQN], the derivative of the solution at XOUT.

Input, int NEQN, the number of equations.

Input, int KOLD, the order used for the last successful step.

Input, double PHI[NEQN\*16], contains information about the interpolating polynomial.

Input, double PSI[12], contains information about the interpolating polynomial.

.. _doxid-ode113_8h_1ab21d05128ddb24988dfbca84c955c6f8:
.. _cid-r8_epsilon:
.. ref-code-block:: cpp
	:class: title-code-block

	double
	r8_epsilon (void)

R8_EPSILON returns the R8 round off unit.

Discussion:

R8_EPSILON is a number R which is a power of 2 with the property that, to the precision of the computer's arithmetic, 1 < 1 + R but 1 = ( 1 + R / 2 )

Licensing:

This code is distributed under the GNU LGPL license.

Modified:

01 September 2012

Author:

John Burkardt

Parameters:

Output, double R8_EPSILON, the R8 round-off unit.

.. _doxid-ode113_8h_1a4441054803552b0a25e8aa1d3b787774:
.. _cid-s2_context_freeode113:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_freeOde113 (s2_context_t* context)

free a numerical integrator datastructure for ode113 integration type

.. _doxid-ode113_8h_1a67844d34b61458283d1060c72a1d7bcd:
.. _cid-s2_context_integrateequationsode113:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_integrateEquationsOde113 (s2_context_t* context)

integrate equations using ode113 integration method

.. _doxid-ode113b_8h_1a3ec1c16e4555d46ae1a1a4a46e1f33f6:
.. _cid-s2_context_allocode113b:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_allocOde113b (s2_context_t*)

allocate memory for ode113b numerical integrator

.. _doxid-ode113b_8h_1a01f571e7d3e58a4018d465e143d72277:
.. _cid-s2_context_freeode113b:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_freeOde113b (s2_context_t*)

free memory for ode113b integrator

.. _doxid-ode113b_8h_1a55e0e1b1dcf8b0236794e961336740b3:
.. _cid-s2_ode113b_fcn:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_ode113b_fcn (
	    double y [],
	    s2_context_t*
	    )

solve for current time step

.. _doxid-ode113b_8h_1a8cc5cb1907a1b5de62e501a7a9ac0096:
.. _cid-s2_ode113b_de:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_ode113b_de (
	    void  f double y[], s2_context_t *context,
	    s2_context_t* context
	    )

subroutine de integrates a system of neqn first order ordinary differential equations of the form

dy(i)/dt = f(t,y(1),y(2),...,y(neqn)) y(i) given at t .

the subroutine integrates from t to tout . on return the parameters in the call list are initialized for continuing the integration. the user has only to define a new value tout and call de again.

de calls two codes, the integrator step and the interpolation routine intrp . step uses a modified divided difference form of the adams pece formulas and local extrapolation. it adjusts the order and step size to control the local error. normally each call to step advances the solution one step in the direction of tout . though never beyond t + 10\*(tout-t), and calls intrp to interpolate the solution at tout . an option is provided to stop the integration at tout but it should be used only if it is impossible to continue the integration beyond tout .

this code is completely explained and documented in the text, computer solution of ordinary differential equations: the initial value problem by l. f. shampine and m. k. gordon.

the parameters for de are: f subroutine f(t,y,yp) to evaluate derivatives yp(i)=dy(i)/dt neqn number of equations to be integrated y(\*) solution vector at t t independent variable tout point at which solution is desired relerr,abserr relative and absolute error tolerances for local error test. at each step the code requires abs(local error) .le. abs(y)\*relerr + abserr for each component of the local error and solution vectors iflag indicates status of integration

first call to de

the user must provide storage in his calling program for the array in the call list, y(neqn), declare f in an external statement, supply the subroutine f(t,y,yp) to evaluate dy(i)/dt = yp(i) = f(t,y(1),y(2),...,y(neqn)) and initialize the parameters: neqn number of equations to be integrated y(\*) vector of initial conditions t starting point of integration tout point at which solution is desired relerr,abserr relative and absolute local error tolerances iflag +1,-1. indicator to initialize the code. normal input is +1. the user should set iflag=-1 only if it is impossible to continue the integration beyond tout . all parameters except f , neqn and tout may be altered by the code on output so must be variables in the calling program.

output from de

neqn unchanged y(\*) solution at t t last point reached in integration. normal return has t = tout . tout unchanged relerr,abserr normal return has tolerances unchanged. iflag=3 signals tolerances increased iflag = 2 normal return. integration reached tout = 3 integration did not reach tout because error tolerances too small. relerr , abserr increased appropriately for continuing = 5 integration did not reach tout because equations appear to be stiff = 6 invalid input parameters (fatal error) the value of iflag is returned negative when the input value is negative and the integration does not reach tout , i.e., -3, -4, -5.

subsequent calls to de

subroutine de returns with all information needed to continue the integration. if the integration reached tout , the user need only define a new tout and call again. if the integration did not reach tout and the user wants to continue, he just calls again. the output value of iflag is the appropriate input value for subsequent calls. the only situation in which it should be altered is to stop the integration internally at the new tout , i.e., change output iflag=2 to input iflag=-2 . error tolerances may be changed by the user before continuing. all other parameters must remain unchanged.

.. _doxid-ode113b_8h_1aafd0cf36ccbf352667a9892ee5310eb7:
.. _cid-s2_context_integrateequationsode113b:
.. ref-code-block:: cpp
	:class: title-code-block

	void
	s2_context_integrateEquationsOde113b (s2_context_t* context)

placeholder for integrating using ode113b method

Macros
------

.. _doxid-shift_8h_1a7d3083ae70159e2452e54760228608ee:
.. _cid-_shift_h_:
.. ref-code-block:: cpp
	:class: title-code-block

	#define _SHIFT_H_

Shift: A dynamics engine and visualization tool for automation and robotics Kristopher Wehage, 2016

Kristopher Wehage

