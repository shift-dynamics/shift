.. _global-namespace:

Global Namespace
================

.. index:: pair: namespace; global

.. toctree::
	:hidden:

	enum_s2_axis_enum_t.rst
	enum_s2_color_enum_t.rst
	enum_s2_coordinate_convention_enum_t.rst
	enum_s2_coordinate_enum_t.rst
	enum_s2_distance_enum_t.rst
	enum_s2_forcemodule_enum_t.rst
	enum_s2_interpolation_enum_t.rst
	enum_s2_joint_enum_t.rst
	enum_s2_kdtree_enum_t.rst
	enum_s2_light_enum_t.rst
	enum_s2_move_enum_t.rst
	enum_s2_numericalintegrator_enum_t.rst
	enum_s2_pid_io_enum_t.rst
	enum_s2_pid_target_enum_t.rst
	enum_s2_projection_enum_t.rst
	enum_s2_queue_enum_t.rst
	enum_s2_restraint_enum_t.rst
	enum_s2_shading_enum_t.rst
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
	struct_s2_assembly_t.rst
	struct_s2_body_t.rst
	struct_s2_camera_t.rst
	struct_s2_chordconstraint_t.rst
	struct_s2_context_t.rst
	struct_s2_convexhull_t.rst
	struct_s2_damper_t.rst
	struct_s2_displacement_t.rst
	struct_s2_displacementf_t.rst
	struct_s2_forcemodule_t.rst
	struct_s2_frame_t.rst
	struct_s2_inertia_t.rst
	struct_s2_io_t.rst
	struct_s2_joint_t.rst
	struct_s2_kdnode_t.rst
	struct_s2_kdtree_t.rst
	struct_s2_light_t.rst
	struct_s2_mat3x3_t.rst
	struct_s2_mat3x3f_t.rst
	struct_s2_mat4x4_t.rst
	struct_s2_mat4x4f_t.rst
	struct_s2_material_t.rst
	struct_s2_numericalintegrator_ode113_t.rst
	struct_s2_numericalintegrator_ode113b_t.rst
	struct_s2_pid_t.rst
	struct_s2_pointcloud_t.rst
	struct_s2_quaternion_t.rst
	struct_s2_quaternionf_t.rst
	struct_s2_queue_t.rst
	struct_s2_queuenode_t.rst
	struct_s2_restraint_t.rst
	struct_s2_screw_t.rst
	struct_s2_screwf_t.rst
	struct_s2_sensor_t.rst
	struct_s2_sphere_t.rst
	struct_s2_spring_t.rst
	struct_s2_stack_t.rst
	struct_s2_stacknode_t.rst
	struct_s2_stackstr_t.rst
	struct_s2_stackstrnode_t.rst
	struct_s2_substructure_t.rst
	struct_s2_terrain_t.rst
	struct_s2_tire_fiala_t.rst
	struct_s2_tire_pac2002_t.rst
	struct_s2_vector_t.rst
	struct_s2_vectorf_t.rst
	struct_s2_vectori_t.rst
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



.. _doxid-queue_8h_1a839e6d6820a1326f833f5c2bd49dc3eb:
.. _cid-queue:
.. _doxid-body_8h_1ae27424868025cb6106a06e46445a5878:
.. _cid-s2_body_append:
.. _doxid-camera_8h_1ad48c432e04d6914a6f0e9d54a073ee67:
.. _cid-s2_camera_orthographic:
.. _doxid-camera_8h_1a966888df9a0e4f813be2e19c6a734f30:
.. _cid-s2_camera:
.. _doxid-context_8h_1aa29a0326cbf8fbf51825f0170cf92289:
.. _cid-s2_context:
.. _doxid-context_8h_1a4dbe7f1b7e502a7f7d4fd3de1c4d331d:
.. _cid-s2_units:
.. _doxid-context_8h_1a1ffee554472578ff517a671d0b950729:
.. _cid-s2_context_setboundingspheres:
.. _doxid-factor_8h_1a31b07a97ab291a876c66f62593c8dad8:
.. _cid-s2_factor_infinity_norm:
.. _doxid-factor_8h_1a2872ab4ab03421ae9aedc65c45a1b147:
.. _cid-s2_iswap:
.. _doxid-frame_8h_1a5e202c005848b1bc7afc590c5f854363:
.. _cid-s2_frame_append:
.. _doxid-inertia_8h_1a7531667824c3885b98c433c50d1bee21:
.. _cid-s2_inertia_rotate2:
.. _doxid-joint_8h_1ab1d29dca5dccab84adc00ce44acca8df:
.. _cid-s2_joint_append:
.. _doxid-joint_8h_1ad5ee561e315d9d8d6cc650da178241ca:
.. _cid-s2_joints_print:
.. _doxid-joint_8h_1a4b4848a11292f937970d529cfdf80332:
.. _cid-s2_joint_floatfixed:
.. _doxid-joint_8h_1a0722cb70cafa70836138c188e3d80be0:
.. _cid-s2_joint_floatfixed_dhva:
.. _doxid-kdtree_8h_1a362203aa1c8ef69404dbacd7ddf87cc2:
.. _cid-s2_kdtreef:
.. _doxid-kdtree_8h_1afa78905e8247b2cf58f2ba4d02198aea:
.. _cid-s2_kdtree_nnsearch:
.. _doxid-kdtree_8h_1a3943c7356d330762dff66d52082c4cf3:
.. _cid-s2_kdtree_nnsearch_within_radius:
.. _doxid-kdtree_8h_1abfa23dc0b798432869a6e15a4786f101:
.. _cid-s2_kdtree_intersection_test:
.. _doxid-kinematics_8h_1a2c62a5c074c0f7a1b5b06d6a983bf8b1:
.. _cid-s2_mat3x3f_getscalar:
.. _doxid-kinematics_8h_1a199cb5e928a16f62c943def9d969b50f:
.. _cid-s2_mat4x4f_get:
.. _doxid-memoize_8h_1a320487035354d221e47bd46c78535148:
.. _cid-s2_memoize_initializeequations:
.. _doxid-queue_8h_1ab5c6e28a1eb87fb65a3a956aa607cbd3:
.. _cid-s2_queue_popint:
.. _doxid-restraint_8h_1a3a85e0c87534916ad6a51aee00cb7139:
.. _cid-s2_restraint_append:
.. _doxid-restraint_8h_1ab35b55ed0c6f13d9db65a6692a71c07e:
.. _cid-s2_restraint_memoize:
.. _doxid-sensor_8h_1ab36081601d50c72996ef45ec93a10340:
.. _cid-s2_sensor:
.. _doxid-stack_8h_1af96f2fddc34de83c644fed987969f8e7:
.. _cid-s2_getrootname:
.. _doxid-terrain_8h_1afba9de7294c7ebda1133e62e64f6b2a3:
.. _cid-generatesampleheightmap:
.. _doxid-damping_8h_1a873aba81c508a23704e626d0e0e601fd:
.. _cid-s2_damping_joint:
.. _doxid-forcemodule_8h_1a77a62d333f3bb60a54e6b9b087b70681:
.. _cid-s2_forcemodule_updatetransient:
.. _doxid-pid_8h_1ac3fcf5ae5ad71668057d00f0ae90c88e:
.. _cid-s2_pid:
.. _doxid-pid_8h_1a185d820d6dcffa61f7e366c2c29de821:
.. _cid-s2_pid_connectforcemodulepointer:
.. _doxid-pid_8h_1a85e32c1d56debf7f65c542282af661d2:
.. _cid-s2_pid_savehistory:
.. _doxid-pac2002_8h_1a2b23b28368c29fdd8a6e6722aafbd971:
.. _cid-s2_tire_pac2002_getslipangle:
.. _doxid-pac2002_8h_1ad14ad96c35ae30de74ca5229755b8ed1:
.. _cid-s2_tire_pac2002_evaluateslips:
.. _doxid-pac2002_8h_1a638d583618f1ac7c30f49fd502052b8a:
.. _cid-s2_tire_pac2002_updatetransient:
.. _doxid-ode113_8h_1adcd75d7f65e131700874564af7d23950:
.. _cid-s2_ode113:
.. _doxid-ode113_8h_1ae7315a7bfae54317d282dafb04db8b4a:
.. _cid-s2_ode113_de:
.. _doxid-ode113_8h_1a76596de5ad03eec0ee220c97d1ed3094:
.. _cid-s2_ode113_step:
.. _doxid-ode113_8h_1a02d8f81e512334c1e2e5be025c41afa8:
.. _cid-timestamp:
.. _doxid-ode113_8h_1a329d308dd8f23198c5f0a260e0a950bf:
.. _cid-s2_ode113_allocate:
.. _doxid-ode113b_8h_1a0771d9f38f3152af927c16eb5400a972:
.. _cid-s2_ode113b_step:
.. _doxid-pointcloud_8h_1aa46b657360daa069e8a5823a5c5aa408:
.. _cid-throwwandexception:
.. _doxid-terrain_8h_1aa46b657360daa069e8a5823a5c5aa408:
.. _cid-throwwandexception-2:
.. ref-code-block:: cpp
	:class: overview-code-block

	// typedefs

	typedef struct :ref:`s2_body_t<doxid-structs2__body__t>` :ref:`s2_body_t<doxid-assembly_8h_1a27dda9bdee20cd214357bb4cf28255b7>`
	typedef struct :ref:`s2_joint_t<doxid-structs2__joint__t>` :ref:`s2_joint_t<doxid-assembly_8h_1a4d755a9c89f71a04e40c30284527fe36>`
	typedef struct :ref:`s2_restraint_t<doxid-structs2__restraint__t>` :ref:`s2_restraint_t<doxid-assembly_8h_1adfc16139f51eb76006f0c6224b59b55b>`
	typedef struct :ref:`s2_io_t<doxid-structs2__io__t>` :ref:`s2_io_t<doxid-assembly_8h_1aebd2492768e60b58c2ce2e2c7254e491>`
	typedef struct :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>` :ref:`s2_forcemodule_t<doxid-assembly_8h_1a6cfa09e2454a092d1d37c32a584f809a>`
	typedef struct :ref:`s2_assembly_t<doxid-structs2__assembly__t>` :ref:`s2_assembly_t<doxid-assembly_8h_1aaf71dd06d1111557c4a6b11d4a7c426d>`
	typedef struct :ref:`s2_context_t<doxid-structs2__context__t>` :ref:`s2_context_t<doxid-assembly_8h_1a1987e656463ddf978b84f49911121113>`
	typedef struct :ref:`s2_body_t<doxid-structs2__body__t>` :ref:`s2_body_t<doxid-body_8h_1ad28decee56fd6e153be7e1c8a29bd157>`
	typedef struct :ref:`s2_frame_t<doxid-structs2__frame__t>` :ref:`s2_frame_t<doxid-body_8h_1a170ad83b57b29a38a6287305c19634e8>`
	typedef struct :ref:`s2_material_t<doxid-structs2__material__t>` :ref:`s2_material_t<doxid-body_8h_1a5ecf8ba3501eacd106b940ed548874ba>`
	typedef struct :ref:`s2_chordconstraint_t<doxid-structs2__chordconstraint__t>` :ref:`s2_chordconstraint_t<doxid-chordconstraint_8h_1a16952dee99f6b367eb7cc57a024cd19d>`
	typedef struct :ref:`s2_context_t<doxid-structs2__context__t>` :ref:`s2_context_t<doxid-context_8h_1ae11b519e2e3234898f7dd62a6b5074e9>`
	typedef struct :ref:`s2_sensor_t<doxid-structs2__sensor__t>` :ref:`s2_sensor_t<doxid-context_8h_1a1f81c82915846be30f4b25043b6297b6>`
	typedef struct :ref:`s2_convexhull_t<doxid-structs2__convexhull__t>` :ref:`s2_convexhull_t<doxid-convexhull_8h_1adec77081a8ea85c9180d59bca2c258f4>`
	typedef struct :ref:`s2_frame_t<doxid-structs2__frame__t>` :ref:`s2_frame_t<doxid-frame_8h_1ac997324df7b2ecdf15ac1a9794c4adf7>`
	typedef struct :ref:`s2_inertia_t<doxid-structs2__inertia__t>` :ref:`s2_inertia_t<doxid-inertia_8h_1a213f599bafaaf13e9bcece9aec49de93>`
	typedef struct :ref:`s2_io_t<doxid-structs2__io__t>` :ref:`s2_io_t<doxid-io_8h_1aff30d428569c4eced0cef97477a8a60e>`
	typedef struct :ref:`s2_joint_t<doxid-structs2__joint__t>` :ref:`s2_joint_t<doxid-joint_8h_1a8db073f35bb50443203b729b364ea632>`
	typedef struct :ref:`s2_light_t<doxid-structs2__light__t>` :ref:`s2_light_t<doxid-light_8h_1acbf63648c4231f4b0451d44f6fe6a988>`
	typedef struct :ref:`s2_material_t<doxid-structs2__material__t>` :ref:`s2_material_t<doxid-material_8h_1a7252505fa938247a5db1425abd8bc13d>`
	typedef struct :ref:`s2_queue_t<doxid-structs2__queue__t>` Queue
	typedef struct :ref:`s2_restraint_t<doxid-structs2__restraint__t>` :ref:`s2_restraint_t<doxid-restraint_8h_1a62184c43e8c59b589eef904750faa69c>`
	typedef struct :ref:`s2_sensor_t<doxid-structs2__sensor__t>` :ref:`s2_sensor_t<doxid-sensor_8h_1ab1cba108cff23659d6989ab6daa4174a>`
	typedef struct :ref:`s2_substructure_t<doxid-structs2__substructure__t>` :ref:`s2_substructure_t<doxid-substructure_8h_1af01a3e27562b78e8b11f9bedc384f4b6>`
	typedef struct :ref:`s2_damper_t<doxid-structs2__damper__t>` :ref:`s2_damper_t<doxid-damping_8h_1ad827256522fc82b2ad4001167238a775>`
	typedef struct :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>` :ref:`s2_forcemodule_t<doxid-forcemodule_8h_1a83a5bc188b2f62e1c16dd941c05a2951>`
	typedef struct :ref:`s2_pid_t<doxid-structs2__pid__t>` :ref:`s2_pid_t<doxid-pid_8h_1ab0e710f017bf8a44db747a1ada3e6bfc>`
	typedef struct :ref:`s2_spring_t<doxid-structs2__spring__t>` :ref:`s2_spring_t<doxid-stiffness_8h_1a810c566c41ba7f20abc258759f13d3b0>`
	typedef struct :ref:`s2_numericalintegrator_ode113_t<doxid-structs2__numericalintegrator__ode113__t>` :ref:`s2_numericalintegrator_ode113_t<doxid-ode113_8h_1a204cc0f0ebd6f99f35ec1a8fdf2816b5>`
	typedef struct :ref:`s2_numericalintegrator_ode113b_t<doxid-structs2__numericalintegrator__ode113b__t>` :ref:`s2_numericalintegrator_ode113b_t<doxid-ode113b_8h_1a749caa804014e62fb4ab043f25398d97>`

	// enums

	enum :ref:`s2_axis_enum_t<doxid-kinematics_8h_1aecbf9cf01094216f8f0b99db494de077>`
	enum :ref:`s2_color_enum_t<doxid-material_8h_1a5c398d161c895ac84d2db459b3c95667>`
	enum :ref:`s2_coordinate_convention_enum_t<doxid-terrain_8h_1a03501e9507988449e75a997381910836>`
	enum :ref:`s2_coordinate_enum_t<doxid-restraint_8h_1a57ade64fd6279202ad85f6ea7232dd63>`
	enum :ref:`s2_distance_enum_t<doxid-restraint_8h_1aacd69c927ddfaa300fb4e07c00e14c0f>`
	enum :ref:`s2_forcemodule_enum_t<doxid-forcemodule_8h_1aef216ac0fc3d71c6419b69df97ab6091>`
	enum :ref:`s2_interpolation_enum_t<doxid-terrain_8h_1aecaa7b02bc380403679f716f60ae49c5>`
	enum :ref:`s2_joint_enum_t<doxid-joint_8h_1a7a43c175bb516fb74df6eb8503a0475c>`
	enum :ref:`s2_kdtree_enum_t<doxid-kdtree_8h_1a69eb9d2aa02768ae2426669ebdf682f8>`
	enum :ref:`s2_light_enum_t<doxid-light_8h_1a19bdf894dc0735ff78ec2d5d08821bef>`
	enum :ref:`s2_move_enum_t<doxid-joint_8h_1a17c0fd52873eda0dcebfe02edce180cc>`
	enum :ref:`s2_numericalintegrator_enum_t<doxid-numericalintegrator_8h_1af852b2c473abbd9d2bf3556aab33755f>`
	enum :ref:`s2_pid_io_enum_t<doxid-pid_8h_1a81311461f9fd6d8fa2460aeb85cfcd8f>`
	enum :ref:`s2_pid_target_enum_t<doxid-pid_8h_1aa2bf678870fa220e7f7e78cad0d64a87>`
	enum :ref:`s2_projection_enum_t<doxid-camera_8h_1aca10cf682258e5987b4c4a8e2330d4ac>`
	enum :ref:`s2_queue_enum_t<doxid-queue_8h_1aebbae532ef0fdb5ccccfe7608223b25d>`
	enum :ref:`s2_restraint_enum_t<doxid-restraint_8h_1a76313e0949644f4f5d871ed6e881fd1d>`
	enum :ref:`s2_shading_enum_t<doxid-material_8h_1a6df9b26600246f1da863389cb03e5084>`
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
	struct :ref:`s2_assembly_t<doxid-structs2__assembly__t>`
	struct :ref:`s2_body_t<doxid-structs2__body__t>`
	struct :ref:`s2_camera_t<doxid-structs2__camera__t>`
	struct :ref:`s2_chordconstraint_t<doxid-structs2__chordconstraint__t>`
	struct :ref:`s2_context_t<doxid-structs2__context__t>`
	struct :ref:`s2_convexhull_t<doxid-structs2__convexhull__t>`
	struct :ref:`s2_damper_t<doxid-structs2__damper__t>`
	struct :ref:`s2_displacement_t<doxid-structs2__displacement__t>`
	struct :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`
	struct :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`
	struct :ref:`s2_frame_t<doxid-structs2__frame__t>`
	struct :ref:`s2_inertia_t<doxid-structs2__inertia__t>`
	struct :ref:`s2_io_t<doxid-structs2__io__t>`
	struct :ref:`s2_joint_t<doxid-structs2__joint__t>`
	struct :ref:`s2_kdnode_t<doxid-structs2__kdnode__t>`
	struct :ref:`s2_kdtree_t<doxid-structs2__kdtree__t>`
	struct :ref:`s2_light_t<doxid-structs2__light__t>`
	struct :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`
	struct :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`
	struct :ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`
	struct :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`
	struct :ref:`s2_material_t<doxid-structs2__material__t>`
	struct :ref:`s2_numericalintegrator_ode113_t<doxid-structs2__numericalintegrator__ode113__t>`
	struct :ref:`s2_numericalintegrator_ode113b_t<doxid-structs2__numericalintegrator__ode113b__t>`
	struct :ref:`s2_pid_t<doxid-structs2__pid__t>`
	struct :ref:`s2_pointcloud_t<doxid-structs2__pointcloud__t>`
	struct :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`
	struct :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`
	struct :ref:`s2_queue_t<doxid-structs2__queue__t>`
	struct :ref:`s2_queuenode_t<doxid-structs2__queuenode__t>`
	struct :ref:`s2_restraint_t<doxid-structs2__restraint__t>`
	struct :ref:`s2_screw_t<doxid-structs2__screw__t>`
	struct :ref:`s2_screwf_t<doxid-structs2__screwf__t>`
	struct :ref:`s2_sensor_t<doxid-structs2__sensor__t>`
	struct :ref:`s2_sphere_t<doxid-structs2__sphere__t>`
	struct :ref:`s2_spring_t<doxid-structs2__spring__t>`
	struct :ref:`s2_stack_t<doxid-structs2__stack__t>`
	struct :ref:`s2_stacknode_t<doxid-structs2__stacknode__t>`
	struct :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`
	struct :ref:`s2_stackstrnode_t<doxid-structs2__stackstrnode__t>`
	struct :ref:`s2_substructure_t<doxid-structs2__substructure__t>`
	struct :ref:`s2_terrain_t<doxid-structs2__terrain__t>`
	struct :ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`
	struct :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`
	struct :ref:`s2_vector_t<doxid-structs2__vector__t>`
	struct :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`
	struct :ref:`s2_vectori_t<doxid-structs2__vectori__t>`
	struct :ref:`scaling_coefficients<doxid-structscaling__coefficients>`
	struct :ref:`shape<doxid-structshape>`
	struct :ref:`slip_angle_range<doxid-structslip__angle__range>`
	struct :ref:`slips<doxid-structslips>`
	struct :ref:`thresholds<doxid-structthresholds>`
	struct :ref:`vertical<doxid-structvertical>`
	struct :ref:`vertical_force_range<doxid-structvertical__force__range>`
	struct :ref:`zetaCoefs<doxid-structzeta_coefs>`

	// global functions

	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* :ref:`s2_assembly<doxid-assembly_8h_1a6c2416f0aee7fe6dba810699aac97815>` ()

	void :ref:`s2_assembly_appendSubAssembly<doxid-assembly_8h_1afe1d2adb24d06738dd61cb7b8b8710b4>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* subAssembly
	    )

	void :ref:`s2_assembly_printBodies<doxid-assembly_8h_1a242f74ec5ced3f9ce78db74128079f4f>` (
	    FILE* f,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly
	    )

	void :ref:`s2_assembly_printJoints<doxid-assembly_8h_1ad710ddb7229ab80512b157d007186bc6>` (
	    FILE* f,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly
	    )

	void :ref:`s2_assembly_print<doxid-assembly_8h_1a3bf5b1cbf1a77c47ebb09ae2614201a1>` (
	    FILE* f,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly
	    )

	void :ref:`s2_assembly_appendBody<doxid-assembly_8h_1ac011b887e7ab8c636abff8f8b47351a9>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_body_t<doxid-structs2__body__t>`* body
	    )

	void :ref:`s2_assembly_appendPointCloud<doxid-assembly_8h_1afd54b0d4e8baa8ec3ce7083e6dd4c955>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_pointcloud_t<doxid-structs2__pointcloud__t>`* pointCloud
	    )

	int :ref:`s2_assembly_loadBodies<doxid-assembly_8h_1ad79e5a749aa80db79b357f8f0cc8239a>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    json_t* assemblyObject,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix
	    )

	int :ref:`s2_assembly_loadJoints<doxid-assembly_8h_1a6db4eacbea9586e671b95c0000b0f3d4>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`*,
	    json_t*,
	    :ref:`s2_context_t<doxid-structs2__context__t>`*,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`*
	    )

	int :ref:`s2_assembly_loadLights<doxid-assembly_8h_1a8f014c06474c3333fc560f169afe7444>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`*,
	    json_t*,
	    :ref:`s2_context_t<doxid-structs2__context__t>`*,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`*
	    )

	int :ref:`s2_assembly_loadRestraints<doxid-assembly_8h_1a786317fdaf0db57437ab625186ff4fd3>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`*,
	    json_t*,
	    :ref:`s2_context_t<doxid-structs2__context__t>`*,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`*
	    )

	int :ref:`s2_assembly_loadForceModules<doxid-assembly_8h_1a7ca9d117d740f43feef007f7e7f77635>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    json_t* assemblyObject,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix
	    )

	void :ref:`s2_assembly_free<doxid-assembly_8h_1ab0365a12ee4d6173a86931afabc8e623>` (:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly)

	void :ref:`s2_assembly_appendJoint<doxid-assembly_8h_1a432a0fa719afdd10490a42215e63f050>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_joint_t<doxid-structs2__joint__t>`* joint
	    )

	void :ref:`s2_assembly_appendForceModule<doxid-assembly_8h_1a07d90382ad4fadb47f4e44f3386221f5>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* forceModule
	    )

	void :ref:`s2_assembly_appendRestraint<doxid-assembly_8h_1aa0263f8de9d65f8ade74e22e0d9dcef2>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_restraint_t<doxid-structs2__restraint__t>`* restraint
	    )

	void :ref:`s2_assembly_appendLight<doxid-assembly_8h_1a6dc022a707b6a4d3dc5e6b096fdf59c3>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_light_t<doxid-structs2__light__t>`* light
	    )

	int :ref:`s2_assembly_load<doxid-assembly_8h_1a3b733cb2b82be6039e9597c23679030c>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* parentAssembly,
	    json_t* assembliesObject,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix
	    )

	void :ref:`s2_body_appendFrame<doxid-body_8h_1a1908ef354ca5400654bd3c77f1e2f510>` (
	    :ref:`s2_body_t<doxid-structs2__body__t>`* body,
	    :ref:`s2_frame_t<doxid-structs2__frame__t>`* frame
	    )

	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`s2_body<doxid-body_8h_1a69b43a33ebea29118334a71134bfd08d>` (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name)

	void :ref:`s2_body_printFrames<doxid-body_8h_1ad3e18bb16e07d20fe281003166a6f2de>` (
	    FILE* f,
	    :ref:`s2_body_t<doxid-structs2__body__t>`* body
	    )

	void s2_body_append (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_body_t<doxid-structs2__body__t>`* body
	    )

	void :ref:`s2_body_free<doxid-body_8h_1aae947d6a1b823f553cff5511ce8cc1e4>` (:ref:`s2_body_t<doxid-structs2__body__t>`* body)
	void :ref:`s2_virtualbody_free<doxid-body_8h_1a308059645984b8f926d8884550112c4b>` (:ref:`s2_body_t<doxid-structs2__body__t>`* body)

	int :ref:`s2_body_load<doxid-body_8h_1aa555276cbc6bd3d4be78fc916dbd2ae6>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix,
	    json_t* bodyObject
	    )

	int :ref:`s2_body_loadGeometry<doxid-body_8h_1a9798b3af24177e61f86edbcebcf45b1c>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_body_t<doxid-structs2__body__t>`* body
	    )

	void :ref:`s2_camera_perspective<doxid-camera_8h_1a64efe9dea50e18dc2beff14bb7f19146>` (
	    float fieldOfView,
	    float aspectRatio,
	    float nearClippingPlane,
	    float farClippingPlane,
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* perspective
	    )

	void s2_camera_orthographic (
	    float left,
	    float right,
	    float bottom,
	    float top,
	    float near,
	    float far,
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* orthographic
	    )

	:ref:`s2_camera_t<doxid-structs2__camera__t>`* s2_camera ()
	:ref:`s2_chordconstraint_t<doxid-structs2__chordconstraint__t>`* :ref:`s2_chordconstraint<doxid-chordconstraint_8h_1ad0d4dded0b142bb4c2fd7fff1a013fac>` ()
	:ref:`s2_context_t<doxid-structs2__context__t>`* s2_context ()
	s2_units_t* s2_units ()
	void :ref:`s2_context_free<doxid-context_8h_1a4c52af26a32a4fdf782380f2dd08295c>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_context_freeRunTimeArrays<doxid-context_8h_1a01523c759bd36b2a45c19c5ecde20290>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_context_print<doxid-context_8h_1aeda8ef92fcd0d59bfab3433fbffcf8b1>` (
	    FILE* f,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`s2_context_getFrameByName<doxid-context_8h_1aa53fa7833e02e071bca07d9500361b8f>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* bodyName,
	    const char* frameName
	    )

	:ref:`s2_body_t<doxid-structs2__body__t>`* :ref:`s2_context_getBodyByName<doxid-context_8h_1acd24505aabdde47892e9132c57852ce4>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* bodyName
	    )

	int :ref:`s2_context_getIConv<doxid-context_8h_1a7f76ce795233b12fd3e19c71792d343d>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	double :ref:`s2_context_getSimulationTime<doxid-context_8h_1abda3e5f9e4d0980f0ec28f0a03e293c9>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	FILE* :ref:`s2_context_getDebugPointer<doxid-context_8h_1a4ac077775d53b94844abf4caa49a5183>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int i
	    )

	void :ref:`s2_context_setOmegaN<doxid-context_8h_1a9223bf1c2c8bc9940295e15ab4864ce7>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double omegaN
	    )

	int :ref:`s2_context_loadFixedFrames<doxid-context_8h_1aaa27470dfd5dda04503732ed5a5daff5>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    json_t* root
	    )

	void :ref:`s2_context_loadModelParameters<doxid-context_8h_1a4dde91f880510bd9d291322d5a4c63ff>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    json_t* modelParametersObject,
	    const char* filePath
	    )

	void :ref:`s2_context_freeModelParameters<doxid-context_8h_1a3a5f3b2fd5c74a7a17f53009aabe422e>` (:ref:`s2_context_t<doxid-structs2__context__t>`*)
	:ref:`s2_context_t<doxid-structs2__context__t>`* :ref:`s2_context_load<doxid-context_8h_1a0edbff197c77bdc73ddb6eb607f39666>` (const char* fileName)
	void :ref:`s2_context_setTerrainPointers<doxid-context_8h_1aabf0a90e7b6c0c728fb97ec0f0d54ed4>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void s2_context_setBoundingSpheres (:ref:`s2_context_t<doxid-structs2__context__t>`*)
	void :ref:`s2_context_updateBoundingSpheres<doxid-context_8h_1a1377a99a9de6a8736510a704ca1aa76b>` (:ref:`s2_context_t<doxid-structs2__context__t>`*)
	int :ref:`s2_convexhull_qhull<doxid-convexhull_8h_1af459673607b4420cd938a11b7e530f52>` (:ref:`s2_body_t<doxid-structs2__body__t>`* body)
	void :ref:`s2_equation_initializeEquations<doxid-equation_8h_1ae07157bcd53cf5af2ef36335316cfb1c>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_equation_setEquations<doxid-equation_8h_1a7a41bd12faba6d99ea43161e63433d52>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_equation_factorAndSolve<doxid-equation_8h_1a75684b0d4d92ff58217623e3aab71f87>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_equation_setMassMatrix<doxid-equation_8h_1a4cc978da410a907b2c46d5d1883bd54b>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_equation_setRHS<doxid-equation_8h_1aa166c8d96fcdf1bed248a28e374bd44a>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_equation_dq_to_dy<doxid-equation_8h_1a8e5464e4ddb188f92ddb5d7132ac086f>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double* dy
	    )

	void :ref:`s2_factor_LU<doxid-factor_8h_1a56d3d8ca169b30b9d858540565e49baa>` (
	    int rowDim,
	    int colDim,
	    int rank,
	    double* A
	    )

	int :ref:`s2_factor_LU_full<doxid-factor_8h_1a1c08ca16f2b8caff47da29e37acfbd7e>` (
	    int rowDim,
	    int colDim,
	    double* A,
	    double eps,
	    int* pr,
	    int* pc
	    )

	int :ref:`s2_factor_LU_full_specified<doxid-factor_8h_1a0c54277861282aab80c6b5fd5067256a>` (
	    int rowDim,
	    int colDim,
	    double* A,
	    double eps,
	    int* pr,
	    int* pc,
	    int specifiedCoordinate
	    )

	double s2_factor_infinity_norm (
	    int rowDim,
	    int colDim,
	    int rank,
	    double* A
	    )

	void s2_iswap (
	    int* a,
	    int* b
	    )

	void :ref:`s2_factor_setBMatrix<doxid-factor_8h_1a35e3120d7d178d4002edbbd5296a1f66>` (
	    int rowDim,
	    int colDim,
	    int rank,
	    double* A
	    )

	void :ref:`s2_factor_setCMatrix<doxid-factor_8h_1a34e822650cfcb87252241777792f769e>` (
	    int rowDim,
	    int colDim,
	    int rank,
	    double* A
	    )

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* :ref:`s2_frame<doxid-frame_8h_1ac05a372808decdafe5796ba49a709758>` ()

	void s2_frame_append (
	    :ref:`s2_body_t<doxid-structs2__body__t>`* body,
	    :ref:`s2_frame_t<doxid-structs2__frame__t>`* frame
	    )

	void :ref:`s2_frame_free<doxid-frame_8h_1a213482cc6c0e6fa2d46302da5e173dc3>` (:ref:`s2_frame_t<doxid-structs2__frame__t>`* frame)

	void :ref:`s2_frame_load<doxid-frame_8h_1a6d9a2da53ebca90b5523e54b78e512c0>` (
	    :ref:`s2_body_t<doxid-structs2__body__t>`*,
	    json_t*
	    )

	void :ref:`s2_graph_digraph<doxid-graph_8h_1ab87c483575dfc67a291ea82a7dbae7f0>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* fileName
	    )

	void :ref:`s2_graph_digraphFrames<doxid-graph_8h_1a75be41e869aa7af837ea3b8080ad24ad>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* fileName
	    )

	:ref:`s2_inertia_t<doxid-structs2__inertia__t>`* :ref:`s2_inertia<doxid-inertia_8h_1a4064ffd99364407e01a3ee14c476b84e>` ()
	void :ref:`s2_context_initializeInertia<doxid-inertia_8h_1a553a8b1b69118115e43a66b9ea803a04>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_inertia_rotate<doxid-inertia_8h_1ae13c1d0c1ce6928544980a50d9948820>` (
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia_i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r,
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia_o
	    )

	void s2_inertia_rotate2 (
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia_i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r,
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia_o
	    )

	void :ref:`s2_inertia_transform<doxid-inertia_8h_1a67902d9bad5b566d73b272e519dd0bb4>` (
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia_i,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d,
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia_o
	    )

	void :ref:`s2_inertia_print<doxid-inertia_8h_1ae62ec7e5a343f66ec0653c51dfef6f73>` (
	    FILE* f,
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia
	    )

	void :ref:`s2_inertia_free<doxid-inertia_8h_1a0facc7213f1b16bf84a9bb90770091c9>` (:ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia)
	void :ref:`s2_context_initializeJacobian<doxid-jacobian_8h_1ab87a970d54b86f82c43b1cbd36299539>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_jacobian_setPO2<doxid-jacobian_8h_1a5f56adbb7a9c2345080903ab52b6426d>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_jacobian_setPO2_substructure<doxid-jacobian_8h_1af0d9613c5bc5b4e454b05c2a7ee305a8>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex
	    )

	int :ref:`s2_jacobian_factorPO2<doxid-jacobian_8h_1a9dc8928636426d9c7591bd8a0e76625d>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_jacobian_factorPO2_substructure<doxid-jacobian_8h_1a566e85ff55a63bb70e7cb353b2d64911>` (:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure)
	void :ref:`s2_jacobian_setPO1<doxid-jacobian_8h_1a80877a6ec2cceaa3e67150dc1fcac7c8>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_jacobian_setPO1_substructure<doxid-jacobian_8h_1a52ddd6ba41fc771d6b9ab7cec530b567>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex
	    )

	void :ref:`s2_jacobian_setPO1Specified_substructure<doxid-jacobian_8h_1a4c6ac044c752da74b0d670db8429fe30>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex,
	    int specifiedCoordinateIndex
	    )

	void :ref:`s2_jacobian_evaluateConstraints<doxid-jacobian_8h_1ae019e00518457225ca52702c5b292fc6>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_jacobian_evaluateConstraints_substructure<doxid-jacobian_8h_1ab9876793ffa9e3ff25f6d32d9b2ff8b2>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure
	    )

	void :ref:`s2_jacobian_analyze<doxid-jacobian_8h_1a3ba961f68079a678bbffce3432d1ac59>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	double :ref:`s2_jacobian_pivotRatio<doxid-jacobian_8h_1ae6c32b68f805cc9caff14750d504ea1e>` (
	    int colDim,
	    int rank,
	    double* J
	    )

	void :ref:`s2_jacobian_print<doxid-jacobian_8h_1a856f0a3f1051e729b40885f13fea757b>` (
	    FILE* f,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

	void :ref:`s2_jacobian_setBMatrix_substructure<doxid-jacobian_8h_1a44db4cd91a5f275669f417189207c1c7>` (:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure)
	void :ref:`s2_jacobian_setBMatrix<doxid-jacobian_8h_1ade8a1c811181a2447453d8b951bf7e19>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_jacobian_setBMatrixSparse_substructure<doxid-jacobian_8h_1a33e15659177517d6190488cf881a125d>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure
	    )

	void :ref:`s2_jacobian_setBMatrixSparse<doxid-jacobian_8h_1a563619b6d2834efe63d48d75d95e9c08>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	int :ref:`s2_jacobian_infinityNormBCheck<doxid-jacobian_8h_1a290c638eabbd342efabc656f759026eb>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_jacobian_setJacobianBooleanPO2<doxid-jacobian_8h_1ad9869ddf4f2cad3987cf8ae69c2a71c6>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_jacobian_setJacobianBooleanPO2_substructure<doxid-jacobian_8h_1a53544ee4ae9566aa65f1d6d67490543e>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex
	    )

	double :ref:`s2_jacobian_infinityNormB<doxid-jacobian_8h_1ae984f4f20342d6f76e43b0512d0a7d12>` (
	    int rowDim,
	    int colDim,
	    int rank,
	    double* J
	    )

	:ref:`s2_joint_t<doxid-structs2__joint__t>`* :ref:`s2_joint<doxid-joint_8h_1aa6f268a4ced5d9664eb611bda2e728a1>` (
	    const char* jointTypeStr,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name
	    )

	:ref:`s2_joint_enum_t<doxid-joint_8h_1a7a43c175bb516fb74df6eb8503a0475c>` :ref:`s2_getJointType<doxid-joint_8h_1ad22141c13a6e6084fe0b5def5633f80d>` (const char* jointTypeStr)

	void s2_joint_append (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_joint_t<doxid-structs2__joint__t>`* joint
	    )

	void s2_joints_print (
	    FILE* f,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly
	    )

	void :ref:`s2_joint_tx<doxid-joint_8h_1a313d59dabeaf3a9ad13134e00f026a81>` (void** dhva)
	void :ref:`s2_joint_txinverse<doxid-joint_8h_1ad46544b7618de87c8bedd8067512be97>` (void** dhva)
	void :ref:`s2_joint_ty<doxid-joint_8h_1a3b95f1c8b5bb0238a0c794b357395f7e>` (void** dhva)
	void :ref:`s2_joint_tyinverse<doxid-joint_8h_1a6e447de36e49f99467ecd4bb1a785692>` (void** dhva)
	void :ref:`s2_joint_tz<doxid-joint_8h_1ac49e1ec5e2c0cc15cdb9f0a8d687c64e>` (void** dhva)
	void :ref:`s2_joint_tzinverse<doxid-joint_8h_1a1c80d10ed48a315bc5ac1a9366814143>` (void** dhva)
	void :ref:`s2_joint_tx_dhva<doxid-joint_8h_1a00ee8658804edec0b274aee52659c422>` (void** dhva)
	void :ref:`s2_joint_txinverse_dhva<doxid-joint_8h_1a7e57f436403fd50f59aa52fda2929b94>` (void** dhva)
	void :ref:`s2_joint_ty_dhva<doxid-joint_8h_1a89fc2603a56e0e18940e3574a500979d>` (void** dhva)
	void :ref:`s2_joint_tyinverse_dhva<doxid-joint_8h_1a6aa5867308d5155f411b0177d249a4f8>` (void** dhva)
	void :ref:`s2_joint_tz_dhva<doxid-joint_8h_1aaf4b8443458ae58835d9da18e2320b23>` (void** dhva)
	void :ref:`s2_joint_tzinverse_dhva<doxid-joint_8h_1a15187216dd548906ef8aaf4eef26eb4e>` (void** dhva)
	void :ref:`s2_joint_rx<doxid-joint_8h_1ac0bd6a131a3a2c3231fcfdfd3315148f>` (void** dhva)
	void :ref:`s2_joint_rxinverse<doxid-joint_8h_1af7f605ed6fa506af8b038ef9cc3c5b54>` (void** dhva)
	void :ref:`s2_joint_ry<doxid-joint_8h_1a6f22c68683c564d9a375de830ab9bdf3>` (void** dhva)
	void :ref:`s2_joint_ryinverse<doxid-joint_8h_1a86e3fbb62ba22b44a85dbcda97066961>` (void** dhva)
	void :ref:`s2_joint_rz<doxid-joint_8h_1ae010faf53791ce3aec07c16a8a863ccd>` (void** dhva)
	void :ref:`s2_joint_rzinverse<doxid-joint_8h_1ac883e86580e6ea41abe9da413932e51b>` (void** dhva)
	void :ref:`s2_joint_rx_dhva<doxid-joint_8h_1a6a18edd5be1952467649a33743d32d31>` (void** dhva)
	void :ref:`s2_joint_rxinverse_dhva<doxid-joint_8h_1a5911e71a6b1d1cec7a89a9fa7daba519>` (void** dhva)
	void :ref:`s2_joint_ry_dhva<doxid-joint_8h_1a6858c0763ed98fdf2e9a0b1c55e692a0>` (void** dhva)
	void :ref:`s2_joint_ryinverse_dhva<doxid-joint_8h_1a6b71f3f1d355f959ae7acb6f49258a88>` (void** dhva)
	void :ref:`s2_joint_rz_dhva<doxid-joint_8h_1a2058cf68a422137e54cf55403034dcca>` (void** dhva)
	void :ref:`s2_joint_rzinverse_dhva<doxid-joint_8h_1a8e9f3ecba5a34780a3b0226a0170bfd4>` (void** dhva)
	void :ref:`s2_joint_rigid<doxid-joint_8h_1ac9ab6d4a5b7dd35fdf2d7f49b9d319f6>` (void** dhva)
	void :ref:`s2_joint_rigidinverse<doxid-joint_8h_1a0b4dad9c13fa3502289d4330120c80bc>` (void** dhva)
	void :ref:`s2_joint_rigid_dhva<doxid-joint_8h_1a2308146d104dda09c52c4d44d49a17d6>` (void** dhva)
	void :ref:`s2_joint_rigidinverse_dhva<doxid-joint_8h_1a44450c1ab097e39c84dfd413af512ca2>` (void** dhva)
	void :ref:`s2_joint_spherical<doxid-joint_8h_1a1f080e6ccd539171410ef72cac993d82>` (void** dhva)
	void :ref:`s2_joint_sphericalinverse<doxid-joint_8h_1a3fd780c6194d664a93671c0438ce1681>` (void** dhva)
	void :ref:`s2_joint_spherical_dhva<doxid-joint_8h_1af81dd6149e55911590071c65e1311faf>` (void** dhva)
	void :ref:`s2_joint_sphericalinverse_dhva<doxid-joint_8h_1a937de4d7bd4045dac874baaa2bd6e336>` (void** dhva)
	void :ref:`s2_joint_float<doxid-joint_8h_1a2d976decccc57a1342ddcaa4f8165e1e>` (void** dhva)
	void :ref:`s2_joint_floatinverse<doxid-joint_8h_1ae0b3f59aacc1124ae5a3ecf7523422a3>` (void** dhva)
	void :ref:`s2_joint_float_dhva<doxid-joint_8h_1abfd8e31fc06dc2d24755b58e2c5564c9>` (void** dhva)
	void :ref:`s2_joint_floatinverse_dhva<doxid-joint_8h_1a7ef38976486d6132770c16cf4e0485ae>` (void** dhva)
	void s2_joint_floatfixed (void** dhva)
	void s2_joint_floatfixed_dhva (void** dhva)
	void :ref:`s2_joint_free<doxid-joint_8h_1aa7022ddeb2f5c5c469c6f4c22c515a55>` (:ref:`s2_joint_t<doxid-structs2__joint__t>`* joint)

	int :ref:`s2_joint_load<doxid-joint_8h_1a00650f29fa56ff3d66bc96a03265a113>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix,
	    json_t* bodyObject
	    )

	:ref:`s2_kdtree_t<doxid-structs2__kdtree__t>`* s2_kdtreef (
	    int vertexCount,
	    float* vertices,
	    int faceCount,
	    int* faces,
	    uint32_t binSize
	    )

	:ref:`s2_kdtree_t<doxid-structs2__kdtree__t>`* :ref:`s2_kdtree<doxid-kdtree_8h_1a1151319024fef1ae2cba63f1dc3df4eb>` (
	    int vertexCount,
	    double* vertices,
	    int faceCount,
	    int* faces,
	    uint32_t binSize
	    )

	void :ref:`s2_kdtree_free<doxid-kdtree_8h_1a135cfea0db85264378e7e6651e3b217a>` (:ref:`s2_kdtree_t<doxid-structs2__kdtree__t>`*)

	int s2_kdtree_nnsearch (
	    const :ref:`s2_kdtree_t<doxid-structs2__kdtree__t>`*,
	    const :ref:`s2_vector_t<doxid-structs2__vector__t>`*,
	    double* minimumDistance
	    )

	int s2_kdtree_nnsearch_within_radius (
	    const :ref:`s2_kdtree_t<doxid-structs2__kdtree__t>`*restrict kdtree,
	    const :ref:`s2_vector_t<doxid-structs2__vector__t>`*restrict vertex,
	    double searchRadius,
	    double* minimumDistance,
	    int* nearestPositions,
	    int maxNearestPositions
	    )

	bool s2_kdtree_intersection_test (
	    const :ref:`s2_kdtree_t<doxid-structs2__kdtree__t>`*restrict kdtree,
	    const :ref:`s2_vector_t<doxid-structs2__vector__t>`*restrict point0,
	    const :ref:`s2_vector_t<doxid-structs2__vector__t>`*restrict point1,
	    const double t1,
	    double*restrict intersectionPoint
	    )

	:ref:`s2_vector_t<doxid-structs2__vector__t>`* :ref:`s2_vector<doxid-kinematics_8h_1add6f5c8455a0c42e9d98e5abc6c832de>` ()
	:ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* :ref:`s2_vectorf<doxid-kinematics_8h_1a504c37f6f2d787b2167848a008413f74>` ()
	:ref:`s2_vectori_t<doxid-structs2__vectori__t>`* :ref:`s2_vectori<doxid-kinematics_8h_1ae9ccea35fa4785c758d9dcb907ad9e07>` ()
	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* :ref:`s2_displacement<doxid-kinematics_8h_1a749edbf573f292b057f81bee69e5e9d4>` ()
	:ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* :ref:`s2_displacementf<doxid-kinematics_8h_1a2ebd3f4328d754fd7186be75ce276bfb>` ()
	:ref:`s2_screw_t<doxid-structs2__screw__t>`* :ref:`s2_screw<doxid-kinematics_8h_1af5ad699cdb031af0a3d317b3cd71ce39>` ()
	:ref:`s2_screwf_t<doxid-structs2__screwf__t>`* :ref:`s2_screwf<doxid-kinematics_8h_1a1452fca3cfc2fb7dbb4991450abee7a7>` ()
	:ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* :ref:`s2_mat3x3<doxid-kinematics_8h_1a36a6a8db2bc1bc46e6261071ff1a6b87>` ()
	:ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* :ref:`s2_mat3x3f<doxid-kinematics_8h_1a636c2728626c32de4e09fc16e8baac24>` ()
	:ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`* :ref:`s2_mat4x4<doxid-kinematics_8h_1af2ed9d8c80c58af7e07cdb4bc3d7a3ea>` ()
	:ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* :ref:`s2_mat4x4f<doxid-kinematics_8h_1a8fe199e9f411819659f2ea5139dea917>` ()

	void :ref:`s2_displacement_toMat4x4<doxid-kinematics_8h_1a62bda6da6588457acf90477d273077c9>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d,
	    :ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`* m
	    )

	void :ref:`s2_displacementf_toMat4x4f<doxid-kinematics_8h_1ad0ae207d0ad99dc99834ac62d33466ca>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* d,
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* m
	    )

	void :ref:`s2_displacement_toMat4x4f<doxid-kinematics_8h_1abae78348ad957e61cf237502c9576900>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d,
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* m
	    )

	void :ref:`s2_vector_print<doxid-kinematics_8h_1a77d1698e6e0b0f2559f3dbac07d09d82>` (
	    FILE* f,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v
	    )

	void :ref:`s2_screw_print<doxid-kinematics_8h_1ab118fa4550fdec3303e3652d41cd57ea>` (
	    FILE* f,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s
	    )

	void :ref:`s2_mat3x3_print<doxid-kinematics_8h_1a1f8bb044d159e83916496910de5b5037>` (
	    FILE* f,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r
	    )

	void :ref:`s2_mat3x3f_print<doxid-kinematics_8h_1a4633d76ec38ba232366bd64336faaa97>` (
	    FILE* f,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r
	    )

	void :ref:`s2_displacement_print<doxid-kinematics_8h_1a417cd55d2af30994e91a339c659aafc6>` (
	    FILE* f,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d
	    )

	void :ref:`s2_mat4x4_print<doxid-kinematics_8h_1ace214610c0710efe4c696826a06d27db>` (
	    FILE* f,
	    :ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`* d
	    )

	void :ref:`s2_mat4x4f_print<doxid-kinematics_8h_1ae57720ec83109c7e72d927d701ae997c>` (
	    FILE* f,
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* d
	    )

	void :ref:`s2_array_print<doxid-kinematics_8h_1a8d23082a7f8cfe085867ac96ba845f82>` (
	    FILE* f,
	    int n,
	    double* a
	    )

	void :ref:`s2_array_print2D<doxid-kinematics_8h_1a4cd08b1aa3089dfc3c21c8b6a6a63b2f>` (
	    FILE* f,
	    int rowDim,
	    int colDim,
	    bool isRowMajor,
	    double* a
	    )

	void :ref:`s2_arrayi_print<doxid-kinematics_8h_1a4c02a7a5bdd4d6c55937ff7917f14ee3>` (
	    FILE* f,
	    int n,
	    int* a
	    )

	void :ref:`s2_triang_print<doxid-kinematics_8h_1adc90ce89806874fa4e1c5ea3cd9d92ce>` (
	    FILE* f,
	    int n,
	    double* a
	    )

	void :ref:`s2_triangi_print<doxid-kinematics_8h_1a633f455ee53f3b0f9b8b3c50d96640f7>` (
	    FILE* f,
	    int n,
	    int* a
	    )

	void :ref:`s2_triangb_print<doxid-kinematics_8h_1ac0a79222986f7db5afc96034d56e2855>` (
	    FILE* f,
	    int n,
	    bool* a
	    )

	void :ref:`s2_dc<doxid-kinematics_8h_1ab8853d41657f078add6796d31d7d6b11>` (
	    double* thetai,
	    double dc [2]
	    )

	void :ref:`s2_displacement_setIdentity<doxid-kinematics_8h_1a92b73b20758ea492170c1b793d97b2bf>` (:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d)
	void :ref:`s2_mat3x3_setIdentity<doxid-kinematics_8h_1acad5b10f882249ebf4ead0ae04d738ed>` (:ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r)

	void :ref:`s2_vector_set<doxid-kinematics_8h_1a7c2af5b65856c46e4717d2d915412d80>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v,
	    int index,
	    double value
	    )

	void :ref:`s2_screw_set<doxid-kinematics_8h_1a4dc6664a5de6655f8a692fe9c581285e>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s,
	    int index,
	    double value
	    )

	double :ref:`s2_vector_l2norm<doxid-kinematics_8h_1ae832f876fe2c48c794069aefd22ecf47>` (:ref:`s2_vector_t<doxid-structs2__vector__t>`* v)
	float :ref:`s2_vectorf_l2norm<doxid-kinematics_8h_1a5944ee7b7d802a4c0496c4db30a57157>` (:ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v)
	double :ref:`s2_screw_l2norm<doxid-kinematics_8h_1a1b9b5c10cf53749a5ba1ae5cebdb0517>` (:ref:`s2_screw_t<doxid-structs2__screw__t>`* v)
	float :ref:`s2_screwf_l2norm<doxid-kinematics_8h_1a470337fffa0e9d8eb4d34afee1246a47>` (:ref:`s2_screwf_t<doxid-structs2__screwf__t>`* v)

	void :ref:`s2_mat3x3_set<doxid-kinematics_8h_1a19cc24b3d8e509bc702bad718e9f1b76>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* R,
	    int row,
	    int column,
	    double value
	    )

	void :ref:`s2_mat3x3_setFromVectors<doxid-kinematics_8h_1a0ad504b914b0e429f0e70d5bed9b4e7f>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* R,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* x,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* z
	    )

	void :ref:`s2_mat4x4_set<doxid-kinematics_8h_1a71679fcf2d23747e870e5b94cf17c425>` (
	    :ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`* R,
	    int row,
	    int column,
	    double value
	    )

	void :ref:`s2_displacement_set<doxid-kinematics_8h_1abd7dff4462439ffd7ee662519b9fa1a5>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D,
	    int row,
	    int column,
	    double value
	    )

	void :ref:`s2_vectorf_set<doxid-kinematics_8h_1a0efd3f2ad9d4d6653bed41c51f25b3a7>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v,
	    int index,
	    float value
	    )

	void :ref:`s2_screwf_set<doxid-kinematics_8h_1ab98b7c7e37e980079292ce25267cbb20>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s,
	    int index,
	    float value
	    )

	void :ref:`s2_mat3x3f_set<doxid-kinematics_8h_1a3fa19b544e3e7ca42d2e3f32b56a9b72>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* R,
	    int row,
	    int column,
	    float value
	    )

	void :ref:`s2_mat4x4f_set<doxid-kinematics_8h_1a0bc58ec94030763ef2881c09b021abf0>` (
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* d,
	    int row,
	    int column,
	    float value
	    )

	void :ref:`s2_mat4x4f_setIdentity<doxid-kinematics_8h_1a90be8306e6c85fd683efd37f3ed61815>` (:ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* R)

	void :ref:`s2_displacementf_set<doxid-kinematics_8h_1aa7c2d7da2785bc887397f7fcd59fd282>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D,
	    int row,
	    int column,
	    float value
	    )

	double :ref:`s2_vector_get<doxid-kinematics_8h_1a5b6a920da91584e7140e607c25d730df>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v,
	    int index
	    )

	double :ref:`s2_screw_get<doxid-kinematics_8h_1a427153d990fba6272ec0f6eb9007cc74>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s,
	    int index
	    )

	double :ref:`s2_mat3x3_get<doxid-kinematics_8h_1a1d3c323bcf02daf7e42c4ea54003eb27>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* R,
	    int row,
	    int column
	    )

	double :ref:`s2_displacement_get<doxid-kinematics_8h_1ab88e75900469688ce2494cc17ddf3603>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D,
	    int row,
	    int column
	    )

	float :ref:`s2_vectorf_get<doxid-kinematics_8h_1a1d746b7f2d16a03ddc4de3268b1f44ff>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v,
	    int index
	    )

	float :ref:`s2_screwf_get<doxid-kinematics_8h_1ab1d04de407dbe52ca4f1ed2cf1cb0b55>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s,
	    int index
	    )

	float :ref:`s2_mat3x3f_get<doxid-kinematics_8h_1a0d56ee07fb89cd9cadbfbef9b5b0a6cb>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* R,
	    int row,
	    int column
	    )

	float :ref:`s2_displacementf_get<doxid-kinematics_8h_1a3e1e6b63dc0b10db2b05aebd0d2f0993>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D,
	    int row,
	    int column
	    )

	void :ref:`s2_displacement_getTranslation<doxid-kinematics_8h_1a88255744f79759af23ef0f2ab6e93ac9>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t
	    )

	void :ref:`s2_vector_normalize<doxid-kinematics_8h_1a17e06d126f8e9aa2072e881716a29dfd>` (:ref:`s2_vector_t<doxid-structs2__vector__t>`* v)
	void :ref:`s2_vectorf_normalize<doxid-kinematics_8h_1a05270e2288eac8f5068a6540f86cd4be>` (:ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v)

	void :ref:`s2_vector_scale<doxid-kinematics_8h_1a3e4971a0f6c89474375f17f097b20e56>` (
	    double k,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v
	    )

	void :ref:`s2_vectorf_scale<doxid-kinematics_8h_1a235ee6a927e8c79675134a67333dc8a4>` (
	    double k,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v
	    )

	void :ref:`s2_vector_copy<doxid-kinematics_8h_1ab1ed5e4f30e6c0c4fcec2b80f1adab73>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_vectorf_copy<doxid-kinematics_8h_1afd5179d14d52b675f7d8bb74c2ab816e>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

	void :ref:`s2_vector_invert<doxid-kinematics_8h_1a41990673712e599b887d6622d81dfeda>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_vectorf_invert<doxid-kinematics_8h_1a2ca1a64beb56e639f7e93e1c8452e598>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

	double :ref:`s2_dot_vv<doxid-kinematics_8h_1a2fe4d8a43696adfa539471aa1f6b9d84>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi0,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi1
	    )

	double :ref:`s2_dot_vvf<doxid-kinematics_8h_1a0faf9d7a7b509dc1627b3e7266377d2e>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi0,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi1
	    )

	double :ref:`s2_dot_ss<doxid-kinematics_8h_1a13c2371ecd04f5088a010169de8d8980>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* vi0,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* vi1
	    )

	double :ref:`s2_dot_ssf<doxid-kinematics_8h_1a30572a558bb7ca3b9ee9d81a2ff1dc27>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* vi0,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* vi1
	    )

	void :ref:`s2_product_vk<doxid-kinematics_8h_1ab3ccf15317ea6f6bacb614406b8bcc01>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    double* ki,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_product_vinversek<doxid-kinematics_8h_1aa1f876ac482c3889df9d271bf5e6794f>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    double* ki,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_product_vkinverse<doxid-kinematics_8h_1aa2c1b479099e4cea4fbdb55949eca2cd>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    double* ki,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_product_vkf<doxid-kinematics_8h_1a3fbb7b15e1fe4c9be29a643e82fb8332>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    float* ki,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

	void :ref:`s2_add_vv<doxid-kinematics_8h_1aba3cdfe09849b39f5360917b9facbe32>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_add_vvf<doxid-kinematics_8h_1adb7ffdbb3ff0be7f553d6e98610594cc>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

	void :ref:`s2_addequal_vv<doxid-kinematics_8h_1a955d368c1dd1653c1d98c0849866d2b8>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i
	    )

	void :ref:`s2_addequal_vvf<doxid-kinematics_8h_1a3f26172696564595b05f90db8391184a>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i
	    )

	void :ref:`s2_subtract_vv<doxid-kinematics_8h_1a622e5cd10e1bf0823800269a52ca5e28>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_subtract_vvf<doxid-kinematics_8h_1afd6809a4a30e3e85fc3be0e03d914aab>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

	void :ref:`s2_subtractequal_vv<doxid-kinematics_8h_1a393fcae617b6ca7eed0eef5ec7dae0e3>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i
	    )

	void :ref:`s2_subtractequal_vvf<doxid-kinematics_8h_1a743a6e60733bfdcade977e19722ea0a4>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i
	    )

	void :ref:`s2_axpy_vv<doxid-kinematics_8h_1a8043d4d0f77534a9b641cc0c9bbc86b0>` (
	    double scalar,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1x,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2y
	    )

	void :ref:`s2_axpy_vvf<doxid-kinematics_8h_1abdeb011fa1685891338636061af8f01e>` (
	    float scalar,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1x,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2y
	    )

	void :ref:`s2_axpy_ss<doxid-kinematics_8h_1a39a54ebe31a49549ae65de7e2df3198b>` (
	    double scalar,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* v1x,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* v2y
	    )

	void :ref:`s2_axpy_ssf<doxid-kinematics_8h_1a3dd686974e8a96866c0a47068b782cb0>` (
	    float scalar,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* v1x,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* v2y
	    )

	void :ref:`s2_cross_vv<doxid-kinematics_8h_1afc672ecd8621e23892a82ff7114dbdfc>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_cross_vvinverse<doxid-kinematics_8h_1a00658b700618d65c0de30da39393cc3f>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_cross_vvf<doxid-kinematics_8h_1a5cc89fe340d475bd643a8c5fc5606d0d>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

	void :ref:`s2_cross_vvinversef<doxid-kinematics_8h_1a55f71dc802f0e1d56e4463015bda9ebf>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

	void :ref:`s2_screw_copy<doxid-kinematics_8h_1ae61b2fd7308f015d5ced655e8038d512>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_screwf_copy<doxid-kinematics_8h_1a877e5f8f1a72568028298dca7293addf>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_screw_invert<doxid-kinematics_8h_1a61e36691d826043c69f51730e699eb9a>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_screwf_invert<doxid-kinematics_8h_1a2e1c630b4a773cfa4c16516182219ed9>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_product_sk<doxid-kinematics_8h_1a4fad361eef66c003cf597ea93b60aec3>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    double* ki,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_product_skf<doxid-kinematics_8h_1a6aa748b1ecd5c6420cff106cca8ea0fb>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    float* ki,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_add_ss<doxid-kinematics_8h_1a4ca4d08f9279e9376e5b33cc27bb3ec4>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_add_ssf<doxid-kinematics_8h_1a1d743152fa7392a74a4b1d83648164bb>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_addequal_ss<doxid-kinematics_8h_1ace4d722685aa59f118c7454ed205fd79>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i
	    )

	void :ref:`s2_addequal_ssf<doxid-kinematics_8h_1a9915c1d4a1225b0e83e5802f663344aa>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i
	    )

	void :ref:`s2_subtract_ss<doxid-kinematics_8h_1a2c5df2ff8239acfdcba13979861e6334>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_subtract_ssf<doxid-kinematics_8h_1a99b0ef809bc8aa44a34e96e1467661ee>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_subtractequal_ss<doxid-kinematics_8h_1a54589f97ab7036b4e73bb32244b4b62e>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i
	    )

	void :ref:`s2_subtractequal_ssf<doxid-kinematics_8h_1a0a6934e7684810978871a12a04012e06>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i
	    )

	void :ref:`s2_add_sst<doxid-kinematics_8h_1a94508d66cd7ac94b9a093b0761b0df6f>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si1,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si2,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_add_sstf<doxid-kinematics_8h_1a9468968388a229041ef56da31f720717>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si1,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si2,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_mat3x3_copy<doxid-kinematics_8h_1a5118f11eb04146c2e45b6a42df95037c>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_mat3x3f_copy<doxid-kinematics_8h_1a770547b16bc893e5aed472d4cd5d5365>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_mat3x3_invert<doxid-kinematics_8h_1a014bb516ce7aeae34da04611a471a1f8>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_mat3x3f_invert<doxid-kinematics_8h_1af5e4b6a44b052ae5c0907424e7d1533a>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_product_rv<doxid-kinematics_8h_1a0fbdf7ad62df14225db715b8054309ac>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_product_rvf<doxid-kinematics_8h_1ab081a7cd63e415447bc5c4166b77dfb1>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

	void :ref:`s2_product_rvinverse<doxid-kinematics_8h_1affce2e1277c70ec5a554130df57cc986>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_product_rvinversef<doxid-kinematics_8h_1a30fd8631549b1c147b460d30bd84e662>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

	void :ref:`s2_product_rinversev<doxid-kinematics_8h_1a160a3132f5237c02c41e93def0c6b93a>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_product_rinversevf<doxid-kinematics_8h_1a4806b16cdf637f6519a702e2d76bb5e6>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

	void :ref:`s2_product_rinversevinverse<doxid-kinematics_8h_1a2e03ed8a43d1abfe8f1b4ce18969f8bc>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_product_rinversevinversef<doxid-kinematics_8h_1af9ee6a78e7332cae56a31fd01963533f>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

	void :ref:`s2_product_rs<doxid-kinematics_8h_1a639f37b45a33d87a661aaeeff0a3e1bb>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_product_rsinverse<doxid-kinematics_8h_1a5ea68301a9575b9a3816cdbcec45e0a6>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_product_rsf<doxid-kinematics_8h_1aa754be86054a660dd684886f04101c9c>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_product_rsinversef<doxid-kinematics_8h_1a9af2e48e750e30dbc823ac7b6abf6b78>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_product_rinverses<doxid-kinematics_8h_1a8b21be5ec0e0242e59c127c0969071fa>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_product_rinversesf<doxid-kinematics_8h_1abdd29a7b2ddc6ba996a17f19af749c53>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_product_rrx<doxid-kinematics_8h_1a76e60ee2e631865bfb7e0f20cc3d478f>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_product_rrxf<doxid-kinematics_8h_1ada4b6634d9c811018b355396963cd246>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_product_rrxinverse<doxid-kinematics_8h_1aa336042cd5ff04aec58fe99f01304351>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_product_rrxinversef<doxid-kinematics_8h_1a0086fe943e51ce213b8e10f46d0ae487>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_product_rry<doxid-kinematics_8h_1a5e0535295f6634c276cee80bb11b6393>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_product_rryf<doxid-kinematics_8h_1a860142e5d56955952bbdf02362b8619e>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_product_rryinverse<doxid-kinematics_8h_1aa2becabdce79ba6a9011033e0c6d960f>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_product_rryinversef<doxid-kinematics_8h_1abca9efb8d54d6077f270bf1591eee21f>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_product_rrz<doxid-kinematics_8h_1aada642fd159a5766c0054caf04198f77>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_product_rrzf<doxid-kinematics_8h_1a102d439aa7987fa288276da9181254db>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_product_rrzinverse<doxid-kinematics_8h_1ac8f74722384fe560c56576469d79592c>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_product_rrzinversef<doxid-kinematics_8h_1a089268725603b4bc5f3eaadda27285f8>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_product_rr<doxid-kinematics_8h_1a85f0ea8e6f148d4ae30f683e9808107a>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r1i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r2i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_product_rrf<doxid-kinematics_8h_1aacbf9acbaea875f49126d88d78b03c24>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r1i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r2i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_product_rrinverse<doxid-kinematics_8h_1aa90e088789da6676c214aee4fe656aa4>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r1i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r2i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_product_rrinversef<doxid-kinematics_8h_1a86d9d081de2374a56076c75925c002e0>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r1i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r2i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_product_rinverser<doxid-kinematics_8h_1a2d734e3ca01496db402fd53167e83a9f>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r1i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r2i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_product_rinverserf<doxid-kinematics_8h_1aeafeed107e283a5b9d3b6051dfa475f3>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r1i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r2i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_product_rtx<doxid-kinematics_8h_1a40c0e09a13372e349f698eb50851b01c>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_product_rtxf<doxid-kinematics_8h_1a787b0b6cff4ce7fb4f0f7a1030505ab6>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_product_rtxinverse<doxid-kinematics_8h_1aab9dce66f436a436aa221e996a60fb69>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_product_rtxinversef<doxid-kinematics_8h_1acd755aca4d26696856a26c30dcb0558d>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_product_rty<doxid-kinematics_8h_1a659b2070a7ad93b7534711b121a8d1a3>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_product_rtyf<doxid-kinematics_8h_1a2350fda6e0eb204d8bdb73b0054036c2>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_product_rtyinverse<doxid-kinematics_8h_1a9b6b83714d7e0c0463c54793fd1a53f3>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_product_rtyinversef<doxid-kinematics_8h_1a02132474bde0fccd7dfa4d7158d87fc0>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_product_rtz<doxid-kinematics_8h_1abaff096f63f7231f9fd6552d4b1e9ad3>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_product_rtzf<doxid-kinematics_8h_1a34e5bb55ed27b6a7a81c85cb3d02c6bd>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_product_rtzinverse<doxid-kinematics_8h_1aa7f502f75aada71ca9d68abbfbcc86b0>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_product_rtzinversef<doxid-kinematics_8h_1a35c4da947bc3ac18aad0e8b0def4c701>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_add_ttx<doxid-kinematics_8h_1aed7340fadb9070e72596a13295c8342d>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_add_ttxf<doxid-kinematics_8h_1ac7490edc0a18b0effc71f358639927b2>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_add_ttxinverse<doxid-kinematics_8h_1a75273eebacbf45018fe5dec91787c10a>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_add_ttxinversef<doxid-kinematics_8h_1a7ddd6ef74933f09dfbb9c343997faf85>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_add_tty<doxid-kinematics_8h_1a3b244d5035a2fe7b41fa71bb506fba22>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_add_ttyf<doxid-kinematics_8h_1a182dfd98f5e3bde779b6e25ea41ca8e5>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_add_ttyinverse<doxid-kinematics_8h_1a36e3bcc1acaf20e8c46fb80a4e01e2e8>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_add_ttyinversef<doxid-kinematics_8h_1ad57b7f09daac9a275d6f31fc2e5bbb2e>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_add_ttz<doxid-kinematics_8h_1a30934cc41b06e8e00ec7a0ec6f6b73b7>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_add_ttzf<doxid-kinematics_8h_1a6fb93284f99e1edc4708cec0cc33e1c9>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_add_ttzinverse<doxid-kinematics_8h_1a5257a8ce5b3e541ef4748e3e0b9b512b>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_add_ttzinversef<doxid-kinematics_8h_1a722198145d80fc9a9d665a7fd05dfe25>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void :ref:`s2_displacement_invert<doxid-kinematics_8h_1abc18154ef1cfe80a9ccbe97ad898d3a4>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_displacementf_invert<doxid-kinematics_8h_1ae1d1cd6efd1d7123168603e4e2606936>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_displacement_copy<doxid-kinematics_8h_1af83c896dbc0ab49e6bd13a103b321aec>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_displacementf_copy<doxid-kinematics_8h_1aaad553f944a00d183d4e2e1a189307fe>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_drx<doxid-kinematics_8h_1a92dac683b8af3865c13b4032272bbbd3>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_drxf<doxid-kinematics_8h_1a9deffee2919fbf40180bccdb813c70a7>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_drxinverse<doxid-kinematics_8h_1a7050dc80a4ed7af1f0fb2a1c1fe54dbb>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_drxinversef<doxid-kinematics_8h_1a83ca602a50fba709304a5255be78913f>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_dry<doxid-kinematics_8h_1ad013eda39e2d8f26663c5c6665e13a12>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_dryf<doxid-kinematics_8h_1a52f69f79ad1f07e1b3a49fab081ef6d3>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_dryinverse<doxid-kinematics_8h_1ab13ac64def852a123b0f679b1c0aea61>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_dryinversef<doxid-kinematics_8h_1aa34eacdcfb3e213a3ca8c4c5a410b7cf>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_drz<doxid-kinematics_8h_1a05371d713133d7963d3bd68bee840370>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_drzf<doxid-kinematics_8h_1aebe91869310ca945d54d008b7144f2f8>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_drzinverse<doxid-kinematics_8h_1aa15f1cfd5260c0078757b3e547626e4b>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_drzinversef<doxid-kinematics_8h_1a4c048adf73cc44db1eba74984808d738>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_dt<doxid-kinematics_8h_1ada3a3f19abd83989a1a799a03b591856>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_dtinverse<doxid-kinematics_8h_1a8e06360d1d46eb7f157abbdf4461c3c8>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_dv<doxid-kinematics_8h_1ad6bdbea17b8900b6cdc3efa1b55d8dfc>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_product_dinversev<doxid-kinematics_8h_1ae5f43e0d3cb88762594473fff8daeb9c>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

	void :ref:`s2_product_dtf<doxid-kinematics_8h_1a6a77ce62726105d8a879db5724b5b875>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_dinverset<doxid-kinematics_8h_1a17263fa52606e787fa5d8bba9c89f40a>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_dinversetf<doxid-kinematics_8h_1a4b80e719835e914359d8a38cb366e16e>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_dtx<doxid-kinematics_8h_1aeee4ecce38faba9bc466af2c1bd3edcf>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_dtxinverse<doxid-kinematics_8h_1a970ad48f1c6a9707e83e666aed99a6de>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_dtxinversef<doxid-kinematics_8h_1a0d8d4ba1c266bacfa2f12540cf92f6a2>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float* ti,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_dty<doxid-kinematics_8h_1ae122c8001e90777ea7fe609068821395>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_dtyf<doxid-kinematics_8h_1a2638411003d29bb69dac0b965ce93a04>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float* ti,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_dtyinverse<doxid-kinematics_8h_1acceeede744d34de6ea2776788ddb0bd5>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_dtyinversef<doxid-kinematics_8h_1a60a46355025f1f2833f25ce3e795652d>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float* ti,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_dtz<doxid-kinematics_8h_1a1aa3681f8b87680175ba0df890b3affb>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_dtzf<doxid-kinematics_8h_1a420856b56c387d8c22f0573cc3ac982f>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float* ti,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_dtzinverse<doxid-kinematics_8h_1acd8930cf0605753897c366584c336cf4>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_dr<doxid-kinematics_8h_1aa77f212503f2e0578b3827355c1e9a8a>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* Ri,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_drf<doxid-kinematics_8h_1ad0eb8ef5f9c9840830c50e2ea3538cd2>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* Ri,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_drinverse<doxid-kinematics_8h_1aa4d4444f8a88684f39f4fd78caaa2c89>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* Ri,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_drinversef<doxid-kinematics_8h_1a9143a8e4a7b2afa8e68635e413d7696d>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* Ri,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_dd<doxid-kinematics_8h_1a27ced4c2a0730cde80969a1d78be23ca>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D1i,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D2i,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_ddf<doxid-kinematics_8h_1a31b0cb8c4cc6ac3a1f7f5d2a0b58472d>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D1i,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D2i,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_product_ddinverse<doxid-kinematics_8h_1a6c6a40a92c022567fa99067d599cdb61>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D1i,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D2i,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	void :ref:`s2_product_ddinversef<doxid-kinematics_8h_1a1547358f17dff0717c8e12e280a92805>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D1i,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D2i,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	void :ref:`s2_htx<doxid-kinematics_8h_1a39432c21f1f424b93ee2c4d910d650bd>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_htxf<doxid-kinematics_8h_1abd6631002506449fb71711d11dcb470b>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

	void :ref:`s2_hty<doxid-kinematics_8h_1af5f0478f99260e9b0e70eed2c14093de>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_htyf<doxid-kinematics_8h_1a73184cc4e2d7243451a44154515a4e05>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

	void :ref:`s2_htz<doxid-kinematics_8h_1a9fcd18da73023ae0baf8375ae728a35d>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* ri,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_htzf<doxid-kinematics_8h_1a1687304ba75e42edef025d08ecbf97e9>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* ri,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

	void :ref:`s2_hrx<doxid-kinematics_8h_1a3a38f85e3e4ccc3c7cbc936c4852143d>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_hrxf<doxid-kinematics_8h_1a7a414a13bc67606532d0575d87837387>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

	void :ref:`s2_hry<doxid-kinematics_8h_1a004c36b2c6363164dd6d48dce004143f>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_hrz<doxid-kinematics_8h_1ab3e368edee7878801f32b851eedee9f1>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_hrzf<doxid-kinematics_8h_1a96541e40cb200d159823089421357a37>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

	void :ref:`s2_htxinverse<doxid-kinematics_8h_1a8ab56e3d8b15362fb200637e5f69a116>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_htxinversef<doxid-kinematics_8h_1ad1efa81d59f93234bdc578514ae0f0fa>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

	void :ref:`s2_htyinverse<doxid-kinematics_8h_1afee0d6416d3465e5483a5f9acdd51d98>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_htyinversef<doxid-kinematics_8h_1a2a43d80b36e7201f3e11eec9da4ef82a>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

	void :ref:`s2_htzinverse<doxid-kinematics_8h_1a1aa156cb200c1143bab7fff45fedfdbc>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_htzinversef<doxid-kinematics_8h_1a38c955050e890a4da54ad1fb8865a54f>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

	void :ref:`s2_hrxinverse<doxid-kinematics_8h_1a44c5133b033dd6ff80bfde1f96afdc90>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_hrxinversef<doxid-kinematics_8h_1ab297a11438b4748cff530fb4da8a2043>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

	void :ref:`s2_hryinverse<doxid-kinematics_8h_1a2f0d67033095676e7031e0bb71e8f3fa>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_hryinversef<doxid-kinematics_8h_1ac0c2f74284fe9459f1148959b0bd048e>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

	void :ref:`s2_hrzinverse<doxid-kinematics_8h_1aa72b91da77d60ce19fb1a4047d059262>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

	void :ref:`s2_hrzinversef<doxid-kinematics_8h_1aa2e93309ef463dddd8e738c9b1514726>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

	void :ref:`s2_screwcross_ss<doxid-kinematics_8h_1aeff653ad6b7f5c62f6f77d99ddb6d93d>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_screwcross_ssf<doxid-kinematics_8h_1a88e4a2073007f92d153fa298ff12ef97>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_screwcross_sst<doxid-kinematics_8h_1af03950ad796422eb9beda69ffb2124af>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_screwcross_sstf<doxid-kinematics_8h_1aaf1335946c59463c4ba81532b6bd217a>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_screwproduct_dts<doxid-kinematics_8h_1a85c2a4a64a3051c63e8f25332426734b>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* ti,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_screwproduct_dtsf<doxid-kinematics_8h_1af0cefda2932c7810e7e7bdccfed7f946>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* ti,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_screwproduct_dtinverses<doxid-kinematics_8h_1a3eab6fdaf8ccc9af0edd32b4dc93c6fc>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* ti,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_screwproduct_dtinversesf<doxid-kinematics_8h_1a308b114b524c248e1c64889879530a2e>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* ti,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_screwproduct_dtsinverse<doxid-kinematics_8h_1a09ed194b68010ab4e25f429f1b79d2b5>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* ti,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_screwproduct_dtsinversef<doxid-kinematics_8h_1a0e746d14b97e6f214f72d5707b49c848>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* ti,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_screwproduct_ds<doxid-kinematics_8h_1a6e34d4bef2ed8f3181a0b2c13731bc7e>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_screwproduct_dsinverse<doxid-kinematics_8h_1a0506b7b95a98175d4d0f75fd153a3433>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_screwproduct_dsf<doxid-kinematics_8h_1a9be1ab951ef968903b0250f2619b6f25>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_screwproduct_dinverses<doxid-kinematics_8h_1ac9d258021e5fe6eef779a912e39bc1a5>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_screwproduct_dinversesf<doxid-kinematics_8h_1abea2d2f876cbde696804af5f91ee4f56>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_coscrewcross_ss<doxid-kinematics_8h_1a1a906e11e10975c319de1ebd09eafed1>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_coscrewcross_ssf<doxid-kinematics_8h_1a68b0b32ea55dba38bea5c4e5e37c20b0>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_coscrewproduct_dts<doxid-kinematics_8h_1a4b04bb7cde275fbf47f18a7d7a00c311>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* ti,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_coscrewproduct_dtsf<doxid-kinematics_8h_1a7b07097f823b02863fc2cefb6d6af0a0>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* ti,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_coscrewproduct_dtsinverse<doxid-kinematics_8h_1a383e5a274d53cc3148695c3738325885>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* ti,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_coscrewproduct_dtsinversef<doxid-kinematics_8h_1afa063e8fd094d7aa717cc3feec7b3922>` (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* ti,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_coscrewproduct_ds<doxid-kinematics_8h_1a7f322f2b6d722b3b24ac429aaa1d3d06>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_coscrewproduct_dsf<doxid-kinematics_8h_1a4ba1fcb1e7150f7d58aad7d1e4e9e797>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_coscrewproduct_dinverses<doxid-kinematics_8h_1ae32c2e74261e720132359d0d5e4a5c1f>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

	void :ref:`s2_coscrewproduct_dinversesf<doxid-kinematics_8h_1ad503267f56b288f35c72007bbd3c2cec>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

	void :ref:`s2_quaternion_normalize<doxid-kinematics_8h_1a354549fff525fa4feca104eb4ce519ad>` (:ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qi)
	void :ref:`s2_quaternionf_normalize<doxid-kinematics_8h_1a85426cef2545af77d69ce3386ea92950>` (:ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* qi)

	void :ref:`s2_mat3x3_toQuaternion<doxid-kinematics_8h_1a86d102e7d45b0a9c27368b46b11a0462>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qo
	    )

	void :ref:`s2_mat3x3f_toQuaternion<doxid-kinematics_8h_1a0da18a57ba2a87e8a2c17f1f73c9d2ad>` (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* qo
	    )

	void :ref:`s2_quaternion_toMat3x3<doxid-kinematics_8h_1a0a1b3bce5132f4ae03192352d1ea8d5c>` (
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qi,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

	void :ref:`s2_quaternionf_toMat3x3f<doxid-kinematics_8h_1aac0f5f19d6f975cdbb14b9acbc700ef8>` (
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* qi,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

	void :ref:`s2_quaternion_slerp<doxid-kinematics_8h_1a49f717f1310c27e85e4bf3df53f9e665>` (
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* q1i,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* q2i,
	    double* t,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qo
	    )

	void :ref:`s2_quaternionf_slerp<doxid-kinematics_8h_1a2f0010db092c86fe7c2f7531dfda91ac>` (
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* q1i,
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* q2i,
	    float* t,
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* qo
	    )

	void :ref:`s2_quaternion_nlerp<doxid-kinematics_8h_1aabc18ea6f7b7d765ed598b8dfd1da89d>` (
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* q1i,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* q2i,
	    double* t,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qo
	    )

	void :ref:`s2_quaternionf_nlerp<doxid-kinematics_8h_1a8bbf8a4063be02eac2296be1a0aa6ac0>` (
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* q1i,
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* q2i,
	    float* t,
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* qo
	    )

	double :ref:`s2_normalizeDegrees<doxid-kinematics_8h_1a72f3e874e98cf8c6abed83a0e3f3342e>` (double angle)
	float :ref:`s2_normalizeDegreesf<doxid-kinematics_8h_1a37ad6a2da9e09c0b1c4b2ad2a94e891f>` (float angle)
	double :ref:`s2_normalizeRadians<doxid-kinematics_8h_1a9d1705e23160bf6ecdee2cc4dc043717>` (double angle)
	float :ref:`s2_normalizeRadiansf<doxid-kinematics_8h_1a65d6444f4ab27cb61cf76f47cf7152f5>` (float angle)

	double :ref:`catan2pi<doxid-kinematics_8h_1ad928cba56a9629532220a1543352990d>` (
	    double y,
	    double x,
	    double theta
	    )

	double :ref:`catan1pi<doxid-kinematics_8h_1a56df7f77228d03737d92529022283ca3>` (
	    double y,
	    double x,
	    double theta
	    )

	void :ref:`s2_mat3x3_getEulerAngles<doxid-kinematics_8h_1a2f7b885ce20d67815d343993c19a815a>` (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* rin,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* angles,
	    s2_axis_enum_t axisType
	    )

	double :ref:`s2_mat3x3_getScalar<doxid-kinematics_8h_1ae373aea0a098137de48a6532d419a35a>` (:ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri)
	float s2_mat3x3f_getScalar (:ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri)

	void :ref:`s2_mat4x4_getTranslation<doxid-kinematics_8h_1a0283d3411d26c8f6bae2e0530e6fd3d2>` (
	    :ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`* Mi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

	void :ref:`s2_mat4x4f_getTranslation<doxid-kinematics_8h_1a9e28efe027651b9c70062f55c92dd4e1>` (
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* Mi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

	void s2_mat4x4f_get (
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* Mi,
	    int row,
	    int column
	    )

	void :ref:`s2_add_dpiToq<doxid-kinematics_8h_1a842ea4c227f675665defe5afafdbfb2a>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* dpi,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qi
	    )

	void :ref:`s2_subtract_dpiFromq<doxid-kinematics_8h_1aa160158656d72b34767a9332aa3fb8ed>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* dpi,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qi
	    )

	void :ref:`s2_omega_to_dquaternion<doxid-kinematics_8h_1a46a7a3faa7ae267d6838f4af751dbd86>` (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* w,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qi,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* dqo
	    )

	void :ref:`s2_array_lerp<doxid-kinematics_8h_1a8553fd17e8c7ce4bf1a39535fcb7994c>` (
	    int qCount,
	    double p,
	    double* y0,
	    double* y1,
	    double* yOut
	    )

	void :ref:`s2_gramSchmidt<doxid-kinematics_8h_1a34b1a2429f9da44179c47182b68064c7>` (
	    int m,
	    int n,
	    double* A
	    )

	bool :ref:`s2_all_close<doxid-kinematics_8h_1ad65acbd7c97bf3da11f2a67fabb83ab6>` (
	    int count,
	    double* a1,
	    double* a2
	    )

	:ref:`s2_light_t<doxid-structs2__light__t>`* :ref:`s2_light<doxid-light_8h_1ad9765a978f23c92d27daf60435862fd8>` ()
	void :ref:`s2_light_free<doxid-light_8h_1a5c94c15851fdc4a29fd6ed14757e999d>` (:ref:`s2_light_t<doxid-structs2__light__t>`* light)

	void :ref:`s2_light_load<doxid-light_8h_1ac13a30ffdcb499e7c60c623cac363790>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`*,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`*,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`*,
	    json_t*
	    )

	:ref:`s2_material_t<doxid-structs2__material__t>`* :ref:`s2_materials<doxid-material_8h_1aea218d524bf86bfcc67505f90d73eeec>` (int materialCount)

	void :ref:`s2_materials_free<doxid-material_8h_1a26270fc1c9c7af76bb38f269fe20f28d>` (
	    :ref:`s2_material_t<doxid-structs2__material__t>`* materials,
	    int materialCount
	    )

	void s2_memoize_initializeEquations (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_memoize_memoizeEquations<doxid-memoize_8h_1ab08412c85f70b3de55c8e9d406cf982e>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_mobility_compute_substructure<doxid-mobility_8h_1afa410441c536e454ca16575d1eda3b66>` (:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure)
	void :ref:`s2_mobility_compute<doxid-mobility_8h_1a5e42cbbf68e159fff14f950cae23db27>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_mobility_initialize<doxid-mobility_8h_1af1e7878c7eb75610777442b9aa586cf1>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_mobility_print<doxid-mobility_8h_1a0d364dc3b600247c3504f3214e1e3fb8>` (
	    FILE* f,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

	void :ref:`s2_mobility_print_substructure<doxid-mobility_8h_1a09ba0060113d1938c45147279e73a54a>` (
	    FILE* f,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex
	    )

	void :ref:`s2_mobility_computeGlobalMobility<doxid-mobility_8h_1a351d36237fc26ddeb6b5ba18360382f9>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	double :ref:`s2_newtonRaphson_evaluateConstraintError_substructure<doxid-newtonraphson_8h_1a4033654cb0e578544930add89add4f4d>` (:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure)
	double :ref:`s2_newtonRaphson_evaluateqError_substructure<doxid-newtonraphson_8h_1a656cf1465235dd983b20f6feea961888>` (:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure)
	int :ref:`s2_newtonRaphson_removeConstraintViolations<doxid-newtonraphson_8h_1a2696b332b31fd7d0a4d55e1bce020b19>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	int :ref:`s2_newtonRaphson_removeConstraintViolations_substructure<doxid-newtonraphson_8h_1ab9c6ee525e16b453216e571bf8056bd0>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex
	    )

	int :ref:`s2_newtonRaphson_removeConstraintViolationsFast_substructure<doxid-newtonraphson_8h_1a64e792a57875231f6c2780e825272d6d>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex,
	    bool useTightTolerance
	    )

	void :ref:`s2_newtonRaphson_setup_specified<doxid-newtonraphson_8h_1a448b4d41be7de62c96f3e4754aa5f10e>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex,
	    int specifiedCoordinateIndex
	    )

	int :ref:`s2_newtonRaphson_increment_specified<doxid-newtonraphson_8h_1a941c03ef0794a5da4c703aac2a74cd92>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex,
	    int specifiedCoordinateIndex,
	    double value,
	    :ref:`s2_joint_enum_t<doxid-joint_8h_1a7a43c175bb516fb74df6eb8503a0475c>` jointType,
	    :ref:`s2_move_enum_t<doxid-joint_8h_1a17c0fd52873eda0dcebfe02edce180cc>` moveType
	    )

	:ref:`s2_pointcloud_t<doxid-structs2__pointcloud__t>`* :ref:`s2_pointcloud<doxid-pointcloud_8h_1a29a5d5879ca5f5c2278f8fe873c98f95>` (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`*)
	void :ref:`s2_pointcloud_free<doxid-pointcloud_8h_1aa3bdcd3f0dc5c11409fd5148bf826e34>` (:ref:`s2_pointcloud_t<doxid-structs2__pointcloud__t>`*)
	:ref:`s2_queue_t<doxid-structs2__queue__t>`* :ref:`s2_queue<doxid-queue_8h_1a8948a6d9b2c828280d6c3dcf2855b805>` ()

	void :ref:`s2_queue_append<doxid-queue_8h_1aac72a13f0c311d229363e5d238ca67e0>` (
	    :ref:`s2_queue_t<doxid-structs2__queue__t>`* queue,
	    void* value,
	    s2_queue_enum_t type
	    )

	void* :ref:`s2_queue_pop<doxid-queue_8h_1a672b3ec9b38b169a0de23576bfb01ad5>` (:ref:`s2_queue_t<doxid-structs2__queue__t>`* queue)

	int s2_queue_popInt (
	    :ref:`s2_queue_t<doxid-structs2__queue__t>`* queue,
	    int integer
	    )

	void :ref:`s2_queue_free<doxid-queue_8h_1a246929c223a14f1eb08a24d5979b381f>` (:ref:`s2_queue_t<doxid-structs2__queue__t>`* queue)
	:ref:`s2_restraint_t<doxid-structs2__restraint__t>`* :ref:`s2_restraint<doxid-restraint_8h_1a48017c62763694dc8244b8680c055b05>` ()

	void s2_restraint_append (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_restraint_t<doxid-structs2__restraint__t>`* restraint
	    )

	void s2_restraint_memoize (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	:ref:`s2_sensor_t<doxid-structs2__sensor__t>`* s2_sensor ()
	void :ref:`s2_solve_integrateEquations<doxid-solve_8h_1a2df56c26f56eca9a44cb0eb78ce900e8>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_solve_momentum<doxid-solve_8h_1ac4a44e1eb5c17a1bc19465d2f93c07f0>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_solve_kineticEnergy<doxid-solve_8h_1a35adcb92a308bbf156488434921c788b>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_solve_gravitationalPotentialEnergy<doxid-solve_8h_1a506f2143d6f10cec3fff7ce949c2346b>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	bool :ref:`s2_solve_verify<doxid-solve_8h_1a0cb4957a1a0b45d9e9d7ab08ccbdbbed>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double* Ml,
	    double* fl
	    )

	void :ref:`s2_equation_fcn<doxid-solve_8h_1ac2e1439d404d7bac5c26ebd6129aa8cc>` (
	    double t,
	    double y [],
	    double dy [],
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

	void :ref:`s2_solve_writeHeaders<doxid-solve_8h_1aba986392ba0e60655634ff6926713443>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_solve_writeStates<doxid-solve_8h_1a640ad4177d355b89cc907fea57180f4f>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int step
	    )

	void :ref:`s2_sphere_approximateMinBoundSphere<doxid-sphere_8h_1acac5b4750ef8472ef0252c4317e978fa>` (
	    :ref:`s2_sphere_t<doxid-structs2__sphere__t>`*,
	    :ref:`s2_body_t<doxid-structs2__body__t>`*
	    )

	void :ref:`s2_sphere_updateCentroid<doxid-sphere_8h_1a9bc196d4fd944d676699ceb8d13a32bc>` (:ref:`s2_sphere_t<doxid-structs2__sphere__t>`*)
	:ref:`s2_stack_t<doxid-structs2__stack__t>`* :ref:`s2_stack<doxid-stack_8h_1afd6037900f9e8b5cbf689b146b1ace10>` ()
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`s2_stackstr<doxid-stack_8h_1a7f24dbcee8d8aee6ccc7fdd2b527469d>` ()

	void :ref:`s2_stack_push<doxid-stack_8h_1a043afb2a321da628790059d5a9edb105>` (
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack,
	    int value
	    )

	void :ref:`s2_stackstr_push<doxid-stack_8h_1a561d62e8b60766d95c165cca29072146>` (
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack,
	    const char* value
	    )

	void :ref:`s2_stack_append<doxid-stack_8h_1a30863701a4fbb815aa2766bab6a7ed2c>` (
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack,
	    int value
	    )

	int :ref:`s2_stack_remove<doxid-stack_8h_1a6887a4ff408c987f03e8b111ed55941b>` (
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack,
	    :ref:`s2_stacknode_t<doxid-structs2__stacknode__t>`* node
	    )

	void :ref:`s2_stackstr_append<doxid-stack_8h_1ac5d5c7c676643be1a68311f5c965a518>` (
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack,
	    const char* value
	    )

	int :ref:`s2_stack_pop<doxid-stack_8h_1a3328beaff6aee8299d4ccd04c62f1397>` (:ref:`s2_stack_t<doxid-structs2__stack__t>`* stack)
	const char* :ref:`s2_stackstr_pop<doxid-stack_8h_1af107df27f1dbe23eaf58fdb7a7f1d478>` (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack)

	void :ref:`s2_stack_print<doxid-stack_8h_1ac0d3dd3cda0476736e640d500776ee67>` (
	    FILE* f,
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack
	    )

	void :ref:`s2_stackstr_print<doxid-stack_8h_1a5fd679a656c3393c43ee188954657e37>` (
	    FILE* f,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack
	    )

	void :ref:`s2_stack_merge<doxid-stack_8h_1ac7a7d8ada71e06e854788bdcc90bf41f>` (
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack1,
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack2
	    )

	void :ref:`s2_stackstr_merge<doxid-stack_8h_1a0f16758ddabf8b1f64603d7a70d53aaa>` (
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack1,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack2
	    )

	int* :ref:`s2_stack_toArray<doxid-stack_8h_1abfe0df16d02f11b5772b7908adfed0a1>` (:ref:`s2_stack_t<doxid-structs2__stack__t>`* stack)
	void :ref:`s2_stack_free<doxid-stack_8h_1ad088b50bfcb9b4392a0046d8c16ec16a>` (:ref:`s2_stack_t<doxid-structs2__stack__t>`* stack)
	void :ref:`s2_stackstr_free<doxid-stack_8h_1a1e0b5156595d851be28b5c84d0900be4>` (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack)

	bool :ref:`s2_stack_in<doxid-stack_8h_1a4d034e4d5260bc873d1fad659fef360f>` (
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack,
	    int value
	    )

	bool :ref:`s2_stackstr_in<doxid-stack_8h_1aa38c1bc6a810bf3e3197ca3050344fe3>` (
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack,
	    const char* value
	    )

	bool :ref:`s2_stackstr_isEqual<doxid-stack_8h_1a339a8dfb0a945016899f53b9855daec7>` (
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack1,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack2
	    )

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`s2_stackstr_copy<doxid-stack_8h_1af37577a33750ddde27c14ae42743e9b1>` (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack)
	char* :ref:`s2_stackstr_getRoot<doxid-stack_8h_1a98afc36883f20654f83c501a1283a88b>` (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack)
	char* :ref:`s2_stackstr_toStr<doxid-stack_8h_1accf9bdf935047c8f248720a28a9ac348>` (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack)
	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* :ref:`s2_stackstr_fromString<doxid-stack_8h_1a5446695107700cb8b8d003095d059411>` (const char*)
	char* s2_getRootName (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack)
	:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* :ref:`s2_substructure<doxid-substructure_8h_1a2237fe00c9594061a85f1fb7329c8382>` ()
	void generateSampleHeightMap (const char* filename)

	double :ref:`s2_terrain_get<doxid-terrain_8h_1a9b9c1297c5055f636e96fc6fb38ed05c>` (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    int i,
	    int j
	    )

	double :ref:`s2_terrain_getZ_bilinear_sae<doxid-terrain_8h_1a0d32a7c840f225e7f6ee3bde9b5c7493>` (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

	double :ref:`s2_terrain_getZ_bilinear_iso<doxid-terrain_8h_1aa357f8946d82e483a238d5031fc139c1>` (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

	double :ref:`s2_terrain_getZ_bicubic_sae<doxid-terrain_8h_1a8274320819cdd8a446b8fff086624351>` (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

	double :ref:`s2_terrain_getZ_bicubic_iso<doxid-terrain_8h_1ac66cd961cd6936c5211afdf87d859816>` (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

	double :ref:`s2_terrain_getZ_catmulrom_sae<doxid-terrain_8h_1aebc3b2daef02c395cf60b49966493d07>` (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

	double :ref:`s2_terrain_getZ_catmulrom_iso<doxid-terrain_8h_1a248c11bb6684c9f8c970ca44334681a3>` (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

	:ref:`s2_terrain_t<doxid-structs2__terrain__t>`* :ref:`s2_terrain<doxid-terrain_8h_1a4090acc0bd8c2dbf3ebe77d634376b75>` (
	    const char* terrainFileName,
	    const char* textureFileName,
	    const char* propertyMap,
	    s2_interpolation_enum_t interpolationType,
	    s2_coordinate_convention_enum_t coordinateConvention,
	    double xOffset,
	    double yOffset,
	    double xScale,
	    double yScale
	    )

	void :ref:`s2_terrain_getHeightAndSAENormal<doxid-terrain_8h_1a67f681544bd6e0983cc63fd610b3f916>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double x,
	    double y,
	    double* height,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

	void :ref:`s2_terrain_getHeightAndISONormal<doxid-terrain_8h_1a9930e944c824ed54c327c78862d5c935>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double x,
	    double y,
	    double* height,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

	int :ref:`s2_terrain_load<doxid-terrain_8h_1ad40245713ff01092a78ef326b10cc77a>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    json_t* assemblyObject,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix
	    )

	void :ref:`s2_context_setTopology<doxid-topology_8h_1a804e0ff9b5a92350672f48b061d67358>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_updateDH<doxid-topology_8h_1a1303aec0a235bc6e16b05d77bdca0e75>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_updateDHVA<doxid-topology_8h_1a16e28aa318129a9c08698a1960987232>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_updateDHVADebug<doxid-topology_8h_1aafb159f6b3cb5306a80a821727143cfe>` (
	    FILE* f,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double t
	    )

	void :ref:`s2_updateGLReferenceFrames<doxid-topology_8h_1a67fe61b3b669f5191fdf381ffea93899>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	int** :ref:`s2_arrayi_malloc2D<doxid-utility_8h_1aee0340f870a70e7d632045f6d4b8a13c>` (
	    int rows,
	    int columns
	    )

	void :ref:`s2_arrayi_free2D<doxid-utility_8h_1a4b448a612428678975832349870e117e>` (int** x)
	void :ref:`s2_arrayi_free<doxid-utility_8h_1a8683a7de3c5b6a462044d998c35072f2>` (int* x)
	void :ref:`s2_array_free<doxid-utility_8h_1ac288fee0fe1a1e7b6489bb44595d0166>` (double* x)
	void :ref:`s2_arrayf_free<doxid-utility_8h_1a58bef2214578639ee0bb8800a558653b>` (float* x)
	void :ref:`s2_arrayb_free<doxid-utility_8h_1a4740fdd9a78a68d4cec6bb7ca3b5ac84>` (bool* x)
	void :ref:`s2_arrays_free<doxid-utility_8h_1aff05ccdb6fa0e375c7363148eafa290a>` (:ref:`s2_screw_t<doxid-structs2__screw__t>`* x)
	void :ref:`s2_arrayd_free<doxid-utility_8h_1a2f2f643fe0c57d3d4784794d10181f1a>` (:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* x)

	double** :ref:`s2_array_malloc2D<doxid-utility_8h_1a641113e0f7789a14d9bc0106146c8f48>` (
	    int rows,
	    int columns
	    )

	float** :ref:`s2_arrayf_malloc2D<doxid-utility_8h_1a28b92c1222ab1725d85ac2edec08671c>` (
	    int rows,
	    int columns
	    )

	void :ref:`s2_array_free2D<doxid-utility_8h_1a0b61c9879dcb264ab4af38a98db625d3>` (double** x)
	void :ref:`s2_arrayf_free2D<doxid-utility_8h_1a72956a50f32bc1c71a4281144b1aa59d>` (float** x)

	char** :ref:`s2_arrayc_malloc2D<doxid-utility_8h_1acb084932ad0d60106ccfe60dfd9aa5c5>` (
	    int rows,
	    int columns
	    )

	void :ref:`s2_arrayc_free2D<doxid-utility_8h_1ae441785d07d353b611b750f87ba13f84>` (char** x)

	void :ref:`s2_array_rowToColumnMajor<doxid-utility_8h_1aa393eef291884b3ca760ffc8bb07f68a>` (
	    int rowDim,
	    int colDim,
	    double* ColumnMajorMatrix,
	    double* RowMajorMatrix
	    )

	int :ref:`s2_displacement_copyUpdate<doxid-utility_8h_1ab546c4df37377474fd48b1662a05cb3e>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`** Di,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

	int :ref:`s2_displacementf_copyUpdate<doxid-utility_8h_1ab21c42a07e3762e5a50ba6d393fddfaa>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`** Di,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

	int :ref:`s2_displacement_copyBack<doxid-utility_8h_1a41b8470b02489f772ebcaba953c27770>` (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`** Do
	    )

	int :ref:`s2_displacementf_copyBack<doxid-utility_8h_1aca1113329136eeed8c2bb42201216418>` (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`** Do
	    )

	int :ref:`s2_screw_copyUpdate<doxid-utility_8h_1a189883fe3a10eaa7fe280f5059898e22>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`** Si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* So
	    )

	int :ref:`s2_screwf_copyUpdate<doxid-utility_8h_1a43eed17b23d6f8ab3da42313ba63e436>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`** Si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* So
	    )

	int :ref:`s2_screw_copyBack<doxid-utility_8h_1a7a4a3483be25b87171483be0811e75ab>` (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* Si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`** So
	    )

	int :ref:`s2_screwf_copyBack<doxid-utility_8h_1a7bf324b3d94c50cccefaa3d9c55ae3b5>` (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* Si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`** So
	    )

	int :ref:`s2_array_copyUpdate<doxid-utility_8h_1a1213ee993cf54a0e118ac033a23e7ed9>` (
	    int count,
	    double** oldArray,
	    double* newArray
	    )

	int :ref:`s2_arrayf_copyUpdate<doxid-utility_8h_1a5a751490c3c7990411c2f9e09e67ccd5>` (
	    int count,
	    float** oldArray,
	    float* newArray
	    )

	int :ref:`s2_array_copyBack<doxid-utility_8h_1a020f9bc6d057069d5ae71090ad988f2a>` (
	    int count,
	    double* oldArray,
	    double** newArray
	    )

	int :ref:`s2_arrayf_copyBack<doxid-utility_8h_1aa53794d55f92436ed056c152f4b8e293>` (
	    int count,
	    float* oldArray,
	    float** newArray
	    )

	char* :ref:`s2_getFilePath<doxid-utility_8h_1a8acece5fdde1300ce4de9cc9a99d29d8>` (const char* fullName)
	const char* :ref:`s2_getFileExtension<doxid-utility_8h_1a334f334da1518648536582cae344d5ce>` (const char*)

	void s2_damping_joint (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_joint_t<doxid-structs2__joint__t>`* joint
	    )

	:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* :ref:`s2_forcemodule<doxid-forcemodule_8h_1a10ba39388d465932d035489090731365>` (:ref:`s2_forcemodule_enum_t<doxid-forcemodule_8h_1aef216ac0fc3d71c6419b69df97ab6091>` forceModuleType)
	void :ref:`s2_forcemodule_flatten<doxid-forcemodule_8h_1add15a798ad8c32c4be95b4f9e638adb3>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_forcemodule_connect<doxid-forcemodule_8h_1a65c554aaea385eb8ec611f9609469d7e>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void s2_forcemodule_updateTransient (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double* y
	    )

	void :ref:`s2_forcemodule_saveHistory<doxid-forcemodule_8h_1afd5e4f09a802a2a80d1fb7c06f71b5f5>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_forcemodule_append<doxid-forcemodule_8h_1a472eef329ecb6c3d3e934ede27279e4c>` (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* forceModule
	    )

	void :ref:`s2_forcemodule_free<doxid-forcemodule_8h_1abd2b86528c4384d622201790c1c6dcd1>` (:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* forceModule)

	void :ref:`s2_forcemodule_load<doxid-forcemodule_8h_1ac28262f7c2ba16e662ff311b36ba2c09>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix,
	    json_t* forceModuleObject
	    )

	:ref:`s2_pid_t<doxid-structs2__pid__t>`* s2_pid ()

	void :ref:`s2_pid_updateFromJointToJoint<doxid-pid_8h_1adfefdc1c61c0e5157a09686e27e4d7ab>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    void** parameters,
	    double* y
	    )

	void :ref:`s2_pid_updateFromFrameToJoint<doxid-pid_8h_1ac8c49e0c6bef94a9f38dde8182c6e8a0>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    void** parameters,
	    double* y
	    )

	void s2_pid_connectForceModulePointer (
	    :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* forceModule,
	    :ref:`s2_pid_t<doxid-structs2__pid__t>`* pid
	    )

	void s2_pid_saveHistory (void* parameters)

	void :ref:`s2_service_setForces<doxid-service_8h_1a848e4950f54f139287ae4a6765fad69f>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double* y
	    )

	void :ref:`s2_stiffness_joint<doxid-stiffness_8h_1ad6dc04dd6bda9c49495fa1ef5f47a210>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_joint_t<doxid-structs2__joint__t>`* joint
	    )

	:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* :ref:`s2_tire_fiala<doxid-fiala_8h_1a8658302eeb1bec4475877e8367172394>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* filePath,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* wheelBody
	    )

	void :ref:`s2_tire_fiala_free<doxid-fiala_8h_1a3e1a2e637efc322442633553baba498c>` (:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire)

	void :ref:`s2_tire_fiala_update<doxid-fiala_8h_1a322fce69ed2f93ae89452a861d8303ca>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    void** parameters,
	    double* y
	    )

	void :ref:`s2_tire_fiala_loadTireDataFile<doxid-fiala_8h_1aef0605aba2e6664ad511704f3adc29fc>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire,
	    const char* filePath
	    )

	:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* :ref:`s2_tire_pac2002<doxid-pac2002_8h_1a52c64dbbbd2c064b8032d7dcce7a10d2>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* filePath,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* wheelBody,
	    bool useTransientSlip
	    )

	void :ref:`s2_tire_pac2002_free<doxid-pac2002_8h_1a75e3fa6c00463a48be7638e921500143>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void :ref:`s2_tire_pac2002_update<doxid-pac2002_8h_1a73243b38626d6cb76ca38ee1d00b83f6>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    void** parameters,
	    double* y
	    )

	void s2_tire_pac2002_getSlipAngle (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void :ref:`s2_tire_pac2002_loadTireDataFile<doxid-pac2002_8h_1a3392f865e9daaa82bbf47793f3a5b8ab>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    const char* filePath
	    )

	void :ref:`s2_tire_pac2002_updateWFrame<doxid-pac2002_8h_1a5075ecf586909626c01f2df725bc3af4>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)
	double :ref:`s2_tire_pac2002_calc_Fz<doxid-pac2002_8h_1aeaaea7981cb1d6b432d05f587fd66ec9>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)
	void :ref:`s2_tire_pac2002_updateRelaxationLengths<doxid-pac2002_8h_1a215220f8e29bf377a51244a6ca6cced0>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)
	void :ref:`s2_tire_pac2002_updateSlipKinematic<doxid-pac2002_8h_1a80e0c5ab777c6793a5e1a74a83dfa5c7>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)
	void :ref:`s2_tire_pac2002_zeroSlips<doxid-pac2002_8h_1a3bd2800b93b071b82df4ef3986561110>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)
	void s2_tire_pac2002_evaluateSlips (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void :ref:`s2_tire_pac2002_evaluateReactions<doxid-pac2002_8h_1ac20ea3476c1d1cbf4dc6f6cfcf52e168>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    bool write_violations,
	    bool enforce_threshold
	    )

	void :ref:`s2_tire_pac2002_slip_from_uv<doxid-pac2002_8h_1ae0239415e4200b821dec893a8bf183b8>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    bool use_besselink,
	    double bessel_Cx,
	    double bessel_Cy,
	    double V_low
	    )

	void :ref:`s2_tire_pac2002_pureSlipReactions<doxid-pac2002_8h_1ae9766ff827292da718194567c723b4eb>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)
	void :ref:`s2_tire_pac2002_combinedSlipReactions<doxid-pac2002_8h_1a2e900f40155187ffbb850e80964d17bd>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	double :ref:`s2_tire_pac2002_Fx_pureLong<doxid-pac2002_8h_1a0c87e22d7746b5f7630c920217b6a3aa>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double kappa
	    )

	double :ref:`s2_tire_pac2002_Fy_pureLat<doxid-pac2002_8h_1aab053dca94043c0115b8c86b755608f3>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma
	    )

	double :ref:`s2_tire_pac2002_Mz_pureLat<doxid-pac2002_8h_1aa1f7798f862f1b1af24b410938970577>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double Fy_pureSlip
	    )

	double :ref:`s2_tire_pac2002_Fx_combined<doxid-pac2002_8h_1a75967c7dc5b18d94b56e03beb9d7cbb4>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double kappa,
	    double Fx_pureSlip
	    )

	double :ref:`s2_tire_pac2002_Fy_combined<doxid-pac2002_8h_1a5873fda415a72e239ea243bed79405c7>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma,
	    double kappa,
	    double Fy_pureSlip
	    )

	double :ref:`s2_tire_pac2002_Mz_combined<doxid-pac2002_8h_1ad36b82bb284940522554a8675cbf9e2f>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha_r,
	    double alpha_t,
	    double gamma,
	    double kappa,
	    double Fx_combined,
	    double Fy_combined
	    )

	double :ref:`s2_tire_pac2002_calc_Mx<doxid-pac2002_8h_1a095bb61ec83dc5a79153f55e4d66f56d>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double Fy_combined
	    )

	double :ref:`s2_tire_pac2002_calc_My<doxid-pac2002_8h_1ae8f86738eda52bbc1d5ea709659c07d7>` (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double Fx_combined
	    )

	bool :ref:`s2_tire_pac2002_discTerrainContact<doxid-pac2002_8h_1ab7a1630c7e0a24a451236c2cb143dc15>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void s2_tire_pac2002_updateTransient (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    void** parameters,
	    double* y
	    )

	void :ref:`s2_tire_pac2002_updateVerticalLoad<doxid-pac2002_8h_1aad631fb210d174a3be0f80a058b42197>` (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

	void :ref:`s2_tire_pac2002_transferForces<doxid-pac2002_8h_1ac1c6d29395ad7cec3dd2d9127b6fdbd9>` (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire
	    )

	void :ref:`s2_numericalintegrator_allocate<doxid-numericalintegrator_8h_1add0ac8b46877a1d5e6189e5c71c4a15f>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_numericalintegrator_free<doxid-numericalintegrator_8h_1a4c29dcac0bbfd00ed84dc52678b62c7e>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void s2_ode113 (
	    void  f double t, double y[], double dy[], s2_context_t *context,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

	void s2_ode113_de (
	    void  f double t, double y[], double dy[], s2_context_t *context,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
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

	int :ref:`i4_sign<doxid-ode113_8h_1a535d66c4385dd7e8c98f78cd4c8b3a2f>` (int i)

	void :ref:`s2_ode113_intrp<doxid-ode113_8h_1ade9963857d52e84c92408c920578c26d>` (
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

	double :ref:`r8_epsilon<doxid-ode113_8h_1ab21d05128ddb24988dfbca84c955c6f8>` (void)

	void s2_ode113_step (
	    double* x,
	    double y [],
	    void  f double t, double y[], double dy[], s2_context_t *context,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
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

	void timestamp (void)
	void s2_ode113_allocate (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_ode113_free<doxid-ode113_8h_1ae5e6e61d2397f04273a1d4a1623bb244>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_ode113_integrateEquations<doxid-ode113_8h_1aeb5c98af7f1932d7afa5a7ed255c81a9>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_ode113b_allocate<doxid-ode113b_8h_1ab276701dcf2866123f1b99ccdc0e9394>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)
	void :ref:`s2_ode113b_free<doxid-ode113b_8h_1ae22ccb46e43e1e319a2584304c944527>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	void :ref:`s2_ode113b_fcn<doxid-ode113b_8h_1adfa68c62baf3abe532197f5ef25eed8b>` (
	    double y [],
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

	void s2_ode113b_step (
	    void  f double y[], s2_context_t *context,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

	void :ref:`s2_ode113b_de<doxid-ode113b_8h_1a8cc5cb1907a1b5de62e501a7a9ac0096>` (
	    void  f double y[], s2_context_t *context,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

	void :ref:`s2_ode113b_integrateEquations<doxid-ode113b_8h_1abff36438c30c864f0e9cb3407d5ac91c>` (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

	// macros

	#define ThrowWandException(wand)
	#define ThrowWandException(wand)
	#define :ref:`_STAR2_H_<doxid-star2_8h_1a5d90259d9c8d01a76dda46dcc46c74cd>`

.. _details-doxid-global:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Typedefs
--------

.. _doxid-assembly_8h_1a27dda9bdee20cd214357bb4cf28255b7:
.. _cid-s2_body_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_body_t<doxid-structs2__body__t>` s2_body_t

Describes properties related to a rigid body.

A Body datastructure contains:

* A descriptive name

* A Frame linked list

* Pointer to Frame defined as origin

The following attributes are assigned by the s2_ProcessTopology() function:

* An index, which is used to identify position in minimal *run-time* arrays

* A level, which indicates level in the spanning tree

* The unique reference frame, which may not be the origin

.. _doxid-assembly_8h_1a4d755a9c89f71a04e40c30284527fe36:
.. _cid-s2_joint_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_joint_t<doxid-structs2__joint__t>` s2_joint_t

properties related to Joints

.. _doxid-assembly_8h_1adfc16139f51eb76006f0c6224b59b55b:
.. _cid-s2_restraint_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_restraint_t<doxid-structs2__restraint__t>` s2_restraint_t

defines a Restraint

.. _doxid-assembly_8h_1aebd2492768e60b58c2ce2e2c7254e491:
.. _cid-s2_io_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_io_t<doxid-structs2__io__t>` s2_io_t

input/output data structure

This structure stores data related to sensors, i.e. cameras, lidar, radar, touch

.. _doxid-assembly_8h_1a6cfa09e2454a092d1d37c32a584f809a:
.. _cid-s2_forcemodule_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>` s2_forcemodule_t

utility to feed back forces into a running simulation

.. _doxid-assembly_8h_1aaf71dd06d1111557c4a6b11d4a7c426d:
.. _cid-s2_assembly_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_assembly_t<doxid-structs2__assembly__t>` s2_assembly_t

Describes properties related to an Assembly.

Each assembly contains a descriptive name, a flag to specify whether the Assembly is active/inactive and a linked list of:

* Subassemblies

* Bodies

* Joints

* Lights

* Restraints

.. _doxid-assembly_8h_1a1987e656463ddf978b84f49911121113:
.. _cid-s2_context_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_context_t<doxid-structs2__context__t>` s2_context_t

star2 context datastructure

This datastructure holds all of the values associated with a model. An empty Context data structure can be initialized using the s2_InitializeContext function or loaded from a file using the s2_LoadJSON function.

.. _doxid-body_8h_1ad28decee56fd6e153be7e1c8a29bd157:
.. _cid-s2_body_t-3:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_body_t<doxid-structs2__body__t>` s2_body_t

Describes properties related to a rigid body.

A Body datastructure contains:

* A descriptive name

* A Frame linked list

* Pointer to Frame defined as origin

The following attributes are assigned by the s2_ProcessTopology() function:

* An index, which is used to identify position in minimal *run-time* arrays

* A level, which indicates level in the spanning tree

* The unique reference frame, which may not be the origin

.. _doxid-body_8h_1a170ad83b57b29a38a6287305c19634e8:
.. _cid-s2_frame_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_frame_t<doxid-structs2__frame__t>` s2_frame_t

Frame or coordinate system.

.. _doxid-body_8h_1a5ecf8ba3501eacd106b940ed548874ba:
.. _cid-s2_material_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_material_t<doxid-structs2__material__t>` s2_material_t

data structure defining a material

.. _doxid-chordconstraint_8h_1a16952dee99f6b367eb7cc57a024cd19d:
.. _cid-s2_chordconstraint_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_chordconstraint_t<doxid-structs2__chordconstraint__t>` s2_chordconstraint_t

chordConstraintNode properties

If chord loops are detected, a virtual massless body is inserted and a chordConstraintNode data structure is created for each chord loop

.. _doxid-context_8h_1ae11b519e2e3234898f7dd62a6b5074e9:
.. _cid-s2_context_t-3:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_context_t<doxid-structs2__context__t>` s2_context_t

star2 context datastructure

This datastructure holds all of the values associated with a model. An empty Context data structure can be initialized using the s2_InitializeContext function or loaded from a file using the s2_LoadJSON function.

.. _doxid-context_8h_1a1f81c82915846be30f4b25043b6297b6:
.. _cid-s2_sensor_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_sensor_t<doxid-structs2__sensor__t>` s2_sensor_t

a sensor datastructure

.. _doxid-convexhull_8h_1adec77081a8ea85c9180d59bca2c258f4:
.. _cid-s2_convexhull_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_convexhull_t<doxid-structs2__convexhull__t>` s2_convexhull_t

convex hull data structure contains vertices and faces in contiguous blocks of memory

.. _doxid-frame_8h_1ac997324df7b2ecdf15ac1a9794c4adf7:
.. _cid-s2_frame_t-3:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_frame_t<doxid-structs2__frame__t>` s2_frame_t

Frame or coordinate system.

.. _doxid-inertia_8h_1a213f599bafaaf13e9bcece9aec49de93:
.. _cid-s2_inertia_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_inertia_t<doxid-structs2__inertia__t>` s2_inertia_t

mass and inertia properties

.. _doxid-io_8h_1aff30d428569c4eced0cef97477a8a60e:
.. _cid-s2_io_t-3:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_io_t<doxid-structs2__io__t>` s2_io_t

input/output data structure

This structure stores data related to sensors, i.e. cameras, lidar, radar, touch

.. _doxid-joint_8h_1a8db073f35bb50443203b729b364ea632:
.. _cid-s2_joint_t-3:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_joint_t<doxid-structs2__joint__t>` s2_joint_t

properties related to Joints

.. _doxid-light_8h_1acbf63648c4231f4b0451d44f6fe6a988:
.. _cid-s2_light_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_light_t<doxid-structs2__light__t>` s2_light_t

Defines a Light.

.. _doxid-material_8h_1a7252505fa938247a5db1425abd8bc13d:
.. _cid-s2_material_t-3:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_material_t<doxid-structs2__material__t>` s2_material_t

data structure defining a material

.. _doxid-restraint_8h_1a62184c43e8c59b589eef904750faa69c:
.. _cid-s2_restraint_t-3:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_restraint_t<doxid-structs2__restraint__t>` s2_restraint_t

defines a Restraint

.. _doxid-sensor_8h_1ab1cba108cff23659d6989ab6daa4174a:
.. _cid-s2_sensor_t-3:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_sensor_t<doxid-structs2__sensor__t>` s2_sensor_t

a sensor datastructure

.. _doxid-substructure_8h_1af01a3e27562b78e8b11f9bedc384f4b6:
.. _cid-s2_substructure_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_substructure_t<doxid-structs2__substructure__t>` s2_substructure_t

Defines properties related to kinematic substructure or module.

At the time of model loading, the topology of a complex mechanism is analyzed to determine if it would benefit from kinematic substructuring. Kinematic substructuring effectively block diagonalizes the Jacobian matrix, which significantly reduces the number of floating point operations required for generalized coordinate partitioning. If the mechanism is an open loop chain, there are no chord constraints to process, and therefore no substructures are returned. If the mechanism does not benefit from substructuring, only one substructure is created (containing the entire mechanism.) A jacobian matrix, a jacobian *mapping* matrix, and an array of constraint violations are computed for each kinematic substructure.

.. _doxid-damping_8h_1ad827256522fc82b2ad4001167238a775:
.. _cid-s2_damper_t-2:
.. ref-code-block:: cpp
	:class: title-code-block

	typedef struct :ref:`s2_damper_t<doxid-structs2__damper__t>` s2_damper_t

damper data structure

piecewise curves (i.e. splines) are not currently supported

.. _doxid-forcemodule_8h_1a83a5bc188b2f62e1c16dd941c05a2951:
.. _cid-s2_forcemodule_t-3:
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

Global Variables
----------------

Global Functions
----------------

.. _doxid-assembly_8h_1a6c2416f0aee7fe6dba810699aac97815:
.. _cid-s2_assembly:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* s2_assembly ()

Initialize an Assembly datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - None

        - 



.. rubric:: Returns:

an assembly datastructure

.. _doxid-assembly_8h_1afe1d2adb24d06738dd61cb7b8b8710b4:
.. _cid-s2_assembly_appendsubassembly:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_assembly_appendSubAssembly (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* subAssembly
	    )

Append a subassembly to a mbdl assembly.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - an Assembly datastructure to append to

    *
        - subAssembly

        - an Assembly datastructure to append

.. _doxid-assembly_8h_1a242f74ec5ced3f9ce78db74128079f4f:
.. _cid-s2_assembly_printbodies:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_assembly_printBodies (
	    FILE* f,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly
	    )

print bodies in a :ref:`s2_assembly_t <doxid-structs2__assembly__t>` datastructure



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - a :ref:`s2_assembly_t <doxid-structs2__assembly__t>` datastructure

.. _doxid-assembly_8h_1ad710ddb7229ab80512b157d007186bc6:
.. _cid-s2_assembly_printjoints:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_assembly_printJoints (
	    FILE* f,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly
	    )

print joints in an Assembly datastructure



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - an Assembly datastructure

.. _doxid-assembly_8h_1a3bf5b1cbf1a77c47ebb09ae2614201a1:
.. _cid-s2_assembly_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_assembly_print (
	    FILE* f,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly
	    )

Recursively print bodies and joints in a STAR assembly datastructure using depth first tree traversal.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - a STAR Assembly datastructure

.. _doxid-assembly_8h_1ac011b887e7ab8c636abff8f8b47351a9:
.. _cid-s2_assembly_appendbody:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_assembly_appendBody (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_body_t<doxid-structs2__body__t>`* body
	    )

Append a body to an :ref:`s2_assembly_t <doxid-structs2__assembly__t>` datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - an Assembly datastructure to append to

    *
        - body

        - a Body datastructure to append

.. _doxid-assembly_8h_1afd54b0d4e8baa8ec3ce7083e6dd4c955:
.. _cid-s2_assembly_appendpointcloud:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_assembly_appendPointCloud (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_pointcloud_t<doxid-structs2__pointcloud__t>`* pointCloud
	    )

Append a body to an :ref:`s2_assembly_t <doxid-structs2__assembly__t>` datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - an Assembly datastructure to append to

    *
        - body

        - a Body datastructure to append

.. _doxid-assembly_8h_1ad79e5a749aa80db79b357f8f0cc8239a:
.. _cid-s2_assembly_loadbodies:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_assembly_loadBodies (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    json_t* assemblyObject,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix
	    )

Load bodies belonging to assembly from json file.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - an :ref:`s2_assembly_t <doxid-structs2__assembly__t>` datastructure

    *
        - assemblyObject

        - a json_t data structure

    *
        - context

        - a Context datastructure

    *
        - prefix

        - a prefix specifying the full path to the body within assembly hierarchy

.. _doxid-assembly_8h_1a6db4eacbea9586e671b95c0000b0f3d4:
.. _cid-s2_assembly_loadjoints:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_assembly_loadJoints (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`*,
	    json_t*,
	    :ref:`s2_context_t<doxid-structs2__context__t>`*,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`*
	    )

Parse joints in a star2 file.

.. _doxid-assembly_8h_1a8f014c06474c3333fc560f169afe7444:
.. _cid-s2_assembly_loadlights:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_assembly_loadLights (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`*,
	    json_t*,
	    :ref:`s2_context_t<doxid-structs2__context__t>`*,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`*
	    )

used to load lights from json data file

.. _doxid-assembly_8h_1a786317fdaf0db57437ab625186ff4fd3:
.. _cid-s2_assembly_loadrestraints:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_assembly_loadRestraints (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`*,
	    json_t*,
	    :ref:`s2_context_t<doxid-structs2__context__t>`*,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`*
	    )

Parse restraints in a JSON assembly.

.. _doxid-assembly_8h_1a7ca9d117d740f43feef007f7e7f77635:
.. _cid-s2_assembly_loadforcemodules:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_assembly_loadForceModules (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    json_t* assemblyObject,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix
	    )

load force modules to an assembly data structure

.. _doxid-assembly_8h_1ab0365a12ee4d6173a86931afabc8e623:
.. _cid-s2_assembly_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_assembly_free (:ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly)

Free an Assembly datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - an Assembly datastructure

.. _doxid-assembly_8h_1a432a0fa719afdd10490a42215e63f050:
.. _cid-s2_assembly_appendjoint:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_assembly_appendJoint (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_joint_t<doxid-structs2__joint__t>`* joint
	    )

Append Joint datastructure to Assembly datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - an Assembly datastructure to append to

    *
        - joint

        - a Joint datastructure to append

.. _doxid-assembly_8h_1a07d90382ad4fadb47f4e44f3386221f5:
.. _cid-s2_assembly_appendforcemodule:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_assembly_appendForceModule (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
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

.. _doxid-assembly_8h_1aa0263f8de9d65f8ade74e22e0d9dcef2:
.. _cid-s2_assembly_appendrestraint:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_assembly_appendRestraint (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_restraint_t<doxid-structs2__restraint__t>`* restraint
	    )

Append a Restraint datastructure to an Assembly.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - a pointer to an Assembly datastructure

    *
        - restraint

        - Restraint datastructure to append

.. _doxid-assembly_8h_1a6dc022a707b6a4d3dc5e6b096fdf59c3:
.. _cid-s2_assembly_appendlight:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_assembly_appendLight (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_light_t<doxid-structs2__light__t>`* light
	    )

Append a Light datastructure to an assembly datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - body

        - an assembly datastructure to append to

    *
        - light

        - a Light datastructure to append

.. _doxid-assembly_8h_1a3b733cb2b82be6039e9597c23679030c:
.. _cid-s2_assembly_load:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_assembly_load (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* parentAssembly,
	    json_t* assembliesObject,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix
	    )

Recursively load assemblies from JSON file.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - assembly

        - An assembly to append subassemblies to

    *
        - assemblyObject

        - a json_t assembly object

    *
        - context

        - a Context datastructure

    *
        - prefix

        - a prefix specifying the full path to the body, joint or restraint within a hierarchical datastructure

.. _doxid-body_8h_1a1908ef354ca5400654bd3c77f1e2f510:
.. _cid-s2_body_appendframe:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_body_appendFrame (
	    :ref:`s2_body_t<doxid-structs2__body__t>`* body,
	    :ref:`s2_frame_t<doxid-structs2__frame__t>`* frame
	    )

Append a frame datastructure to an :ref:`s2_body_t <doxid-structs2__body__t>` datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - body

        - a Body datastructure to append to

    *
        - frame

        - a Frame datastructure to append

.. _doxid-body_8h_1a69b43a33ebea29118334a71134bfd08d:
.. _cid-s2_body:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* s2_body (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name)

Initialize an :ref:`s2_body_t <doxid-structs2__body__t>` datastructure. Appends the origin frame to the body frame.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - None

        - 



.. rubric:: Returns:

a :ref:`s2_body_t <doxid-structs2__body__t>` datastructure

.. _doxid-body_8h_1ad3e18bb16e07d20fe281003166a6f2de:
.. _cid-s2_body_printframes:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_body_printFrames (
	    FILE* f,
	    :ref:`s2_body_t<doxid-structs2__body__t>`* body
	    )

print frames in a :ref:`s2_body_t <doxid-structs2__body__t>` datastructure



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - body

        - a Body datastructure

.. _doxid-body_8h_1aae947d6a1b823f553cff5511ce8cc1e4:
.. _cid-s2_body_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_body_free (:ref:`s2_body_t<doxid-structs2__body__t>`* body)

Free a :ref:`s2_body_t <doxid-structs2__body__t>` datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - body

        - an :ref:`s2_body_t <doxid-structs2__body__t>` datastructure

.. _doxid-body_8h_1a308059645984b8f926d8884550112c4b:
.. _cid-s2_virtualbody_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_virtualbody_free (:ref:`s2_body_t<doxid-structs2__body__t>`* body)

Free a virtual :ref:`s2_body_t <doxid-structs2__body__t>` datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - body

        - a virtual Body datastructure to free

.. _doxid-body_8h_1aa555276cbc6bd3d4be78fc916dbd2ae6:
.. _cid-s2_body_load:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_body_load (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix,
	    json_t* bodyObject
	    )

load body to an assembly data structure



.. rubric:: Returns:

status: 0 indicates that body geometry was successfully loaded, -1 indicates failure in parsing geometry file, -2 indicates no body geometry was found, -3 indicates body is inactive and nothing was loaded

.. _doxid-body_8h_1a9798b3af24177e61f86edbcebcf45b1c:
.. _cid-s2_body_loadgeometry:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_body_loadGeometry (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_body_t<doxid-structs2__body__t>`* body
	    )

Loads geometry specified in Body properties from an external file using ASSIMP (open asset importer)



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - a Context datastructure

    *
        - body

        - a Body datastructure

.. _doxid-camera_8h_1a64efe9dea50e18dc2beff14bb7f19146:
.. _cid-s2_camera_perspective:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_camera_perspective (
	    float fieldOfView,
	    float aspectRatio,
	    float nearClippingPlane,
	    float farClippingPlane,
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* perspective
	    )

used to construct a perspective projection matrix

.. _doxid-chordconstraint_8h_1ad0d4dded0b142bb4c2fd7fff1a013fac:
.. _cid-s2_chordconstraint:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_chordconstraint_t<doxid-structs2__chordconstraint__t>`* s2_chordconstraint ()

Initialize a ChordConstraint.



.. rubric:: Returns:

a ChordConstraint datastructure

.. _doxid-context_8h_1a4c52af26a32a4fdf782380f2dd08295c:
.. _cid-s2_context_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_context_free (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

Free a Context datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - a Context datastructure

.. _doxid-context_8h_1a01523c759bd36b2a45c19c5ecde20290:
.. _cid-s2_context_freeruntimearrays:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_context_freeRunTimeArrays (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

Free run time arrays in a Context datastructure.

Copy values back to original Body, Joint and Frame datastructures and free run time arrays.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - an :ref:`s2_context_t <doxid-structs2__context__t>` datastructure

.. _doxid-context_8h_1aeda8ef92fcd0d59bfab3433fbffcf8b1:
.. _cid-s2_context_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_context_print (
	    FILE* f,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

Print a Context datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - a Context datastructure

.. _doxid-context_8h_1aa53fa7833e02e071bca07d9500361b8f:
.. _cid-s2_context_getframebyname:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* s2_context_getFrameByName (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* bodyName,
	    const char* frameName
	    )

get a frame pointer by specifying root body name and root frame name



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - data structure

    *
        - root

        - descriptive body name as specified in json file

    *
        - root

        - descriptive frame name as specified in json file



.. rubric:: Returns:

frame pointer, returns NULL if none found

.. _doxid-context_8h_1acd24505aabdde47892e9132c57852ce4:
.. _cid-s2_context_getbodybyname:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_body_t<doxid-structs2__body__t>`* s2_context_getBodyByName (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* bodyName
	    )

get a body pointer by specifying root body name, matches first body with root name match searching through all assemblies



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - data structure

    *
        - root

        - descriptive body name as specified in json file

    *
        - root

        - descriptive frame name as specified in json file



.. rubric:: Returns:

frame pointer, returns NULL if none found

.. _doxid-context_8h_1a7f76ce795233b12fd3e19c71792d343d:
.. _cid-s2_context_geticonv:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_context_getIConv (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

get current iconv value from numerical integrator

.. _doxid-context_8h_1abda3e5f9e4d0980f0ec28f0a03e293c9:
.. _cid-s2_context_getsimulationtime:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_context_getSimulationTime (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

get current simulation time

.. _doxid-context_8h_1a4ac077775d53b94844abf4caa49a5183:
.. _cid-s2_context_getdebugpointer:
.. ref-code-block:: cpp
	:class: title-code-block

	FILE* s2_context_getDebugPointer (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int i
	    )

get a pointer to debug file pointer

.. _doxid-context_8h_1a9223bf1c2c8bc9940295e15ab4864ce7:
.. _cid-s2_context_setomegan:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_context_setOmegaN (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double omegaN
	    )

used to set omegaN for baumgarte constraint stabilization

.. _doxid-context_8h_1aaa27470dfd5dda04503732ed5a5daff5:
.. _cid-s2_context_loadfixedframes:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_context_loadFixedFrames (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    json_t* root
	    )

loads fixed frames into an :ref:`s2_context_t <doxid-structs2__context__t>` datastructure

.. _doxid-context_8h_1a4dde91f880510bd9d291322d5a4c63ff:
.. _cid-s2_context_loadmodelparameters:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_context_loadModelParameters (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    json_t* modelParametersObject,
	    const char* filePath
	    )

Load model parameters from a star2 JSON file.

.. _doxid-context_8h_1a3a5f3b2fd5c74a7a17f53009aabe422e:
.. _cid-s2_context_freemodelparameters:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_context_freeModelParameters (:ref:`s2_context_t<doxid-structs2__context__t>`*)

free memory associated with modelParameters substructure

.. _doxid-context_8h_1a0edbff197c77bdc73ddb6eb607f39666:
.. _cid-s2_context_load:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_context_t<doxid-structs2__context__t>`* s2_context_load (const char* fileName)

Parse a STAR2 JSON file and return in "context", a SetupContext data structure.

This function parses a STAR2 JSON file and returns a SetupContext data structure. The JSON file is parsed using the Jansson C library.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - fileName

        - path to star2 JSON file.



.. rubric:: Returns:

a setupContext data structure.

.. _doxid-context_8h_1aabf0a90e7b6c0c728fb97ec0f0d54ed4:
.. _cid-s2_context_setterrainpointers:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_context_setTerrainPointers (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

Sets parent frame and body pointers for terrains.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - an :ref:`s2_context_t <doxid-structs2__context__t>` datastructure

.. _doxid-context_8h_1a1377a99a9de6a8736510a704ca1aa76b:
.. _cid-s2_context_updateboundingspheres:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_context_updateBoundingSpheres (:ref:`s2_context_t<doxid-structs2__context__t>`*)

update global centroid of bounding spheres

.. _doxid-convexhull_8h_1af459673607b4420cd938a11b7e530f52:
.. _cid-s2_convexhull_qhull:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_convexhull_qhull (:ref:`s2_body_t<doxid-structs2__body__t>`* body)

Compute single convex hull for :ref:`s2_body_t <doxid-structs2__body__t>` datastructure convex hull encompasses all submeshes associated with a body.

code derived from Qhull template at https://github.com/qhull/qhull

Currently, only one convex hull is generated to encompass body using the qhull algorithm. Later, this could be modified to fit a number of conformal convex hulls with adjustable level of detail using the v-hacd algorithm or similar.

.. _doxid-equation_8h_1ae07157bcd53cf5af2ef36335316cfb1c:
.. _cid-s2_equation_initializeequations:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_equation_initializeEquations (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

initializes quantities needed to solve system of equations

.. _doxid-equation_8h_1a7a41bd12faba6d99ea43161e63433d52:
.. _cid-s2_equation_setequations:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_equation_setEquations (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

evaluates Jacobians in PO2, factors Jacobian in PO2, sets mass matrix and RHS in PO3.

.. _doxid-equation_8h_1a75684b0d4d92ff58217623e3aab71f87:
.. _cid-s2_equation_factorandsolve:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_equation_factorAndSolve (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

factor and solve a system of equations using source and destination pointer arrays set up in ``memoize.c``

This algorithm is intended to be called after s2_equation_memoize()

.. _doxid-equation_8h_1a4cc978da410a907b2c46d5d1883bd54b:
.. _cid-s2_equation_setmassmatrix:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_equation_setMassMatrix (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

evaluate generalized mass matrix

.. _doxid-equation_8h_1aa166c8d96fcdf1bed248a28e374bd44a:
.. _cid-s2_equation_setrhs:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_equation_setRHS (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

evaluate right hand side of equations of motion Step 1 is carried out in PO1

f^{l} = f + H^T * C^{-T} (g + {v}^T(MV) - M )

"aq" (accelerations that are quadratic in velocity) is equivalent to "\gamma"

setRHS must be called before setMassMatrix

.. _doxid-equation_8h_1a8e5464e4ddb188f92ddb5d7132ac086f:
.. _cid-s2_equation_dq_to_dy:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_equation_dq_to_dy (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double* dy
	    )

convert angular velocities to time derivative of quaternions copy by value other terms from dq to dY array

.. _doxid-factor_8h_1a56d3d8ca169b30b9d858540565e49baa:
.. _cid-s2_factor_lu:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_factor_LU (
	    int rowDim,
	    int colDim,
	    int rank,
	    double* A
	    )

LU Decomposition with no row or column pivoting.

factor LU factors a double precision matrix, A[rowDim][colDim], using no row or column pivoting. Only the square rank x colDim submatrix of matrix A will be processed.

The lower triangular (rank+1) by (rank+1) Lr matrix, except the unity diagonal, is stored below the diagonal in A[0:rank][0:rank]. The upper triangular (rank+1) by (rank+1) Ur matrix is stored on and above the diagonal in A[0:rank][0:rank].

Note that this minimal algorithm variation does not compute the product C=LR*inverse(Lr)



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - rowDim

        - [in] row dimension of matrix A.

    *
        - colDim

        - [in] column dimension of matrix A.

    *
        - rank

        - [in] only the square rank x colDim submatrix of A will be processed

    *
        - *A

        - [in/out] matrix to be factored with dimensions A[0:rowDim-1][0:colDim-1]:

.. _doxid-factor_8h_1a1c08ca16f2b8caff47da29e37acfbd7e:
.. _cid-s2_factor_lu_full:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_factor_LU_full (
	    int rowDim,
	    int colDim,
	    double* A,
	    double eps,
	    int* pr,
	    int* pc
	    )

LU Decomposition with complete row and column pivoting.

mbdl)factor_LU_full factors a double precision matrix, A[rowDim * colDim], stored in row major order using full row and column pivoting. Matrix A need not have full row or column rank. The integer variable 'rank' represents the matrix rank and internally is always one less than the true rank to be consistent with C's indexing from zero. Upon return from the function, rank will be set to the correct value.

The lower triangular (rank+1) by (rank+1) Lr matrix, except the unity diagonal, is stored below the diagonal in A[0:rank][0:rank]. The upper triangular (rank+1) by (rank+1) Ur matrix is stored on and above the diagonal in A[0:rank][0:rank].

Following the first major matrix decomposition step, A[0:rank][rank+1:colDim] stores the residual matrix UR and A[rank+1:rowDim][0:rank] stores the residual matrix LR. The product C=LR*inverse(Lr) is then computed and stored back in A[rank+1:rowDim][0:rank] and the product B=-inverse(Ur)*UR is computed and stored back in A[0:rank][rank+1:colDim].

The remaining A[rank+1:rowDim][rank+1:colDim] submatrix contains numbers whose absolute values are all smaller than eps times the absolutely largest entry in the original matrix. This part of the matrix is not used.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - rowDim

        - [in] row dimension of matrix A.

    *
        - colDim

        - [in] column dimension of matrix A.

    *
        - *A

        - [in/out] matrix to be factored with dimensions A[0:rowDim-1][0:colDim-1], LU factors are overwritten in original matrix

    *
        - eps

        - [in] input tolerance on the order of machine roundoff.

    *
        - pr

        - [out] output row permutation array with dimensions pr[0:rowDim-1]

    *
        - pc

        - [out] output column permutation array with dimensions pc[0:colDim-1]



.. rubric:: Returns:

rank of matrix A

.. _doxid-factor_8h_1a0c54277861282aab80c6b5fd5067256a:
.. _cid-s2_factor_lu_full_specified:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_factor_LU_full_specified (
	    int rowDim,
	    int colDim,
	    double* A,
	    double eps,
	    int* pr,
	    int* pc,
	    int specifiedCoordinate
	    )

LU Decomposition with complete row and column pivoting with one specified coordinate.

s2_factor_LU_full_specified factors a double precision matrix, A[rowDim * colDim], stored in row major order using full row and column pivoting. Matrix A need not have full row or column rank. The integer variable 'rank' represents the matrix rank and internally is always one less than the true rank to be consistent with C's indexing from zero. Upon return from the function, rank will be set to the correct value.

The lower triangular (rank+1) by (rank+1) Lr matrix, except the unity diagonal, is stored below the diagonal in A[0:rank][0:rank]. The upper triangular (rank+1) by (rank+1) Ur matrix is stored on and above the diagonal in A[0:rank][0:rank].

Following the first major matrix decomposition step, A[0:rank][rank+1:colDim] stores the residual matrix UR and A[rank+1:rowDim][0:rank] stores the residual matrix LR. The product C=LR*inverse(Lr) is then computed and stored back in A[rank+1:rowDim][0:rank] and the product B=-inverse(Ur)*UR is computed and stored back in A[0:rank][rank+1:colDim].

The remaining A[rank+1:rowDim][rank+1:colDim] submatrix contains numbers whose absolute values are all smaller than eps times the absolutely largest entry in the original matrix. This part of the matrix is not used.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - rowDim

        - input row dimension of matrix A.

    *
        - colDim

        - input column dimension of matrix A.

    *
        - **A

        - input/output matrix to be factored with dimensions A[0:rowDim-1][0:colDim-1]:

    *
        - eps

        - input tolerance on the order of machine roundoff.

    *
        - pr

        - output row permutation array with dimensions pr[0:rowDim-1]

    *
        - pc

        - output column permutation array with dimensions pc[0:colDim-1]



.. rubric:: Returns:

rank of matrix A

.. _doxid-factor_8h_1a35e3120d7d178d4002edbbd5296a1f66:
.. _cid-s2_factor_setbmatrix:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_factor_setBMatrix (
	    int rowDim,
	    int colDim,
	    int rank,
	    double* A
	    )

compute B matrix (inverse(Ur) * UR)

This block overwrites UR with inverse(Ur) * UR If rank == 0, nothing to process. This loop computes A[0:rank][rank+1:colDim-1] = inverse(Ur)*UR where UR is stored in A[0:rank][rank+1:colDim-1] and Ur is stored in upper part of A[0:rank][0:rank]. i is the row number in Ur. j is the column number in Ur & row number in UR. k is the column number in UR.

.. _doxid-factor_8h_1a34e822650cfcb87252241777792f769e:
.. _cid-s2_factor_setcmatrix:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_factor_setCMatrix (
	    int rowDim,
	    int colDim,
	    int rank,
	    double* A
	    )

compute C matrix

This block overwrites LR with LR*inverse(Lr) Here rank>0 means true rank is greater than 1. If rank==0, the Lr matrix is a 1 by 1 identity matrix, so there is nothing to do here. This loop computes A[rank+1:rowDim-1][0:rank]=LR*inverse(Lr) where LR is stored in A[rank+1:rowDim-1][0:rank] and Lr is stored in A[0:rank][0:rank]. j is the column number in LR. It ends at 1 because the diagonal entry in row 0 of Lr is 1. i is the row number in LR. k is the column number in Lr

.. _doxid-frame_8h_1ac05a372808decdafe5796ba49a709758:
.. _cid-s2_frame:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_frame_t<doxid-structs2__frame__t>`* s2_frame ()

Initialize an :ref:`s2_frame_t <doxid-structs2__frame__t>` datastructure and set to identity.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - None

        - 



.. rubric:: Returns:

an initialized :ref:`s2_frame_t <doxid-structs2__frame__t>` datastructure

.. _doxid-frame_8h_1a213482cc6c0e6fa2d46302da5e173dc3:
.. _cid-s2_frame_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_frame_free (:ref:`s2_frame_t<doxid-structs2__frame__t>`* frame)

Free a :ref:`s2_frame_t <doxid-structs2__frame__t>` datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - frame

        - an :ref:`s2_frame_t <doxid-structs2__frame__t>` datastructure

.. _doxid-frame_8h_1a6d9a2da53ebca90b5523e54b78e512c0:
.. _cid-s2_frame_load:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_frame_load (
	    :ref:`s2_body_t<doxid-structs2__body__t>`*,
	    json_t*
	    )

load frame from json file

.. _doxid-graph_8h_1ab87c483575dfc67a291ea82a7dbae7f0:
.. _cid-s2_graph_digraph:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_graph_digraph (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* fileName
	    )

Generate a directed graph using graphviz api library.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - setup

        - a pointer to a Context data structure.

.. _doxid-graph_8h_1a75be41e869aa7af837ea3b8080ad24ad:
.. _cid-s2_graph_digraphframes:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_graph_digraphFrames (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* fileName
	    )

Generate a directed graph using graphviz api library.

must be called after s2_setTopology

The graphviz library appears to leak memory



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - setup

        - a pointer to a Context data structure.

.. _doxid-inertia_8h_1a4064ffd99364407e01a3ee14c476b84e:
.. _cid-s2_inertia:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_inertia_t<doxid-structs2__inertia__t>`* s2_inertia ()

Initialize an Inertia datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - None

        - 



.. rubric:: Returns:

an Inertia datastructure

.. _doxid-inertia_8h_1a553a8b1b69118115e43a66b9ea803a04:
.. _cid-s2_context_initializeinertia:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_context_initializeInertia (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

initializes array of inertia matrices in level increasing order

if a body is connected to another body through a rigid joint, the child body's inertia properties are merged with the parent body

.. _doxid-inertia_8h_1ae13c1d0c1ce6928544980a50d9948820:
.. _cid-s2_inertia_rotate:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_inertia_rotate (
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia_i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r,
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia_o
	    )

rotate a centroidal inertia matrix, does not modify m{t} terms

.. _doxid-inertia_8h_1a67902d9bad5b566d73b272e519dd0bb4:
.. _cid-s2_inertia_transform:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_inertia_transform (
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia_i,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d,
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia_o
	    )

transform spatial centroidal inertia matrix to global reference frame

.. _doxid-inertia_8h_1ae62ec7e5a343f66ec0653c51dfef6f73:
.. _cid-s2_inertia_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_inertia_print (
	    FILE* f,
	    :ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia
	    )

print an inertia matrix

.. _doxid-inertia_8h_1a0facc7213f1b16bf84a9bb90770091c9:
.. _cid-s2_inertia_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_inertia_free (:ref:`s2_inertia_t<doxid-structs2__inertia__t>`* inertia)

Free an Inertia datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - inertia

        - Inertia datastructure to free

.. _doxid-jacobian_8h_1ab87a970d54b86f82c43b1cbd36299539:
.. _cid-s2_context_initializejacobian:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_context_initializeJacobian (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

initialize and set initial jacobian in PO1

initializes and set ups an initial jacobian in PO1

.. _doxid-jacobian_8h_1a5f56adbb7a9c2345080903ab52b6426d:
.. _cid-s2_jacobian_setpo2:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_setPO2 (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

evaluate permuted Jacobian matrix for all substructures in PO2

.. _doxid-jacobian_8h_1af0d9613c5bc5b4e454b05c2a7ee305a8:
.. _cid-s2_jacobian_setpo2_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_setPO2_substructure (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex
	    )

evaluate permuted Jacobian matrix for one substructure in PO2

note: does not zero out the jacobian matrix for the substructure. zeroing the Jacobian is performed in s2_jacobian_setPO2. If it is desired to set the substructure jacobian without setting other jacobians, the array should explicitly be zeroed before this function is called.

.. _doxid-jacobian_8h_1a9dc8928636426d9c7591bd8a0e76625d:
.. _cid-s2_jacobian_factorpo2:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_jacobian_factorPO2 (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

factors LU Decomposition with no row or column pivoting, stores preconditioned constraints in PO3

fullFactor LU factors a double precision matrix, A[rowDim][colDim], using no row or column pivoting. Only the square rank x rank submatrix of matrix A will be processed.

The lower triangular (rank+1) by (rank+1) Lr matrix, except the unity diagonal, is stored below the diagonal in A[0:rank][0:rank]. The upper triangular (rank+1) by (rank+1) Ur matrix is stored on and above the diagonal in A[0:rank][0:rank].

Note that this minimal algorithm variation does not compute the product C=LR*inverse(Lr) or B=-inverse(Ur)*UR.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - rowDim

        - input row dimension of matrix A.

    *
        - colDim

        - input column dimension of matrix A.

    *
        - only

        - the square rank x rank submatrix of A will be factored

    *
        - *A

        - input/output matrix to be factored with dimensions A[0:rowDim-1][0:colDim-1]:

.. _doxid-jacobian_8h_1a566e85ff55a63bb70e7cb353b2d64911:
.. _cid-s2_jacobian_factorpo2_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_factorPO2_substructure (:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure)

wrapper function for s2_factor_LU to evaluate the jacobian for a substructure that has been evaluated in PO2 order

.. _doxid-jacobian_8h_1a80877a6ec2cceaa3e67150dc1fcac7c8:
.. _cid-s2_jacobian_setpo1:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_setPO1 (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

set initial jacobian in permutation order 1 and perform an initial call to GCP using s2_factor_LU_full

a global Jacobian is stored in the linear array context->J a pointer to the local Jacobian is stored in context->substructure[i]->J

note the Jacobian is stored in row-major order

.. _doxid-jacobian_8h_1a52ddd6ba41fc771d6b9ab7cec530b567:
.. _cid-s2_jacobian_setpo1_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_setPO1_substructure (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex
	    )

set initial jacobian in permutation order 1 and perform an initial call to GCP using s2_factor_LU_full on the basis of one substructure

a global Jacobian is stored in the linear array context->J a pointer to the local Jacobian is stored in context->substructure[i]->J

the Jacobian is stored in row-major order

.. _doxid-jacobian_8h_1a4c6ac044c752da74b0d670db8429fe30:
.. _cid-s2_jacobian_setpo1specified_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_setPO1Specified_substructure (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex,
	    int specifiedCoordinateIndex
	    )

set initial jacobian in permutation order 1 and perform an initial call to GCP using s2_factor_LU_full on the basis of one substructure with one specified coordinate

note the Jacobian is stored in row-major order



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - [in] :ref:`s2_context_t <doxid-structs2__context__t>` data structure

    *
        - substructureIndex

        - [in] index of substructure

    *
        - dqIndex

        - [in] global dqIndex of specified joint

.. _doxid-jacobian_8h_1ae019e00518457225ca52702c5b292fc6:
.. _cid-s2_jacobian_evaluateconstraints:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_evaluateConstraints (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

evaluates all position level constraints

.. _doxid-jacobian_8h_1ab9876793ffa9e3ff25f6d32d9b2ff8b2:
.. _cid-s2_jacobian_evaluateconstraints_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_evaluateConstraints_substructure (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure
	    )

evaluate position level constraints for one substructure

.. _doxid-jacobian_8h_1a3ba961f68079a678bbffce3432d1ac59:
.. _cid-s2_jacobian_analyze:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_analyze (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

called after s2_context_initializeJacobian

.. _doxid-jacobian_8h_1ae6c32b68f805cc9caff14750d504ea1e:
.. _cid-s2_jacobian_pivotratio:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_jacobian_pivotRatio (
	    int colDim,
	    int rank,
	    double* J
	    )

pivot ratio, performed on a Jacobian that has been factored

returns the ratio of the largest to the smallest pivot

.. _doxid-jacobian_8h_1a856f0a3f1051e729b40885f13fea757b:
.. _cid-s2_jacobian_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_print (
	    FILE* f,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

used to print properties of substructure datastructures related to Jacobians

.. _doxid-jacobian_8h_1a44db4cd91a5f275669f417189207c1c7:
.. _cid-s2_jacobian_setbmatrix_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_setBMatrix_substructure (:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure)

wrapper function for s2_factor_setBMatrix for a substructure

.. _doxid-jacobian_8h_1ade8a1c811181a2447453d8b951bf7e19:
.. _cid-s2_jacobian_setbmatrix:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_setBMatrix (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

sets B matrix for all substructures

.. _doxid-jacobian_8h_1a33e15659177517d6190488cf881a125d:
.. _cid-s2_jacobian_setbmatrixsparse_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_setBMatrixSparse_substructure (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure
	    )

sparse version of set B matrix evaluates B matrix into compressed b3 array in PO3, to align with the permutation order of the coefficient matrix. Evaluates infinityNormB.

to be called after evaluate sparsity

.. _doxid-jacobian_8h_1a563619b6d2834efe63d48d75d95e9c08:
.. _cid-s2_jacobian_setbmatrixsparse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_setBMatrixSparse (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

sparse version of set B matrix evaluates B matrix into compressed b3 array in PO3 for all substructures

to be called after evaluate sparsity

.. _doxid-jacobian_8h_1a290c638eabbd342efabc656f759026eb:
.. _cid-s2_jacobian_infinitynormbcheck:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_jacobian_infinityNormBCheck (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

check pivot ratios in all jacobians and call full factor if pivot ratio has changed a specified threshold from initial starting value

.. _doxid-jacobian_8h_1ad9869ddf4f2cad3987cf8ae69c2a71c6:
.. _cid-s2_jacobian_setjacobianbooleanpo2:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_setJacobianBooleanPO2 (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

set jacobian boolean matrix for all substructures in PO2

.. _doxid-jacobian_8h_1a53544ee4ae9566aa65f1d6d67490543e:
.. _cid-s2_jacobian_setjacobianbooleanpo2_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_jacobian_setJacobianBooleanPO2_substructure (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex
	    )

evaluate permuted Jacobian boolean matrix in PO2 for a single substructure

.. _doxid-jacobian_8h_1ae984f4f20342d6f76e43b0512d0a7d12:
.. _cid-s2_jacobian_infinitynormb:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_jacobian_infinityNormB (
	    int rowDim,
	    int colDim,
	    int rank,
	    double* J
	    )

returns infinity norm of B matrix

.. _doxid-joint_8h_1aa6f268a4ced5d9664eb611bda2e728a1:
.. _cid-s2_joint:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_joint_t<doxid-structs2__joint__t>`* s2_joint (
	    const char* jointTypeStr,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* name
	    )

Initialize a Joint datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - None

        - 



.. rubric:: Returns:

a Joint datastructure

.. _doxid-joint_8h_1ad22141c13a6e6084fe0b5def5633f80d:
.. _cid-s2_getjointtype:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_joint_enum_t<doxid-joint_8h_1a7a43c175bb516fb74df6eb8503a0475c>` s2_getJointType (const char* jointTypeStr)

Converts jointType descriptive name to JOINTTYPE enum.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - jointType

        - descriptive name



.. rubric:: Returns:

JOINTTYPE enum

.. _doxid-joint_8h_1a313d59dabeaf3a9ad13134e00f026a81:
.. _cid-s2_joint_tx:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_tx (void** dhva)

Prismatic joint along x-axis (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dPrimitive = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1ad46544b7618de87c8bedd8067512be97:
.. _cid-s2_joint_txinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_txinverse (void** dhva)

Prismatic Joint aligned with x-axis - inverse (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dPrimitive = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a3b95f1c8b5bb0238a0c794b357395f7e:
.. _cid-s2_joint_ty:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_ty (void** dhva)

Prismatic Joint aligned with y-axis (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dPrimitive = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a6e447de36e49f99467ecd4bb1a785692:
.. _cid-s2_joint_tyinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_tyinverse (void** dhva)

Prismatic Joint aligned with y-axis - inverse (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dPrimitive = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1ac49e1ec5e2c0cc15cdb9f0a8d687c64e:
.. _cid-s2_joint_tz:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_tz (void** dhva)

Prismatic Joint aligned with z-axis (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dPrimitive = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a1c80d10ed48a315bc5ac1a9366814143:
.. _cid-s2_joint_tzinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_tzinverse (void** dhva)

Prismatic Joint aligned with z-axis - inverse (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dPrimitive = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a00ee8658804edec0b274aee52659c422:
.. _cid-s2_joint_tx_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_tx_dhva (void** dhva)

Prismatic joint along x-axis (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations

The dhva array contains pointers to the following quantities: s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a7e57f436403fd50f59aa52fda2929b94:
.. _cid-s2_joint_txinverse_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_txinverse_dhva (void** dhva)

Prismatic joint along x-axis - inverse (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a89fc2603a56e0e18940e3574a500979d:
.. _cid-s2_joint_ty_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_ty_dhva (void** dhva)

Prismatic joint along y-axis (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a6aa5867308d5155f411b0177d249a4f8:
.. _cid-s2_joint_tyinverse_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_tyinverse_dhva (void** dhva)

Prismatic joint along y-axis - inverse (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1aaf4b8443458ae58835d9da18e2320b23:
.. _cid-s2_joint_tz_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_tz_dhva (void** dhva)

Prismatic joint along z-axis (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a15187216dd548906ef8aaf4eef26eb4e:
.. _cid-s2_joint_tzinverse_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_tzinverse_dhva (void** dhva)

Prismatic joint along z-axis - inverse (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1ac0bd6a131a3a2c3231fcfdfd3315148f:
.. _cid-s2_joint_rx:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rx (void** dhva)

Revolute Joint aligned with x-axis (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dc = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1af7f605ed6fa506af8b038ef9cc3c5b54:
.. _cid-s2_joint_rxinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rxinverse (void** dhva)

Revolute Joint aligned with x-axis - inverse (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dc = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a6f22c68683c564d9a375de830ab9bdf3:
.. _cid-s2_joint_ry:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_ry (void** dhva)

Revolute Joint aligned with y-axis (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dc = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a86e3fbb62ba22b44a85dbcda97066961:
.. _cid-s2_joint_ryinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_ryinverse (void** dhva)

Revolute Joint aligned with y-axis - inverse (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dc = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1ae010faf53791ce3aec07c16a8a863ccd:
.. _cid-s2_joint_rz:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rz (void** dhva)

Revolute Joint aligned with z-axis (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dc = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1ac883e86580e6ea41abe9da413932e51b:
.. _cid-s2_joint_rzinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rzinverse (void** dhva)

Revolute Joint aligned with z-axis - inverse (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
double* dc = dhva[1];
s2_displacement_t* dChild = dhva[2];
s2_screw_t* h = dhva[3];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a6a18edd5be1952467649a33743d32d31:
.. _cid-s2_joint_rx_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rx_dhva (void** dhva)

Revolute joint aligned with x-axis (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a5911e71a6b1d1cec7a89a9fa7daba519:
.. _cid-s2_joint_rxinverse_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rxinverse_dhva (void** dhva)

Revolute joint aligned with x-axis - inverse (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a6858c0763ed98fdf2e9a0b1c55e692a0:
.. _cid-s2_joint_ry_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_ry_dhva (void** dhva)

Revolute joint aligned with y-axis (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a6b71f3f1d355f959ae7acb6f49258a88:
.. _cid-s2_joint_ryinverse_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_ryinverse_dhva (void** dhva)

Revolute joint aligned with y-axis - inverse (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a2058cf68a422137e54cf55403034dcca:
.. _cid-s2_joint_rz_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rz_dhva (void** dhva)

Revolute joint aligned with z-axis (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a8e9f3ecba5a34780a3b0226a0170bfd4:
.. _cid-s2_joint_rzinverse_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rzinverse_dhva (void** dhva)

Revolute joint aligned with z-axis - inverse (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* h = dhva[3];

s2_screw_t* vParent = dhva[4];
double* vPrimitive = dhva[5];
s2_screw_t* vRelative = dhva[6];
s2_screw_t* vChild = dhva[7];

s2_screw_t* aqParent = dhva[8];
s2_screw_t* aqRelative = dhva[9];
s2_screw_t* aqChild = dhva[10];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1ac9ab6d4a5b7dd35fdf2d7f49b9d319f6:
.. _cid-s2_joint_rigid:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rigid (void** dhva)

Rigid Joint (dh)

Updates displacement and influence coefficient matrices Displacement dChild = dhva[0];

Displacement dParent = dhva[1];

Displacement dPrimitive = dhva[2];

double* dc = dhva[3];

s2_screw_t* h = dhva[4];

s2_screw_t* vParent = dhva[5];

s2_screw_t* vChild = dhva[6];

double* vPrimitive = dhva[7];

s2_screw_t* vRelative = dhva[8];

s2_screw_t* aqChild = dhva[9];

s2_screw_t* aqParent = dhva[10];

s2_screw_t* aqRelative = dhva[11];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a0b4dad9c13fa3502289d4330120c80bc:
.. _cid-s2_joint_rigidinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rigidinverse (void** dhva)

Rigid Joint - inverse (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];
s2_displacement_t* dPrimitive = dhva[1];
s2_displacement_t* dChild = dhva[2];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a2308146d104dda09c52c4d44d49a17d6:
.. _cid-s2_joint_rigid_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rigid_dhva (void** dhva)

Rigid Joint (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* vParent = dhva[3];
s2_screw_t* vChild = dhva[4];
s2_screw_t* aqChild = dhva[5];
s2_screw_t* aqParent = dhva[6];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a44450c1ab097e39c84dfd413af512ca2:
.. _cid-s2_joint_rigidinverse_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_rigidinverse_dhva (void** dhva)

Rigid Joint - inverse (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_screw_t* vParent = dhva[3];
s2_screw_t* vChild = dhva[4];
s2_screw_t* aqChild = dhva[5];
s2_screw_t* aqParent = dhva[6];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a1f080e6ccd539171410ef72cac993d82:
.. _cid-s2_joint_spherical:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_spherical (void** dhva)

Spherical Joint (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];

s2_displacement_t* dPrimitive = dhva[1];

s2_displacement_t* dWRTReference = dhva[2];

s2_displacement_t* dChild = dhva[3];

s2_screw_t* h = dhva[4];

s2_screw_t* vParent = dhva[5];

s2_screw_t* vPrimitive = dhva[6];

s2_screw_t* vRelative = dhva[7];

s2_screw_t* vChild = dhva[8];

s2_screw_t* aqParent = dhva[9];

s2_screw_t* aqRelative = dhva[10];

s2_screw_t* aqChild = dhva[11];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a3fd780c6194d664a93671c0438ce1681:
.. _cid-s2_joint_sphericalinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_sphericalinverse (void** dhva)

Float Joint - inverse (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];

s2_displacement_t* dPrimitive = dhva[1];

s2_displacement_t* dWRTReference = dhva[2];

s2_displacement_t* dChild = dhva[3];

s2_screw_t* h = dhva[4];

s2_screw_t* vParent = dhva[5];

s2_screw_t* vPrimitive = dhva[6];

s2_screw_t* vRelative = dhva[7];

s2_screw_t* vChild = dhva[8];

s2_screw_t* aqParent = dhva[9];

s2_screw_t* aqRelative = dhva[10];

s2_screw_t* aqChild = dhva[11];

Todo add sphericalinverse



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1af81dd6149e55911590071c65e1311faf:
.. _cid-s2_joint_spherical_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_spherical_dhva (void** dhva)

Float Joint (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_displacement_t* dParent = dhva[0];

s2_displacement_t* dPrimitive = dhva[1];

s2_displacement_t* dWRTReference = dhva[2];

s2_displacement_t* dChild = dhva[3];

s2_screw_t* h = dhva[4];

s2_screw_t* vParent = dhva[5];

s2_screw_t* vPrimitive = dhva[6];

s2_screw_t* vRelative = dhva[7];

s2_screw_t* vChild = dhva[8];

s2_screw_t* aqParent = dhva[9];

s2_screw_t* aqRelative = dhva[10];

s2_screw_t* aqChild = dhva[11];

Todo add spherical joint



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a937de4d7bd4045dac874baaa2bd6e336:
.. _cid-s2_joint_sphericalinverse_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_sphericalinverse_dhva (void** dhva)

Float Joint - inverse (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_displacement_t* dParent = dhva[0];

s2_displacement_t* dPrimitive = dhva[1];

s2_displacement_t* dWRTReference = dhva[2];

s2_displacement_t* dChild = dhva[3];

s2_screw_t* h = dhva[4];

s2_screw_t* vParent = dhva[5];

s2_screw_t* vPrimitive = dhva[6];

s2_screw_t* vRelative = dhva[7];

s2_screw_t* vChild = dhva[8];

s2_screw_t* aqParent = dhva[9];

s2_screw_t* aqRelative = dhva[10];

s2_screw_t* aqChild = dhva[11];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a2d976decccc57a1342ddcaa4f8165e1e:
.. _cid-s2_joint_float:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_float (void** dhva)

Rigid Joint (dh)

Updates displacement and influence coefficient matrices

s2_displacement_t* dParent = dhva[0];

s2_displacement_t* dPrimitive = dhva[1];

s2_displacement_t* dWRTReference = dhva[2];

s2_displacement_t* dChild = dhva[3];

s2_screw_t* h = dhva[4];

s2_screw_t* vParent = dhva[5];

s2_screw_t* vPrimitive = dhva[6];

s2_screw_t* vRelative = dhva[7];

s2_screw_t* vChild = dhva[8];

s2_screw_t* aqParent = dhva[9];

s2_screw_t* aqRelative = dhva[10];

s2_screw_t* aqChild = dhva[11];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1ae0b3f59aacc1124ae5a3ecf7523422a3:
.. _cid-s2_joint_floatinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_floatinverse (void** dhva)

Float Joint - inverse (dh)

Updates displacement and influence coefficient matrices s2_displacement_t* dParent = dhva[0];

s2_displacement_t* dPrimitive = dhva[1];

s2_displacement_t* dWRTReference = dhva[2];

s2_displacement_t* dChild = dhva[3];

s2_screw_t* h = dhva[4];

s2_screw_t* vParent = dhva[5];

s2_screw_t* vPrimitive = dhva[6];

s2_screw_t* vRelative = dhva[7];

s2_screw_t* vChild = dhva[8];

s2_screw_t* aqParent = dhva[9];

s2_screw_t* aqRelative = dhva[10];

s2_screw_t* aqChild = dhva[11];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1abfd8e31fc06dc2d24755b58e2c5564c9:
.. _cid-s2_joint_float_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_float_dhva (void** dhva)

Float Joint (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_displacement_t* dParent = dhva[0];

s2_displacement_t* dPrimitive = dhva[1];

s2_displacement_t* dWRTReference = dhva[2];

s2_displacement_t* dChild = dhva[3];

s2_screw_t* h = dhva[4];

s2_screw_t* vParent = dhva[5];

s2_screw_t* vPrimitive = dhva[6];

s2_screw_t* vRelative = dhva[7];

s2_screw_t* vChild = dhva[8];

s2_screw_t* aqParent = dhva[9];

s2_screw_t* aqRelative = dhva[10];

s2_screw_t* aqChild = dhva[11];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1a7ef38976486d6132770c16cf4e0485ae:
.. _cid-s2_joint_floatinverse_dhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_floatinverse_dhva (void** dhva)

Float Joint - inverse (dhva)

Updates displacement and influence coefficient matrices, velocities and accelerations s2_displacement_t* dParent = dhva[0];

s2_displacement_t* dPrimitive = dhva[1];

s2_displacement_t* dWRTReference = dhva[2];

s2_displacement_t* dChild = dhva[3];

s2_screw_t* h = dhva[4];

s2_screw_t* vParent = dhva[5];

s2_screw_t* vPrimitive = dhva[6];

s2_screw_t* vRelative = dhva[7];

s2_screw_t* vChild = dhva[8];

s2_screw_t* aqParent = dhva[9];

s2_screw_t* aqRelative = dhva[10];

s2_screw_t* aqChild = dhva[11];



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - dhva

        - an array of pointers to properties related to displacement, influence coefficient matrices, velocities, and accelerations.

.. _doxid-joint_8h_1aa7022ddeb2f5c5c469c6f4c22c515a55:
.. _cid-s2_joint_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_joint_free (:ref:`s2_joint_t<doxid-structs2__joint__t>`* joint)

free a :ref:`s2_joint_t <doxid-structs2__joint__t>` data structure

.. _doxid-joint_8h_1a00650f29fa56ff3d66bc96a03265a113:
.. _cid-s2_joint_load:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_joint_load (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix,
	    json_t* bodyObject
	    )

loads joint from star2 json file

.. _doxid-kdtree_8h_1a1151319024fef1ae2cba63f1dc3df4eb:
.. _cid-s2_kdtree:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_kdtree_t<doxid-structs2__kdtree__t>`* s2_kdtree (
	    int vertexCount,
	    double* vertices,
	    int faceCount,
	    int* faces,
	    uint32_t binSize
	    )

constructor for double precision :ref:`s2_kdtree_t <doxid-structs2__kdtree__t>` datastructure

.. _doxid-kdtree_8h_1a135cfea0db85264378e7e6651e3b217a:
.. _cid-s2_kdtree_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_kdtree_free (:ref:`s2_kdtree_t<doxid-structs2__kdtree__t>`*)

free memory for a :ref:`s2_kdtree_t <doxid-structs2__kdtree__t>` datastructure

.. _doxid-kinematics_8h_1add6f5c8455a0c42e9d98e5abc6c832de:
.. _cid-s2_vector:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vector_t<doxid-structs2__vector__t>`* s2_vector ()

s a vector_t datastructure (an array of doubles of length 3)

.. _doxid-kinematics_8h_1a504c37f6f2d787b2167848a008413f74:
.. _cid-s2_vectorf:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* s2_vectorf ()

s a vectorf_t datastructure (an array of floats of length 3)

.. _doxid-kinematics_8h_1ae9ccea35fa4785c758d9dcb907ad9e07:
.. _cid-s2_vectori:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_vectori_t<doxid-structs2__vectori__t>`* s2_vectori ()

s a vectori_t datastructure (an array of ints of length 3)

.. _doxid-kinematics_8h_1a749edbf573f292b057f81bee69e5e9d4:
.. _cid-s2_displacement:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* s2_displacement ()

initializes a general :ref:`s2_displacement_t <doxid-structs2__displacement__t>` datastructure

A general displacement is an array of doubles of length 12, consisting of a rotation matrix in column major order followed by a translation vector.



.. rubric:: Returns:

A displacement with the rotation component set to identity and translation set to zero.

.. _doxid-kinematics_8h_1a2ebd3f4328d754fd7186be75ce276bfb:
.. _cid-s2_displacementf:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* s2_displacementf ()

s a general :ref:`s2_displacementf_t <doxid-structs2__displacementf__t>` datastructure

A general displacement is an array of float of length 12, consisting of a rotation matrix in column major order followed by a translation vector

.. _doxid-kinematics_8h_1af5ad699cdb031af0a3d317b3cd71ce39:
.. _cid-s2_screw:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_screw_t<doxid-structs2__screw__t>`* s2_screw ()

s a :ref:`s2_screw_t <doxid-structs2__screw__t>` datastructure (an array of doubles of length 6)

.. _doxid-kinematics_8h_1a1452fca3cfc2fb7dbb4991450abee7a7:
.. _cid-s2_screwf:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2_screwf ()

s a :ref:`s2_screw_t <doxid-structs2__screw__t>` datastructure (an array of floats of length 6)

.. _doxid-kinematics_8h_1a36a6a8db2bc1bc46e6261071ff1a6b87:
.. _cid-s2_mat3x3:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* s2_mat3x3 ()

s a double precision :ref:`s2_mat3x3_t <doxid-structs2__mat3x3__t>` datastructure for a general rotation (i.e. a SO3 matrix). The matrix is specified in column major order.



.. rubric:: Returns:

A :ref:`s2_mat3x3_t <doxid-structs2__mat3x3__t>` rotation matrix set to identity.

.. _doxid-kinematics_8h_1a636c2728626c32de4e09fc16e8baac24:
.. _cid-s2_mat3x3f:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* s2_mat3x3f ()

s a single precision :ref:`s2_mat3x3f_t <doxid-structs2__mat3x3f__t>` datastructure for a general rotation (i.e. a SO3 matrix). The matrix is specified in column major order.



.. rubric:: Returns:

A :ref:`s2_mat3x3f_t <doxid-structs2__mat3x3f__t>` rotation matrix set to identity.

.. _doxid-kinematics_8h_1af2ed9d8c80c58af7e07cdb4bc3d7a3ea:
.. _cid-s2_mat4x4:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`* s2_mat4x4 ()

s a double precision s2_Mat4 datastructure for a general transformation (i.e. a SE3 matrix). The matrix is specified in column major order.



.. rubric:: Returns:

A s2_Mat4 SE3 matrix set to identity.

.. _doxid-kinematics_8h_1a8fe199e9f411819659f2ea5139dea917:
.. _cid-s2_mat4x4f:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* s2_mat4x4f ()

s a single precision s2_Mat4f datastructure for a general transformation (i.e. a SE3 matrix). The matrix is specified in column major order.



.. rubric:: Returns:

A s2_Mat4f SE3 matrix set to identity.

.. _doxid-kinematics_8h_1a62bda6da6588457acf90477d273077c9:
.. _cid-s2_displacement_tomat4x4:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacement_toMat4x4 (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d,
	    :ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`* m
	    )

converts a double precision :ref:`s2_displacement_t <doxid-structs2__displacement__t>` datastructure to s2_Mat4 format.

.. _doxid-kinematics_8h_1ad0ae207d0ad99dc99834ac62d33466ca:
.. _cid-s2_displacementf_tomat4x4f:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacementf_toMat4x4f (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* d,
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* m
	    )

converts a single precision :ref:`s2_displacementf_t <doxid-structs2__displacementf__t>` datastructure to s2_Mat4f format.

.. _doxid-kinematics_8h_1abae78348ad957e61cf237502c9576900:
.. _cid-s2_displacement_tomat4x4f:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacement_toMat4x4f (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d,
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* m
	    )

converts a double precision :ref:`s2_displacement_t <doxid-structs2__displacement__t>` datastructure to a single precision s2_Mat4f format.

.. _doxid-kinematics_8h_1a77d1698e6e0b0f2559f3dbac07d09d82:
.. _cid-s2_vector_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_vector_print (
	    FILE* f,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v
	    )

Utility to print a 3x1 column vector.

prints a 3x1 column vector as a row vector



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ti

        - 3x1 column vector

.. _doxid-kinematics_8h_1ab118fa4550fdec3303e3652d41cd57ea:
.. _cid-s2_screw_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screw_print (
	    FILE* f,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s
	    )

Utility to print a 6x1 screw vector.

prints a 6x1 column vector as a row vector



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - si

        - 6x1 column vector

.. _doxid-kinematics_8h_1a1f8bb044d159e83916496910de5b5037:
.. _cid-s2_mat3x3_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3_print (
	    FILE* f,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r
	    )

Utility to print a 3x3 rotation matrix.

prints a rotation matrix



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - 3x3 rotation matrix

.. _doxid-kinematics_8h_1a4633d76ec38ba232366bd64336faaa97:
.. _cid-s2_mat3x3f_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3f_print (
	    FILE* f,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r
	    )

Utility to print a 3x3 rotation matrix.

prints a rotation matrix



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - 3x3 rotation matrix

.. _doxid-kinematics_8h_1a417cd55d2af30994e91a339c659aafc6:
.. _cid-s2_displacement_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacement_print (
	    FILE* f,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d
	    )

Utility to print a general displacement, $D_i$ .

prints a general displacement as a 4x4 matrix for transforming homogeneous coordinates. Note that the displacement is stored in memory as a 12 x 1 contiguous array with terms of the rotation matrix in the first 9 entries of the array (in row major order), followed by translation terms in the last 3 entries of the array.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - A general displacement, $D_i$

.. _doxid-kinematics_8h_1ace214610c0710efe4c696826a06d27db:
.. _cid-s2_mat4x4_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat4x4_print (
	    FILE* f,
	    :ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`* d
	    )

Utility to print a SE3 mat4 matrix.

prints a s2_Mat4f as a 4x4 matrix for transforming homogeneous coordinates. Note that the displacement is stored in memory as a 12 x 1 contiguous array with terms of the rotation matrix in the first 9 entries of the array (in row major order), followed by translation terms in the last 3 entries of the array.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - A general displacement, $D_i$

.. _doxid-kinematics_8h_1ae57720ec83109c7e72d927d701ae997c:
.. _cid-s2_mat4x4f_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat4x4f_print (
	    FILE* f,
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* d
	    )

Utility to print a SE3 mat4 matrix.

prints a s2_Mat4f as a 4x4 matrix for transforming homogeneous coordinates. Note that the displacement is stored in memory as a 12 x 1 contiguous array with terms of the rotation matrix in the first 9 entries of the array (in row major order), followed by translation terms in the last 3 entries of the array.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - A general displacement, $D_i$

.. _doxid-kinematics_8h_1a8d23082a7f8cfe085867ac96ba845f82:
.. _cid-s2_array_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_array_print (
	    FILE* f,
	    int n,
	    double* a
	    )

utility to print a general array of double of length n on one line

.. _doxid-kinematics_8h_1a4cd08b1aa3089dfc3c21c8b6a6a63b2f:
.. _cid-s2_array_print2d:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_array_print2D (
	    FILE* f,
	    int rowDim,
	    int colDim,
	    bool isRowMajor,
	    double* a
	    )

utility to print a 2D array stored in a single pointer with dimension rowDim * colDim

.. _doxid-kinematics_8h_1a4c02a7a5bdd4d6c55937ff7917f14ee3:
.. _cid-s2_arrayi_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_arrayi_print (
	    FILE* f,
	    int n,
	    int* a
	    )

utility to print a general array of ints of length n on one line

.. _doxid-kinematics_8h_1adc90ce89806874fa4e1c5ea3cd9d92ce:
.. _cid-s2_triang_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_triang_print (
	    FILE* f,
	    int n,
	    double* a
	    )

utility to print a lower triangular matrix of doubles stored in a linear array

.. _doxid-kinematics_8h_1a633f455ee53f3b0f9b8b3c50d96640f7:
.. _cid-s2_triangi_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_triangi_print (
	    FILE* f,
	    int n,
	    int* a
	    )

utility to print a lower triangular matrix of integers stored in a linear array

.. _doxid-kinematics_8h_1ac0a79222986f7db5afc96034d56e2855:
.. _cid-s2_triangb_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_triangb_print (
	    FILE* f,
	    int n,
	    bool* a
	    )

utility to print a lower triangular matrix of integers stored in a linear array

.. _doxid-kinematics_8h_1ab8853d41657f078add6796d31d7d6b11:
.. _cid-s2_dc:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_dc (
	    double* thetai,
	    double dc [2]
	    )

Computes direction cosine and sine terms.

Computes direction cosine and sine terms, terms are saved to array dc



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - anglqi

        - angle, in radians

    *
        - dco

        - Array of cos, sin terms, $cos(theta_i) \rightarrow dc_o[0]$ , $sin(thetai) \rightarrow dc_o[1]$

.. _doxid-kinematics_8h_1a92b73b20758ea492170c1b793d97b2bf:
.. _cid-s2_displacement_setidentity:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacement_setIdentity (:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d)

Set an SE3 displacement to identity.

Overwrites an SE3 matrix with the identity matrix.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - D

        - an SE3 matrix to set to identity, the resulting matrix will be overwritten

.. _doxid-kinematics_8h_1acad5b10f882249ebf4ead0ae04d738ed:
.. _cid-s2_mat3x3_setidentity:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3_setIdentity (:ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r)

Set a rotation matrix to identity.

Overwrites a rotation matrix with the identity matrix.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - R

        - an SE3 matrix to set to identity, the resulting matrix will be overwritten

.. _doxid-kinematics_8h_1a7c2af5b65856c46e4717d2d915412d80:
.. _cid-s2_vector_set:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_vector_set (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v,
	    int index,
	    double value
	    )

set a value of vector

.. _doxid-kinematics_8h_1a4dc6664a5de6655f8a692fe9c581285e:
.. _cid-s2_screw_set:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screw_set (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s,
	    int index,
	    double value
	    )

set a value of a screw

.. _doxid-kinematics_8h_1ae832f876fe2c48c794069aefd22ecf47:
.. _cid-s2_vector_l2norm:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_vector_l2norm (:ref:`s2_vector_t<doxid-structs2__vector__t>`* v)

l2 norm (length) of double precision vector

.. _doxid-kinematics_8h_1a5944ee7b7d802a4c0496c4db30a57157:
.. _cid-s2_vectorf_l2norm:
.. ref-code-block:: cpp
	:class: title-code-block

	float s2_vectorf_l2norm (:ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v)

l2 norm (length) of single precision vector

.. _doxid-kinematics_8h_1a1b9b5c10cf53749a5ba1ae5cebdb0517:
.. _cid-s2_screw_l2norm:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_screw_l2norm (:ref:`s2_screw_t<doxid-structs2__screw__t>`* v)

l2 norm of double precision screw

.. _doxid-kinematics_8h_1a470337fffa0e9d8eb4d34afee1246a47:
.. _cid-s2_screwf_l2norm:
.. ref-code-block:: cpp
	:class: title-code-block

	float s2_screwf_l2norm (:ref:`s2_screwf_t<doxid-structs2__screwf__t>`* v)

l2 norm of single precision screw

.. _doxid-kinematics_8h_1a19cc24b3d8e509bc702bad718e9f1b76:
.. _cid-s2_mat3x3_set:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3_set (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* R,
	    int row,
	    int column,
	    double value
	    )

set a value of a mat3x3

note that mat3x3 is stored in column major order

.. _doxid-kinematics_8h_1a0ad504b914b0e429f0e70d5bed9b4e7f:
.. _cid-s2_mat3x3_setfromvectors:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3_setFromVectors (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* R,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* x,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* z
	    )

set a mat3x3 rotation matrix from three vectors

note that mat3x3 is stored in column major order

.. _doxid-kinematics_8h_1a71679fcf2d23747e870e5b94cf17c425:
.. _cid-s2_mat4x4_set:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat4x4_set (
	    :ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`* R,
	    int row,
	    int column,
	    double value
	    )

set a value of a mat3x3

note that mat3x3 is stored in column major order

.. _doxid-kinematics_8h_1abd7dff4462439ffd7ee662519b9fa1a5:
.. _cid-s2_displacement_set:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacement_set (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D,
	    int row,
	    int column,
	    double value
	    )

set a value of a displacement

.. _doxid-kinematics_8h_1a0efd3f2ad9d4d6653bed41c51f25b3a7:
.. _cid-s2_vectorf_set:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_vectorf_set (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v,
	    int index,
	    float value
	    )

set a value of vectorf

.. _doxid-kinematics_8h_1ab98b7c7e37e980079292ce25267cbb20:
.. _cid-s2_screwf_set:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwf_set (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s,
	    int index,
	    float value
	    )

set a value of a screwf

.. _doxid-kinematics_8h_1a3fa19b544e3e7ca42d2e3f32b56a9b72:
.. _cid-s2_mat3x3f_set:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3f_set (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* R,
	    int row,
	    int column,
	    float value
	    )

set a value of a mat3x3f

note that mat3x3 is stored in column major order

.. _doxid-kinematics_8h_1a0bc58ec94030763ef2881c09b021abf0:
.. _cid-s2_mat4x4f_set:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat4x4f_set (
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* d,
	    int row,
	    int column,
	    float value
	    )

set a value of a mat4x4f

note that mat3x3 is stored in column major order

.. _doxid-kinematics_8h_1a90be8306e6c85fd683efd37f3ed61815:
.. _cid-s2_mat4x4f_setidentity:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat4x4f_setIdentity (:ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* R)

Set a rotation matrix to identity.

Overwrites a rotation matrix with the identity matrix.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - R

        - an SE3 matrix to set to identity, the resulting matrix will be overwritten

.. _doxid-kinematics_8h_1aa7c2d7da2785bc887397f7fcd59fd282:
.. _cid-s2_displacementf_set:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacementf_set (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D,
	    int row,
	    int column,
	    float value
	    )

set a value of a displacementf

.. _doxid-kinematics_8h_1a5b6a920da91584e7140e607c25d730df:
.. _cid-s2_vector_get:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_vector_get (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v,
	    int index
	    )

get a value of vector

.. _doxid-kinematics_8h_1a427153d990fba6272ec0f6eb9007cc74:
.. _cid-s2_screw_get:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_screw_get (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s,
	    int index
	    )

set a value of a screw

.. _doxid-kinematics_8h_1a1d3c323bcf02daf7e42c4ea54003eb27:
.. _cid-s2_mat3x3_get:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_mat3x3_get (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* R,
	    int row,
	    int column
	    )

set a value of a mat3x3

note that mat3x3 is stored in column major order

.. _doxid-kinematics_8h_1ab88e75900469688ce2494cc17ddf3603:
.. _cid-s2_displacement_get:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_displacement_get (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D,
	    int row,
	    int column
	    )

set a value of a displacement

.. _doxid-kinematics_8h_1a1d746b7f2d16a03ddc4de3268b1f44ff:
.. _cid-s2_vectorf_get:
.. ref-code-block:: cpp
	:class: title-code-block

	float s2_vectorf_get (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v,
	    int index
	    )

set a value of vectorf

.. _doxid-kinematics_8h_1ab1d04de407dbe52ca4f1ed2cf1cb0b55:
.. _cid-s2_screwf_get:
.. ref-code-block:: cpp
	:class: title-code-block

	float s2_screwf_get (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s,
	    int index
	    )

set a value of a screwf

.. _doxid-kinematics_8h_1a0d56ee07fb89cd9cadbfbef9b5b0a6cb:
.. _cid-s2_mat3x3f_get:
.. ref-code-block:: cpp
	:class: title-code-block

	float s2_mat3x3f_get (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* R,
	    int row,
	    int column
	    )

set a value of a mat3x3f

note that mat3x3 is stored in column major order

.. _doxid-kinematics_8h_1a3e1e6b63dc0b10db2b05aebd0d2f0993:
.. _cid-s2_displacementf_get:
.. ref-code-block:: cpp
	:class: title-code-block

	float s2_displacementf_get (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D,
	    int row,
	    int column
	    )

set a value of a displacementf

.. _doxid-kinematics_8h_1a88255744f79759af23ef0f2ab6e93ac9:
.. _cid-s2_displacement_gettranslation:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacement_getTranslation (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* d,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t
	    )

get translation component of a general displacement

.. _doxid-kinematics_8h_1a17e06d126f8e9aa2072e881716a29dfd:
.. _cid-s2_vector_normalize:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_vector_normalize (:ref:`s2_vector_t<doxid-structs2__vector__t>`* v)

Normalize a double precision vector.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v

        - [in/out] vector to normalize

.. _doxid-kinematics_8h_1a05270e2288eac8f5068a6540f86cd4be:
.. _cid-s2_vectorf_normalize:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_vectorf_normalize (:ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v)

Normalize a double precision vector.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v

        - [in/out] vector to normalize

.. _doxid-kinematics_8h_1a3e4971a0f6c89474375f17f097b20e56:
.. _cid-s2_vector_scale:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_vector_scale (
	    double k,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v
	    )

scale a double precision vector



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - [in/out]

        - vector to scale

.. _doxid-kinematics_8h_1a235ee6a927e8c79675134a67333dc8a4:
.. _cid-s2_vectorf_scale:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_vectorf_scale (
	    double k,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v
	    )

scale a double precision vector



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - [in/out]

        - vector to scale

.. _doxid-kinematics_8h_1ab1ed5e4f30e6c0c4fcec2b80f1adab73:
.. _cid-s2_vector_copy:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_vector_copy (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Copy a double precision Vector vi into Vector vo.

$V_i \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi

        - input vector $V_i$

    *
        - vo

        - output vector $V_o$

.. _doxid-kinematics_8h_1afd5179d14d52b675f7d8bb74c2ab816e:
.. _cid-s2_vectorf_copy:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_vectorf_copy (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

Copy a single precision Vectorf vi into Vectorf vo.

$V_i \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi

        - input vector $V_i$

    *
        - vo

        - output vector $V_o$

.. _doxid-kinematics_8h_1a41990673712e599b887d6622d81dfeda:
.. _cid-s2_vector_invert:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_vector_invert (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Copy a double precision Vector vi into vo.

$-V_i \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi

        - input vector $V_i$

    *
        - vo

        - output vector $V_o$

.. _doxid-kinematics_8h_1a2ca1a64beb56e639f7e93e1c8452e598:
.. _cid-s2_vectorf_invert:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_vectorf_invert (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

Copy a single prevision Vectorf vi into vo.

$-V_i \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi

        - input vector $V_i$

    *
        - vo

        - output vector $V_o$

.. _doxid-kinematics_8h_1a2fe4d8a43696adfa539471aa1f6b9d84:
.. _cid-s2_dot_vv:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_dot_vv (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi0,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi1
	    )

double precision Dot product of two vectors

$ V_i0 \cdot V_i1 \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi0

        - first input vector

    *
        - vi1

        - second input vector

.. _doxid-kinematics_8h_1a0faf9d7a7b509dc1627b3e7266377d2e:
.. _cid-s2_dot_vvf:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_dot_vvf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi0,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi1
	    )

single precision Dot product of two vectors

$ V_i0 \cdot V_i1 \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi0

        - first input vector

    *
        - vi1

        - second input vector

.. _doxid-kinematics_8h_1a13c2371ecd04f5088a010169de8d8980:
.. _cid-s2_dot_ss:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_dot_ss (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* vi0,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* vi1
	    )

double precision Dot product of two screws

$ V_i0 \cdot V_i1 \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi0

        - first input vector

    *
        - vi1

        - second input vector

.. _doxid-kinematics_8h_1a30572a558bb7ca3b9ee9d81a2ff1dc27:
.. _cid-s2_dot_ssf:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_dot_ssf (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* vi0,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* vi1
	    )

single precision Dot product of two screws

$ V_i0 \cdot V_i1 \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi0

        - first input vector

    *
        - vi1

        - second input vector

.. _doxid-kinematics_8h_1ab3ccf15317ea6f6bacb614406b8bcc01:
.. _cid-s2_product_vk:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_vk (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    double* ki,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Computes the product of a vector and a scalar.

$V_i * k_i \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi[3]

        - A 3x1 input vector

    *
        - *ki

        - a pointer to a scalar

    *
        - vo[3]

        - the results are stored in vo[3]

.. _doxid-kinematics_8h_1aa1f876ac482c3889df9d271bf5e6794f:
.. _cid-s2_product_vinversek:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_vinversek (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    double* ki,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Computes the product of a vector and a 1/scalar.

$V_i * k_i \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi[3]

        - A 3x1 input vector

    *
        - *ki

        - a pointer to a scalar

    *
        - vo[3]

        - the results are stored in vo[3]

.. _doxid-kinematics_8h_1aa2c1b479099e4cea4fbdb55949eca2cd:
.. _cid-s2_product_vkinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_vkinverse (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    double* ki,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Computes the product of a vector and a 1/scalar.

$V_i * k_i \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi[3]

        - A 3x1 input vector

    *
        - *ki

        - a pointer to a scalar

    *
        - vo[3]

        - the results are stored in vo[3]

.. _doxid-kinematics_8h_1a3fbb7b15e1fe4c9be29a643e82fb8332:
.. _cid-s2_product_vkf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_vkf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    float* ki,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

Computes the product of a vector and a scalar.

$V_i * k_i \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - vi[3]

        - A 3x1 input vector

    *
        - *ki

        - a pointer to a scalar

    *
        - vo[3]

        - the results are stored in vo[3]

.. _doxid-kinematics_8h_1aba3cdfe09849b39f5360917b9facbe32:
.. _cid-s2_add_vv:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_vv (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Adds two double precision vectors.

Add 3 by 1 column matrices v1i and v2i and store results in vo.

$V_{1i} + V_{2i} \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first column vector input

    *
        - v2i

        - second column vector

    *
        - vo

        - the results of v1i + v2i are stored in vo

.. _doxid-kinematics_8h_1adb7ffdbb3ff0be7f553d6e98610594cc:
.. _cid-s2_add_vvf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_vvf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

Adds two single precision vectors.

Add 3 by 1 column matrices v1i and v2i and store results in vo.

$V_{1i} + V_{2i} \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first column vector input

    *
        - v2i

        - second column vector

    *
        - vo

        - the results of v1i + v2i are stored in vo

.. _doxid-kinematics_8h_1a955d368c1dd1653c1d98c0849866d2b8:
.. _cid-s2_addequal_vv:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_addequal_vv (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i
	    )

Adds two double precision vectors.

Add 3 by 1 column matrix v1i to v2i

$V_{2i} += V_{1i} $



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first column vector input

    *
        - v2i

        - second column vector

.. _doxid-kinematics_8h_1a3f26172696564595b05f90db8391184a:
.. _cid-s2_addequal_vvf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_addequal_vvf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i
	    )

Adds two single precision vectors.

Add 3 by 1 column matrix v1i to v2i

$V_{2i} += V_{1i} $



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - [in] a first column vector input

    *
        - v2i

        - [in/out] second column vector

.. _doxid-kinematics_8h_1a622e5cd10e1bf0823800269a52ca5e28:
.. _cid-s2_subtract_vv:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_subtract_vv (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Subtract two double precision vectors.

Subtract 3 by 1 column matrix v2i from v1i and store results in vo.

$V_{1i} - V_{2i} \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first 3x1 vector

    *
        - V21

        - a second 3x1 vector

    *
        - vo

        - the results of v1i - v2i are returned in vo

.. _doxid-kinematics_8h_1afd6809a4a30e3e85fc3be0e03d914aab:
.. _cid-s2_subtract_vvf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_subtract_vvf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

Subtract two single precision vectors.

Subtract 3 by 1 column matrix v2i from v1i and store results in vo.

$V_{1i} - V_{2i} \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first 3x1 vector

    *
        - V21

        - a second 3x1 vector

    *
        - vo

        - the results of v1i - v2i are returned in vo

.. _doxid-kinematics_8h_1a393fcae617b6ca7eed0eef5ec7dae0e3:
.. _cid-s2_subtractequal_vv:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_subtractequal_vv (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i
	    )

Adds two double precision vectors.

subtract 3 by 1 column matrix v1i from v2i

$V_{2i} -= V_{1i} $



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first column vector input

    *
        - v2i

        - second column vector

.. _doxid-kinematics_8h_1a743a6e60733bfdcade977e19722ea0a4:
.. _cid-s2_subtractequal_vvf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_subtractequal_vvf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i
	    )

Adds two single precision vectors.

Subtract 3 by 1 column matrix v1i from v2i

$V_{2i} -= V_{1i} $



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - [in] a first column vector input

    *
        - v2i

        - [in/out] second column vector

.. _doxid-kinematics_8h_1a8043d4d0f77534a9b641cc0c9bbc86b0:
.. _cid-s2_axpy_vv:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_axpy_vv (
	    double scalar,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1x,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2y
	    )

axpy (ax plus y stored in y) for double precision vectors

add 3 by 1 column matrix v1i scaled by k to v2i

$V_{2i} += k * V_{1i} $



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first column vector input

    *
        - v2i

        - second column vector

.. _doxid-kinematics_8h_1abdeb011fa1685891338636061af8f01e:
.. _cid-s2_axpy_vvf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_axpy_vvf (
	    float scalar,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1x,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2y
	    )

axpy (ax plus y stored in y) for single precision vectors

add 3 by 1 column matrix v1i scaled by k to v2i

$V_{2i} += k * V_{1i} $



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - [in] a first column vector input

    *
        - v2i

        - [in/out] second column vector

.. _doxid-kinematics_8h_1a39a54ebe31a49549ae65de7e2df3198b:
.. _cid-s2_axpy_ss:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_axpy_ss (
	    double scalar,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* v1x,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* v2y
	    )

axpy (ax plus y stored in y) for double precision screws

add 3 by 1 column matrix v1i scaled by k to v2i

$V_{2i} += k * V_{1i} $



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first column vector input

    *
        - v2i

        - second column vector

.. _doxid-kinematics_8h_1a3dd686974e8a96866c0a47068b782cb0:
.. _cid-s2_axpy_ssf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_axpy_ssf (
	    float scalar,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* v1x,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* v2y
	    )

axpy (ax plus y stored in y) for single precision screws

add 3 by 1 column matrix v1i scaled by k to v2i

$V_{2i} += k * V_{1i} $



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - [in] a first column vector input

    *
        - v2i

        - [in/out] second column vector

.. _doxid-kinematics_8h_1afc672ecd8621e23892a82ff7114dbdfc:
.. _cid-s2_cross_vv:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_cross_vv (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Compute double precision cross product of v1i with v2i.

$V_{1i} \times V_{2i} \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first 3x1 vector

    *
        - v2i

        - a second 3x1 vector

    *
        - vo

        - the results are stored in vo

.. _doxid-kinematics_8h_1a00658b700618d65c0de30da39393cc3f:
.. _cid-s2_cross_vvinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_cross_vvinverse (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v1i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* v2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Compute double precision cross product of v1i with v2i^-1.

$V_{1i} \times V_{2i} \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first 3x1 vector

    *
        - v2i

        - a second 3x1 vector

    *
        - vo

        - the results are stored in vo

.. _doxid-kinematics_8h_1a5cc89fe340d475bd643a8c5fc5606d0d:
.. _cid-s2_cross_vvf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_cross_vvf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

Compute single precision cross product of v1i with v2i.

$V_{1i} \times V_{2i} \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first 3x1 vector

    *
        - v2i

        - a second 3x1 vector

    *
        - vo

        - the results are stored in vo

.. _doxid-kinematics_8h_1a55f71dc802f0e1d56e4463015bda9ebf:
.. _cid-s2_cross_vvinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_cross_vvinversef (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v1i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* v2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

Compute single precision cross product of v1i with v2i inverse.

$V_{1i} \times V_{2i} \rightarrow V_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - v1i

        - a first 3x1 vector

    *
        - v2i

        - a second 3x1 vector

    *
        - vo

        - the results are stored in vo

.. _doxid-kinematics_8h_1ae61b2fd7308f015d5ced655e8038d512:
.. _cid-s2_screw_copy:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screw_copy (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

Copy double precision :ref:`s2_screw_t <doxid-structs2__screw__t>` si to so.

$S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - si

        - 6x1 screw input

    *
        - so

        - si is copied to so

.. _doxid-kinematics_8h_1a877e5f8f1a72568028298dca7293addf:
.. _cid-s2_screwf_copy:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwf_copy (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

Copy single precision :ref:`s2_screw_t <doxid-structs2__screw__t>` si to so.

$S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - si

        - 6x1 screw input

    *
        - so

        - si is copied to so

.. _doxid-kinematics_8h_1a61e36691d826043c69f51730e699eb9a:
.. _cid-s2_screw_invert:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screw_invert (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

Invert a double precision :ref:`s2_screw_t <doxid-structs2__screw__t>` si, store the results in so.

$-S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - si

        - input vector $S_i$

    *
        - so

        - output vector $S_o$

.. _doxid-kinematics_8h_1a2e1c630b4a773cfa4c16516182219ed9:
.. _cid-s2_screwf_invert:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwf_invert (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

Invert a single precision :ref:`s2_screwf_t <doxid-structs2__screwf__t>` si, store the results in so.

$-S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - si

        - input vector $S_i$

    *
        - so

        - output vector $S_o$

.. _doxid-kinematics_8h_1a4fad361eef66c003cf597ea93b60aec3:
.. _cid-s2_product_sk:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_sk (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    double* ki,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

Computes the product of a a double precision :ref:`s2_screw_t <doxid-structs2__screw__t>` and a scalar.

$S_i * k_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - si[6]

        - A 3x1 input vector

    *
        - *ki

        - a pointer to a scalar

    *
        - so[6]

        - the results are stored in so[6]

.. _doxid-kinematics_8h_1a6aa748b1ecd5c6420cff106cca8ea0fb:
.. _cid-s2_product_skf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_skf (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    float* ki,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

Computes the product of a single precision :ref:`s2_screwf_t <doxid-structs2__screwf__t>` and a scalar.

$S_i * k_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - si[6]

        - A 3x1 input vector

    *
        - *ki

        - a pointer to a scalar

    *
        - so[6]

        - the results are stored in so[6]

.. _doxid-kinematics_8h_1a4ca4d08f9279e9376e5b33cc27bb3ec4:
.. _cid-s2_add_ss:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ss (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

Add double precision s2_screw_ts s1i and s2i and store results in so.

$S_{1i} + S_{2i} \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first 6x1 screw

    *
        - s2i

        - a second 6x1 screw

    *
        - so

        - The results of s1i + s2i are stored in so

.. _doxid-kinematics_8h_1a1d743152fa7392a74a4b1d83648164bb:
.. _cid-s2_add_ssf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ssf (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

Add single precision s2_screw_ts s1i and s2i and store results in so.

$S_{1i} + S_{2i} \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first 6x1 screw

    *
        - s2i

        - a second 6x1 screw

    *
        - so

        - The results of s1i + s2i are stored in so

.. _doxid-kinematics_8h_1ace4d722685aa59f118c7454ed205fd79:
.. _cid-s2_addequal_ss:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_addequal_ss (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i
	    )

Add double precision s2_screw_ts s1i to s2i.

$S_{2i} += S_{1i}$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first 6x1 screw

    *
        - s2i

        - a second 6x1 screw

    *
        - so

        - The results of s1i + s2i are stored in so

.. _doxid-kinematics_8h_1a9915c1d4a1225b0e83e5802f663344aa:
.. _cid-s2_addequal_ssf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_addequal_ssf (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i
	    )

Add double precision s2_screw_ts s1i to s2i.

$S_{2i} += S_{1i}$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first 6x1 screw

    *
        - s2i

        - a second 6x1 screw

    *
        - so

        - The results of s1i + s2i are stored in so

.. _doxid-kinematics_8h_1a2c5df2ff8239acfdcba13979861e6334:
.. _cid-s2_subtract_ss:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_subtract_ss (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

Subtract double precision :ref:`s2_screw_t <doxid-structs2__screw__t>` s2i from s1i and store results in so.

$S_{1i} - S_{2i} \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first :ref:`s2_screw_t <doxid-structs2__screw__t>`

    *
        - s2i

        - a second :ref:`s2_screw_t <doxid-structs2__screw__t>`

    *
        - so

        - The results of s1i - s2i are stored in so

.. _doxid-kinematics_8h_1a99b0ef809bc8aa44a34e96e1467661ee:
.. _cid-s2_subtract_ssf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_subtract_ssf (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

Subtract single precision :ref:`s2_screw_t <doxid-structs2__screw__t>` s2i from s1i and store results in so.

$S_{1i} - S_{2i} \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first :ref:`s2_screw_t <doxid-structs2__screw__t>`

    *
        - s2i

        - a second :ref:`s2_screw_t <doxid-structs2__screw__t>`

    *
        - so

        - The results of s1i - s2i are stored in so

.. _doxid-kinematics_8h_1a54589f97ab7036b4e73bb32244b4b62e:
.. _cid-s2_subtractequal_ss:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_subtractequal_ss (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i
	    )

Add double precision s2_screw_ts s1i to s2i.

$S_{2i} -= S_{1i}$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first 6x1 screw

    *
        - s2i

        - a second 6x1 screw

    *
        - so

        - The results of s1i + s2i are stored in so

.. _doxid-kinematics_8h_1a0a6934e7684810978871a12a04012e06:
.. _cid-s2_subtractequal_ssf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_subtractequal_ssf (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i
	    )

Add double precision s2_screw_ts s1i to s2i.

$S_{2i} += S_{1i}$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first 6x1 screw

    *
        - s2i

        - a second 6x1 screw

    *
        - so

        - The results of s1i + s2i are stored in so

.. _doxid-kinematics_8h_1a94508d66cd7ac94b9a093b0761b0df6f:
.. _cid-s2_add_sst:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_sst (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si1,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si2,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

Adds a double prevision :ref:`s2_screw_t <doxid-structs2__screw__t>` and Vector.

$S_i[0:3] + V_i \rightarrow S_o[0:3]$

$S_i[3:6] \rightarrow S_o[3:6]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - si

        - a 6x1 screw

    *
        - vi

        - a 3x1 vector

    *
        - so

        - the results of the operation are returned in so

.. _doxid-kinematics_8h_1a9468968388a229041ef56da31f720717:
.. _cid-s2_add_sstf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_sstf (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si1,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si2,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

Adds a single prevision :ref:`s2_screwf_t <doxid-structs2__screwf__t>` and Vectorf.

$S_i[0:3] + V_i \rightarrow S_o[0:3]$

$S_i[3:6] \rightarrow S_o[3:6]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - si

        - a 6x1 screw

    *
        - vi

        - a 3x1 vector

    *
        - so

        - the results of the operation are returned in so

.. _doxid-kinematics_8h_1a5118f11eb04146c2e45b6a42df95037c:
.. _cid-s2_mat3x3_copy:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3_copy (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Copy a double precision :ref:`s2_mat3x3_t <doxid-structs2__mat3x3__t>` direction cosine matrix ri into ro.

$R_i \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - rotation matrix input

    *
        - ro

        - rotation matrix output

.. _doxid-kinematics_8h_1a770547b16bc893e5aed472d4cd5d5365:
.. _cid-s2_mat3x3f_copy:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3f_copy (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

Copy a single precision :ref:`s2_mat3x3f_t <doxid-structs2__mat3x3f__t>` direction cosine matrix ri into ro.

$R_i \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - rotation matrix input

    *
        - ro

        - rotation matrix output

.. _doxid-kinematics_8h_1a014bb516ce7aeae34da04611a471a1f8:
.. _cid-s2_mat3x3_invert:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3_invert (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Inverts (transposes) a double precision :ref:`s2_mat3x3_t <doxid-structs2__mat3x3__t>` rotation matrix.

$R_i^T \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - rotation matrix to be inverted

    *
        - ro

        - transposed rotation matrix returned in ro

.. _doxid-kinematics_8h_1af5e4b6a44b052ae5c0907424e7d1533a:
.. _cid-s2_mat3x3f_invert:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3f_invert (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

Inverts (transposes) a single precision :ref:`s2_mat3x3f_t <doxid-structs2__mat3x3f__t>` rotation matrix.

$R_i^T \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - rotation matrix to be inverted

    *
        - ro

        - transposed rotation matrix returned in ro

.. _doxid-kinematics_8h_1a0fbdf7ad62df14225db715b8054309ac:
.. _cid-s2_product_rv:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rv (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Store product of a double precision direction cosine matrix ri with vector vi in vo.

$R_i * V_i \rightarrow V_o$

.. _doxid-kinematics_8h_1ab081a7cd63e415447bc5c4166b77dfb1:
.. _cid-s2_product_rvf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rvf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

Store product of direction cosine matrix ri with vector vi in vo.

$R_i * V_i \rightarrow V_o$

.. _doxid-kinematics_8h_1affce2e1277c70ec5a554130df57cc986:
.. _cid-s2_product_rvinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rvinverse (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Store product of a double precision direction cosine matrix ri with vector -vi in vo.

$R_i * -V_i \rightarrow V_o$

.. _doxid-kinematics_8h_1a30fd8631549b1c147b460d30bd84e662:
.. _cid-s2_product_rvinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rvinversef (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

Store product of a single precision direction cosine matrix ri with vector -vi in vo.

$R_i * -V_i \rightarrow V_o$

.. _doxid-kinematics_8h_1a160a3132f5237c02c41e93def0c6b93a:
.. _cid-s2_product_rinversev:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rinversev (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Store product of a double precision direction cosine matrix ri^-1 with vector vi in vo.

$R_i^T * V_i \rightarrow V_o$

.. _doxid-kinematics_8h_1a4806b16cdf637f6519a702e2d76bb5e6:
.. _cid-s2_product_rinversevf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rinversevf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

Store product of a single precision direction cosine matrix ri^-1 with vector vi in vo.

$R_i^T * V_i \rightarrow V_o$

.. _doxid-kinematics_8h_1a2e03ed8a43d1abfe8f1b4ce18969f8bc:
.. _cid-s2_product_rinversevinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rinversevinverse (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Store product of a double precision direction cosine matrix ri^-1 with vector -vi in vo.

$R_i^T * -V_i \rightarrow V_o$

.. _doxid-kinematics_8h_1af9ee6a78e7332cae56a31fd01963533f:
.. _cid-s2_product_rinversevinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rinversevinversef (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vo
	    )

Store product of a double precision direction cosine matrix ri^-1 with vector -vi in vo.

$R_i^T * -V_i \rightarrow V_o$

.. _doxid-kinematics_8h_1a639f37b45a33d87a661aaeeff0a3e1bb:
.. _cid-s2_product_rs:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rs (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

rotates a double precision spatial screw

Store product of direction cosine matrix ri with spatial vector si in so.

$ R_i * S_i[0:2] \rightarrow S_o[0:2]$

$ R_i * S_i[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a 1x9 rotation matrix

    *
        - si

        - a 1x6 screw

    *
        - so

        - the output of ri * si is returned in so

.. _doxid-kinematics_8h_1a5ea68301a9575b9a3816cdbcec45e0a6:
.. _cid-s2_product_rsinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rsinverse (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

rotates a double precision spatial screw inverse

Store product of direction cosine matrix ri with spatial vector si in so.

$ R_i * S_i[0:2] \rightarrow S_o[0:2]$

$ R_i * S_i[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a 1x9 rotation matrix

    *
        - si

        - a 1x6 screw

    *
        - so

        - the output of ri * si is returned in so

.. _doxid-kinematics_8h_1aa754be86054a660dd684886f04101c9c:
.. _cid-s2_product_rsf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rsf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

rotates a single precision spatial screw

Store product of direction cosine matrix ri with spatial vector si in so.

$ R_i * S_i[0:2] \rightarrow S_o[0:2]$

$ R_i * S_i[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a 1x9 rotation matrix

    *
        - si

        - a 1x6 screw

    *
        - so

        - the output of ri * si is returned in so

.. _doxid-kinematics_8h_1a9af2e48e750e30dbc823ac7b6abf6b78:
.. _cid-s2_product_rsinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rsinversef (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

rotates a double precision spatial screw inverse

Store product of direction cosine matrix ri with spatial vector si in so.

$ R_i * S_i[0:2] \rightarrow S_o[0:2]$

$ R_i * S_i[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a 1x9 rotation matrix

    *
        - si

        - a 1x6 screw

    *
        - so

        - the output of ri * si is returned in so

.. _doxid-kinematics_8h_1a8b21be5ec0e0242e59c127c0969071fa:
.. _cid-s2_product_rinverses:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rinverses (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

rotates a single precision spatial screw's inverse

Store product of direction cosine matrix ri with spatial vector si in so.

$R_i * -S_i[0:2] \rightarrow S_o[0:2]$ $R_i * -S_i[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - rotation matrix

    *
        - si

        - screw

    *
        - so

        - the output of ri * -si is returned in so

.. _doxid-kinematics_8h_1abdd29a7b2ddc6ba996a17f19af749c53:
.. _cid-s2_product_rinversesf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rinversesf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

rotates a single precision spatial screw's inverse

Store product of direction cosine matrix ri with spatial vector si in so.

$R_i * -S_i[0:2] \rightarrow S_o[0:2]$ $R_i * -S_i[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - rotation matrix

    *
        - si

        - screw

    *
        - so

        - the output of ri * -si is returned in so

.. _doxid-kinematics_8h_1a76e60ee2e631865bfb7e0f20cc3d478f:
.. _cid-s2_product_rrx:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rrx (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Double precision product of R and Rx.

Computes the product of a general rotation matrix, R, with a primitive rotation about the x axis, Rx, characterized by its direction cosines.

$R_i * R_x \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1ada4b6634d9c811018b355396963cd246:
.. _cid-s2_product_rrxf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rrxf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

Product of R and Rx.

Computes the product of a general rotation matrix, R, with a primitive rotation about the x axis, Rx, characterized by its direction cosines.

$R_i * R_x \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1aa336042cd5ff04aec58fe99f01304351:
.. _cid-s2_product_rrxinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rrxinverse (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Double precision product of R and Rxinverse.

Computes the product of a general rotation matrix, R, with the inverse of a primitive rotation about the x axis, Rx^T, characterized by its direction cosines.

$R_i * R_x^T \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a0086fe943e51ce213b8e10f46d0ae487:
.. _cid-s2_product_rrxinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rrxinversef (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

single precision product of R and Rxinverse

Computes the product of a general rotation matrix, R, with the inverse of a primitive rotation about the x axis, Rx^T, characterized by its direction cosines.

$R_i * R_x^T \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a5e0535295f6634c276cee80bb11b6393:
.. _cid-s2_product_rry:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rry (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Product of R and Ry.

Computes the product of a general rotation matrix, R, with a primitive rotation about the x axis, Rx, characterized by its direction cosines.

$R_i * R_y \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a860142e5d56955952bbdf02362b8619e:
.. _cid-s2_product_rryf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rryf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

Product of R and Ry.

Computes the product of a general rotation matrix, R, with a primitive rotation about the x axis, Rx, characterized by its direction cosines.

$R_i * R_y \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1aa2becabdce79ba6a9011033e0c6d960f:
.. _cid-s2_product_rryinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rryinverse (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Double precision product of R and Ryinverse.

Computes the product of a general rotation matrix, R, with the inverse of a primitive rotation about the y axis, Ry^T, characterized by its direction cosines.

$R_i * R_y^T \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1abca9efb8d54d6077f270bf1591eee21f:
.. _cid-s2_product_rryinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rryinversef (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

Double precision product of R and Ryinverse.

Computes the product of a general rotation matrix, R, with the inverse of a primitive rotation about the y axis, Ry^T, characterized by its direction cosines.

$R_i * R_y^T \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1aada642fd159a5766c0054caf04198f77:
.. _cid-s2_product_rrz:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rrz (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Double precision product of R and Rz.

Computes the product of a general rotation matrix, R, with a primitive rotation about the z axis, Rz, characterized by its direction cosines.

$R_i * R_z \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a102d439aa7987fa288276da9181254db:
.. _cid-s2_product_rrzf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rrzf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

single precision product of R and Rz

Computes the product of a general rotation matrix, R, with a primitive rotation about the z axis, Rz, characterized by its direction cosines.

$R_i * R_z \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1ac8f74722384fe560c56576469d79592c:
.. _cid-s2_product_rrzinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rrzinverse (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double dc [2],
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Product of R and Rzinverse.

Computes the product of a general rotation matrix, R, with the inverse of a primitive rotation about the z axis, Rz^T, characterized by its direction cosines.

$R_i * R_z^T \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a089268725603b4bc5f3eaadda27285f8:
.. _cid-s2_product_rrzinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rrzinversef (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float dc [2],
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

Product of R and Rzinverse.

Computes the product of a general rotation matrix, R, with the inverse of a primitive rotation about the z axis, Rz^T, characterized by its direction cosines.

$R_i * R_z^T \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - dc

        - a pointer to an array of direction cosines

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a85f0ea8e6f148d4ae30f683e9808107a:
.. _cid-s2_product_rr:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rr (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r1i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r2i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Double precision product of two general rotation matrices.

Computes the product of two general rotation matrices, R1i and R2i.

$R_{1i} * R_{2i} \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - R1i

        - a pointer to a general rotation matrix

    *
        - R2i

        - a pointer to a general rotation matrix

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1aacbf9acbaea875f49126d88d78b03c24:
.. _cid-s2_product_rrf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rrf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r1i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r2i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

single precision product of two general rotation matrices

Computes the product of two general rotation matrices, R1i and R2i.

$R_{1i} * R_{2i} \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - R1i

        - a pointer to a general rotation matrix

    *
        - R2i

        - a pointer to a general rotation matrix

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1aa90e088789da6676c214aee4fe656aa4:
.. _cid-s2_product_rrinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rrinverse (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r1i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r2i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Product of a general rotation matrix and the inverse of a general rotation matrix.

Computes the product of a general rotation matrix, R1i, and the inverse of a second rotation matrix, R2i.

$R_{1i} * R_{2i}^T \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - R1i

        - a pointer to a general rotation matrix

    *
        - R2i

        - a pointer to a general rotation matrix

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a86d9d081de2374a56076c75925c002e0:
.. _cid-s2_product_rrinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rrinversef (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r1i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r2i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

Product of a general rotation matrix and the inverse of a general rotation matrix.

Computes the product of a general rotation matrix, R1i, and the inverse of a second rotation matrix, R2i.

$R_{1i} * R_{2i}^T \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - R1i

        - a pointer to a general rotation matrix

    *
        - R2i

        - a pointer to a general rotation matrix

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a2d734e3ca01496db402fd53167e83a9f:
.. _cid-s2_product_rinverser:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rinverser (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r1i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* r2i,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Double precision product of the inverse of a general rotation matrix and a general rotation matrix.

Computes the product of the inverse of a general rotation matrix, R1i, and a second rotation matrix, R2i.

$R_{1i}^T * R_{2i} \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - R1i

        - a pointer to a general rotation matrix

    *
        - R2i

        - a pointer to a general rotation matrix

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1aeafeed107e283a5b9d3b6051dfa475f3:
.. _cid-s2_product_rinverserf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rinverserf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r1i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* r2i,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

single precision product of the inverse of a general rotation matrix and a general rotation matrix

Computes the product of the inverse of a general rotation matrix, R1i, and a second rotation matrix, R2i.

$R_{1i}^T * R_{2i} \rightarrow R_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - R1i

        - a pointer to a general rotation matrix

    *
        - R2i

        - a pointer to a general rotation matrix

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a40c0e09a13372e349f698eb50851b01c:
.. _cid-s2_product_rtx:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rtx (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Double precision product of the a general rotation matrix and a primitive translation along the x axis.

Computes the product of a general rotation matrix and a primitive translation along the x axis using sparse algebra.

$R_{i} * t_{x} \rightarrow t_o$

is equivalent to:

$R_i[0,3,6] * t_x \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - to

        - a pointer to a vector where result is stored.

.. _doxid-kinematics_8h_1a787b0b6cff4ce7fb4f0f7a1030505ab6:
.. _cid-s2_product_rtxf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rtxf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

single precision product of the a general rotation matrix and a primitive translation along the x axis

Computes the product of a general rotation matrix and a primitive translation along the x axis using sparse algebra.

$R_{i} * t_{x} \rightarrow t_o$

is equivalent to:

$R_i[0,3,6] * t_x \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1aab9dce66f436a436aa221e996a60fb69:
.. _cid-s2_product_rtxinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rtxinverse (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Double precision product of the a general rotation matrix and the inverse of a primitive translation along the x axis.

Computes the product of a general rotation matrix and the inverse of a primitive translation along the x axis using sparse algebra.

$R_{i} * -t_{x} \rightarrow t_o$

is equivalent to:

$R_i[0,3,6] * -t_x \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1acd755aca4d26696856a26c30dcb0558d:
.. _cid-s2_product_rtxinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rtxinversef (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

single precision product of the a general rotation matrix and the inverse of a primitive translation along the x axis

Computes the product of a general rotation matrix and the inverse of a primitive translation along the x axis using sparse algebra.

$R_{i} * -t_{x} \rightarrow t_o$

is equivalent to:

$R_i[0,3,6] * -t_x \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a659b2070a7ad93b7534711b121a8d1a3:
.. _cid-s2_product_rty:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rty (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Double precision product of the a general rotation matrix and a primitive translation along the y axis.

Computes the product of a general rotation matrix and a primitive translation along the y axis using sparse algebra.

$R_{i} * t_{y} \rightarrow t_o$

is equivalent to:

$R_i[1,4,7] * t_y \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a2350fda6e0eb204d8bdb73b0054036c2:
.. _cid-s2_product_rtyf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rtyf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

single precision product of the a general rotation matrix and a primitive translation along the y axis

Computes the product of a general rotation matrix and a primitive translation along the y axis using sparse algebra.

$R_{i} * t_{y} \rightarrow t_o$

is equivalent to:

$R_i[1,4,7] * t_y \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a9b6b83714d7e0c0463c54793fd1a53f3:
.. _cid-s2_product_rtyinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rtyinverse (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Double precision product of a general rotation matrix and the inverse of a primitive translation along the y axis.

Computes the product of a general rotation matrix and the inverse of a primitive translation along the x axis using sparse algebra.

$R_{i} * -t_{y} \rightarrow t_o$

is equivalent to:

$R_i[1,4,7] * -t_y \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a02132474bde0fccd7dfa4d7158d87fc0:
.. _cid-s2_product_rtyinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rtyinversef (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

single precision product of a general rotation matrix and the inverse of a primitive translation along the y axis

Computes the product of a general rotation matrix and the inverse of a primitive translation along the x axis using sparse algebra.

$R_{i} * -t_{y} \rightarrow t_o$

is equivalent to:

$R_i[1,4,7] * -t_y \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1abaff096f63f7231f9fd6552d4b1e9ad3:
.. _cid-s2_product_rtz:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rtz (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Double precision product of a general rotation matrix and a primitive translation along the z axis.

Computes the product of a general rotation matrix and a primitive translation along the z axis using sparse algebra.

$R_{i} * t_{z} \rightarrow t_o$

is equivalent to:

$R_i[2,5,8] * t_z \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a34e5bb55ed27b6a7a81c85cb3d02c6bd:
.. _cid-s2_product_rtzf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rtzf (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

Double precision product of a general rotation matrix and a primitive translation along the z axis.

Computes the product of a general rotation matrix and a primitive translation along the z axis using sparse algebra.

$R_{i} * t_{z} \rightarrow t_o$

is equivalent to:

$R_i[2,5,8] * t_z \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1aa7f502f75aada71ca9d68abbfbcc86b0:
.. _cid-s2_product_rtzinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rtzinverse (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    double* ti,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Double precision product of the a general rotation matrix and the inverse of a primitive translation along the y axis.

Computes the product of a general rotation matrix and the inverse of a primitive translation along the x axis using sparse algebra.

$R_{i} * -t_{z} \rightarrow t_o$

is equivalent to:

$R_i[2,5,8] * -t_z \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1a35c4da947bc3ac18aad0e8b0def4c701:
.. _cid-s2_product_rtzinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_rtzinversef (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    float* ti,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

single precision product of the a general rotation matrix and the inverse of a primitive translation along the y axis

Computes the product of a general rotation matrix and the inverse of a primitive translation along the x axis using sparse algebra.

$R_{i} * -t_{z} \rightarrow t_o$

is equivalent to:

$R_i[2,5,8] * -t_z \rightarrow t_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a pointer to a general rotation matrix

    *
        - ti

        - a pointer to a double

    *
        - ro

        - a pointer to a rotation matrix where the result is stored.

.. _doxid-kinematics_8h_1aed7340fadb9070e72596a13295c8342d:
.. _cid-s2_add_ttx:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ttx (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Add a double precision Vector with a transformation along x-axis.

Computes the result of a general 3-d translation, followed by a translation along the x axis using sparse algebra.

$t_{i} + t_{x} \rightarrow t_o$

is equivalent to:

$t_{i} \rightarrow t_o$

$t_o[0] + t_x \rightarrow t_o[0]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1ac7490edc0a18b0effc71f358639927b2:
.. _cid-s2_add_ttxf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ttxf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

Add a single precision Vector with a transformation along x-axis.

Computes the result of a general 3-d translation, followed by a translation along the x axis using sparse algebra.

$t_{i} + t_{x} \rightarrow t_o$

is equivalent to:

$t_{i} \rightarrow t_o$

$t_o[0] + t_x \rightarrow t_o[0]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1a75273eebacbf45018fe5dec91787c10a:
.. _cid-s2_add_ttxinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ttxinverse (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Add a double precision Vector with the inverse of a translation along the x-axis.

Computes the result of a general 3-d transformation, followed by a tranformation along the x axis using sparse algebra.

$t_{i} + t_{x} \rightarrow t_o$

is equivalent to:

$t_{i} \rightarrow t_o$

$t_o[0] + t_x \rightarrow t_o[0]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1a7ddd6ef74933f09dfbb9c343997faf85:
.. _cid-s2_add_ttxinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ttxinversef (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

Add a single precision Vectorf with the inverse of a translation along the x-axis.

Computes the result of a general 3-d transformation, followed by a tranformation along the x axis using sparse algebra.

$t_{i} + t_{x} \rightarrow t_o$

is equivalent to:

$t_{i} \rightarrow t_o$

$t_o[0] + t_x \rightarrow t_o[0]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1a3b244d5035a2fe7b41fa71bb506fba22:
.. _cid-s2_add_tty:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_tty (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Add a double precision Vector with a translation along the y-axis.

Computes the result of a general 3-d transformation, followed by a tranformation along the y axis using sparse algebra.

$t_{i} + t_{y} \rightarrow t_o$

is equivalent to:

$t_{i}[0:2] \rightarrow t_o[0:2]$

$t_o[1] + t_y \rightarrow t_o[1]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1a182dfd98f5e3bde779b6e25ea41ca8e5:
.. _cid-s2_add_ttyf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ttyf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

Add a single precision Vectorf with a translation along the y-axis.

Computes the result of a general 3-d transformation, followed by a tranformation along the y axis using sparse algebra.

$t_{i} + t_{y} \rightarrow t_o$

is equivalent to:

$t_{i}[0:2] \rightarrow t_o[0:2]$

$t_o[1] + t_y \rightarrow t_o[1]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1a36e3bcc1acaf20e8c46fb80a4e01e2e8:
.. _cid-s2_add_ttyinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ttyinverse (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Add a double precision Vector with the inverse of a translation along the y-axis.

Computes the result of a general 3-d transformation, followed by a tranformation along the y axis using sparse algebra.

$t_{i} + t_{y} \rightarrow t_o$

is equivalent to:

$t_{i}[0:2] \rightarrow t_o[0:2]$

$t_o[1] - t_y \rightarrow t_o[1]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1ad57b7f09daac9a275d6f31fc2e5bbb2e:
.. _cid-s2_add_ttyinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ttyinversef (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

Add a single precision Vector with the inverse of a translation along the y-axis.

Computes the result of a general 3-d transformation, followed by a tranformation along the y axis using sparse algebra.

$t_{i} + t_{y} \rightarrow t_o$

is equivalent to:

$t_{i}[0:2] \rightarrow t_o[0:2]$

$t_o[1] - t_y \rightarrow t_o[1]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1a30934cc41b06e8e00ec7a0ec6f6b73b7:
.. _cid-s2_add_ttz:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ttz (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Add a double precision Vector with a translation along the z-axis.

Computes the result of a general 3-d transformation, followed by a tranformation along the z axis using sparse algebra.

$t_{i} + t_{z} \rightarrow t_o$

is equivalent to:

$t_{i}[0:2] \rightarrow t_o[0:2]$

$t_o[2] + t_z \rightarrow t_o[2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1a6fb93284f99e1edc4708cec0cc33e1c9:
.. _cid-s2_add_ttzf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ttzf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

Add a single precision Vectorf with a translation along the z-axis.

Computes the result of a general 3-d transformation, followed by a tranformation along the z axis using sparse algebra.

$t_{i} + t_{z} \rightarrow t_o$

is equivalent to:

$t_{i}[0:2] \rightarrow t_o[0:2]$

$t_o[2] + t_z \rightarrow t_o[2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1a5257a8ce5b3e541ef4748e3e0b9b512b:
.. _cid-s2_add_ttzinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ttzinverse (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* t1i,
	    double* t2i,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Add a double precision Vector with the inverse of a translation along the z-axis.

Computes the result of a general 3-d transformation, followed by a tranformation along the z axis using sparse algebra.

$t_{i} + t_{z} \rightarrow t_o$

is equivalent to:

$t_{i}[0:2] \rightarrow t_o[0:2]$

$t_o[2] - t_z \rightarrow t_o[2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1a722198145d80fc9a9d665a7fd05dfe25:
.. _cid-s2_add_ttzinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_ttzinversef (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* t1i,
	    float* t2i,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

Add a single precision Vector with the inverse of a translation along the z-axis.

Computes the result of a general 3-d transformation, followed by a tranformation along the z axis using sparse algebra.

$t_{i} + t_{z} \rightarrow t_o$

is equivalent to:

$t_{i}[0:2] \rightarrow t_o[0:2]$

$t_o[2] - t_z \rightarrow t_o[2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - t1i

        - a pointer to a general 3-d translation

    *
        - t2i

        - a pointer to a double

    *
        - to

        - a pointer to a general 3-d translation where the result is stored.

.. _doxid-kinematics_8h_1abc18154ef1cfe80a9ccbe97ad898d3a4:
.. _cid-s2_displacement_invert:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacement_invert (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Compute double precision inverse of a :ref:`s2_displacement_t <doxid-structs2__displacement__t>` .

$D_i^{-1} \rightarrow D_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement to take the inverse of

    *
        - Do

        - inverse of Di is returned in Do

.. _doxid-kinematics_8h_1ae1d1cd6efd1d7123168603e4e2606936:
.. _cid-s2_displacementf_invert:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacementf_invert (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

Compute single precision inverse of a :ref:`s2_displacement_t <doxid-structs2__displacement__t>` .

$D_i^{-1} \rightarrow D_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement to take the inverse of

    *
        - Do

        - inverse of Di is returned in Do

.. _doxid-kinematics_8h_1af83c896dbc0ab49e6bd13a103b321aec:
.. _cid-s2_displacement_copy:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacement_copy (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Copy a double precision :ref:`s2_displacement_t <doxid-structs2__displacement__t>` consisting of a rotation and translation.

$D_i \rightarrow D_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement input

    *
        - Do

        - Di is copied to Do

.. _doxid-kinematics_8h_1aaad553f944a00d183d4e2e1a189307fe:
.. _cid-s2_displacementf_copy:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_displacementf_copy (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

Copy a single precision :ref:`s2_displacementf_t <doxid-structs2__displacementf__t>` consisting of a rotation and translation.

$D_i \rightarrow D_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement input

    *
        - Do

        - Di is copied to Do

.. _doxid-kinematics_8h_1a92dac683b8af3865c13b4032272bbbd3:
.. _cid-s2_product_drx:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_drx (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of a :ref:`s2_displacement_t <doxid-structs2__displacement__t>` with a primitive rotation about the x-axis.

$R_i * R_x \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a pointer to a general displacement

    *
        - dc

        - direction cosines characterizing a primitive rotation

    *
        - the

        - result of the computation is stored in Do

.. _doxid-kinematics_8h_1a9deffee2919fbf40180bccdb813c70a7:
.. _cid-s2_product_drxf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_drxf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision product of a :ref:`s2_displacement_t <doxid-structs2__displacement__t>` with a primitive rotation about the x-axis

$R_i * R_x \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a pointer to a general displacement

    *
        - dc

        - direction cosines characterizing a primitive rotation

    *
        - the

        - result of the computation is stored in Do

.. _doxid-kinematics_8h_1a7050dc80a4ed7af1f0fb2a1c1fe54dbb:
.. _cid-s2_product_drxinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_drxinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of a general displacement with the inverse of a primitive rotation about the x-axis.

$D_i * R_x^T \rightarrow D_o$

is equivalent to:

$R_i * R_x^T \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a pointer to a general displacement

    *
        - dc

        - direction cosines characterizing a primitive rotation

    *
        - the

        - result of the computation is stored in Do

.. _doxid-kinematics_8h_1a83ca602a50fba709304a5255be78913f:
.. _cid-s2_product_drxinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_drxinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision product of a general displacement with the inverse of a primitive rotation about the x-axis

$D_i * R_x^T \rightarrow D_o$

is equivalent to:

$R_i * R_x^T \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a pointer to a general displacement

    *
        - dc

        - direction cosines characterizing a primitive rotation

    *
        - the

        - result of the computation is stored in Do

.. _doxid-kinematics_8h_1ad013eda39e2d8f26663c5c6665e13a12:
.. _cid-s2_product_dry:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dry (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of a :ref:`s2_displacement_t <doxid-structs2__displacement__t>` , $D$ , and a rotation about the y axis, $R_y$ .

The following operation: $D_i * R_y \rightarrow D_o$

is equivalent to:

$R_i * R_y \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - dc[2]

        - direction cosines for a rotation about the Y axis.

    *
        - Do

        - the results are stored in a general displacement, $D_o$

.. _doxid-kinematics_8h_1a52f69f79ad1f07e1b3a49fab081ef6d3:
.. _cid-s2_product_dryf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dryf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision product of a :ref:`s2_displacementf_t <doxid-structs2__displacementf__t>` , $D$ , and a rotation about the y axis, $R_y$

The following operation: $D_i * R_y \rightarrow D_o$

is equivalent to:

$R_i * R_y \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - dc[2]

        - direction cosines for a rotation about the Y axis.

    *
        - Do

        - the results are stored in a general displacement, $D_o$

.. _doxid-kinematics_8h_1ab13ac64def852a123b0f679b1c0aea61:
.. _cid-s2_product_dryinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dryinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of a :ref:`s2_displacement_t <doxid-structs2__displacement__t>` , $D$ , and the inverse of a rotation about the y axis, $R_y^T$ .

The following operation: $D_i * R_y^T \rightarrow D_o$

Can be split in to:

$R_i * R_y^T \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - dc[2]

        - direction cosines for a rotation about the Y axis.

    *
        - Do

        - the results are stored in a general displacement, $D_o$

.. _doxid-kinematics_8h_1aa34eacdcfb3e213a3ca8c4c5a410b7cf:
.. _cid-s2_product_dryinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dryinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision product of a :ref:`s2_displacementf_t <doxid-structs2__displacementf__t>` , $D$ , and the inverse of a rotation about the y axis, $R_y^T$

The following operation: $D_i * R_y^T \rightarrow D_o$

Can be split in to:

$R_i * R_y^T \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - dc[2]

        - direction cosines for a rotation about the Y axis.

    *
        - Do

        - the results are stored in a general displacement, $D_o$

.. _doxid-kinematics_8h_1a05371d713133d7963d3bd68bee840370:
.. _cid-s2_product_drz:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_drz (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of a general displacement, $D$ , and a rotation about the z axis, $R_z$ .

The following operation: $D_i * R_z \rightarrow D_o$

Can be split in to:

$R_i*R_z \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - dc[2]

        - direction cosines for a rotation about the Z axis.

    *
        - Do

        - the results are stored in a general displacement, $D_o$

.. _doxid-kinematics_8h_1aebe91869310ca945d54d008b7144f2f8:
.. _cid-s2_product_drzf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_drzf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision product of a :ref:`s2_displacementf_t <doxid-structs2__displacementf__t>` , $D$ , and a rotation about the z axis, $R_z$

The following operation: $D_i * R_z \rightarrow D_o$

Can be split in to:

$R_i*R_z \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - dc[2]

        - direction cosines for a rotation about the Z axis.

    *
        - Do

        - the results are stored in a general displacement, $D_o$

.. _doxid-kinematics_8h_1aa15f1cfd5260c0078757b3e547626e4b:
.. _cid-s2_product_drzinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_drzinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double dc [2],
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of a :ref:`s2_displacement_t <doxid-structs2__displacement__t>` , $D$ , and an inverse rotation about the z axis, $R_z^T$ .

The following operation: $D_i * R_z^T \rightarrow D_o$

Can be split in to:

$R_i * R_z^T \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - dc[2]

        - direction cosines for a rotation about the Z axis.

    *
        - Do

        - the results are stored in a general displacement, $D_o$

.. _doxid-kinematics_8h_1a4c048adf73cc44db1eba74984808d738:
.. _cid-s2_product_drzinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_drzinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float dc [2],
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision product of a :ref:`s2_displacement_t <doxid-structs2__displacement__t>` , $D$ , and an inverse rotation about the z axis, $R_z^T$

The following operation: $D_i * R_z^T \rightarrow D_o$

Can be split in to:

$R_i * R_z^T \rightarrow R_o$

$T_i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - dc[2]

        - direction cosines for a rotation about the Z axis.

    *
        - Do

        - the results are stored in a general displacement, $D_o$

.. _doxid-kinematics_8h_1ada3a3f19abd83989a1a799a03b591856:
.. _cid-s2_product_dt:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dt (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of a general :ref:`s2_displacement_t <doxid-structs2__displacement__t>` , followed by a 3D translation.

Computes the result of a general displacement, followed by a general 3-d translation.

$R_i \rightarrow R_o$

$v1i + ri*v2i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - vi[3]

        - a general 3-d Vector

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a8e06360d1d46eb7f157abbdf4461c3c8:
.. _cid-s2_product_dtinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dtinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of a general :ref:`s2_displacement_t <doxid-structs2__displacement__t>` , followed by a 3D translation.

Computes the result of a general displacement, followed by the inverse of a general 3-d translation.

$R_i \rightarrow R_o$

$v1i + ri*v2i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - vi[3]

        - a general 3-d Vector

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1ad6bdbea17b8900b6cdc3efa1b55d8dfc:
.. _cid-s2_product_dv:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dv (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Double precision product of a general :ref:`s2_displacement_t <doxid-structs2__displacement__t>` with a vector.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - vi

        - a general 3-d Vector

    *
        - vo

        - the results are stored in vector, $v_o$

.. _doxid-kinematics_8h_1ae5f43e0d3cb88762594473fff8daeb9c:
.. _cid-s2_product_dinversev:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dinversev (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vo
	    )

Double precision product of a general :ref:`s2_displacement_t <doxid-structs2__displacement__t>` inverse with a vector.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D$

    *
        - vi

        - a general 3-d Vector

    *
        - vo

        - the results are stored in vector, $v_o$

.. _doxid-kinematics_8h_1a6a77ce62726105d8a879db5724b5b875:
.. _cid-s2_product_dtf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dtf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

Product of a general :ref:`s2_displacement_t <doxid-structs2__displacement__t>` and a displacement consisting of only a translation component.

Computes the result of a general displacement, followed by a general 3-d translation.

$R_i \rightarrow R_o$

$v1i + ri*v2i \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - vi[3]

        - a general 3-d translation

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a17263fa52606e787fa5d8bba9c89f40a:
.. _cid-s2_product_dinverset:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dinverset (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* vi,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of the inverse of a general displacement, $D_i^{-1}$ , and a general translation $v_{i}$ .

The following operation: $D_i^{-1} * V \rightarrow D_o$

Can be split in to:

$R_i^{T} \rightarrow R_o$

$R_i^T * (V_i - T_i) \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - dc[2]

        - direction cosines for a rotation about the Z axis.

    *
        - Do

        - the results are returned in a general displacement, $D_o$

.. _doxid-kinematics_8h_1a4b80e719835e914359d8a38cb366e16e:
.. _cid-s2_product_dinversetf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dinversetf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* vi,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision product of the inverse of a general displacement, $D_i^{-1}$ , and a general translation $v_{i}$

The following operation: $D_i^{-1} * V \rightarrow D_o$

Can be split in to:

$R_i^{T} \rightarrow R_o$

$R_i^T * (V_i - T_i) \rightarrow T_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - dc[2]

        - direction cosines for a rotation about the Z axis.

    *
        - Do

        - the results are returned in a general displacement, $D_o$

.. _doxid-kinematics_8h_1aeee4ecce38faba9bc466af2c1bd3edcf:
.. _cid-s2_product_dtx:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dtx (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Product of a general displacement and a primitive translation along the x-axis.

Computes the result of a general displacement, followed by a primitive 1-d translation.

$R_i \rightarrow R_o$

$t_i + R_i * t_x \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - ti

        - a double representing a primitive 1-d translation

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a970ad48f1c6a9707e83e666aed99a6de:
.. _cid-s2_product_dtxinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dtxinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of a :ref:`s2_displacement_t <doxid-structs2__displacement__t>` and a primitive translation.

Computes the result of a general displacement, followed by a primitive 1-d translation.

$R_i \rightarrow R_o$

$t_i - R_i * t_x \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - ti

        - a double representing a primitive 1-d translation

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a0d8d4ba1c266bacfa2f12540cf92f6a2:
.. _cid-s2_product_dtxinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dtxinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float* ti,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision product of a :ref:`s2_displacement_t <doxid-structs2__displacement__t>` and a primitive translation

Computes the result of a general displacement, followed by a primitive 1-d translation.

$R_i \rightarrow R_o$

$t_i - R_i * t_x \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - ti

        - a double representing a primitive 1-d translation

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1ae122c8001e90777ea7fe609068821395:
.. _cid-s2_product_dty:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dty (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision general :ref:`s2_displacement_t <doxid-structs2__displacement__t>` multiplied with a primitive translation along the y-axis.

$R_i \rightarrow R_o$

$t_i + R_i * t_y \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - ti

        - a double representing a primitive 1-d translation

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a2638411003d29bb69dac0b965ce93a04:
.. _cid-s2_product_dtyf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dtyf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float* ti,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision :ref:`s2_displacementf_t <doxid-structs2__displacementf__t>` multiplied with a primitive translation along the y-axis

$R_i \rightarrow R_o$

$t_i + R_i * t_y \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - ti

        - a double representing a primitive 1-d translation

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1acceeede744d34de6ea2776788ddb0bd5:
.. _cid-s2_product_dtyinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dtyinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision :ref:`s2_displacement_t <doxid-structs2__displacement__t>` multiplied with the inverse of a primitive translation along y axis.

Computes the result of a general displacement, followed by a primitive 1-d translation.

$R_i \rightarrow R_o$

$t_i - R_i * t_y \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - ti

        - a double representing a primitive 1-d translation

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a60a46355025f1f2833f25ce3e795652d:
.. _cid-s2_product_dtyinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dtyinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float* ti,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision :ref:`s2_displacementf_t <doxid-structs2__displacementf__t>` multiplied with the inverse of a primitive translation along y axis

Computes the result of a general displacement, followed by a primitive 1-d translation.

$R_i \rightarrow R_o$

$t_i - R_i * t_y \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - ti

        - a double representing a primitive 1-d translation

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a1aa3681f8b87680175ba0df890b3affb:
.. _cid-s2_product_dtz:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dtz (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision :ref:`s2_displacement_t <doxid-structs2__displacement__t>` multiplied with a primitive translation along z axis.

Computes the result of a general displacement, followed by a primitive 1-d translation along z axis.

$R_i \rightarrow R_o$

$t_i + R_i * t_z \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - ti

        - a double representing a primitive 1-d translation

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a420856b56c387d8c22f0573cc3ac982f:
.. _cid-s2_product_dtzf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dtzf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    float* ti,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision :ref:`s2_displacementf_t <doxid-structs2__displacementf__t>` multiplied with a primitive translation along z axis

Computes the result of a general displacement, followed by a primitive 1-d translation along z axis.

$R_i \rightarrow R_o$

$t_i + R_i * t_z \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - ti

        - a double representing a primitive 1-d translation

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1acd8930cf0605753897c366584c336cf4:
.. _cid-s2_product_dtzinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dtzinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    double* ti,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision :ref:`s2_displacement_t <doxid-structs2__displacement__t>` , multiplied with the inverse of a primitive translation along z axis.

Computes the result of a general displacement, followed by the inverse of a primitive 1-d translation along the z axis.

$R_i \rightarrow R_o$

$t_i - R_i * t_z \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement, $D_i$

    *
        - ti

        - a double representing a primitive 1-d translation

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1aa77f212503f2e0578b3827355c1e9a8a:
.. _cid-s2_product_dr:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dr (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* Ri,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of q displacement matrix with a mat3x3 rotation matrix.

Computes the product of two general displacement matrices consisting of both a rotation and translation component

$D_{1i} * D_{2i} \rightarrow D_{0}$

can be simplified as:

$R_{1i}*R_{2i} \rightarrow R_{o}$

$t_{1i} \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - D1i

        - a first general displacement, $D_{1i}$

    *
        - D2i

        - a second general displacement, $D_{2i}$

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1ad0eb8ef5f9c9840830c50e2ea3538cd2:
.. _cid-s2_product_drf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_drf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* Ri,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

Double precision product of q displacement matrix with a mat3x3 rotation matrix.

Computes the product of two general displacement matrices consisting of both a rotation and translation component

$D_{1i} * D_{2i} \rightarrow D_{0}$

can be simplified as:

$R_{1i}*R_{2i} \rightarrow R_{o}$

$t_{1i} \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - D1i

        - a first general displacement, $D_{1i}$

    *
        - D2i

        - a second general displacement, $D_{2i}$

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1aa4d4444f8a88684f39f4fd78caaa2c89:
.. _cid-s2_product_drinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_drinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* Ri,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of q displacement matrix with a mat3x3 rotation matrix.

Computes the product of two general displacement matrices consisting of both a rotation and translation component

$D_{1i} * D_{2i} \rightarrow D_{0}$

can be simplified as:

$R_{1i}*R_{2i} \rightarrow R_{o}$

$t_{1i} \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - D1i

        - a first general displacement, $D_{1i}$

    *
        - D2i

        - a second general displacement, $D_{2i}$

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a9143a8e4a7b2afa8e68635e413d7696d:
.. _cid-s2_product_drinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_drinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* Ri,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

Double precision product of q displacement matrix with a mat3x3 rotation matrix.

Computes the product of two general displacement matrices consisting of both a rotation and translation component

$D_{1i} * D_{2i} \rightarrow D_{0}$

can be simplified as:

$R_{1i}*R_{2i} \rightarrow R_{o}$

$t_{1i} \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - D1i

        - a first general displacement, $D_{1i}$

    *
        - D2i

        - a second general displacement, $D_{2i}$

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a27ced4c2a0730cde80969a1d78be23ca:
.. _cid-s2_product_dd:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_dd (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D1i,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D2i,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of two general displacement matrices.

Computes the product of two general displacement matrices consisting of both a rotation and translation component

$D_{1i} * D_{2i} \rightarrow D_{0}$

can be simplified as:

$R_{1i}*R_{2i} \rightarrow R_{o}$

$t_{1i} + R_{1i} * t_{2i} \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - D1i

        - a first general displacement, $D_{1i}$

    *
        - D2i

        - a second general displacement, $D_{2i}$

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a31b0cb8c4cc6ac3a1f7f5d2a0b58472d:
.. _cid-s2_product_ddf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_ddf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D1i,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D2i,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision product of two general displacement matrices

Computes the product of two general displacement matrices consisting of both a rotation and translation component

$D_{1i} * D_{2i} \rightarrow D_{0}$

can be simplified as:

$R_{1i}*R_{2i} \rightarrow R_{o}$

$t_{1i} + R_{1i} * t_{2i} \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - D1i

        - a first general displacement, $D_{1i}$

    *
        - D2i

        - a second general displacement, $D_{2i}$

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a6c6a40a92c022567fa99067d599cdb61:
.. _cid-s2_product_ddinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_ddinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D1i,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* D2i,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Double precision product of a first general displacement matrix and the inverse of a second displacement matrix.

Computes the product of two general displacement matrices consisting of both a rotation and translation component

$D_{1i} * D_{2i}^{-1} \rightarrow D_{0}$

can be simplified as:

$R_{1i}*R_{2i}^T \rightarrow R_{o}$

$t_{1i} - R_{1i} * t_{2i} \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - D1i

        - a first general displacement, $D_{1i}$

    *
        - D2i

        - a second general displacement, $D_{2i}$

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a1547358f17dff0717c8e12e280a92805:
.. _cid-s2_product_ddinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_product_ddinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D1i,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* D2i,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

single precision product of a first general displacement matrix and the inverse of a second displacement matrix

Computes the product of two general displacement matrices consisting of both a rotation and translation component

$D_{1i} * D_{2i}^{-1} \rightarrow D_{0}$

can be simplified as:

$R_{1i}*R_{2i}^T \rightarrow R_{o}$

$t_{1i} - R_{1i} * t_{2i} \rightarrow t_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - D1i

        - a first general displacement, $D_{1i}$

    *
        - D2i

        - a second general displacement, $D_{2i}$

    *
        - Do

        - the results are stored in the general displacement, $D_o$

.. _doxid-kinematics_8h_1a39432c21f1f424b93ee2c4d910d650bd:
.. _cid-s2_htx:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_htx (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for a primitive translation along x-axis.

Computes the influence coefficient matrix for a translation along the x-axis.

$D[0,3,6] \rightarrow h_{tx}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is initially set to zero.

$0 \rightarrow h_{tx}[3:5]$

.. _doxid-kinematics_8h_1abd6631002506449fb71711d11dcb470b:
.. _cid-s2_htxf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_htxf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

single precision influence coefficient matrix for a primitive translation along x-axis

Computes the influence coefficient matrix for a translation along the x-axis.

$D[0,3,6] \rightarrow h_{tx}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is d to zero.

$0 \rightarrow h_{tx}[3:5]$

.. _doxid-kinematics_8h_1af5f0478f99260e9b0e70eed2c14093de:
.. _cid-s2_hty:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hty (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for a primitive translation along y-axis.

Computes the influence coefficient matrix for a translation along the y-axis.

$D[1,4,7] \rightarrow h_{tx}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is d to zero.

$0 \rightarrow h_{tx}[3:5]$

.. _doxid-kinematics_8h_1a73184cc4e2d7243451a44154515a4e05:
.. _cid-s2_htyf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_htyf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

single precision influence coefficient matrix for a primitive translation along y-axis

Computes the influence coefficient matrix for a translation along the y-axis.

$D[1,4,7] \rightarrow h_{tx}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is d to zero.

$0 \rightarrow h_{tx}[3:5]$

.. _doxid-kinematics_8h_1a9fcd18da73023ae0baf8375ae728a35d:
.. _cid-s2_htz:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_htz (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* ri,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for a primitive translation along z-axis.

Computes the influence coefficient matrix for a translation along the z-axis.

$D[2,5,8] \rightarrow h_{tx}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is d to zero.

$0 \rightarrow h_{tx}[3:5]$

.. _doxid-kinematics_8h_1a1687304ba75e42edef025d08ecbf97e9:
.. _cid-s2_htzf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_htzf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* ri,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

single precision influence coefficient matrix for a primitive translation along z-axis

Computes the influence coefficient matrix for a translation along the z-axis.

$D[2,5,8] \rightarrow h_{tx}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is d to zero.

$0 \rightarrow h_{tx}[3:5]$

.. _doxid-kinematics_8h_1a3a38f85e3e4ccc3c7cbc936c4852143d:
.. _cid-s2_hrx:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hrx (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for a primitive rotation about x-axis.

Computes the influence coefficient matrix for a rotation about x-axis.

$D[0,3,6] \rightarrow h_{tx}[3:5]$

$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]$

.. _doxid-kinematics_8h_1a7a414a13bc67606532d0575d87837387:
.. _cid-s2_hrxf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hrxf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

single precision influence coefficient matrix for a primitive rotation about x-axis

Computes the influence coefficient matrix for a rotation about x-axis.

$D[0,3,6] \rightarrow h_{tx}[3:5]$

$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]$

.. _doxid-kinematics_8h_1a004c36b2c6363164dd6d48dce004143f:
.. _cid-s2_hry:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hry (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for a primitive rotation about y-axis.

Computes the influence coefficient matrix for a rotation about y-axis.

$D[1,4,7] \rightarrow h_{tx}[3:5]$

$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]$

.. _doxid-kinematics_8h_1ab3e368edee7878801f32b851eedee9f1:
.. _cid-s2_hrz:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hrz (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for a primitive rotation about z-axis.

Computes the influence coefficient matrix for a rotation about z-axis.

$D[2,5,8] \rightarrow h_{tz}[3:5]$

$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]$

.. _doxid-kinematics_8h_1a96541e40cb200d159823089421357a37:
.. _cid-s2_hrzf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hrzf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

single precision influence coefficient matrix for a primitive rotation about z-axis

Computes the influence coefficient matrix for a rotation about z-axis.

$D[2,5,8] \rightarrow h_{tz}[3:5]$

$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]$

.. _doxid-kinematics_8h_1a8ab56e3d8b15362fb200637e5f69a116:
.. _cid-s2_htxinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_htxinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for inverse primitive translation along x-axis.

Computes the influence coefficient matrix for an inverse primitive translation along the x-axis.

$-D[0,3,6] \rightarrow h_{tx}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is d to zero.

$0 \rightarrow h_{tx}[3:5]$

.. _doxid-kinematics_8h_1ad1efa81d59f93234bdc578514ae0f0fa:
.. _cid-s2_htxinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_htxinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

single precision influence coefficient matrix for inverse primitive translation along x-axis

Computes the influence coefficient matrix for an inverse primitive translation along the x-axis.

$-D[0,3,6] \rightarrow h_{tx}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is d to zero.

$0 \rightarrow h_{tx}[3:5]$

.. _doxid-kinematics_8h_1afee0d6416d3465e5483a5f9acdd51d98:
.. _cid-s2_htyinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_htyinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for inverse primitive translation along y-axis.

Computes the influence coefficient matrix for an inverse primitive translation along the y-axis.

$-D[1,4,7] \rightarrow h_{ty}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is d to zero.

$0 \rightarrow h_{ty}[3:5]$

.. _doxid-kinematics_8h_1a2a43d80b36e7201f3e11eec9da4ef82a:
.. _cid-s2_htyinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_htyinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

single precision influence coefficient matrix for inverse primitive translation along y-axis

Computes the influence coefficient matrix for an inverse primitive translation along the y-axis.

$-D[1,4,7] \rightarrow h_{ty}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is d to zero.

$0 \rightarrow h_{ty}[3:5]$

.. _doxid-kinematics_8h_1a1aa156cb200c1143bab7fff45fedfdbc:
.. _cid-s2_htzinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_htzinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for inverse primitive translation along z-axis.

Computes the influence coefficient matrix for an inverse primitive translation along the z-axis.

$-D[1,4,7] \rightarrow h_{tz}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is d to zero.

$0 \rightarrow h_{tz}[3:5]$

.. _doxid-kinematics_8h_1a38c955050e890a4da54ad1fb8865a54f:
.. _cid-s2_htzinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_htzinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

single precision influence coefficient matrix for inverse primitive translation along z-axis

Computes the influence coefficient matrix for an inverse primitive translation along the z-axis.

$-D[1,4,7] \rightarrow h_{tz}[0:2]$

Note that the last three elements of ho will always be zero, so the following operation need not be computed if ho is d to zero.

$0 \rightarrow h_{tz}[3:5]$

.. _doxid-kinematics_8h_1a44c5133b033dd6ff80bfde1f96afdc90:
.. _cid-s2_hrxinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hrxinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for inverse of a primitive rotation about x-axis.

Computes the influence coefficient matrix for the inverse of a primitive rotation about x-axis.

$-D[0,3,6] \rightarrow h_{tx}[3:5]$

$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]$

.. _doxid-kinematics_8h_1ab297a11438b4748cff530fb4da8a2043:
.. _cid-s2_hrxinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hrxinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

single precision influence coefficient matrix for inverse of a primitive rotation about x-axis

Computes the influence coefficient matrix for the inverse of a primitive rotation about x-axis.

$-D[0,3,6] \rightarrow h_{tx}[3:5]$

$T_i \times h_{tx}[3:5] \rightarrow h_{rx}[0:2]$

.. _doxid-kinematics_8h_1a2f0d67033095676e7031e0bb71e8f3fa:
.. _cid-s2_hryinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hryinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for inverse of a primitive rotation about y-axis.

Computes the influence coefficient matrix for the inverse of a primitive rotation about y-axis.

$-D[1,4,7] \rightarrow h_{ty}[3:5]$

$T_i \times h_{ty}[3:5] \rightarrow h_{ry}[0:2]$

.. _doxid-kinematics_8h_1ac0c2f74284fe9459f1148959b0bd048e:
.. _cid-s2_hryinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hryinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

single precision influence coefficient matrix for inverse of a primitive rotation about y-axis

Computes the influence coefficient matrix for the inverse of a primitive rotation about y-axis.

$-D[1,4,7] \rightarrow h_{ty}[3:5]$

$T_i \times h_{ty}[3:5] \rightarrow h_{ry}[0:2]$

.. _doxid-kinematics_8h_1aa72b91da77d60ce19fb1a4047d059262:
.. _cid-s2_hrzinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hrzinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* ho
	    )

Double precision influence coefficient matrix for inverse of a primitive rotation about z-axis.

Computes the influence coefficient matrix for the inverse of a primitive rotation about z-axis.

$-D[2,5,8] \rightarrow h_{tz}[3:5]$

$T_i \times h_{tz}[3:5] \rightarrow h_{rz}[0:2]$

.. _doxid-kinematics_8h_1aa2e93309ef463dddd8e738c9b1514726:
.. _cid-s2_hrzinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_hrzinversef (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* ho
	    )

single precision influence coefficient matrix for inverse of a primitive rotation about z-axis

Computes the influence coefficient matrix for the inverse of a primitive rotation about z-axis.

$-D[2,5,8] \rightarrow h_{tz}[3:5]$

$T_i \times h_{tz}[3:5] \rightarrow h_{rz}[0:2]$

.. _doxid-kinematics_8h_1aeff653ad6b7f5c62f6f77d99ddb6d93d:
.. _cid-s2_screwcross_ss:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwcross_ss (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

Compute double precision :ref:`s2_screw_t <doxid-structs2__screw__t>` Cross product in configuration space.

$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]$

$S_{1i}[0:2] \times S_{2i}[0:2] + S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first 6x1 screw

    *
        - s2i

        - a second 6x1 screw

    *
        - The

        - results of s1i crossed with s2i are stored in so

.. _doxid-kinematics_8h_1a88e4a2073007f92d153fa298ff12ef97:
.. _cid-s2_screwcross_ssf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwcross_ssf (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

Compute single precision :ref:`s2_screwf_t <doxid-structs2__screwf__t>` Cross product in configuration space.

$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]$

$S_{1i}[0:2] \times S_{2i}[0:2] + S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first 6x1 screw

    *
        - s2i

        - a second 6x1 screw

    *
        - The

        - results of s1i crossed with s2i are stored in so

.. _doxid-kinematics_8h_1af03950ad796422eb9beda69ffb2124af:
.. _cid-s2_screwcross_sst:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwcross_sst (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

Compute a double precision :ref:`s2_screw_t <doxid-structs2__screw__t>` Cross product in configuration space Compute :ref:`s2_screw_t <doxid-structs2__screw__t>` Cross product in configuration space, where the second screw consists of only a translation component.

$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]$

$0 \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first 6x1 screw

    *
        - s2i

        - a second 6x1 screw

    *
        - The

        - results of s1i crossed with s2i are stored in so

.. _doxid-kinematics_8h_1aaf1335946c59463c4ba81532b6bd217a:
.. _cid-s2_screwcross_sstf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwcross_sstf (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

Compute a single precision :ref:`s2_screwf_t <doxid-structs2__screwf__t>` Cross product in configuration space Compute :ref:`s2_screw_t <doxid-structs2__screw__t>` Cross product in configuration space, where the second screw consists of only a translation component.

$S_{1i}[3:5] \times S_{2i}[0:2] \rightarrow S_o[0:2]$

$0 \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first 6x1 screw

    *
        - s2i

        - a second 6x1 screw

    *
        - The

        - results of s1i crossed with s2i are stored in so

.. _doxid-kinematics_8h_1a85c2a4a64a3051c63e8f25332426734b:
.. _cid-s2_screwproduct_dts:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwproduct_dts (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* ti,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

Double precision translate a screw, si, in configuration space by ti and store in so.

$S_i[0:2] + T_i \times S_i[3:5] \rightarrow S_o[0:2]$

$si[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ti

        - a general 3-d translation

    *
        - si

        - a screw to translate

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1af0cefda2932c7810e7e7bdccfed7f946:
.. _cid-s2_screwproduct_dtsf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwproduct_dtsf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* ti,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

single precision translate a screw, si, in configuration space by ti and store in so.

$S_i[0:2] + T_i \times S_i[3:5] \rightarrow S_o[0:2]$

$si[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ti

        - a general 3-d translation

    *
        - si

        - a screw to translate

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a3eab6fdaf8ccc9af0edd32b4dc93c6fc:
.. _cid-s2_screwproduct_dtinverses:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwproduct_dtinverses (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* ti,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

In configuration space, double precision translate screw si by ti^-1 and store in so.

$S_i[0:2] - T_i \times S_i[3:5] \rightarrow S_o[0:2]$

$si[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ti

        - a general 3-d translation

    *
        - si

        - a screw to translate

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a308b114b524c248e1c64889879530a2e:
.. _cid-s2_screwproduct_dtinversesf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwproduct_dtinversesf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* ti,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

In configuration space, single precision translate screw si by ti^1 and store in so.

$S_i[0:2] - T_i \times S_i[3:5] \rightarrow S_o[0:2]$

$si[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ti

        - a general 3-d translation

    *
        - si

        - a screw to translate

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a09ed194b68010ab4e25f429f1b79d2b5:
.. _cid-s2_screwproduct_dtsinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwproduct_dtsinverse (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* ti,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

In configuration space, double precision translate screw si by ti^-1 and store in so.

$S_i[0:2] - T_i \times S_i[3:5] \rightarrow S_o[0:2]$

$si[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ti

        - a general 3-d translation

    *
        - si

        - a screw to translate

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a0e746d14b97e6f214f72d5707b49c848:
.. _cid-s2_screwproduct_dtsinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwproduct_dtsinversef (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* ti,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

In configuration space, single precision translate screw si by ti^1 and store in so.

$S_i[0:2] - T_i \times S_i[3:5] \rightarrow S_o[0:2]$

$si[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ti

        - a general 3-d translation

    *
        - si

        - a screw to translate

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a6e34d4bef2ed8f3181a0b2c13731bc7e:
.. _cid-s2_screwproduct_ds:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwproduct_ds (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

Double precision displace screw si by general displacement and store in so.

$D_i * S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement

    *
        - si

        - a screw to transform

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a0506b7b95a98175d4d0f75fd153a3433:
.. _cid-s2_screwproduct_dsinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwproduct_dsinverse (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

Double precision displace negative screw si by general displacement and store in so.

$D_i * S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement

    *
        - si

        - a screw to transform

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a9be1ab951ef968903b0250f2619b6f25:
.. _cid-s2_screwproduct_dsf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwproduct_dsf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

single precision displace screw si by general displacement and store in Do.

$D_i * S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement

    *
        - si

        - a screw to transform

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1ac9d258021e5fe6eef779a912e39bc1a5:
.. _cid-s2_screwproduct_dinverses:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwproduct_dinverses (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

In configuration space, double precision displace screw si by ti^-1 & ri^-1 and store in so.

$D_i^{-1} * S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement

    *
        - si

        - a screw to transform

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1abea2d2f876cbde696804af5f91ee4f56:
.. _cid-s2_screwproduct_dinversesf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_screwproduct_dinversesf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

In configuration space, single precision displace screw si by ti^-1 & ri^-1 and store in so.

$D_i^{-1} * S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement

    *
        - si

        - a screw to transform

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a1a906e11e10975c319de1ebd09eafed1:
.. _cid-s2_coscrewcross_ss:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_coscrewcross_ss (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s1i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* s2i,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

In function space, store spatial cross product of s1i with s2i in so.

$ S_{1i}[3:5]\times S_{2i}[0:2] + S_{1i}[0:2] \times S_{2i}[3:5] \rightarrow S_o[0:2]$

$ S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first coscrew

    *
        - s2i

        - a second coscrew

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a68b0b32ea55dba38bea5c4e5e37c20b0:
.. _cid-s2_coscrewcross_ssf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_coscrewcross_ssf (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s1i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* s2i,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

In function space, store single precision spatial cross product of s1i with s2i in so.

$ S_{1i}[3:5]\times S_{2i}[0:2] + S_{1i}[0:2] \times S_{2i}[3:5] \rightarrow S_o[0:2]$

$ S_{1i}[3:5] \times S_{2i}[3:5] \rightarrow S_o[3:5]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - s1i

        - a first coscrew

    *
        - s2i

        - a second coscrew

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a4b04bb7cde275fbf47f18a7d7a00c311:
.. _cid-s2_coscrewproduct_dts:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_coscrewproduct_dts (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* ti,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

In function space displace spatial vector si by ti^T and store in so.

$S_i[3:5] - T_i \times S_i[0:2] \rightarrow S_o[3:5] $

$S_i[0:2] \rightarrow S_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ti

        - a 3-d translation

    *
        - si

        - a coscrew

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a7b07097f823b02863fc2cefb6d6af0a0:
.. _cid-s2_coscrewproduct_dtsf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_coscrewproduct_dtsf (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* ti,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

In function space displace single precision spatial vector si by ti^T and store in so.

$S_i[3:5] - T_i \times S_i[0:2] \rightarrow S_o[3:5] $

$S_i[0:2] \rightarrow S_o[0:2]$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ti

        - a 3-d translation

    *
        - si

        - a coscrew

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a383e5a274d53cc3148695c3738325885:
.. _cid-s2_coscrewproduct_dtsinverse:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_coscrewproduct_dtsinverse (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* ti,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

In function space displace spatial vector si by ti^-T and store in so.

$ S_i[3:5] + T_i \times S_i[0:2] \rightarrow S_o[3:5]$

$ S_i[0:2] \rightarrow S_o[0:2] $

.. _doxid-kinematics_8h_1afa063e8fd094d7aa717cc3feec7b3922:
.. _cid-s2_coscrewproduct_dtsinversef:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_coscrewproduct_dtsinversef (
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* ti,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

In function space displace single precision spatial vector si by ti^-T and store in so.

$ S_i[3:5] + T_i \times S_i[0:2] \rightarrow S_o[3:5]$

$ S_i[0:2] \rightarrow S_o[0:2] $

.. _doxid-kinematics_8h_1a7f322f2b6d722b3b24ac429aaa1d3d06:
.. _cid-s2_coscrewproduct_ds:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_coscrewproduct_ds (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

In function space displace spatial vector si by ti^T and ri^-1 and store in so.

$ R_i^{-1} * T_i^T * S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement

    *
        - si

        - a coscrew

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a4ba1fcb1e7150f7d58aad7d1e4e9e797:
.. _cid-s2_coscrewproduct_dsf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_coscrewproduct_dsf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

In function space displace single precision spatial vector si by ti^T and ri^-1 and store in so.

$ R_i^{-1} * T_i^T * S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement

    *
        - si

        - a coscrew

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1ae32c2e74261e720132359d0d5e4a5c1f:
.. _cid-s2_coscrewproduct_dinverses:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_coscrewproduct_dinverses (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* so
	    )

In function space, displace spatial vector si by ri and ti^-T and store in so.

$ T_i^{-T} * R_i * S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement

    *
        - si

        - a coscrew

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1ad503267f56b288f35c72007bbd3c2cec:
.. _cid-s2_coscrewproduct_dinversesf:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_coscrewproduct_dinversesf (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* so
	    )

In function space, displace single precision spatial vector si by ri and ti^-T and store in so.

$ T_i^{-T} * R_i * S_i \rightarrow S_o$



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a general displacement

    *
        - si

        - a coscrew

    *
        - so

        - the results are stored in so

.. _doxid-kinematics_8h_1a354549fff525fa4feca104eb4ce519ad:
.. _cid-s2_quaternion_normalize:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_quaternion_normalize (:ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qi)

Normalize a double precision quaternion.

Normalize a quaternion and overwrite.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - qi

        - euler parameters to normalize

.. _doxid-kinematics_8h_1a85426cef2545af77d69ce3386ea92950:
.. _cid-s2_quaternionf_normalize:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_quaternionf_normalize (:ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* qi)

Normalize single precision quaternion.

Normalize a quaternion and overwrite.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - qi

        - euler parameters to normalize

.. _doxid-kinematics_8h_1a86d102e7d45b0a9c27368b46b11a0462:
.. _cid-s2_mat3x3_toquaternion:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3_toQuaternion (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qo
	    )

Double precision function to convert :ref:`s2_mat3x3_t <doxid-structs2__mat3x3__t>` rotation matrix to quaternion representation.

$R = \begin{bmatrix} e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 & e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\ e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 + e_2e_3 - e_0e_1 - e_0e_1 \\ e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 & e_0^2-e_1^2-e_2^2+e_3^2 \\ \end{bmatrix}$

Todo Supply *ER* , a reference Euler Parameter. If necessary, flip the sign of *(E) so that each *(E+i) is the closest match to ER[i]. For this algorithm to work, *(E) must be a close follow-on to ER. This function would then prove more useful when continuous Euler Parameters are desired for interpolation purposes. Matrix R is quadratic in *(E) so *(E+i) and -*(E+i) give the same R.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a rotation matrix in a 9x1 column format

    *
        - qo

        - the resulting euler parameters are stored in the 4x1 array, qo

.. _doxid-kinematics_8h_1a0da18a57ba2a87e8a2c17f1f73c9d2ad:
.. _cid-s2_mat3x3f_toquaternion:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3f_toQuaternion (
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ri,
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* qo
	    )

single precision function to convert :ref:`s2_mat3x3_t <doxid-structs2__mat3x3__t>` rotation matrix to quaternion representation

$R = \begin{bmatrix} e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 & e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\ e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 + e_2e_3 - e_0e_1 - e_0e_1 \\ e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 & e_0^2-e_1^2-e_2^2+e_3^2 \\ \end{bmatrix}$

Todo Supply *ER* , a reference Euler Parameter. If necessary, flip the sign of *(E) so that each *(E+i) is the closest match to ER[i]. For this algorithm to work, *(E) must be a close follow-on to ER. This function would then prove more useful when continuous Euler Parameters are desired for interpolation purposes. Matrix R is quadratic in *(E) so *(E+i) and -*(E+i) give the same R.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - ri

        - a rotation matrix in a 9x1 column format

    *
        - qo

        - the resulting euler parameters are stored in the 4x1 array, qo

.. _doxid-kinematics_8h_1a0a1b3bce5132f4ae03192352d1ea8d5c:
.. _cid-s2_quaternion_tomat3x3:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_quaternion_toMat3x3 (
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qi,
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ro
	    )

Double precision function to convert quaternion to rotation matrix representation.

The following formula is used to perform conversion:

$R = \begin{bmatrix} e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 & e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\ e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 + e_2e_3 - e_0e_1 - e_0e_1 \\ e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 & e_0^2-e_1^2-e_2^2+e_3^2 \\ \end{bmatrix}$

The Quaternion will be normalized during conversion.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - qi

        - euler parameters to convert in a 4x1 array, in the form [w,x,y,z]

    *
        - ro

        - the resulting rotation matrix is stored in ro

.. _doxid-kinematics_8h_1aac0f5f19d6f975cdbb14b9acbc700ef8:
.. _cid-s2_quaternionf_tomat3x3f:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_quaternionf_toMat3x3f (
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* qi,
	    :ref:`s2_mat3x3f_t<doxid-structs2__mat3x3f__t>`* ro
	    )

single precision function to convert Quaternionf to rotation matrix ( :ref:`s2_mat3x3f_t <doxid-structs2__mat3x3f__t>` ) representation

The following formula is used to perform conversion:

$R = \begin{bmatrix} e_0^2 + e_1^2-e_2^2-e_3^2 & e_1e_2+e_1e_2-e_0e_3+e_0e_3 & e_3e_1+e_3e_1+e_0e_2+e_0e_2 \\ e_1e_2 + e_1e_2 + e_0e_3 + e_0e_3 & e_0^2 - e_1^2 + e_2^2 - e_3^2 & e_2e_3 + e_2e_3 - e_0e_1 - e_0e_1 \\ e_3e_1+e_3e_1-e_0e_2-e_0e_2 & e_2e_3+e_2e_3+e_0e_1+e_0e_1 & e_0^2-e_1^2-e_2^2+e_3^2 \\ \end{bmatrix}$

The Quaternion will be normalized during conversion.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - qi

        - euler parameters to convert in a 4x1 array, in the form [w,x,y,z]

    *
        - ro

        - the resulting rotation matrix is stored in ro

.. _doxid-kinematics_8h_1a49f717f1310c27e85e4bf3df53f9e665:
.. _cid-s2_quaternion_slerp:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_quaternion_slerp (
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* q1i,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* q2i,
	    double* t,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qo
	    )

Double precision, vectorized version of Shoemake's Spherical Linear Interpolation algorithm.

Refer to Shoemake, Animating rotation with Quaternion Curves, SIGGRAPH, vol 19, No. 3, 1985 for specific information on the algorithm.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - q1i

        - a first quaternion in the form [w,x,y,z] to interpolate from

    *
        - q2i

        - a second quaternion in the form [w,x,y,z] to interpolate to

    *
        - t

        - fraction from q1i to q2i (0<t<1)

    *
        - qo

        - the resulting quaternion is returned in qo

.. _doxid-kinematics_8h_1a2f0010db092c86fe7c2f7531dfda91ac:
.. _cid-s2_quaternionf_slerp:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_quaternionf_slerp (
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* q1i,
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* q2i,
	    float* t,
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* qo
	    )

single precision, vectorized version of Shoemake's Spherical Linear Interpolation algorithm

Refer to Shoemake, Animating rotation with Quaternion Curves, SIGGRAPH, vol 19, No. 3, 1985 for specific information on the algorithm.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - q1i

        - a first quaternion in the form [w,x,y,z] to interpolate from

    *
        - q2i

        - a second quaternion in the form [w,x,y,z] to interpolate to

    *
        - t

        - fraction from q1i to q2i (0<t<1)

    *
        - qo

        - the resulting quaternion is returned in qo

.. _doxid-kinematics_8h_1aabc18ea6f7b7d765ed598b8dfd1da89d:
.. _cid-s2_quaternion_nlerp:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_quaternion_nlerp (
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* q1i,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* q2i,
	    double* t,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qo
	    )

Vectorized version of quaternion Linear Interpolation (Lerp)

Less expensive numerically than Shoemake's Slerp algorithm (s2_slerp) but does not interpolate with constant velocity



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - q1i

        - a first quaternion in the form [w,x,y,z] to interpolate from

    *
        - q2i

        - a second quaternion in the form [w,x,y,z] to interpolate to

    *
        - t

        - fraction from q1i to q2i (0<t<1)

    *
        - qo

        - the resulting quaternion is returned in qo

.. _doxid-kinematics_8h_1a8bbf8a4063be02eac2296be1a0aa6ac0:
.. _cid-s2_quaternionf_nlerp:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_quaternionf_nlerp (
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* q1i,
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* q2i,
	    float* t,
	    :ref:`s2_quaternionf_t<doxid-structs2__quaternionf__t>`* qo
	    )

Vectorized version of quaternion Linear Interpolation (Lerp)

Less expensive numerically than Shoemake's Slerp algorithm (s2_Slerp) but does not interpolate with constant velocity



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - q1i

        - a first quaternion in the form [w,x,y,z] to interpolate from

    *
        - q2i

        - a second quaternion in the form [w,x,y,z] to interpolate to

    *
        - t

        - fraction from q1i to q2i (0<t<1)

    *
        - qo

        - the resulting quaternion is returned in qo

.. _doxid-kinematics_8h_1a72f3e874e98cf8c6abed83a0e3f3342e:
.. _cid-s2_normalizedegrees:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_normalizeDegrees (double angle)

Normalize double precision angle in degrees.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - normalizes

        - degrees to be between 0 and 360 degrees

.. _doxid-kinematics_8h_1a37ad6a2da9e09c0b1c4b2ad2a94e891f:
.. _cid-s2_normalizedegreesf:
.. ref-code-block:: cpp
	:class: title-code-block

	float s2_normalizeDegreesf (float angle)

Normalize single precision angle in degrees.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - normalizes

        - degrees to be between 0 and 360 degrees

.. _doxid-kinematics_8h_1a9d1705e23160bf6ecdee2cc4dc043717:
.. _cid-s2_normalizeradians:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_normalizeRadians (double angle)

Normalize angle in radians.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - normalizes

        - double precision radians to be between 0 and 2 * PI radians

.. _doxid-kinematics_8h_1a65d6444f4ab27cb61cf76f47cf7152f5:
.. _cid-s2_normalizeradiansf:
.. ref-code-block:: cpp
	:class: title-code-block

	float s2_normalizeRadiansf (float angle)

Normalize angle in radians.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - normalizes

        - single precision radians to be between 0 and 2 * PI radians

.. _doxid-kinematics_8h_1ad928cba56a9629532220a1543352990d:
.. _cid-catan2pi:
.. ref-code-block:: cpp
	:class: title-code-block

	double catan2pi (
	    double y,
	    double x,
	    double theta
	    )

returns a continuous angle obtained from atan2(y, x)

Caution: atan2(y,x) will return an angle with +-2m*pi accuracy only if the sign on x and y are both correct. If x and y are both off by (-1) then atan2 will return an angle with accuracy of only +-m*pi. If the sign of x and y are uncertain, use the alternate function, catan1pi.

This function returns a continuous angle obtained from the standard math atan2(y,x) function. On input, the argument theta contains the previous value associated with the angle that is being computed. catan2() adjusts the value returned from atan2(), so it will be as close to theta as possible. The assumption is that the current value to be returned by catan2() is not more than (+-)pi/4 from the previous value. This should be a valid assumption, because angles generally don't change very fast over the course of a simulation. Both input arguments x and y must have the correct sign for catan2pi to work correctly.

.. _doxid-kinematics_8h_1a56df7f77228d03737d92529022283ca3:
.. _cid-catan1pi:
.. ref-code-block:: cpp
	:class: title-code-block

	double catan1pi (
	    double y,
	    double x,
	    double theta
	    )

returns a continuous angle from the atan2(y, x) function

Caution: atan2(y,x) will return an angle with +-2m*pi accuracy only if the sign on x and y are both correct. If x and y are both off by (-1) then atan2 will return an angle with accuracy of only +-m*pi. If the sign of x and y are certain, this or the alternate function, catan2pi will work. If the sign of x and y are uncertain, do not use catan2pi. This function returns a continuous angle from the standard math atan2(y,x) function. On input, the argument theta contains the previous value associated with the angle that is being computed. catan2() adjusts the value returned from atan2(), so it will be as close to theta as possible. The assumption is that the current value to be returned by catan2() is not more than (+-)pi/4 from the previous value. This should be a valid assumption, because angles generally don't change very fast over the course of a simulation. Both input arguments x and y must either have the correct sign or be off by the same sign for catan1pia() to work correctly.

.. _doxid-kinematics_8h_1a2f7b885ce20d67815d343993c19a815a:
.. _cid-s2_mat3x3_geteulerangles:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat3x3_getEulerAngles (
	    :ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* rin,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* angles,
	    s2_axis_enum_t axisType
	    )

extract Euler angles from a :ref:`s2_mat3x3_t <doxid-structs2__mat3x3__t>` rotation matrix

This function decomposes a general direction cosine matrix into three primitive direction cosine matrices, whose axes are determined by the integers passed in through axis (see below). The direction cosine matrix is passed in through ri, and the three computed angles are updated and returned in angle. The previous values assigned to angle are passed in to RtoA in angle. RtoA calls catan2, which uses the values stored in angle to maintain continuity if at all possible.

This function does not check for bad axis indices. These checks could easily be added. For actual reporting purposes, especially when a large number of user output requests are to be processed, it would be more efficient to program four or twelve different functions and have a preprocessor generate pointers to the appropriate functions as needed. That way, no logical tests would have to be performed at runtime, except in the function catan2().

If four functions were coded, each would handle one of the four Cases below. In this case, the if statements would be eliminated, and only the axis indices would be needed. If twelve functions were coded, each would handle one of the twelve Cases. Again, the if statements would be eliminated, and the axis indices would be hard coded, eliminating the need to pass them in or to perform any other tests.

Used to determine if the next axis is forward '+' or reverse '-' sequence. It also identifies index locations in the direction cosine matrix when axes are repeated, such as in the sequence 1 > 2 > 1.

The function can be modified to decompose a matrix in any sequence of angles by by changing "axis", but presently the sequence [rx=0, ry=1, rz=2] is hard coded in to the function.

.. _doxid-kinematics_8h_1ae373aea0a098137de48a6532d419a35a:
.. _cid-s2_mat3x3_getscalar:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_mat3x3_getScalar (:ref:`s2_mat3x3_t<doxid-structs2__mat3x3__t>`* ri)

Extract scalar from a uniformly scaled :ref:`s2_mat3x3_t <doxid-structs2__mat3x3__t>` matrix.

The norm of each column or row of an orthonormal rotation matrix is one. Therefore, to find the scale factor for a rotation matrix that has been <uniformly> scaled, simply find the norm of any row or column of the rotation matrix. Warning, this approach will not work for nonuniform scaling. For nonuniform scaling, one needs to find the eigenvectors using SVD or other means.

.. _doxid-kinematics_8h_1a0283d3411d26c8f6bae2e0530e6fd3d2:
.. _cid-s2_mat4x4_gettranslation:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat4x4_getTranslation (
	    :ref:`s2_mat4x4_t<doxid-structs2__mat4x4__t>`* Mi,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* to
	    )

Extract scalar from a uniformly scaled single precision :ref:`s2_mat3x3f_t <doxid-structs2__mat3x3f__t>` matrix.

The norm of each column or row of an orthonormal rotation matrix is one.

Therefore, to find the scale factor for a rotation matrix that has been <uniformly> scaled, simply find the norm of any row or column of the rotation matrix. Warning, this approach will not work for nonuniform scaling. For nonuniform scaling, one needs to find the eigenvectors using SVD or other means.

extract translation components of a s2_Mat4 matrix

.. _doxid-kinematics_8h_1a9e28efe027651b9c70062f55c92dd4e1:
.. _cid-s2_mat4x4f_gettranslation:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mat4x4f_getTranslation (
	    :ref:`s2_mat4x4f_t<doxid-structs2__mat4x4f__t>`* Mi,
	    :ref:`s2_vectorf_t<doxid-structs2__vectorf__t>`* to
	    )

extract translation component of a s2_mat4x4f matrix

.. _doxid-kinematics_8h_1a842ea4c227f675665defe5afafdbfb2a:
.. _cid-s2_add_dpitoq:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_add_dpiToq (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* dpi,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qi
	    )

utility to convert dpi, an array of length 3, to deltaq (an incremental change in quaternion), add incremental change to an input quaternion

.. _doxid-kinematics_8h_1aa160158656d72b34767a9332aa3fb8ed:
.. _cid-s2_subtract_dpifromq:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_subtract_dpiFromq (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* dpi,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qi
	    )

utility to convert dpi, an array of length 3, to deltaq (an incremental change in quaternion), subtract incremental change from an input quaternion

.. _doxid-kinematics_8h_1a46a7a3faa7ae267d6838f4af751dbd86:
.. _cid-s2_omega_to_dquaternion:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_omega_to_dquaternion (
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* w,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* qi,
	    :ref:`s2_quaternion_t<doxid-structs2__quaternion__t>`* dqo
	    )

utility to convert angular velocity to time derivative of quaternion

$ \underline{\dot{e}}^{ij} \leftarrow 1/2 \mathscr{R}(\underline{e}) \underline{\omega}_{ij}^i$

The result is then normalized by adding a feedback term

$ \underline{\dot{e}}^{ij} - \frac{0.5}{\tau}(1 - \underline{e}^{ijT} \underline{e}^{ij})\underline{e}^{ij} $

where $\tau=0.5$ is hard coded



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - w

        - [in] omega, angular velocity

    *
        - qi

        - [in] reference quaternion

    *
        - dqo

        - [out] output quaternion time derivative

.. _doxid-kinematics_8h_1a8553fd17e8c7ce4bf1a39535fcb7994c:
.. _cid-s2_array_lerp:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_array_lerp (
	    int qCount,
	    double p,
	    double* y0,
	    double* y1,
	    double* yOut
	    )

linear interpolation nlerp



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - count

        - [in] number of entries to interpolate

    *
        - p

        - [in] interpolant value, ranges from 0 to 1

    *
        - y0

        - [in] value of function at y=0

    *
        - y1

        - [in] value of function at y=1

    *
        - yOut

        - [out] interpolated data returned in yOut

.. _doxid-kinematics_8h_1a34b1a2429f9da44179c47182b68064c7:
.. _cid-s2_gramschmidt:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_gramSchmidt (
	    int m,
	    int n,
	    double* A
	    )

Function for performing modified Gram-Schmidt orthonormalization.

This function implements a compact version of the Gram-Schmidt algorithm to orthonormalize the columns of a matrix which has full column rank. The matrix, whose columns are to be orthonormalized, is passed in as A[m][n] where m and n are the respective row and column dimensions of A[][]. The orthonormalized columns are passed back in A[][].



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - m

        - row dimension

    *
        - n

        - column dimension

    *
        - A[m][n]

        - a rectangular matrix to be orthonormalized, the orthonormalized matrix is returned in A

.. _doxid-kinematics_8h_1ad65acbd7c97bf3da11f2a67fabb83ab6:
.. _cid-s2_all_close:
.. ref-code-block:: cpp
	:class: title-code-block

	bool s2_all_close (
	    int count,
	    double* a1,
	    double* a2
	    )

used to determine if two double arrays are close to each other within an error tolerance of 1e-6



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - count

        - [in] number of entries in array

    *
        - a1

        - [in] first double array

    *
        - a2

        - [in] second double array



.. rubric:: Returns:

true if values are all close, false if values are not all close

.. _doxid-light_8h_1ad9765a978f23c92d27daf60435862fd8:
.. _cid-s2_light:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_light_t<doxid-structs2__light__t>`* s2_light ()

Initialize an :ref:`s2_light_t <doxid-structs2__light__t>` datastructure.

creates memory for :ref:`s2_light_t <doxid-structs2__light__t>` datastructure and initializes to default values



.. rubric:: Returns:

a Light datastructure

.. _doxid-light_8h_1a5c94c15851fdc4a29fd6ed14757e999d:
.. _cid-s2_light_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_light_free (:ref:`s2_light_t<doxid-structs2__light__t>`* light)

Free a Light datastructure.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - light

        - a Light datastructure

.. _doxid-light_8h_1ac13a30ffdcb499e7c60c623cac363790:
.. _cid-s2_light_load:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_light_load (
	    :ref:`s2_context_t<doxid-structs2__context__t>`*,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`*,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`*,
	    json_t*
	    )

load light from json file

.. _doxid-material_8h_1aea218d524bf86bfcc67505f90d73eeec:
.. _cid-s2_materials:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_material_t<doxid-structs2__material__t>`* s2_materials (int materialCount)

Initialize a Material datastructure.

.. _doxid-material_8h_1a26270fc1c9c7af76bb38f269fe20f28d:
.. _cid-s2_materials_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_materials_free (
	    :ref:`s2_material_t<doxid-structs2__material__t>`* materials,
	    int materialCount
	    )

free materials data structures

.. _doxid-memoize_8h_1ab08412c85f70b3de55c8e9d406cf982e:
.. _cid-s2_memoize_memoizeequations:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_memoize_memoizeEquations (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

memoize performs an initial factorization to count operations, then performs factorization again, recording the operations in integer arrays, finally, the instructions encoded in integer arrays are recorded in arrays of source and destination memory pointers

.. _doxid-mobility_8h_1afa410441c536e454ca16575d1eda3b66:
.. _cid-s2_mobility_compute_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mobility_compute_substructure (:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure)

minimal version of computing mobility numbers, assumes jacobian of substructre has already been factored using full factor and mobility numbers have been initialized

this function requires that s2_jacobian_setPO2_substructure, s2_jacobian_factorPO2_substructure and s2_jacobian_setBMatrix_substructure has been called prior to execution

.. _doxid-mobility_8h_1a5e42cbbf68e159fff14f950cae23db27:
.. _cid-s2_mobility_compute:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mobility_compute (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

Function for computing mobility numbers, s2_jacobian_setPO1 must be called prior to this function.

This function evaluates the variable mobility numbers in PO2. It first forms an n by n-r matrix from the submatrix J[0:r-1][r:n-1] stacked above an (n-r) by (n-r) identity matrix. Then it calls GramSchmidt() to orthonormalize the columns and return them in the same matrix. Finally it returns the square of row norms of this matrix as mobility numbers in the mn[] array.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - [in]

.. _doxid-mobility_8h_1af1e7878c7eb75610777442b9aa586cf1:
.. _cid-s2_mobility_initialize:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mobility_initialize (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

Initialize quantities necessary to compute mobility numbers.

This function evaluates the variable mobility numbers in permuted order. It first forms an n by n-r matrix from the submatrix J[0:r-1][r:n-1] stacked above an (n-r) by (n-r) identity matrix. Then it calls GramSchmidt() to orthonormalize the columns and return them in the same matrix. Finally it returns the square of row norms of this matrix as mobility numbers in the mn[] array.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - m

        - row dimension

    *
        - n

        -

.. _doxid-mobility_8h_1a0d364dc3b600247c3504f3214e1e3fb8:
.. _cid-s2_mobility_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mobility_print (
	    FILE* f,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

print mobility information,

.. _doxid-mobility_8h_1a09ba0060113d1938c45147279e73a54a:
.. _cid-s2_mobility_print_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mobility_print_substructure (
	    FILE* f,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex
	    )

prints mobility numbers for one substructure

.. _doxid-mobility_8h_1a351d36237fc26ddeb6b5ba18360382f9:
.. _cid-s2_mobility_computeglobalmobility:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_mobility_computeGlobalMobility (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

compute global instantaneous mobility, to be called after mobility has been computed within each substructure

.. _doxid-newtonraphson_8h_1a4033654cb0e578544930add89add4f4d:
.. _cid-s2_newtonraphson_evaluateconstrainterror_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_newtonRaphson_evaluateConstraintError_substructure (:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure)

computes constraint error in a substructure

error = sqrt(cs[] . cs[]) / constraintEquationCount

.. _doxid-newtonraphson_8h_1a656cf1465235dd983b20f6feea961888:
.. _cid-s2_newtonraphson_evaluateqerror_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_newtonRaphson_evaluateqError_substructure (:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* substructure)

computes error in change of delta q for a substructure

error = sqrt(deltaq[] . deltaq[]) / dqCount

.. _doxid-newtonraphson_8h_1a2696b332b31fd7d0a4d55e1bce020b19:
.. _cid-s2_newtonraphson_removeconstraintviolations:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_newtonRaphson_removeConstraintViolations (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

This function removes constraints for the entire mechanism using a strict error tolerance (context->epsNewtonRaphsonRankDetermination)



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - [in] an :ref:`s2_context_t <doxid-structs2__context__t>` datastructure

.. _doxid-newtonraphson_8h_1ab9c6ee525e16b453216e571bf8056bd0:
.. _cid-s2_newtonraphson_removeconstraintviolations_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_newtonRaphson_removeConstraintViolations_substructure (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex
	    )

This function removes constraints for the entire mechanism using a strict error tolerance (context->epsNewtonRaphsonRankDetermination)

on exit, sets the sparsity pattern of J in the Jb boolean matrix



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - [in] an :ref:`s2_context_t <doxid-structs2__context__t>` datastructure

.. _doxid-newtonraphson_8h_1a64e792a57875231f6c2780e825272d6d:
.. _cid-s2_newtonraphson_removeconstraintviolationsfast_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_newtonRaphson_removeConstraintViolationsFast_substructure (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex,
	    bool useTightTolerance
	    )

This function advances the solution of a set of constraints under user control of one or more of its independent variables.

Does not set the jacobian boolean matrix, which is necessary for solving the equations of motion.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - [in] :ref:`s2_context_t <doxid-structs2__context__t>` datastructure

.. _doxid-newtonraphson_8h_1a448b4d41be7de62c96f3e4754aa5f10e:
.. _cid-s2_newtonraphson_setup_specified:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_newtonRaphson_setup_specified (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex,
	    int specifiedCoordinateIndex
	    )

drive down constraint violations, then perform an initial full factor to set up slider for repositioning mechanism

removes constraint violations and performs a specified full factor in PO1, permutations are used for subsequent fast factorization

.. _doxid-newtonraphson_8h_1a941c03ef0794a5da4c703aac2a74cd92:
.. _cid-s2_newtonraphson_increment_specified:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_newtonRaphson_increment_specified (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int substructureIndex,
	    int specifiedCoordinateIndex,
	    double value,
	    :ref:`s2_joint_enum_t<doxid-joint_8h_1a7a43c175bb516fb74df6eb8503a0475c>` jointType,
	    :ref:`s2_move_enum_t<doxid-joint_8h_1a17c0fd52873eda0dcebfe02edce180cc>` moveType
	    )

This function advances the solution of a set of constraints under user control of one of its independent variables.

Full factor with a specified coordinate must be called prior to calling this function. A lower error tolerance (context->epsNewtonRaphson) is used to advance the position of the mechanism for visualization purposes, but a higher tolerance (context->epsNewtonRaphsonRankDetermination) is used for the purpose of determining the mechanism rank.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - [in] an :ref:`s2_context_t <doxid-structs2__context__t>` datastructure

    *
        - substructureIndex

        - [in] index of substructure to process

    *
        - specifiedCoordinateIndex

        - [in] global index of coordinate to set as specified

.. _doxid-pointcloud_8h_1a29a5d5879ca5f5c2278f8fe873c98f95:
.. _cid-s2_pointcloud:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_pointcloud_t<doxid-structs2__pointcloud__t>`* s2_pointcloud (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`*)

constructor for :ref:`s2_pointcloud_t <doxid-structs2__pointcloud__t>`

.. _doxid-pointcloud_8h_1aa3bdcd3f0dc5c11409fd5148bf826e34:
.. _cid-s2_pointcloud_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_pointcloud_free (:ref:`s2_pointcloud_t<doxid-structs2__pointcloud__t>`*)

destructor for :ref:`s2_pointcloud_t <doxid-structs2__pointcloud__t>`

.. _doxid-queue_8h_1a8948a6d9b2c828280d6c3dcf2855b805:
.. _cid-s2_queue:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_queue_t<doxid-structs2__queue__t>`* s2_queue ()

used to initialize a queue datastructure

.. _doxid-queue_8h_1aac72a13f0c311d229363e5d238ca67e0:
.. _cid-s2_queue_append:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_queue_append (
	    :ref:`s2_queue_t<doxid-structs2__queue__t>`* queue,
	    void* value,
	    s2_queue_enum_t type
	    )

append a value to the end of a queue datastructure

.. _doxid-queue_8h_1a672b3ec9b38b169a0de23576bfb01ad5:
.. _cid-s2_queue_pop:
.. ref-code-block:: cpp
	:class: title-code-block

	void* s2_queue_pop (:ref:`s2_queue_t<doxid-structs2__queue__t>`* queue)

pop the value off the front of a queue datastructure

.. _doxid-queue_8h_1a246929c223a14f1eb08a24d5979b381f:
.. _cid-s2_queue_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_queue_free (:ref:`s2_queue_t<doxid-structs2__queue__t>`* queue)

free memory for a queue datastructure

.. _doxid-restraint_8h_1a48017c62763694dc8244b8680c055b05:
.. _cid-s2_restraint:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_restraint_t<doxid-structs2__restraint__t>`* s2_restraint ()

Initialize an :ref:`s2_restraint_t <doxid-structs2__restraint__t>` datastructure.



.. rubric:: Returns:

a Restraint datastructure

.. _doxid-solve_8h_1a2df56c26f56eca9a44cb0eb78ce900e8:
.. _cid-s2_solve_integrateequations:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_solve_integrateEquations (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

wrapper function to ode integrator

.. _doxid-solve_8h_1ac4a44e1eb5c17a1bc19465d2f93c07f0:
.. _cid-s2_solve_momentum:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_solve_momentum (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

compute total spatial momentum for system

.. _doxid-solve_8h_1a35adcb92a308bbf156488434921c788b:
.. _cid-s2_solve_kineticenergy:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_solve_kineticEnergy (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

compute kinetic energy for system

.. _doxid-solve_8h_1a506f2143d6f10cec3fff7ce949c2346b:
.. _cid-s2_solve_gravitationalpotentialenergy:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_solve_gravitationalPotentialEnergy (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

computes total gravitational potential energy for system

result is stored in context->gravitationalPotentialEnergy



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - [in] context datastructure

.. _doxid-solve_8h_1a0cb4957a1a0b45d9e9d7ab08ccbdbbed:
.. _cid-s2_solve_verify:
.. ref-code-block:: cpp
	:class: title-code-block

	bool s2_solve_verify (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double* Ml,
	    double* fl
	    )

set coefficient matrix and multiply with answer to verify we get the right hand side back

.. _doxid-solve_8h_1ac2e1439d404d7bac5c26ebd6129aa8cc:
.. _cid-s2_equation_fcn:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_equation_fcn (
	    double t,
	    double y [],
	    double dy [],
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

generic function to solve equations of motion for one time step of numerical integration

.. _doxid-solve_8h_1aba986392ba0e60655634ff6926713443:
.. _cid-s2_solve_writeheaders:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_solve_writeHeaders (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

writes headers for output file to stdout and to the output file pointer specified in context->modelParameters->outFilePointer

.. _doxid-solve_8h_1a640ad4177d355b89cc907fea57180f4f:
.. _cid-s2_solve_writestates:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_solve_writeStates (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    int step
	    )

writes states to output file, ydata array and/or stdout

.. _doxid-sphere_8h_1acac5b4750ef8472ef0252c4317e978fa:
.. _cid-s2_sphere_approximateminboundsphere:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_sphere_approximateMinBoundSphere (
	    :ref:`s2_sphere_t<doxid-structs2__sphere__t>`*,
	    :ref:`s2_body_t<doxid-structs2__body__t>`*
	    )

Find a bounding sphere for a set of M points in 3-dimensional space using Ritter's algorithm. In 3D, the resulting sphere is approximately 5% bigger than the optimal minimum radius sphere.

REFERENCES: [1] Ritter, J. (1990), 'An efficient bounding sphere', in Graphics Gems, A. Glassner, Ed. Academic Press, pp.301303



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - radius

        - and centroid of bounding sphere returned in

    *
        - convexhull

        - 

    *
        - computing

        - using :ref:`s2_convexhull() <doxid-convexhull_8c_1acba435f489bbd775a98f583108bdeb1e>` defined in ``convexhull.h``

.. _doxid-sphere_8h_1a9bc196d4fd944d676699ceb8d13a32bc:
.. _cid-s2_sphere_updatecentroid:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_sphere_updateCentroid (:ref:`s2_sphere_t<doxid-structs2__sphere__t>`*)

update sphere's global centroid

.. _doxid-stack_8h_1afd6037900f9e8b5cbf689b146b1ace10:
.. _cid-s2_stack:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stack_t<doxid-structs2__stack__t>`* s2_stack ()

initialize a stack data structure

.. _doxid-stack_8h_1a7f24dbcee8d8aee6ccc7fdd2b527469d:
.. _cid-s2_stackstr:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* s2_stackstr ()

initialize a stack data structure

.. _doxid-stack_8h_1a043afb2a321da628790059d5a9edb105:
.. _cid-s2_stack_push:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_stack_push (
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack,
	    int value
	    )

push an integer onto the front of a stack

.. _doxid-stack_8h_1a561d62e8b60766d95c165cca29072146:
.. _cid-s2_stackstr_push:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_stackstr_push (
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack,
	    const char* value
	    )

push a const char* onto the front of a stack

.. _doxid-stack_8h_1a30863701a4fbb815aa2766bab6a7ed2c:
.. _cid-s2_stack_append:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_stack_append (
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack,
	    int value
	    )

append an integer onto the end of a stack

.. _doxid-stack_8h_1a6887a4ff408c987f03e8b111ed55941b:
.. _cid-s2_stack_remove:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_stack_remove (
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack,
	    :ref:`s2_stacknode_t<doxid-structs2__stacknode__t>`* node
	    )

remove a stacknode from a stack



.. rubric:: Returns:

int -1 for failure, 0 for success

.. _doxid-stack_8h_1ac5d5c7c676643be1a68311f5c965a518:
.. _cid-s2_stackstr_append:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_stackstr_append (
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack,
	    const char* value
	    )

append an integer onto the end of a stack

.. _doxid-stack_8h_1a3328beaff6aee8299d4ccd04c62f1397:
.. _cid-s2_stack_pop:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_stack_pop (:ref:`s2_stack_t<doxid-structs2__stack__t>`* stack)

pop the first value off of the stack

.. _doxid-stack_8h_1af107df27f1dbe23eaf58fdb7a7f1d478:
.. _cid-s2_stackstr_pop:
.. ref-code-block:: cpp
	:class: title-code-block

	const char* s2_stackstr_pop (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack)

pop the first value off of the stack

.. _doxid-stack_8h_1ac0d3dd3cda0476736e640d500776ee67:
.. _cid-s2_stack_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_stack_print (
	    FILE* f,
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack
	    )

print the contents of a stack datastructure

.. _doxid-stack_8h_1a5fd679a656c3393c43ee188954657e37:
.. _cid-s2_stackstr_print:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_stackstr_print (
	    FILE* f,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack
	    )

print the contents of a stack datastructure

.. _doxid-stack_8h_1ac7a7d8ada71e06e854788bdcc90bf41f:
.. _cid-s2_stack_merge:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_stack_merge (
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack1,
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack2
	    )

used to merge two stack data structures together. the merged stack will be in stack1, memory for stack2 will be freed after operation is complete

.. _doxid-stack_8h_1a0f16758ddabf8b1f64603d7a70d53aaa:
.. _cid-s2_stackstr_merge:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_stackstr_merge (
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack1,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack2
	    )

used to merge two stack data structures together. the merged stack will be in stack1, memory for stack2 will be freed after operation is complete

.. _doxid-stack_8h_1abfe0df16d02f11b5772b7908adfed0a1:
.. _cid-s2_stack_toarray:
.. ref-code-block:: cpp
	:class: title-code-block

	int* s2_stack_toArray (:ref:`s2_stack_t<doxid-structs2__stack__t>`* stack)

convert stack to a linear array

.. _doxid-stack_8h_1ad088b50bfcb9b4392a0046d8c16ec16a:
.. _cid-s2_stack_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_stack_free (:ref:`s2_stack_t<doxid-structs2__stack__t>`* stack)

free memory for a stack data structure

.. _doxid-stack_8h_1a1e0b5156595d851be28b5c84d0900be4:
.. _cid-s2_stackstr_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_stackstr_free (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack)

free memory for a stackStr data structure

.. _doxid-stack_8h_1a4d034e4d5260bc873d1fad659fef360f:
.. _cid-s2_stack_in:
.. ref-code-block:: cpp
	:class: title-code-block

	bool s2_stack_in (
	    :ref:`s2_stack_t<doxid-structs2__stack__t>`* stack,
	    int value
	    )

test to see if a value is in a stack



.. rubric:: Returns:

bool true if value is in stack, false otherwise

.. _doxid-stack_8h_1aa38c1bc6a810bf3e3197ca3050344fe3:
.. _cid-s2_stackstr_in:
.. ref-code-block:: cpp
	:class: title-code-block

	bool s2_stackstr_in (
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack,
	    const char* value
	    )

test to see if a value is in a stack



.. rubric:: Returns:

bool true if value is in stack, false otherwise

.. _doxid-stack_8h_1a339a8dfb0a945016899f53b9855daec7:
.. _cid-s2_stackstr_isequal:
.. ref-code-block:: cpp
	:class: title-code-block

	bool s2_stackstr_isEqual (
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack1,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack2
	    )

test to see if two stackStr data structures are equal

.. _doxid-stack_8h_1af37577a33750ddde27c14ae42743e9b1:
.. _cid-s2_stackstr_copy:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* s2_stackstr_copy (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack)

used to copy a :ref:`s2_stackstr_t <doxid-structs2__stackstr__t>` to a new :ref:`s2_stackstr_t <doxid-structs2__stackstr__t>`

.. _doxid-stack_8h_1a98afc36883f20654f83c501a1283a88b:
.. _cid-s2_stackstr_getroot:
.. ref-code-block:: cpp
	:class: title-code-block

	char* s2_stackstr_getRoot (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack)

get :ref:`s2_stackstr_t <doxid-structs2__stackstr__t>` root/base name

.. _doxid-stack_8h_1accf9bdf935047c8f248720a28a9ac348:
.. _cid-s2_stackstr_tostr:
.. ref-code-block:: cpp
	:class: title-code-block

	char* s2_stackstr_toStr (:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* stack)

allocates memory for a new char array, prints formatted :ref:`s2_stackstr_t <doxid-structs2__stackstr__t>`

.. _doxid-stack_8h_1a5446695107700cb8b8d003095d059411:
.. _cid-s2_stackstr_fromstring:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* s2_stackstr_fromString (const char*)

initialize a new stackstr and push first value onto list

.. _doxid-substructure_8h_1a2237fe00c9594061a85f1fb7329c8382:
.. _cid-s2_substructure:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_substructure_t<doxid-structs2__substructure__t>`* s2_substructure ()

Initialize a Substructure datastructure.



.. rubric:: Returns:

a Substructure datastructure

.. _doxid-terrain_8h_1a9b9c1297c5055f636e96fc6fb38ed05c:
.. _cid-s2_terrain_get:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_terrain_get (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    int i,
	    int j
	    )

get value at specified row and column index.

.. _doxid-terrain_8h_1a0d32a7c840f225e7f6ee3bde9b5c7493:
.. _cid-s2_terrain_getz_bilinear_sae:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_terrain_getZ_bilinear_sae (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

get z using bilinear interpolation at specified x and y position following SAE convention with positive z "down". The normal is returned in the "up" direction.

normal vector not computed yet

.. _doxid-terrain_8h_1aa357f8946d82e483a238d5031fc139c1:
.. _cid-s2_terrain_getz_bilinear_iso:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_terrain_getZ_bilinear_iso (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

get z using bilinear interpolation at specified x and y position following ISO convention with positive z "up". The normal is returned in the "up" direction.

normal vector not computed yet

.. _doxid-terrain_8h_1a8274320819cdd8a446b8fff086624351:
.. _cid-s2_terrain_getz_bicubic_sae:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_terrain_getZ_bicubic_sae (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

bicubic interpolation following sae convention with positive z down.

Surface normal is returned in the -z (up) direction.

.. _doxid-terrain_8h_1ac66cd961cd6936c5211afdf87d859816:
.. _cid-s2_terrain_getz_bicubic_iso:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_terrain_getZ_bicubic_iso (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

bicubic interpolation following iso convention with positive z up.

Surface normal is returned in the +z (up) direction.

.. _doxid-terrain_8h_1aebc3b2daef02c395cf60b49966493d07:
.. _cid-s2_terrain_getz_catmulrom_sae:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_terrain_getZ_catmulrom_sae (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

not implented yet

.. _doxid-terrain_8h_1a248c11bb6684c9f8c970ca44334681a3:
.. _cid-s2_terrain_getz_catmulrom_iso:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_terrain_getZ_catmulrom_iso (
	    :ref:`s2_terrain_t<doxid-structs2__terrain__t>`* terrain,
	    double x,
	    double y,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

not implented yet

.. _doxid-terrain_8h_1a4090acc0bd8c2dbf3ebe77d634376b75:
.. _cid-s2_terrain:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_terrain_t<doxid-structs2__terrain__t>`* s2_terrain (
	    const char* terrainFileName,
	    const char* textureFileName,
	    const char* propertyMap,
	    s2_interpolation_enum_t interpolationType,
	    s2_coordinate_convention_enum_t coordinateConvention,
	    double xOffset,
	    double yOffset,
	    double xScale,
	    double yScale
	    )

constructor for :ref:`s2_terrain_t <doxid-structs2__terrain__t>`

.. _doxid-terrain_8h_1a67f681544bd6e0983cc63fd610b3f916:
.. _cid-s2_terrain_getheightandsaenormal:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_terrain_getHeightAndSAENormal (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double x,
	    double y,
	    double* height,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

gets height and surface normal at specified x,y coordinates



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - [in] star2 context data structure containing model data

    *
        - x

        - [in] x coordinate

    *
        - y

        - [in] y coordinate

    *
        - height

        - [out] height at x, y coordinate

    *
        - normal

        - [out] surface normal at x, y coordinate (in global coordinates) This is a place holder function for now. In the future need to add functions to load terrain profiles and report values based on terrain. The surface normal follows SAE / Aerospace conventions with normal in the "down" direction, i.e. positive z

.. _doxid-terrain_8h_1a9930e944c824ed54c327c78862d5c935:
.. _cid-s2_terrain_getheightandisonormal:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_terrain_getHeightAndISONormal (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double x,
	    double y,
	    double* height,
	    :ref:`s2_vector_t<doxid-structs2__vector__t>`* normal
	    )

gets height and surface normal at specified x,y coordinates



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - context

        - [in] star2 context data structure containing model data

    *
        - x

        - [in] x coordinate

    *
        - y

        - [in] y coordinate

    *
        - height

        - [out] height at x, y coordinate

    *
        - normal

        - [out] surface normal at x, y coordinate (in global coordinates) This is a place holder function for now. In the future need to add functions to load terrain profiles and report values based on terrain. The surface normal follows ISO convention with normal in the "up" direction, i.e. {0, 0, -1}

.. _doxid-terrain_8h_1ad40245713ff01092a78ef326b10cc77a:
.. _cid-s2_terrain_load:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_terrain_load (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    json_t* assemblyObject,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix
	    )

loads terrain datastructure from json file

.. _doxid-topology_8h_1a804e0ff9b5a92350672f48b061d67358:
.. _cid-s2_context_settopology:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_context_setTopology (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

reset or initialize runtime context (optimized directed graph) from user's directed acyclic (linked list) representation

.. _doxid-topology_8h_1a1303aec0a235bc6e16b05d77bdca0e75:
.. _cid-s2_updatedh:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_updateDH (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

updates only displacements and influence coefficient matrices

.. _doxid-topology_8h_1a16e28aa318129a9c08698a1960987232:
.. _cid-s2_updatedhva:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_updateDHVA (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

optimized function to update displacements, coefficient matrices, velocities and accelerations

.. _doxid-topology_8h_1aafb159f6b3cb5306a80a821727143cfe:
.. _cid-s2_updatedhvadebug:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_updateDHVADebug (
	    FILE* f,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double t
	    )

update dhva and print values at all dhva pointer locations

.. _doxid-topology_8h_1a67fe61b3b669f5191fdf381ffea93899:
.. _cid-s2_updateglreferenceframes:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_updateGLReferenceFrames (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

update open gl frames (i.e. the origin of each body)

.. _doxid-utility_8h_1aee0340f870a70e7d632045f6d4b8a13c:
.. _cid-s2_arrayi_malloc2d:
.. ref-code-block:: cpp
	:class: title-code-block

	int** s2_arrayi_malloc2D (
	    int rows,
	    int columns
	    )

Allocates memory for a two dimensional integer array.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - int

        - rows in two dimensional array to allocate

    *
        - int

        - columns in two dimensional array to allocate



.. rubric:: Returns:

two dimensional integer array

.. _doxid-utility_8h_1a4b448a612428678975832349870e117e:
.. _cid-s2_arrayi_free2d:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_arrayi_free2D (int** x)

Free memory for a two dimensional integer array.

.. _doxid-utility_8h_1a8683a7de3c5b6a462044d998c35072f2:
.. _cid-s2_arrayi_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_arrayi_free (int* x)

Free memory for a one dimensional integer array.

.. _doxid-utility_8h_1ac288fee0fe1a1e7b6489bb44595d0166:
.. _cid-s2_array_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_array_free (double* x)

Free memory for a one dimensional double array.

.. _doxid-utility_8h_1a58bef2214578639ee0bb8800a558653b:
.. _cid-s2_arrayf_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_arrayf_free (float* x)

Free memory for a one dimensional float array.

.. _doxid-utility_8h_1a4740fdd9a78a68d4cec6bb7ca3b5ac84:
.. _cid-s2_arrayb_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_arrayb_free (bool* x)

Free memory for a one dimensional boolean array.

.. _doxid-utility_8h_1aff05ccdb6fa0e375c7363148eafa290a:
.. _cid-s2_arrays_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_arrays_free (:ref:`s2_screw_t<doxid-structs2__screw__t>`* x)

Free memory for a one dimensional array of screws.

.. _doxid-utility_8h_1a2f2f643fe0c57d3d4784794d10181f1a:
.. _cid-s2_arrayd_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_arrayd_free (:ref:`s2_displacement_t<doxid-structs2__displacement__t>`* x)

Free memory for a one dimensional array of displacement data structures.

.. _doxid-utility_8h_1a641113e0f7789a14d9bc0106146c8f48:
.. _cid-s2_array_malloc2d:
.. ref-code-block:: cpp
	:class: title-code-block

	double** s2_array_malloc2D (
	    int rows,
	    int columns
	    )

Allocates memory for a two dimensional double array.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - int

        - rows in two dimensional array to allocate

    *
        - int

        - columns in two dimensional array to allocate



.. rubric:: Returns:

two dimensional double array

.. _doxid-utility_8h_1a28b92c1222ab1725d85ac2edec08671c:
.. _cid-s2_arrayf_malloc2d:
.. ref-code-block:: cpp
	:class: title-code-block

	float** s2_arrayf_malloc2D (
	    int rows,
	    int columns
	    )

Allocates memory for a two dimensional single precision array.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - int

        - rows in two dimensional array to allocate

    *
        - int

        - columns in two dimensional array to allocate



.. rubric:: Returns:

two dimensional double array

.. _doxid-utility_8h_1a0b61c9879dcb264ab4af38a98db625d3:
.. _cid-s2_array_free2d:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_array_free2D (double** x)

Frees memory for a two dimensional double array.

.. _doxid-utility_8h_1a72956a50f32bc1c71a4281144b1aa59d:
.. _cid-s2_arrayf_free2d:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_arrayf_free2D (float** x)

Frees memory for a two dimensional float array.

.. _doxid-utility_8h_1acb084932ad0d60106ccfe60dfd9aa5c5:
.. _cid-s2_arrayc_malloc2d:
.. ref-code-block:: cpp
	:class: title-code-block

	char** s2_arrayc_malloc2D (
	    int rows,
	    int columns
	    )

Allocates memory for a two dimensional char array.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - int

        - rows in two dimensional array to allocate

    *
        - int

        - columns in two dimensional array to allocate



.. rubric:: Returns:

two dimensional char array

.. _doxid-utility_8h_1ae441785d07d353b611b750f87ba13f84:
.. _cid-s2_arrayc_free2d:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_arrayc_free2D (char** x)

Frees memory for a two dimensional double array.

.. _doxid-utility_8h_1aa393eef291884b3ca760ffc8bb07f68a:
.. _cid-s2_array_rowtocolumnmajor:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_array_rowToColumnMajor (
	    int rowDim,
	    int colDim,
	    double* ColumnMajorMatrix,
	    double* RowMajorMatrix
	    )

Converts a row major double precision matrix to column major.

.. _doxid-utility_8h_1ab546c4df37377474fd48b1662a05cb3e:
.. _cid-s2_displacement_copyupdate:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_displacement_copyUpdate (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`** Di,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Do
	    )

Copies data by value from a rotation matrix and translation vector to a 12*1 array.

Frees memory at old location and updates pointer address to point to new location.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a 12 x 1 displacement

    *
        - Do

        - Di is copied by value to Do, Di is updated to point to the address of Do

    *
        - d

        - a 12x1 vector

.. _doxid-utility_8h_1ab21c42a07e3762e5a50ba6d393fddfaa:
.. _cid-s2_displacementf_copyupdate:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_displacementf_copyUpdate (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`** Di,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Do
	    )

Copies data by value from a rotation matrix and translation vector to a 12*1 array.

Frees memory at old location and updates pointer address to point to new location.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a 12 x 1 displacement

    *
        - Do

        - Di is copied by value to Do, Di is updated to point to the address of Do

    *
        - d

        - a 12x1 vector

.. _doxid-utility_8h_1a41b8470b02489f772ebcaba953c27770:
.. _cid-s2_displacement_copyback:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_displacement_copyBack (
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`* Di,
	    :ref:`s2_displacement_t<doxid-structs2__displacement__t>`** Do
	    )

Copies data by value from a rotation matrix and translation vector to a 12*1 array.

Frees memory at old location and updates pointer address to point to new location.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a 12x1 vector, memory for Di is not freed after function execution.

    *
        - Do

        - Do is malloced and Di is copied by value to Do, after function.

.. _doxid-utility_8h_1aca1113329136eeed8c2bb42201216418:
.. _cid-s2_displacementf_copyback:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_displacementf_copyBack (
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`* Di,
	    :ref:`s2_displacementf_t<doxid-structs2__displacementf__t>`** Do
	    )

Copies data by value from a rotation matrix and translation vector to a 12*1 array.

Frees memory at old location and updates pointer address to point to new location.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a 12x1 vector, memory for Di is not freed after function execution.

    *
        - Do

        - Do is malloced and Di is copied by value to Do, after function.

.. _doxid-utility_8h_1a189883fe3a10eaa7fe280f5059898e22:
.. _cid-s2_screw_copyupdate:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_screw_copyUpdate (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`** Si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* So
	    )

Copies data by value from a rotation matrix and translation vector to a 12*1 array.

Frees memory at old location and updates pointer address to point to new location.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a 12 x 1 displacement

    *
        - Do

        - Di is copied by value to Do, Di is updated to point to the address of Do

    *
        - d

        - a 12x1 vector

.. _doxid-utility_8h_1a43eed17b23d6f8ab3da42313ba63e436:
.. _cid-s2_screwf_copyupdate:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_screwf_copyUpdate (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`** Si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* So
	    )

Copies data by value from a rotation matrix and translation vector to a 12*1 array.

Frees memory at old location and updates pointer address to point to new location.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a 12 x 1 displacement

    *
        - Do

        - Di is copied by value to Do, Di is updated to point to the address of Do

    *
        - d

        - a 12x1 vector

.. _doxid-utility_8h_1a7a4a3483be25b87171483be0811e75ab:
.. _cid-s2_screw_copyback:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_screw_copyBack (
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`* Si,
	    :ref:`s2_screw_t<doxid-structs2__screw__t>`** So
	    )

Copies data by value from a rotation matrix and translation vector to a 12*1 array.

Frees memory at old location and updates pointer address to point to new location.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a 12x1 vector, memory for Di is not freed after function execution.

    *
        - Do

        - Do is malloced and Di is copied by value to Do, after function.

.. _doxid-utility_8h_1a7bf324b3d94c50cccefaa3d9c55ae3b5:
.. _cid-s2_screwf_copyback:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_screwf_copyBack (
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`* Si,
	    :ref:`s2_screwf_t<doxid-structs2__screwf__t>`** So
	    )

Copies data by value from a rotation matrix and translation vector to a 12*1 array.

Frees memory at old location and updates pointer address to point to new location.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - Di

        - a 12x1 vector, memory for Di is not freed after function execution.

    *
        - Do

        - Do is malloced and Di is copied by value to Do, after function.

.. _doxid-utility_8h_1a1213ee993cf54a0e118ac033a23e7ed9:
.. _cid-s2_array_copyupdate:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_array_copyUpdate (
	    int count,
	    double** oldArray,
	    double* newArray
	    )

Copies data by value from elements of a linked list into a contiguous array.

Frees memory at old location and updates pointer address to point to new location.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - count

        - number of entries to copy

    *
        - old

        - pointer to old location

    *
        - new

        - pointer to new location



.. rubric:: Returns:

success=0, failure=1

.. _doxid-utility_8h_1a5a751490c3c7990411c2f9e09e67ccd5:
.. _cid-s2_arrayf_copyupdate:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_arrayf_copyUpdate (
	    int count,
	    float** oldArray,
	    float* newArray
	    )

Copies data by value from elements of a linked list into a contiguous array.

Frees memory at old location and updates pointer address to point to new location.



.. rubric:: Parameters:

.. list-table::
    :widths: 20 80

    *
        - count

        - number of entries to copy

    *
        - old

        - pointer to old location

    *
        - new

        - pointer to new location



.. rubric:: Returns:

success=0, failure=1

.. _doxid-utility_8h_1a020f9bc6d057069d5ae71090ad988f2a:
.. _cid-s2_array_copyback:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_array_copyBack (
	    int count,
	    double* oldArray,
	    double** newArray
	    )

copies a double precision array by value back to original location

Note this function does not free memory at old location, the memory must be freed in another function

.. _doxid-utility_8h_1aa53794d55f92436ed056c152f4b8e293:
.. _cid-s2_arrayf_copyback:
.. ref-code-block:: cpp
	:class: title-code-block

	int s2_arrayf_copyBack (
	    int count,
	    float* oldArray,
	    float** newArray
	    )

copies a single precision array by value back to original location

Note this function does not free memory at old location, the memory must be freed in another function

.. _doxid-utility_8h_1a8acece5fdde1300ce4de9cc9a99d29d8:
.. _cid-s2_getfilepath:
.. ref-code-block:: cpp
	:class: title-code-block

	char* s2_getFilePath (const char* fullName)

get file path of assembly

.. _doxid-utility_8h_1a334f334da1518648536582cae344d5ce:
.. _cid-s2_getfileextension:
.. ref-code-block:: cpp
	:class: title-code-block

	const char* s2_getFileExtension (const char*)

get file extension

.. _doxid-forcemodule_8h_1a10ba39388d465932d035489090731365:
.. _cid-s2_forcemodule:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* s2_forcemodule (:ref:`s2_forcemodule_enum_t<doxid-forcemodule_8h_1aef216ac0fc3d71c6419b69df97ab6091>` forceModuleType)

initialize a forcemodule data structure

.. _doxid-forcemodule_8h_1add15a798ad8c32c4be95b4f9e638adb3:
.. _cid-s2_forcemodule_flatten:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_forcemodule_flatten (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

flatten force modules into array of array of pointers to force modules with static and transient properties

.. _doxid-forcemodule_8h_1a65c554aaea385eb8ec611f9609469d7e:
.. _cid-s2_forcemodule_connect:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_forcemodule_connect (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

connect force module dy array pointer and set yOffset, called after numerical integrator set up

.. _doxid-forcemodule_8h_1afd5e4f09a802a2a80d1fb7c06f71b5f5:
.. _cid-s2_forcemodule_savehistory:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_forcemodule_saveHistory (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

save time histories of force modules

ode113 does not integrate equations in monotonically time increasing order, any values that need to be saved are set in this function and called in a different location when integrating the equations only when convergence is true

.. _doxid-forcemodule_8h_1a472eef329ecb6c3d3e934ede27279e4c:
.. _cid-s2_forcemodule_append:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_forcemodule_append (
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
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

.. _doxid-forcemodule_8h_1abd2b86528c4384d622201790c1c6dcd1:
.. _cid-s2_forcemodule_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_forcemodule_free (:ref:`s2_forcemodule_t<doxid-structs2__forcemodule__t>`* forceModule)

initialize a forcemodule data structure

.. _doxid-forcemodule_8h_1ac28262f7c2ba16e662ff311b36ba2c09:
.. _cid-s2_forcemodule_load:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_forcemodule_load (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_assembly_t<doxid-structs2__assembly__t>`* assembly,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* prefix,
	    json_t* forceModuleObject
	    )

load forcemodule properties from json file

.. _doxid-pid_8h_1adfefdc1c61c0e5157a09686e27e4d7ab:
.. _cid-s2_pid_updatefromjointtojoint:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_pid_updateFromJointToJoint (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    void** parameters,
	    double* y
	    )

update a force in a joint as a function of velocity in a given frame

.. _doxid-pid_8h_1ac8c49e0c6bef94a9f38dde8182c6e8a0:
.. _cid-s2_pid_updatefromframetojoint:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_pid_updateFromFrameToJoint (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    void** parameters,
	    double* y
	    )

update a force in a joint as a function of velocity in a given frame

.. _doxid-service_8h_1a848e4950f54f139287ae4a6765fad69f:
.. _cid-s2_service_setforces:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_service_setForces (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    double* y
	    )

clear and set externally applied forces to model

.. _doxid-stiffness_8h_1ad6dc04dd6bda9c49495fa1ef5f47a210:
.. _cid-s2_stiffness_joint:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_stiffness_joint (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_joint_t<doxid-structs2__joint__t>`* joint
	    )

sets stiffness within a joint

.. _doxid-fiala_8h_1a8658302eeb1bec4475877e8367172394:
.. _cid-s2_tire_fiala:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* s2_tire_fiala (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* filePath,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* wheelBody
	    )

allocate memory for tire with default parameters

.. _doxid-fiala_8h_1a3e1a2e637efc322442633553baba498c:
.. _cid-s2_tire_fiala_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_fiala_free (:ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire)

free memory for :ref:`s2_tire_fiala_t <doxid-structs2__tire__fiala__t>`

.. _doxid-fiala_8h_1a322fce69ed2f93ae89452a861d8303ca:
.. _cid-s2_tire_fiala_update:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_fiala_update (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    void** parameters,
	    double* y
	    )

placeholder function to update transient properties associated with fiala tire

.. _doxid-fiala_8h_1aef0605aba2e6664ad511704f3adc29fc:
.. _cid-s2_tire_fiala_loadtiredatafile:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_fiala_loadTireDataFile (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_tire_fiala_t<doxid-structs2__tire__fiala__t>`* tire,
	    const char* filePath
	    )

Load a .tir tire specification file.

.. _doxid-pac2002_8h_1a52c64dbbbd2c064b8032d7dcce7a10d2:
.. _cid-s2_tire_pac2002:
.. ref-code-block:: cpp
	:class: title-code-block

	:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* s2_tire_pac2002 (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    const char* filePath,
	    :ref:`s2_stackstr_t<doxid-structs2__stackstr__t>`* wheelBody,
	    bool useTransientSlip
	    )

allocate memory for tire with default parameters

.. _doxid-pac2002_8h_1a75e3fa6c00463a48be7638e921500143:
.. _cid-s2_tire_pac2002_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_free (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

free memory for :ref:`s2_tire_pac2002_t <doxid-structs2__tire__pac2002__t>`

.. _doxid-pac2002_8h_1a73243b38626d6cb76ca38ee1d00b83f6:
.. _cid-s2_tire_pac2002_update:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_update (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    void** parameters,
	    double* y
	    )

update slip angle, longtitudinal slip and camber angle, update forces to model

.. _doxid-pac2002_8h_1a3392f865e9daaa82bbf47793f3a5b8ab:
.. _cid-s2_tire_pac2002_loadtiredatafile:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_loadTireDataFile (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    const char* filePath
	    )

Load a .tir tire specification file.

.. _doxid-pac2002_8h_1a5075ecf586909626c01f2df725bc3af4:
.. _cid-s2_tire_pac2002_updatewframe:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_updateWFrame (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

updates wFrame, wheel normal (in iso convention), terrain normal,

.. _doxid-pac2002_8h_1aeaaea7981cb1d6b432d05f587fd66ec9:
.. _cid-s2_tire_pac2002_calc_fz:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_tire_pac2002_calc_Fz (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

calculate Fz using spring - damper model

.. _doxid-pac2002_8h_1a215220f8e29bf377a51244a6ca6cced0:
.. _cid-s2_tire_pac2002_updaterelaxationlengths:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_updateRelaxationLengths (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

compute relaxation lengths

.. _doxid-pac2002_8h_1a80e0c5ab777c6793a5e1a74a83dfa5c7:
.. _cid-s2_tire_pac2002_updateslipkinematic:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_updateSlipKinematic (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

Calculate kinematic slip quantities from the current wheel state.

Note: when not in contact, all slips are assumed zero, but velocities still set

.. _doxid-pac2002_8h_1a3bd2800b93b071b82df4ef3986561110:
.. _cid-s2_tire_pac2002_zeroslips:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_zeroSlips (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

set the tire m_slip vector to all zeros

.. _doxid-pac2002_8h_1ac20ea3476c1d1cbf4dc6f6cfcf52e168:
.. _cid-s2_tire_pac2002_evaluatereactions:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_evaluateReactions (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    bool write_violations,
	    bool enforce_threshold
	    )

after calculating reactions, enforce limits when forces/moments exceed specified thresholds (except on Fz)

.. _doxid-pac2002_8h_1ae0239415e4200b821dec893a8bf183b8:
.. _cid-s2_tire_pac2002_slip_from_uv:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_slip_from_uv (
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

	void s2_tire_pac2002_pureSlipReactions (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

Calculate tire reactions.

pure slip reactions, if the tire is in contact with the ground, in the TYDEX W-Axis system. calcFx alphaP ~= 0 calcFy and calcMz kappaP ~= 0 NOTE: alphaP, gammaP and kappaP defined with respect to tire side specified from *.tir input file. e.g., positive alpha = turn wheel toward vehicle centerline e.g., positive gamma = wheel vertical axis top pointing toward vehicle center

.. _doxid-pac2002_8h_1a2e900f40155187ffbb850e80964d17bd:
.. _cid-s2_tire_pac2002_combinedslipreactions:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_combinedSlipReactions (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

calculate combined slip reactions combined slip reactions, if the tire is in contact with the ground MUST call pureSlipReactions() first, as these functions rely on intermediate values

NOTE: alphaP and gammaP have already been modified for being on the L/R side e.g., positive alpha = turn wheel toward vehicle centerline e.g., positive gamma = wheel vertical axis top pointing toward vehicle center

.. _doxid-pac2002_8h_1a0c87e22d7746b5f7630c920217b6a3aa:
.. _cid-s2_tire_pac2002_fx_purelong:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_tire_pac2002_Fx_pureLong (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double kappa
	    )

longitudinal force, alpha ~= 0 assign to m_FM.force.x assign m_pureLong, trionometric function calculated constants

.. _doxid-pac2002_8h_1aab053dca94043c0115b8c86b755608f3:
.. _cid-s2_tire_pac2002_fy_purelat:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_tire_pac2002_Fy_pureLat (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double alpha,
	    double gamma
	    )

lateral force, kappa ~= 0 assign to m_FM.force.y assign m_pureLong, trionometric function calculated constants

.. _doxid-pac2002_8h_1aa1f7798f862f1b1af24b410938970577:
.. _cid-s2_tire_pac2002_mz_purelat:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_tire_pac2002_Mz_pureLat (
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

	double s2_tire_pac2002_Fx_combined (
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

	double s2_tire_pac2002_Fy_combined (
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

	double s2_tire_pac2002_Mz_combined (
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

	double s2_tire_pac2002_calc_Mx (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double gamma,
	    double Fy_combined
	    )

calculate the overturning couple moment assign m_FM.moment.x and m_FM_combined.moment.x

.. _doxid-pac2002_8h_1ae8f86738eda52bbc1d5ea709659c07d7:
.. _cid-s2_tire_pac2002_calc_my:
.. ref-code-block:: cpp
	:class: title-code-block

	double s2_tire_pac2002_calc_My (
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire,
	    double Fx_combined
	    )

calculate the rolling resistance moment, assign m_FM.moment.y and m_FM_combined.moment.y

.. _doxid-pac2002_8h_1ab7a1630c7e0a24a451236c2cb143dc15:
.. _cid-s2_tire_pac2002_discterraincontact:
.. ref-code-block:: cpp
	:class: title-code-block

	bool s2_tire_pac2002_discTerrainContact (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

check for contact, set tire->inContact flag and set penetration depth

.. _doxid-pac2002_8h_1aad631fb210d174a3be0f80a058b42197:
.. _cid-s2_tire_pac2002_updateverticalload:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_updateVerticalLoad (:ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire)

Calculate the tire vertical load at the current configuration and update the tire deflection and tire rolling radius.

.. _doxid-pac2002_8h_1ac1c6d29395ad7cec3dd2d9127b6fdbd9:
.. _cid-s2_tire_pac2002_transferforces:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_tire_pac2002_transferForces (
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context,
	    :ref:`s2_tire_pac2002_t<doxid-structs2__tire__pac2002__t>`* tire
	    )

wrapper function to update for use with force module data structure

.. _doxid-numericalintegrator_8h_1add0ac8b46877a1d5e6189e5c71c4a15f:
.. _cid-s2_numericalintegrator_allocate:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_numericalintegrator_allocate (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

Initialize and allocate memory for numericalIntegrator datastructure.



.. rubric:: Returns:

a numericalIntegrator datastructure

.. _doxid-numericalintegrator_8h_1a4c29dcac0bbfd00ed84dc52678b62c7e:
.. _cid-s2_numericalintegrator_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_numericalintegrator_free (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

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

	int i4_sign (int i)

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

	void s2_ode113_intrp (
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

Input, double PHI[NEQN*16], contains information about the interpolating polynomial.

Input, double PSI[12], contains information about the interpolating polynomial.

.. _doxid-ode113_8h_1ab21d05128ddb24988dfbca84c955c6f8:
.. _cid-r8_epsilon:
.. ref-code-block:: cpp
	:class: title-code-block

	double r8_epsilon (void)

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

.. _doxid-ode113_8h_1ae5e6e61d2397f04273a1d4a1623bb244:
.. _cid-s2_ode113_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_ode113_free (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

free a numerical integrator datastructure for ode113 integration type

.. _doxid-ode113_8h_1aeb5c98af7f1932d7afa5a7ed255c81a9:
.. _cid-s2_ode113_integrateequations:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_ode113_integrateEquations (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

integrate equations using ode113 integration method

.. _doxid-ode113b_8h_1ab276701dcf2866123f1b99ccdc0e9394:
.. _cid-s2_ode113b_allocate:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_ode113b_allocate (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

allocate memory for ode113b numerical integrator

.. _doxid-ode113b_8h_1ae22ccb46e43e1e319a2584304c944527:
.. _cid-s2_ode113b_free:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_ode113b_free (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

free memory for ode113b integrator

.. _doxid-ode113b_8h_1adfa68c62baf3abe532197f5ef25eed8b:
.. _cid-s2_ode113b_fcn:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_ode113b_fcn (
	    double y [],
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

solve for current time step

.. _doxid-ode113b_8h_1a8cc5cb1907a1b5de62e501a7a9ac0096:
.. _cid-s2_ode113b_de:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_ode113b_de (
	    void  f double y[], s2_context_t *context,
	    :ref:`s2_context_t<doxid-structs2__context__t>`* context
	    )

subroutine de integrates a system of neqn first order ordinary differential equations of the form

dy(i)/dt = f(t,y(1),y(2),...,y(neqn)) y(i) given at t .

the subroutine integrates from t to tout . on return the parameters in the call list are initialized for continuing the integration. the user has only to define a new value tout and call de again.

de calls two codes, the integrator step and the interpolation routine intrp . step uses a modified divided difference form of the adams pece formulas and local extrapolation. it adjusts the order and step size to control the local error. normally each call to step advances the solution one step in the direction of tout . though never beyond t + 10*(tout-t), and calls intrp to interpolate the solution at tout . an option is provided to stop the integration at tout but it should be used only if it is impossible to continue the integration beyond tout .

this code is completely explained and documented in the text, computer solution of ordinary differential equations: the initial value problem by l. f. shampine and m. k. gordon.

the parameters for de are: f subroutine f(t,y,yp) to evaluate derivatives yp(i)=dy(i)/dt neqn number of equations to be integrated y(*) solution vector at t t independent variable tout point at which solution is desired relerr,abserr relative and absolute error tolerances for local error test. at each step the code requires abs(local error) .le. abs(y)*relerr + abserr for each component of the local error and solution vectors iflag indicates status of integration

first call to de

the user must provide storage in his calling program for the array in the call list, y(neqn), declare f in an external statement, supply the subroutine f(t,y,yp) to evaluate dy(i)/dt = yp(i) = f(t,y(1),y(2),...,y(neqn)) and initialize the parameters: neqn number of equations to be integrated y(*) vector of initial conditions t starting point of integration tout point at which solution is desired relerr,abserr relative and absolute local error tolerances iflag +1,-1. indicator to initialize the code. normal input is +1. the user should set iflag=-1 only if it is impossible to continue the integration beyond tout . all parameters except f , neqn and tout may be altered by the code on output so must be variables in the calling program.

output from de

neqn unchanged y(*) solution at t t last point reached in integration. normal return has t = tout . tout unchanged relerr,abserr normal return has tolerances unchanged. iflag=3 signals tolerances increased iflag = 2 normal return. integration reached tout = 3 integration did not reach tout because error tolerances too small. relerr , abserr increased appropriately for continuing = 5 integration did not reach tout because equations appear to be stiff = 6 invalid input parameters (fatal error) the value of iflag is returned negative when the input value is negative and the integration does not reach tout , i.e., -3, -4, -5.

subsequent calls to de

subroutine de returns with all information needed to continue the integration. if the integration reached tout , the user need only define a new tout and call again. if the integration did not reach tout and the user wants to continue, he just calls again. the output value of iflag is the appropriate input value for subsequent calls. the only situation in which it should be altered is to stop the integration internally at the new tout , i.e., change output iflag=2 to input iflag=-2 . error tolerances may be changed by the user before continuing. all other parameters must remain unchanged.

.. _doxid-ode113b_8h_1abff36438c30c864f0e9cb3407d5ac91c:
.. _cid-s2_ode113b_integrateequations:
.. ref-code-block:: cpp
	:class: title-code-block

	void s2_ode113b_integrateEquations (:ref:`s2_context_t<doxid-structs2__context__t>`* context)

placeholder for integrating using ode113b method

Macros
------

.. _doxid-star2_8h_1a5d90259d9c8d01a76dda46dcc46c74cd:
.. _cid-_star2_h_:
.. ref-code-block:: cpp
	:class: title-code-block

	#define _STAR2_H_

star2: A Simulation Tool for Automation and Robotics Kristopher Wehage, 2016

Kristopher Wehage

