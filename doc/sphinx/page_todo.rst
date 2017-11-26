.. index:: pair: page; Todo List
.. _doxid-todo:

Todo List
=========

.. list-table::
    :widths: 20 80

    *
        - Global :ref:`s2_body_loadGeometry <doxid-body_8h_1a9798b3af24177e61f86edbcebcf45b1c>` ( :ref:`s2_context_t <doxid-structs2__context__t>` \*context, :ref:`s2_body_t <doxid-structs2__body__t>` \*body)

        - figure out where specular highlight term is stored in assimp

    *
        - Global :ref:`s2_joint_spherical_dhva <doxid-joint_8h_1af81dd6149e55911590071c65e1311faf>` (void \*\*dhva)

        - add spherical joint

    *
        - Global :ref:`s2_joint_sphericalinverse <doxid-joint_8h_1a3fd780c6194d664a93671c0438ce1681>` (void \*\*dhva)

        - add sphericalinverse

    *
        - Global :ref:`s2_mat3x3_toQuaternion <doxid-kinematics_8h_1a86d102e7d45b0a9c27368b46b11a0462>` ( :ref:`s2_mat3x3_t <doxid-structs2__mat3x3__t>` \*ri, :ref:`s2_quaternion_t <doxid-structs2__quaternion__t>` \*qo)

        - Supply *ER* , a reference Euler Parameter. If necessary, flip the sign of \*(E) so that each \*(E+i) is the closest match to ER[i]. For this algorithm to work, \*(E) must be a close follow-on to ER. This function would then prove more useful when continuous Euler Parameters are desired for interpolation purposes. Matrix R is quadratic in \*(E) so \*(E+i) and -\*(E+i) give the same R.

    *
        - Global :ref:`s2_mat3x3f_toQuaternion <doxid-kinematics_8h_1a0da18a57ba2a87e8a2c17f1f73c9d2ad>` ( :ref:`s2_mat3x3f_t <doxid-structs2__mat3x3f__t>` \*ri, :ref:`s2_quaternionf_t <doxid-structs2__quaternionf__t>` \*qo)

        - Supply *ER* , a reference Euler Parameter. If necessary, flip the sign of \*(E) so that each \*(E+i) is the closest match to ER[i]. For this algorithm to work, \*(E) must be a close follow-on to ER. This function would then prove more useful when continuous Euler Parameters are desired for interpolation purposes. Matrix R is quadratic in \*(E) so \*(E+i) and -\*(E+i) give the same R.

    *
        - Global :ref:`topology_setActiveObjectPointers <doxid-topology_8c_1a62f84ab28d5fa679e80005b89cd1f320>` ( :ref:`s2_assembly_t <doxid-structs2__assembly__t>` \*assembly, :ref:`s2_context_t <doxid-structs2__context__t>` \*context)

        - add array of io pointers

    *
        - Global :ref:`topology_setFramesInTree <doxid-topology_8c_1a07be85f007a240eeef35c7ffda6e343c>` ( :ref:`s2_assembly_t <doxid-structs2__assembly__t>` \*assembly, :ref:`s2_context_t <doxid-structs2__context__t>` \*context, :ref:`s2_stack_t <doxid-structs2__stack__t>` \*frameStack)

        - add counting devices

    *
        - Global :ref:`topology_setIndicesInitial <doxid-topology_8c_1a62edb22af9be6d2aa3b64652c3c902c9>` ( :ref:`s2_assembly_t <doxid-structs2__assembly__t>` \*assembly, :ref:`s2_context_t <doxid-structs2__context__t>` \*context)

        - add logic to detect if either parent or child frame is on an inactive assembly and do not count joint if true
