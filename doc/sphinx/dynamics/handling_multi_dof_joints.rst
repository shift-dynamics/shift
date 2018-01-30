.. title:: Handling MultiDOF Joints

Handling MultiDOF Joints
========================

==================== ========================= ================== ===========
**variable name**    **description**           **size**           **type**
-------------------- ------------------------- ------------------ -----------
*context->mhProduct* product of *inertiaP*     dqCount            s2_screw_t
                     and *h*

*context->fl*        generalized forces        dqCount            double

*context->g*         externally applied        nonRigidJointCount s2_screw_t
                     forces including gravity

*context->p*         spatial momentum          nonRigidJointCount s2_screw_t

*context->gamma_d*   constraint error          dqCount            s2_screw_t

*context->inertiaP*  spatial inertia in fixed  nonRigidJointCount s2_inertia_t
                     inertial frame

*context->inertia*   spatial inertia in body   nonRigidJointCount s2_inertia_t
                     frame

*context->h*         influence coefficient     dqCount            s2_screw_t
                     matrices

*context->f*         working forces within a   dqCount            double
                     joint

*context->la*        lagrange multipliers      dqCount            double

*context->v*         spatial body velocities   nonRigidJointCount s2_screw_t

*context->aq*        quadratic accelerations   nonRigidJointCount s2_screw_t

==================== ========================= ================== ===========


Evaluating RHS
--------------

Evaluating the RHS involves evaluating the expression:

.. math::
  :label: spatial_1

  \underline{f}^{\ell} = \underline{f} + H^T * C^{-T} (\underline{g} + \tilde{v}^T(M\underline{v}) – M \underline{\gamma}


which can be broken down into the following steps:

1. Transform inertia to inertiaP
