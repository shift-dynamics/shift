.. title:: Generalized Coordinate Partitioning

.. _generalized_coordinate_partitioning:


Dynamics
========

Spatial Equations of Motion for a Rigid Body
---------------------------------------------

In the present notation, the *spatial* equations of motion for a rigid body :math:`b` expressed in the fixed inertial reference frame :math:`f` can be found from the time derivative of spatial momentum. Frame :math:`c` is fixed in body :math:`b`, with its origin at the center of mass and its axes along the principal axes of inertia.

.. math::
  :label: newton_1

  \underline{\mathbf{g}}_{b,f}^{*f} = \frac{d}{dt}\underline{\mathbf{p}}_{b,f}^{*f} = \frac{d}{dt}\left( M_{b,ff}^{ff}\underline{\mathbf{v}}_{fc,f}^{f}\right)


where :math:`\underline{\mathbf{g}}_{b,f}^{*f}` is the wrench acting on body :math:`b`, such as from gravity or externally applied forces and torques, taken at the origin of :math:`f` and in :math:`f` coordinates, :math:`\underline{\mathbf{p}}_{b,f}^{*f}` is the spatial momentum of body :math:`b`, and :math:`\underline{\mathbf{v}}_{fc,f}^{f}` is the velocity screw of frame :math:`c` relative to :math:`f`, taken at the origin of :math:`f` and in :math:`f` coordinates. Using the chain rule, the right hand side is expanded as:

.. math::
  :label: spatial_1

  \underline{\mathbf{g}}_{b,f}^{*f} = \dot{M}_{b,ff}^{ff}\underline{\mathbf{v}}_{fc,f}^{f} + M_{b,ff}^{ff}\dot{\underline{\mathbf{v}}}_{fc,f}^{f}


The spatial inertia at the origin of :math:`f` is found from the inertia :math:`M_{b,cc}^{cc}` at the center of mass:

.. math::
  :label: eq:M_ff

  M_{b,ff}^{ff} = [Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf}


Noting that :math:`M_{b,cc}^{cc}` in the centroidal reference frame is constant, :math:`\dot{M}_{b,ff}^{ff}` can be found using the chain rule as:

.. math::
  :label: eq:M_der

  \dot{M}_{b,ff}^{ff} = \dot{[Ad]}^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf} + [Ad]^{*fc}_{fc}M_{b,cc}^{cc}\dot{[Ad]}_{cf}^{cf}


where :math:`\dot{[Ad]}^{*fc}_{fc} = [ad]^{*f}_{fc,f}[Ad]^{*fc}_{fc}` and :math:`\dot{[Ad]}_{cf}^{cf} = -[Ad]_{cf}^{cf}[ad]_{fc,f}^{f}`, with :math:`[ad]_{fc,f}^{f}` and :math:`[ad]^{*f}_{fc,f}` formed from :math:`\underline{\mathbf{v}}_{fc,f}^{f}`:

.. math::
  :label: eq:M_der2

  \dot{M}_{b,ff}^{ff} = [ad]^{*f}_{fc,f}[Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf} -
  [Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf}[ad]_{fc,f}^{f}


yielding:

.. math::
  :label: eq:spatial2

  \underline{\mathbf{g}}_{b,f}^{*f} = \left([ad]^{*f}_{fc,f}[Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf} -
  [Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf}[ad]_{fc,f}^{f}\right)\underline{\mathbf{v}}_{fc,f}^{f} + [Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf}\dot{\underline{\mathbf{v}}}_{fc,f}^{f}


Which is further simplified as:

.. math::
  :label: eq:spatial3

  \require{cancel}
  \underline{\mathbf{g}}_{b,f}^{*f} = [ad]^{*f}_{fc,f}M_{b,ff}^{ff}\underline{\mathbf{v}}_{fc,f}^{f} - \cancel{M_{b,ff}^{ff}[ad]_{fc,f}^{f}\underline{\mathbf{v}}_{fc,f}^{f}} + M_{b,ff}^{ff}\dot{\underline{\mathbf{v}}}_{fc,f}^{f}


The term :math:`M_{b,ff}^{ff}[ad]_{fc,f}^{f}\underline{\mathbf{v}}_{fc,f}^{f}` cancels because the matrix product :math:`\begin{bmatrix}\tilde{\omega}_{fc}^{f} & \tilde{v}_{fc,f}^{f}\\ 0 & \tilde{\omega}_{fc}^{f}\end{bmatrix} \begin{Bmatrix}\underline{v}_{fc,f}^{f}\\ \underline{\omega}_{fc}^{f} \end{Bmatrix}` is zero.

The final form of the spatial equations of motion may be written as:

.. math::
  :label: eq:spatial_final

  M_{b,ff}^{ff}\dot{\underline{\mathbf{v}}}_{fc,f}^{f} = \underline{\mathbf{g}}_{b,f}^{*f} - [ad]^{*f}_{fc,f}M_{b,ff}^{ff}\underline{\mathbf{v}}_{fc,f}^{f}


Comparison to Newton-Euler Equations
************************************

The equations take the same form in the coordinates of frame :math:`c`, at its origin. With the velocity screw :math:`\underline{\mathbf{v}}_{fc,c}^{c}` of :math:`c` relative to :math:`f`, taken at the center of mass and in :math:`c` coordinates, and the inertia :math:`M_{b,cc}^{cc}`, they expand as:

.. math::
  :label: eq:spatial_expand

  \begin{Bmatrix}
    \underline{f}_{b}^{c} \\
    \underline{\tau}_{b,c}^{c}
  \end{Bmatrix}=
  \begin{bmatrix}
    mI & 0\\ 0 & \bar{I}_{b,cc}^{cc}
  \end{bmatrix}
  \begin{Bmatrix}
    \underline{\dot{v}}_{fc,c}^{c}\\
    \dot{\underline{\omega}}_{fc}^{c}
  \end{Bmatrix} +
  \begin{bmatrix}
    \tilde{\omega}_{fc}^{c} & 0 \\
    \tilde{v}_{fc,c}^{c} & \tilde{\omega}_{fc}^{c}
  \end{bmatrix}
  \begin{bmatrix}
    mI & 0\\
    0 & \bar{I}_{b,cc}^{cc}
  \end{bmatrix}
  \begin{Bmatrix}
    \underline{v}_{fc,c}^{c}\\
    \underline{\omega}_{fc}^{c}
  \end{Bmatrix}


where :math:`\underline{f}_{b}^{c}` is the force on body :math:`b` and :math:`\underline{\tau}_{b,c}^{c}` the torque on it about its center of mass. Since :math:`\tilde{v}_{fc,c}^{c}\,m\underline{v}_{fc,c}^{c} = 0`, this is:

.. math::
  :label: eq:spatial_expand2

  \begin{Bmatrix}
    \underline{f}_{b}^{c} \\
    \underline{\tau}_{b,c}^{c}
  \end{Bmatrix}=
  \begin{bmatrix}
    mI & 0\\ 0 & \bar{I}_{b,cc}^{cc}
  \end{bmatrix}
  \begin{Bmatrix}
    \underline{\dot{v}}_{fc,c}^{c} \\
    \dot{\underline{\omega}}_{fc}^{c}
  \end{Bmatrix} +
  \begin{Bmatrix}
    m\tilde{\omega}_{fc}^{c}\underline{v}_{fc,c}^{c}\\
    \tilde{\omega}_{fc}^{c}\bar{I}_{b,cc}^{cc}\underline{\omega}_{fc}^{c}
  \end{Bmatrix}


The Newton-Euler equations of motion are written with the classical acceleration of the center of mass, :math:`\underline{a}_{fc,c}^{c} = \underline{\dot{v}}_{fc,c}^{c} + \tilde{\omega}_{fc}^{c}\underline{v}_{fc,c}^{c}`:

.. math::
  :label: eq:newton_euler

  \begin{Bmatrix}
    \underline{f}_{b}^{c} \\
    \underline{\tau}_{b,c}^{c}
  \end{Bmatrix}=
  \begin{bmatrix}
  mI & 0\\
  0 & \bar{I}_{b,cc}^{cc}
  \end{bmatrix}
  \begin{Bmatrix}
  \underline{a}_{fc,c}^{c}\\
  \dot{\underline{\omega}}_{fc}^{c}
  \end{Bmatrix}+
  \begin{Bmatrix}
  0\\
  \tilde{\omega}_{fc}^{c}\bar{I}_{b,cc}^{cc}\underline{\omega}_{fc}^{c}
  \end{Bmatrix}


so the two agree. The translational component of the spatial acceleration, :math:`\underline{\dot{v}}_{fc,c}^{c}`, differs from the classical acceleration by :math:`\tilde{\omega}_{fc}^{c}\underline{v}_{fc,c}^{c}`. Care must be taken when interpreting spatial accelerations: they are equal only when :math:`\tilde{\omega}_{fc}^{c}\underline{v}_{fc,c}^{c}` is zero, for example when the body does not rotate.


Dynamic Equations for a Multi-rigid Body System
-----------------------------------------------

The constrained dynamical equations of motion, a form of Lagrange's equations of motion expressed in joint coordinate space, may be represented compactly as

.. math::
    :label: eq:lagrange_eqn

    \begin{bmatrix}
    M^{\ell} & J^T\\
    J& 0
    \end{bmatrix}
    \begin{Bmatrix}
    \ddot{\underline{q}}\\
    \underline{\lambda}
    \end{Bmatrix} =
    \begin{Bmatrix}
    \underline{f}^{\ell}\\
    \underline{\gamma}_c
    \end{Bmatrix}


where :math:`M^{\ell}` is a symmetric, positive semidefinite generalized mass matrix, :math:`J` is a rectangular constraint Jacobian matrix that may lack full row and column rank, :math:`\underline{f}^{\ell}` is a column matrix of externally applied and generalized dynamic wrenches, :math:`\underline{\ddot{q}}` is a column matrix of generalized coordinate second time derivatives, :math:`\underline{\lambda}` is a column matrix of Lagrange multipliers, and :math:`\underline{\gamma}_c=-\dot{J}\underline{\dot{q}}-\dot{\underline{e}}_t`, where :math:`\dot{\underline{e}}_t` represents the time derivative of constraint partial derivatives, :math:`\underline{e}_t`, with respect to time.

Equation :eq:`eq:lagrange_eqn` can be derived from Newton-Euler equations for a rigid body:

.. math::
    :label: eq:newton_euler

    \underline{\mathbf{g}}^* = \begin{bmatrix}mI & 0 \\ 0 & I_c\end{bmatrix}\dot{\underline{\mathbf{v}}} +
    \begin{bmatrix}0 & 0 \\ 0 & \tilde{\omega}\end{bmatrix}\begin{bmatrix}mI & 0 \\ 0 & I_c\end{bmatrix}\underline{\mathbf{v}}


where :math:`\underline{\ddot{q}}` and :math:`\underline{\dot{q}}` represent body acceleration and velocity, :math:`\underline{\mathbf{g}}^*` represent external wrenches acting on a body, including gravity, :math:`m` is the body mass, and :math:`I_c` is the moment of inertia tensor. If :math:`\underline{\mathbf{g}}^*`, :math:`I_c` and :math:`\underline{\ddot{q}}` are expressed in the inertial reference frame, coriolis forces can be neglected and Eq. :eq:`eq:newton_euler` simplifies as:

.. math::
    :label: eq:newton_euler2

    \underline{\mathbf{g}}^* = M\dot{\underline{\mathbf{v}}}


For an unconstrained multibody system, :math:`\underline{f}^*` and :math:`\underline{\ddot{q}}` are replaced by a column matrix of body wrenches and velocities, and :math:`M` by a block matrix of 6x6 body inertia matrices along the diagonal. An important tool in formulating the equations of motion is the incidence matrix, :math:`\Gamma`, a representation of an *oriented mechanical graph* or *oriented assembly* of a mechanism :cite:`uicker2013`, with columns corresponding joints and rows corresponding to bodies. An entry of row b and column h of :math:`\Gamma` is defined as:

.. math::
    :label: eq:incidence_matrix_def

    \Gamma (b,h) = \begin{cases}
    +1&\text{if joint h is positively incident with body b}\\
    -1& \text{if joint h is negatively incident with body b}\\
    0&\text{if joint h is not incident with body b}
    \end{cases}


For a detailed, worked example of an incidence matrix, refer to the next section. A corollary to the incidence matrix, the connectivity matrix, is formed by element-wise replacement of the +1 and -1s in :math:`\Gamma^T` with positive or negative 6x6 identity matrices. The partial velocity matrix, :math:`H`, is a block matrix containing 6x1 influence coefficient matrices along the diagonal and provides a mapping of joint velocities to relative velocities across joints.

.. math::
    :label: eq:mapping

    \underline{\mathbf{v}}_{rel} = C \underline{\mathbf{v}}_{abs} = H\dot{\underline{q}}


Taking the derivative of Eq. :eq:`eq:mapping` yields:

.. math::
    :label: eq:mapping_der

    C\dot{\underline{\mathbf{v}}}_{abs} = H\ddot{\underline{q}} + \dot{H}\dot{\underline{q}} = H\ddot{\underline{q}} + \underline{\gamma}


where :math:`\underline{\gamma}=\dot{H}\dot{\underline{q}}`. Eq. :eq:`eq:mapping_der` can be expressed in factored matrix form as:

.. math::
    :label: eq:mapping_der_matrix

    \begin{bmatrix}C & -H\end{bmatrix}\begin{Bmatrix} \dot{\underline{\mathbf{v}}}_{abs} \\ \underline{\ddot{q}}\end{Bmatrix} = \underline{\gamma}


Additionally the equation:

.. math::
    :label: eq:map_force_to_joint_space

    H^T\underline{\mathbf{k}}^*=\underline{f}


maps constraint reaction wrenches, :math:`\underline{\mathbf{k}}^*` onto joint space. The column matrix :math:`\underline{f}` represents working generalized forces (such as from springs, dampers, actuators, friction, etc.) applied within a joint. If no such effects exist, :math:`\underline{f}` will be zero.

Equation :eq:`eq:mapping_der_matrix` and :eq:`eq:map_force_to_joint_space` can be appended to Eq. :eq:`eq:newton_euler2` as:

.. math::
    :label: eq:augmented_newton_euler

    \begin{bmatrix}M & 0 & C^T\\ 0 & 0 & -H^T\\ C & -H & 0\end{bmatrix}
    \begin{Bmatrix}\dot{\underline{\mathbf{v}}}_{abs}\\ \underline{\ddot{q}}\\ \underline{\mathbf{k}}^* \end{Bmatrix} =
    \begin{Bmatrix}\underline{\mathbf{g}}^* \\ \underline{f} \\ \underline{\gamma}\end{Bmatrix}


If the connectivity matrix is structurally nonsingular and invertible, then the absolute spatial accelerations, :math:`\underline{\dot{v}}_{abs}` and reaction wrenches, :math:`\underline{\mathbf{k}}^*` can be symbolically eliminated to obtain the generalized equations of motion as:

.. math::
    :label: eq:generalized_eqns_motion

    H^TC^{-T}MC^{-1}H \underline{\ddot{q}} = \underline{f} + H^TC^{-T}(\underline{\mathbf{g}}^* - MC^{-1}\underline{\gamma})


which can be expressed as:

.. math::
    :label: eq:generalized_eqns_motion_no_constraints

    M^{\ell} \underline{\ddot{q}} = \underline{f}^{\ell}


where:

.. math::
    :label: eq:gen_mass_matrix

    M^{\ell} = H^TC^{-T}MC^{-1}H


and:

.. math::
    :label: eq:generalized_forces

    \underline{f}^{\ell} = \underline{f} + H^TC^{-T}(\underline{\mathbf{g}}^* - MC^{-1}\underline{\gamma})


The symmetric, positive semidefinite :math:`M^{\ell}` corresponds to the generalized mass matrix. It contains the same quantities that would have been obtained by expanding a Lagrangian, hence the superscript :math:`\ell` :cite:`pars1979`. :math:`\underline{f}^{\ell}` represents the generalized forces, and additionally contains quantities that would have been obtained from expanding a Lagrangian. While equation :eq:`eq:generalized_eqns_motion_no_constraints` is suitable to describe the dynamic behavior of mechanisms with star configurations, for mechanisms with multiple loops it is necessary to append constraints to the system of equations. Setting up constraint equations for a multibody system requires identifying a *spanning* tree, i.e. a root-oriented directed graph that provides a unique path between each body and the fixed inertial reference frame.


which was introduced in Eq. :eq:`eq:lagrange_eqn`. Uicker :cite:`uicker1964` and Wittenberg :cite:`roberson1966dynamical` showed that the Jacobian matrix can be generated from the *circuit*, or loop matrix, which is found from the right null space of the incidence matrix transposed, :math:`\Gamma^T` :cite:`seshu1961linear`. Note that the result of Eq. :eq:`eq:jacobian` and the methods of Uicker and Wittenberg are equivalent.

A mechanisms constraint equations may include time varying constraints, :math:`\underline{e}(t)`, such as from springs and dampers. Taking the time derivative of the constraint equations yields:

.. math::
    :label: eq:jacobian_der

    \frac{d}{dt}J \dot{q} + \underline{e}(t) = J \ddot{\underline{q}} + \dot{J}\underline{\dot{q}} + \dot{\underline{e}}(t)= J \ddot{\underline{q}} - \underline{\gamma}_c= 0


where:

.. math::
    :label: eq:constraint_term

    \underline{\gamma}_c = -\dot{J}\underline{\dot{q}} - \dot{\underline{e}}(t)


Appending the Lagrange multipliers and Eq. :eq:`eq:jacobian_der` to Eq. :eq:`eq:newton_euler2` yields Eq. :eq:`eq:lagrange_eqn`. Note that the terms :math:`M^{\ell}`, :math:`J` and :math:`\underline{\gamma}_c` are dependent on a mechanism's present configuration and must be updated at every time step. Fortunately, the overhead in updating these quantities is offset by using only the relative joint coordinates during numerical integration. Furthermore, efficient recursive methods exist to carry out the operations for updating the generalized mass matrix (i.e. Eq. :eq:`eq:gen_mass_matrix`) such as described in :cite:`lilly1993,rodriguez1991`. Another advantage to the form of Eq. :eq:`eq:lagrange_eqn` is that the coefficient matrix is symmetric -- therefore, only the terms on and below the diagonal need be processed, requiring fewer operations to factor and solve the system of equations.


Generalized Coordinate Partitioning
-----------------------------------

Despite the advantages outlined above, there are some challenges associated with the solution process. If the coefficient matrix in Eq. :eq:`eq:lagrange_eqn` is nonsingular, a unique solution can be obtained using basic linear algebra methods :cite:`nikravesh1984`. However, in practice, the Jacobian matrix almost never has full row rank, so the coefficient matrix in Eq. :eq:`eq:lagrange_eqn` is generally singular. In addition, bifurcations, end--of--stroke conditions, or instantaneous alignment of joints may occur as a system moves to different postures. Only the latter of these changes Jacobian matrix rank and overall mobility, but the solution processes remains complicated even for relatively simple mechanisms with few degrees of freedom. Robust solution methods must reliably handle these situations with minimal demands on the analyst, who may lack the training to intervene.

The various approaches to deal with rank--deficient Jacobian matrices can be classified into two groups. In the first group, all variables and equations are considered to contribute equally to the solution, and the entire set of equations is solved using a best root--mean--squared (RMS) approximation approach, such as the Moore-Penrose pseudo-inverse method described in :cite:`uicker1964`. Unfortunately, the RMS approach not only requires many additional floating--point operations, but it also may fail to converge in conditions of poor mechanical advantage (see the discussion in Section 6.5 of :cite:`uicker2013`).

In the second group, a *cutset*, or minimal set of independent variables (often referred to as generalized coordinates); a minimal set of constraint equations (often referred to as primary constraints); and a minimal set of equations of motion (often referred to as generalized equations of motion) are identified. Once the independent variables are computed from the equations of motion, the primary constraint equations are used to evaluate the dependent variables.

This second, minimal approach has demonstrated an order of magnitude speedup over the RMS method :cite:`wehage1981` and has therefore attracted attention in recent decades---although splitting the complete set of equations into primary and secondary sets was first proposed as early as the beginning of the 20th century: see, for example :cite:`maggi1896,maggi1901,woronetz1910`.

While the second approach is appealing from an efficiency standpoint, ensuring robust solutions requires care in selecting the primary equations and independent variables. Particularly, choosing variables as independent that would result in large changes in system configuration for small incremental inputs must be avoided, because this may cause numerical instability and inaccuracy problems. Therefore, the *optimal* independent generalized coordinates are those that induce the smallest changes in system configuration. It follows that this set of generalized coordinates will have roughly the greatest mechanical advantage.

Historically with the second approach, specialists choose the cutset of primary constraints and generalized coordinates based on their experience with similar classes of problems. Manual selection, however, has its limitations. For complex systems, the manual approach is prone to error and imposes a burden on the analyst. The optimal set of generalized coordinates often depends on a system's posture and may require periodic updating during an analysis. In addition, a mechanism's changing posture may periodically require a different set of primary constraint equations, or even a different number of primary equations when the Jacobian matrix rank changes.

In recent decades numerical methods have been devised to automatically identify optimal sets of independent generalized coordinates and primary constraint equations. Generalized Coordinate Partitioning (GCP), introduced by Uicker and Sheth in the context of kinematics :cite:`sheth1972digital,sheth1972imp` and extended by Wehage and Haug in the context of dynamics :cite:`wehage1982`, is a row and column permutation and partitioning method applied to the Jacobian matrix. The GCP method is a byproduct of Gaussian Elimination with Complete Pivoting (GECP). GECP factors a matrix into row--echelon form, reliably revealing its rank :cite:`turing1948`. At each successive factorization step, GECP permutes the largest remaining pivotal element to the upper--left position in the residual matrix, thereby selecting the next dependent variable with possibly the smallest mechanical advantage. After factorization is complete, rows of the largest nonsingular matrix identify the primary constraint equations, and columns identify the dependent variables.

After the GECP decomposition process, the Jacobian will have been partitioned as:

.. math::
    :label: eq:conventional_gcp

    \begin{bmatrix}P^p_e\\P^s_e\end{bmatrix}J\begin{bmatrix}P^d_v\\P^i_v\end{bmatrix}^T
    \begin{Bmatrix}\underline{\ddot{q}}^d\\ \underline{\ddot{q}}^i\end{Bmatrix}=
    \begin{bmatrix}
    J^{pd} & J^{pi}\\
    J^{sd} & J^{si}
    \end{bmatrix}
    \begin{Bmatrix}\underline{\ddot{q}}^d\\ \underline{\ddot{q}}^i\end{Bmatrix}=
    \begin{Bmatrix}\underline{\gamma}_c^p\\ \underline{\gamma}_c^s\end{Bmatrix}


The matrices :math:`P^p_e` and :math:`P^s_e` represent two pieces of an orthonormal permutation matrix that rearrange and partition rows of the Jacobian matrix and equations into primary and secondary components, and the matrices :math:`P^d_v` and :math:`P^i_v` represent two pieces of an orthonormal permutation matrix that rearrange and partition columns of the Jacobian matrix and rows of the column matrix of variables into dependent and independent components. Thus the column matrices :math:`\underline{\ddot{q}}` and :math:`\underline{\gamma}_c` have been appropriately permuted and partitioned according to the equation and variable permutation matrices.  As the GECP algorithm factors the Jacobian matrix, the equation and variable permutation information is stored and available for subsequent use.

The primary constraint equations are extracted from Eq. :eq:`eq:conventional_gcp` as

.. math::
    :label: eq:conventional_primary_constraint

    \begin{bmatrix}
    J^{pi} & J^{pd}
    \end{bmatrix}
    \begin{Bmatrix}
    \underline{\ddot{q}}^i\\
    \underline{\ddot{q}}^d\\
    \end{Bmatrix}=\underline{\gamma}_c^p


where this reduced Jacobian matrix now has full row rank. The secondary constraint equations are removed from the system of equations, yielding:

.. math::
    :label: eq:reduced_system_of_equations

    \begin{bmatrix}M^{\ell ii} & M^{\ell id} & J^{piT}\\
    M^{\ell di} & M^{\ell dd} & J^{pdT}\\
    J^{pi} & J^{pd} & 0 \end{bmatrix}
    \begin{Bmatrix}\underline{\ddot{q}}^i \\ \underline{\ddot{q}}^d \\ \underline{\lambda}^d
    \end{Bmatrix}=
    \begin{Bmatrix}\underline{f}^{\ell i}\\ \underline{f}^{\ell d} \\ \underline{\gamma}_c^p
    \end{Bmatrix}


The size and complexity of Eq. :eq:`eq:reduced_system_of_equations` has now been reduced compared to the form of Eq: :eq:`lagrange_eqn`. Furthermore, the coefficient matrix will be nonsingular, and all unknowns may be solved for. However, further numerical reduction can be obtained by *preconditioning* the primary constraints.

From Eq. :eq:`eq:conventional_primary_constraint` an alternate set of primary constraint equations can be formed. Equation :eq:`eq:conventional_primary_constraint` can be expressed as:

.. math::
    :label: eq:alternate_constraint_eqns

    \begin{bmatrix}
    J^{pd} & J^{pi}\\
    J^{sd} & J^{si}
    \end{bmatrix}
    \begin{Bmatrix}\underline{\ddot{q}}^d\\ \underline{\ddot{q}}^i\end{Bmatrix}=
    \begin{bmatrix}L\\L_R\end{bmatrix}\begin{bmatrix}U & U_R\end{bmatrix}
    \begin{Bmatrix}\underline{\ddot{q}}^d\\ \underline{\ddot{q}}^i\end{Bmatrix}=
    \begin{Bmatrix}
    \underline{\gamma}_c^{p}\\
    \underline{\gamma}_c^{s}
    \end{Bmatrix}


From Eq. :eq:`eq:alternate_constraint_eqns` an alternative set of primary constraint equations can be extracted as:

.. math::
    :label: eq:alternate_primary_constraints

    L\begin{bmatrix}U & U_R
    \end{bmatrix}
    \begin{Bmatrix}\underline{\ddot{q}}^d\\ \underline{\ddot{q}}^i
    \end{Bmatrix}
    =\underline{\gamma}_c^p


Since :math:`L` and :math:`U` are invertible, Eq. :eq:`eq:alternate_primary_constraints` may be revised to

.. math::
    :label: eq:alternative_primary_constraint2

    \begin{bmatrix}
    I & U^{-1}U_R
    \end{bmatrix}
    \begin{Bmatrix}
    \underline{\ddot{q}}^d \\ \underline{\ddot{q}}^i
    \end{Bmatrix}=
    \begin{bmatrix}
    I & B^{di}
    \end{bmatrix}
    \begin{Bmatrix}
    \underline{\ddot{q}}^d \\ \underline{\ddot{q}}^i
    \end{Bmatrix}
    =U^{-1}L^{-1}\underline{\gamma}_c^p=\underline{\gamma}^d


Equation :eq:`eq:alternative_primary_constraint2` identifies a replacement for the primary equations in Eq. :eq:`eq:conventional_primary_constraint` as:

.. math::
    :label: eq:alternative_primary_constraint3

    \begin{bmatrix}
    B^{di} & I
    \end{bmatrix}
    \begin{Bmatrix}
    \underline{\ddot{q}}^i\\
    \underline{\ddot{q}}^d
    \end{Bmatrix}=\underline{\gamma}^d


This new set of constraint equations can be used in place of Eq. :eq:`eq:alternate_primary_constraints`, yielding a *preconditioned* form of dynamics equations as:

.. math::
    :label: eq:preconditioned_eqns

    \begin{bmatrix}M^{\ell ii} & M^{\ell id} & B^{idT}\\
    M^{\ell di} & M^{\ell dd} & I\\
    B^{id} & I & 0 \end{bmatrix}
    \begin{Bmatrix}\underline{\ddot{q}}^i \\ \underline{\ddot{q}}^d \\ \underline{\lambda}^d
    \end{Bmatrix}=
    \begin{Bmatrix}\underline{f}^{\ell i}\\ \underline{f}^{\ell d} \\ \underline{\gamma}^d
    \end{Bmatrix}


Dynamics Equations with Kinematic Substructures
------------------------------------------------

In the previous section, a compact form of the dynamics equations were derived from the incidence matrix. In this section, it will be shown what modifications are necessary to account for the kinematic substructures.

With the addition of virtual bodies, the incidence matrix, :math:`\Gamma^T`, is now square and invertible. Accounting for the joint and body ordering and block partitioning identified above, :math:`\Gamma^T` becomes

.. math::
    :label: eq:incidence_matrix_w_substructures

    \tiny
    \Gamma^T =
    \begin{array}{l|rr:rrrrrrr:rrrr:rrrrrrr}
        & 1  & 2  & 3  & 8  & 9  & 11 & 10 & 16 & 17 & 4  & 12 & 13 & 18 & 7  & 5  & 14 & 6  & 15 & 19 & 20 \\
        \hline
    A & 1  &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &   \\
    B & -1 & 1  &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &   \\
    \hdashline
    C &    & -1 & 1  &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &   \\
    H &    & -1 &    & 1  &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &   \\
    I &    & -1 &    &    & 1  &    &    &    &    &    &    &    &    &    &    &    &    &    &    &   \\
    Q &    &    & -1 &    &    & 1  &    &    &    &    &    &    &    &    &    &    &    &    &    &   \\
    P &    &    & -1 &    &    &    & 1  &    &    &    &    &    &    &    &    &    &    &    &    &   \\
    L &    &    &    & -1 &    &    &    & 1  &    &    &    &    &    &    &    &    &    &    &    &   \\
    M &    &    &    &    & -1 &    &    &    & 1  &    &    &    &    &    &    &    &    &    &    &   \\
    \hdashline
    D &    &    & -1 &    &    &    &    &    &    & 1  &    &    &    &    &    &    &    &    &    &   \\
    J &    &    & -1 &    &    &    &    &    &    &    & 1  &    &    &    &    &    &    &    &    &   \\
    R &    &    &    &    &    &    &    &    &    & -1 &    & 1  &    &    &    &    &    &    &    &   \\
    N &    &    &    &    &    &    &    &    &    &    & -1 &    & 1  &    &    &    &    &    &    &   \\
    \hdashline
    F &    &    &    &    &    &    &    &    &    & -1 &    &    &    & 1  &    &    &    &    &    &   \\
    E &    &    &    &    &    &    &    &    &    & -1 &    &    &    &    &  1 &    &    &    &    &   \\
    K &    &    &    &    &    &    &    &    &    & -1 &    &    &    &    &    & 1  &    &    &    &   \\
    T &    &    &    &    &    &    &    &    &    &    &    &    &    & -1 &    &    & 1  &    &    &   \\
    O &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    & -1 &    & 1  &    &   \\
    G &    &    &    &    &    &    &    &    &    &    &    &    &    &    & -1 &    &    &    & 1  &   \\
    S &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    &    & -1 &    & 1 \\ \end{array}


The block pattern in Eq. :eq:`eq:incidence_matrix_w_substructures` results because the kinematic substructures are coupled to each other only through their base bodies.

With the bodies and joints now grouped/ordered according to their kinematic substructure assignments, a new mass matrix, :math:`M` may be formed with :math:`6x6` inertia matrices along the diagonal. The resulting matrix may be block partitioned as:

.. math::
    :label: eq:mass_matrix

    M=\left[
    \begin{array}{cccc}
    M_{ii} &  &  & \\
    & M_{11} & & \\
    & & M_{22} & \\
    & & & M_{33}\\
    \end{array}\right]


where :math:`M_{ii}` consists of the :math:`6\times 6` inertia terms associated with bodies 1 and 2, :math:`M_{11}` consists of inertia terms associated with bodies 3, 8, 9, 11, 10, 16 and 17, :math:`M_{22}` consists of inertia terms associated with bodies 4, 12, 13 and 18, and :math:`M_{33}` consists of inertia terms associated with bodies 7, 5, 14, 6, 15, 19 and 20. Note that a body may technically be a member of more than one kinematic substructure, but its parent joint will be included in at most one kinematic substructure.

Equation :eq:`eq:gen_mass_matrix` from the previous section defines the mapping of the mass matrix :math:`M` to the generalized mass matrix, :math:`M^{\ell}`. After the projection defined in Eq. :eq:`eq:gen_mass_matrix` is applied, the generalized mass matrix may be partitioned as:

.. math::
    :label: eq:gen_mass_matrix_substructures

    M^{\ell}= \left[
    \begin{array}{cccc}
    M^{\ell}_{ii} & M^{\ell}_{i1} & M^{\ell}_{i2} & M^{\ell}_{i3}\\
    M^{\ell}_{1i} & M^{\ell}_{11} & M^{\ell}_{12} & M^{\ell}_{13}\\
    M^{\ell}_{2i} & M^{\ell}_{21} & M^{\ell}_{22} & M^{\ell}_{23}\\
    M^{\ell}_{3i} & M^{\ell}_{31} & M^{\ell}_{32} & M^{\ell}_{33}
    \end{array}
    \right]


where :math:`M^{\ell}_{ii}`, :math:`M^{\ell}_{11}`, :math:`M^{\ell}_{22}`, and :math:`M^{\ell}_{33}` each have dimensions :math:`1/6th` the size of the respective submatrices :math:`M_{ii}`, :math:`M_{11}`, :math:`M_{22}`, and :math:`M_{33}` in Eq. :eq:`eq:gen_mass_matrix_substructures`.

The block–partitioned Jacobian submatrices in Eq. :eq:`eq:jacobian2` and the constraint equations may be interspersed within the generalized equations of motion as:

.. math::
    :label: eq:28

    \left[
    \begin{array}{c:cc:cc:cc}
    M^{\ell}_{ii} & M^{\ell}_{i1} & & M^{\ell}_{i2} & & M^{\ell}_{i3} & \\
    \hdashline
    M^{\ell}_{1i} & M^{\ell}_{11} & J_I^T & M^{\ell}_{12} & & M^{\ell}_{13} & \\
    & J_{1} & & & & & \\
    \hdashline
    M^{\ell}_{2i} & M^{\ell}_{21} & & M^{\ell}_{22} & J_{2}^T & M^{\ell}_{23} & \\
    & & & J_{2} & & & \\
    \hdashline
    M^{\ell}_{3i} & M^{\ell}_{31} & & M^{\ell}_{32} & & M^{\ell}_{33} & J_{3}^T \\
    & & & & & J_{3} & \\
    \end{array}
    \right]
    \begin{Bmatrix}
    \underline{\ddot{q}}_i \\
    \hdashline
    \underline{\ddot{q}}_1 \\
    \underline{\lambda}_1 \\
    \hdashline
    \underline{\ddot{q}}_2 \\
    \underline{\lambda}_2 \\
    \hdashline
    \underline{\ddot{q}}_3 \\
    \underline{\lambda}_3
    \end{Bmatrix}
    =
    \begin{Bmatrix}
    \underline{f}^{\ell}_i \\
    \hdashline
    \underline{f}^{\ell}_1 \\
    \underline{\gamma}_{c,1} \\
    \hdashline
    \underline{f}_{2}^{\ell} \\
    \underline{\gamma}_{c,2} \\
    \hdashline
    \underline{f}_{3}^{\ell} \\
    \underline{\gamma}_{c,3}
    \end{Bmatrix}


The kinematic substructures appear as the three block :math:`2 \times 2` submatrices along the diagonal of the coefficient matrix in Eq. :eq:`eq:28`. Coupling between these substructures is through the off-diagonal submatrices, which are relatively sparse. As in the previous chapter, the coefficient matrix in this system of equations typically does not have full rank. Therefore, GECP is applied to each of the submatrices :math:`J_1` -- :math:`J_3` to identify the primary constraint equations. The secondary constraint equations are removed and the preconditioning process that was described in the previous section is applied to each of the submatrices :math:`J_1` -- :math:`J_3`, yielding:

.. math::
    :label: eq:36

    \scriptsize
    \left[
    \begin{array}
    {c:ccc:ccc:ccc}
    M^{\ell ii}_{ii} & M^{\ell ii}_{i1} & M^{\ell id}_{i1} & & M^{\ell ii}_{i2} & M^{\ell id}_{i2} & & M^{\ell ii}_{i3} & M^{\ell id}_{i3} & \\
    \hdashline
    M^{\ell ii}_{1i} & M_{11}^{\ell ii} & M_{11}^{\ell id} & B_{11}^{id} & M^{\ell ii}_{12} & M^{\ell id}_{12} & & M^{\ell ii}_{13} & M^{\ell id}_{13} & \\
    M^{\ell di}_{1i} & M_{11}^{\ell di} & M_{11}^{\ell dd} & I & M^{\ell di}_{12} & M^{\ell dd}_{12} & & M^{\ell di}_{13} & M^{\ell dd}_{13} & \\
    & B^{di}_{11} & I & & & & & & & \\
    \hdashline
    M^{\ell ii}_{2i} & M^{\ell ii}_{21} & M^{\ell id}_{21} & & M_{22}^{\ell ii} & M_{22}^{\ell id} & B^{id}_{22} & M^{\ell ii}_{23} & M^{\ell id}_{23} & \\
    M^{\ell di}_{2i} & M^{\ell di}_{21} & M_{21}^{\ell dd} & & M_{22}^{\ell di} & M_{22}^{\ell dd} & I & M^{\ell di}_{23} & M^{\ell dd}_{23} & \\
    & & & & B^{di}_{22} & I & & & & \\
    \hdashline
    M^{\ell ii}_{3i} & M^{\ell ii}_{31} & M^{\ell id}_{31} & & M_{32}^{\ell ii} & M_{32}^{\ell id} & & M_{33}^{\ell ii} & M_{33}^{\ell id} & B^{id}_{33} \\
    M^{\ell di}_{3i} & M^{\ell di}_{31} & M_{31}^{\ell dd} & & M_{32}^{\ell di} & M_{32}^{\ell dd} & & M^{\ell di}_{33} & M_{33}^{\ell dd} & I \\
    & & & & & & & B^{di}_{33} & I & \\
    \end{array}
    \right]
    \begin{Bmatrix}
    \underline{\ddot{q}}_{i}\\
    \hdashline
    \underline{\ddot{q}}_{1}^i\\
    \underline{\ddot{q}}_{1}^d\\
    \underline{\lambda}_{1}^d\\
    \hdashline
    \underline{\ddot{q}}_{2}^i\\
    \underline{\ddot{q}}_{2}^d\\
    \underline{\lambda}_{2}^d\\
    \hdashline
    \underline{\ddot{q}}_{3}^i\\
    \underline{\ddot{q}}_{3}^d\\
    \underline{\lambda}_{3}^d\\
    \end{Bmatrix} =
    \begin{Bmatrix}
    \underline{f}^{\ell}_i\\
    \hdashline
    \underline{f}_{1}^{\ell i}\\
    \underline{f}_{1}^{\ell d}\\
    \underline{\gamma}_{1}^{d}\\
    \hdashline
    \underline{f}_{2}^{\ell i}\\
    \underline{f}_{2}^{\ell d}\\
    \underline{\gamma}_{2}^{d}\\
    \hdashline
    \underline{f}_{3}^{\ell i}\\
    \underline{f}_{3}^{\ell d}\\
    \underline{\gamma}_{3}^{d}\\
    \end{Bmatrix}


In this form, inverting the block :math:`2 \times 2` pivotal submatrices containing the identity matrices is trivial. Refer to the next section for details of the solution process.


Numerical Solution of Dynamics Equations
----------------------------------------

Method of the Schur Complement
******************************
The method of the Schur complement can be used to solve the preconditioned dynamics equations with kinematic substructures. The Schur complement method is a block matrix factorization algorithm :cite:`zhang2006schur`. Before illustrating how the Schur complement method would be used to solve a system of dynamics equations, a simple example is provided to demonstrate how the Schur complement method is used in general.

Consider the block partitioned system of equations:

.. math::
  :label: schur_example

  \begin{bmatrix}
  A & C^T \\ C & D
  \end{bmatrix}
  \begin{Bmatrix}\underline{x}\\\underline{y}\end{Bmatrix}=
  \begin{Bmatrix}\underline{a} \\ \underline{b}\end{Bmatrix}


To solve the system of equations:

  * Multiply the lower block row by :math:`C^TD^{-1}`, then subtract the second block row from the first block row:

  .. math::
    :label: eq:schur_1

    \require{cancel}
    A\underline{x} + \cancel{C^T\underline{y}} - C^TD^{-1}C\underline{x}  - \cancel{C^TD^{-1}D\underline{y}} = \underline{a} - C^TD^{-1}\underline{b}


  .. math::
    :label: eq:schur_2

    \left(A - C^TD^{-1}C\right)\underline{x} = \underline{a} - C^TD^{-1}\underline{b} \rightarrow   A'\underline{x} = \underline{a}'

  * Store the results of backward elimination in the coefficient matrix and right--hand side using in--place assignment. Here, :math:`A'` is the Schur complement.

  * Now :math:`\underline{x}` and :math:`\underline{y}` can be solved for using the smaller submatrices as:

  .. math::
    :label: eq:schur_4

    A'\underline{x} = \underline{a}'


  .. math::
    :label: eq:schur_5

    \underline{x} = A'^{-1}\underline{a}'


  Once x is known, y can be found from the second block row of :eq:`eq:schur_1`.

  .. math::
    :label: schur_example2

    \underline{y} = D^{-1}(\underline{b} - C\underline{x})


Next, consider how the method of the Schur complement can be used to solve the preconditioned dynamics equations. Consider a simple example, consisting of only one kinematic substructure:

.. math::
  :label: eq:simple_system

  \left[
  \begin{array}{c:cc}
  M^{\ell ii}&M^{\ell id}&B^{id}\\
  \hdashline
  M^{\ell di}&M^{\ell dd}&I\\
  B^{di}&I&0
  \end{array}
  \right]
  \begin{Bmatrix}
  \ddot{\underline{q}}^i\\ \hdashline \ddot{\underline{q}}^d \\ \underline{\lambda}^d
  \end{Bmatrix}=
  \begin{Bmatrix}
  \underline{f}^{\ell i} \\ \hdashline \underline{f}^{\ell d}\\ \underline{\gamma}^d
  \end{Bmatrix}


The lower right block submatrix of the coefficient matrix in Eq. :eq:`eq:simple_system` has a trivial inverse, given as:

.. math::
  :label: eq:trivial_inverse

  \begin{bmatrix}M^{dd}& I\\ I & 0\end{bmatrix}^{-1} = \begin{bmatrix}0& I\\ I & -M^{dd}\end{bmatrix}


The block submatrix in Eq. :eq:`eq:trivial_inverse` has an inverse, even if :math:`M_{dd}` is singular.

Carrying out the Schur complement projects the quantities :math:`\begin{bmatrix}M^{\ell id} & B^T\end{bmatrix} \begin{bmatrix}0 & I \\ I & -M^{\ell dd} \end{bmatrix} \begin{bmatrix}M^{\ell di} \\ B\end{bmatrix}` and :math:`\begin{bmatrix}M^{\ell id} & B^T\end{bmatrix} \begin{bmatrix}0 & I \\ I & -M^{\ell dd} \end{bmatrix}\begin{Bmatrix} \underline{f}^{\ell d} \\ \underline{\gamma}^d \end{Bmatrix}` onto :math:`M^{\ell ii}` and :math:`\underline{f}^{\ell i}` respectively.


.. math::
  :label: eq:schur_7

  \left(M^{\ell ii} - \begin{bmatrix}M^{\ell id} & B^T\end{bmatrix} \begin{bmatrix}0 & I \\ I & -M^{\ell dd} \end{bmatrix} \begin{bmatrix}M^{\ell di} \\ B\end{bmatrix}\right)\underline{\ddot{q}}^i =
  \left(\underline{f}^{\ell i}  -
  \begin{bmatrix}M^{\ell id} & B^T\end{bmatrix} \begin{bmatrix}0 & I \\ I & -M^{\ell dd} \end{bmatrix}
  \begin{Bmatrix} \underline{f}^{\ell d} \\ \underline{\gamma}^d \end{Bmatrix}\right)


Again, no inversion of pivots is necessary to carry out the projection detailed in Eq. :eq:`eq:schur_7`. The resulting coefficient matrix in Eq. :eq:`eq:schur_7` will be positive-definite and nonsingular if all bodies associated with independent variables have mass or if massless bodies associated with independent variables have mass projected on them by dependent variables in application of the Schur complement. For this to occur, any massless independent bodies should lie in the kinematic path of bodies associated with dependent variables with mass. After the projection step, if full, dense matrix algebra were applied, Eq. :eq:`eq:schur_7` could be solved using standard methods such as :math:`L^TL` or :math:`L^TDL` decomposition.

Once :math:`\ddot{\underline{q}}^i` is known, :math:`\ddot{\underline{q}}^d` can be found from the third block row of Eq. :eq:`eq:simple_system`.

.. math::
  :label: eq:schur_8

  \ddot{\underline{q}}^d = \underline{\gamma}-B^{di}\underline{\ddot{q}}^i


Equivalence to Maggi's Equations
********************************

The system of preconditioned dynamics equations is equivalent to a form of Maggi's equations, as presented in :cite:`dejalon2012efficient`, shown below:

.. math::
  :label: eq:maggi

  Z^TM^{\ell}Z \underline{\ddot{q}}^i = Z^T\underline{f}^{\ell} + Z^TM^{\ell}\begin{bmatrix}(J^{pd})^{-1}\\ 0\end{bmatrix}\underline{\gamma}_c

One benefit to using the preconditioned form of the dynamics equations is that it is not necessary to carry out the full matrix product :math:`Z^TM^{\ell}Z`. Using the preconditioning method, it is possible to carry out factorization working with the smaller block submatrices.

To illustrate how the two methods are equivalent, consider how the Maggi's method can be block partitioned and rearranged as:

.. math::
  :label: eq:maggi1

  Z^TM^{\ell}Z \underline{\ddot{q}}^i = Z^T\underline{f}^{\ell} + Z^TM^{\ell}\begin{bmatrix}(J^{pd})^{-1}\\ 0\end{bmatrix}\underline{\gamma}_c


.. math::
  :label: eq:maggi2

  \begin{bmatrix}-B^T & I\end{bmatrix}
  \begin{bmatrix}M^{\ell dd} & M^{\ell di}\\M^{\ell id} & M^{\ell ii}\end{bmatrix}
  \begin{bmatrix}-B \\ I\end{bmatrix}\underline{\ddot{q}}^i =
  \begin{bmatrix}-B^T & I\end{bmatrix}
  \begin{Bmatrix}\underline{f}^{\ell d}\\ \underline{f}^{\ell i}\end{Bmatrix}
  + \begin{bmatrix}-B^T & I\end{bmatrix}
  \begin{bmatrix}M^{\ell dd} & M^{\ell di}\\M^{\ell id} & M^{\ell ii}\end{bmatrix}
  \begin{Bmatrix}\underline{\gamma}^d \\ 0 \end{Bmatrix}


.. math::
  :label: eq:maggi3

  \left(M^{\ell ii} - B^TM^{\ell di} - M^{\ell id}B + B^TM^{\ell dd}B\right)
  \underline{\ddot{q}}^i=
  \begin{bmatrix}-B^T & I\end{bmatrix}
  \begin{Bmatrix}\underline{f}^{\ell d}\\ \underline{f}^{\ell i}\end{Bmatrix}
  + \begin{bmatrix}-B^T & I\end{bmatrix}
  \begin{bmatrix}M^{\ell dd} \\M^{\ell id}\end{bmatrix}
  \underline{\gamma}^d


Similarly, the preconditioned equations can be block partitioned and rearranged as:

.. math::
  :label: eq:preconditioned_eqns1

  \label{eq:preconditioned_eqns}
  \begin{bmatrix}M^{\ell ii} & M^{\ell id} & B^T\\
  M^{\ell di} & M^{\ell dd} & I\\
  B & I & 0 \end{bmatrix}
  \begin{Bmatrix}\underline{\ddot{q}}^i \\ \underline{\ddot{q}}^d \\ \underline{\lambda}^d
  \end{Bmatrix}=
  \begin{Bmatrix}\underline{f}^{\ell i}\\ \underline{f}^{\ell d} \\ \underline{\gamma}^d
  \end{Bmatrix}


.. math::
  :label: eq:preconditioned_eqns2

  \label{eq:schur_complement}
  \left(M^{\ell ii} - \begin{bmatrix}M^{\ell id} & B^T\end{bmatrix} \begin{bmatrix}0 & I \\ I & -M^{\ell dd} \end{bmatrix} \begin{bmatrix}M^{\ell di} \\ B\end{bmatrix}\right)\underline{\ddot{q}}^i =
  \left(\underline{f}^{\ell i}  -
  \begin{bmatrix}M^{\ell id} & B^T\end{bmatrix} \begin{bmatrix}0 & I \\ I & -M^{\ell dd} \end{bmatrix}
  \begin{bmatrix} \underline{f}^{\ell d} \\ \underline{\gamma}^d \end{bmatrix}\right)


.. math::
  :label: eq:preconditioned_eqns3

  \left(M^{\ell ii} -
  \begin{bmatrix}B^T & M^{\ell id}-B^TM^{\ell dd}\end{bmatrix}
    \begin{bmatrix}M^{\ell di} \\ B
      \end{bmatrix}\right)
    \underline{\ddot{q}}^i=
    \left(\underline{f}^{\ell i}  -
    \begin{bmatrix}B^T & M^{\ell id}-B^TM^{\ell dd}\end{bmatrix}
    \begin{bmatrix} \underline{f}^{\ell d} \\ \underline{\gamma}^d \end{bmatrix}\right)


.. math::
  :label: eq:preconditioned_eqns4

  \left(M^{\ell ii} - B^TM^{\ell di} - M^{\ell id}B + B^TM^{\ell dd}B\right)
  \underline{\ddot{q}}^i=
  \underline{f}^{\ell i} - B^T \underline{f}^{\ell d} + \left(M^{\ell id} - B^TM^{\ell dd}\right)\underline{\gamma}^d


.. math::
  :label: eq:preconditioned_eqns5

  \left(M^{\ell ii} - B^TM^{\ell di} - M^{\ell id}B + B^TM^{\ell dd}B\right)
  \underline{\ddot{q}}^i=
  \begin{bmatrix}-B^T & I\end{bmatrix}
  \begin{Bmatrix}\underline{f}^{\ell d}\\ \underline{f}^{\ell i}\end{Bmatrix} +
  \begin{bmatrix}-B^T & I\end{bmatrix}
  \begin{bmatrix}M^{\ell dd} \\ M^{\ell id}\end{bmatrix}\underline{\gamma}^d


Equations :eq:`eq:maggi3` and :eq:`eq:preconditioned_eqns5` are equivalent.


Method of Preconditioning / Schur Complement with Multiple Substructures
************************************************************************

Although the Schur complement solution method was described in context to a system of equations with one substructure, the solution method generalizes to multiple substructures, such as the example of the excavator. The excavator's preconditioned system of equations is shown again below with additional block partitioning:

.. math::
    :label: eq:37

    \scriptsize
    \left[
    \begin{array}
    {c:c:cc:c:cc:c:cc}
    M^{\ell ii}_{ii} & M^{\ell ii}_{i1} & M^{\ell id}_{i1} & & M^{\ell ii}_{i2} & M^{\ell id}_{i2} & & M^{\ell ii}_{i3} & M^{\ell id}_{i3} & \\
    \hdashline
    M^{\ell ii}_{1i} & M_{11}^{\ell ii} & M_{11}^{\ell id} & B_{11}^{id} & M^{\ell ii}_{12} & M^{\ell id}_{12} & & M^{\ell ii}_{13} & M^{\ell id}_{13} & \\
    \hdashline
    M^{\ell di}_{1i} & M_{11}^{\ell di} & M_{11}^{\ell dd} & I & M^{\ell di}_{12} & M^{\ell dd}_{12} & & M^{\ell di}_{13} & M^{\ell dd}_{13} & \\
    & B^{di}_{11} & I & & & & & & & \\
    \hdashline
    M^{\ell ii}_{2i} & M^{\ell ii}_{21} & M^{\ell id}_{21} & & M_{22}^{\ell ii} & M_{22}^{\ell id} & B^{id}_{22} & M^{\ell ii}_{23} & M^{\ell id}_{23} & \\
    \hdashline
    M^{\ell di}_{2i} & M^{\ell di}_{21} & M_{21}^{\ell dd} & & M_{22}^{\ell di} & M_{22}^{\ell dd} & I & M^{\ell di}_{23} & M^{\ell dd}_{23} & \\
    & & & & B^{di}_{22} & I & & & & \\
    \hdashline
    M^{\ell ii}_{3i} & M^{\ell ii}_{31} & M^{\ell id}_{31} & & M_{32}^{\ell ii} & M_{32}^{\ell id} & & M_{33}^{\ell ii} & M_{33}^{\ell id} & B^{id}_{33} \\
    \hdashline
    M^{\ell di}_{3i} & M^{\ell di}_{31} & M_{31}^{\ell dd} & & M_{32}^{\ell di} & M_{32}^{\ell dd} & & M^{\ell di}_{33} & M_{33}^{\ell dd} & I \\
    & & & & & & & B^{di}_{33} & I & \\
    \end{array}
    \right]
    \begin{Bmatrix}
    \underline{\ddot{q}}_{i}\\
    \hdashline
    \underline{\ddot{q}}_{1}^i\\
    \underline{\ddot{q}}_{1}^d\\
    \underline{\lambda}_{1}^d\\
    \hdashline
    \underline{\ddot{q}}_{2}^i\\
    \underline{\ddot{q}}_{2}^d\\
    \underline{\lambda}_{2}^d\\
    \hdashline
    \underline{\ddot{q}}_{3}^i\\
    \underline{\ddot{q}}_{3}^d\\
    \underline{\lambda}_{3}^d\\
    \end{Bmatrix} =
    \begin{Bmatrix}
    \underline{f}^{\ell}_i\\
    \hdashline
    \underline{f}_{1}^{\ell i}\\
    \underline{f}_{1}^{\ell d}\\
    \underline{\gamma}_{1}^{d}\\
    \hdashline
    \underline{f}_{2}^{\ell i}\\
    \underline{f}_{2}^{\ell d}\\
    \underline{\gamma}_{2}^{d}\\
    \hdashline
    \underline{f}_{3}^{\ell i}\\
    \underline{f}_{3}^{\ell d}\\
    \underline{\gamma}_{3}^{d}\\
    \end{Bmatrix}


To solve the system of equations using the Schur complement, start from the lower right of the coefficient matrix. Using the labels from the example above, let:

.. math::
  :label: eq:A

  A = \left[
  \begin{array}
  {c:c:cc:c:cc:c}
  M^{\ell ii}_{ii} & M^{\ell ii}_{i1} & M^{\ell id}_{i1} & & M^{\ell ii}_{i2} & M^{\ell id}_{i2} & & M^{\ell ii}_{i3} \\
  \hdashline
  M^{\ell ii}_{1i} & M_{11}^{\ell ii} & M_{11}^{\ell id} & B_{11}^{id} & M^{\ell ii}_{12} & M^{\ell id}_{12} & & M^{\ell ii}_{13} \\
  \hdashline
  M^{\ell di}_{1i} & M_{11}^{\ell di} & M_{11}^{\ell dd} & I & M^{\ell di}_{12} & M^{\ell dd}_{12} & & M^{\ell di}_{13} \\
  & B^{di}_{11} & I & & & & & \\
  \hdashline
  M^{\ell ii}_{2i} & M^{\ell ii}_{21} & M^{\ell id}_{21} & & M_{22}^{\ell ii} & M_{22}^{\ell id} & B^{id}_{22} & M^{\ell ii}_{23} \\
  \hdashline
  M^{\ell di}_{2i} & M^{\ell di}_{21} & M_{21}^{\ell dd} & & M_{22}^{\ell di} & M_{22}^{\ell dd} & I & M^{\ell di}_{23} \\
  & & & & B^{di}_{22} & I & & \\
  \hdashline
  M^{\ell ii}_{3i} & M^{\ell ii}_{31} & M^{\ell id}_{31} & & M_{32}^{\ell ii} & M_{32}^{\ell id} & & M_{33}^{\ell ii} \\
  \end{array}
  \right]


.. math::
  :label: eq:C

  C = \left[
  \begin{array}{c:c:cc:c:cc:c}
  M^{\ell di}_{3i} & M^{\ell di}_{31} & M_{31}^{\ell dd} & & M_{32}^{\ell di} & M_{32}^{\ell dd} & & M^{\ell di}_{33}\\
  & & & & & & & B^{di}_{33} \\
  \end{array}
  \right]


and

.. math::
  :label: eq:D

  D = \begin{bmatrix}
  M_{33}^{\ell dd} & I\\I &
  \end{bmatrix}

The matrix product :math:`C^TD^{-1}C` is given as:

.. math::
  :label: eq:projection

  \scriptsize
  C^TD^{-1}C =
  \begin{bmatrix}
  & & & & & & & M_{i3}^{\ell}B_{33}^{di}\\
  & & & & & & & M_{13}^{\ell di}B_{33}^{di}\\
  & & & & & & & M_{13}^{\ell dd}B_{33}^{di}\\
  & & & & & & & \\
  & & & & & & & M_{23}^{\ell di}B_{33}^{di}\\
  & & & & & & & M_{23}^{\ell dd}B_{33}^{di}\\
  & & & & & & & \\
  B_{33}^{id}M_{3i}^{\ell}& B_{33}^{id}M_{31}^{\ell di} & B_{33}^{id}M_{31}^{\ell dd} & &  B_{33}^{id}M_{32}^{\ell di} & B_{33}^{id}M_{32}^{\ell dd} & & B_{33}^{id}M_{33}^{dd} + M_{23}^{dd}B_{33}^{di} - B_{33}^{id}M_{33}^{\ell dd}B_{33}^{di}
  \end{bmatrix}


Carrying out the Schur complement on the coefficient matrix yields:

.. math::
  :label: eq:schur_substructure

  \scriptsize
  \begin{array}{l}
  A - C^TD^{-1}C =\\
  \left[
  \begin{array}
  {c:c:cc:c:cc:c}
  M^{\ell ii}_{ii} & M^{\ell ii}_{i1} & M^{\ell id}_{i1} & & M^{\ell ii}_{i2} & M^{\ell id}_{i2} & & \color{red}{M^{\ell ii}_{i3} - M_{i3}^{\ell}B_{33}^{di}} \\
  \hdashline
  M^{\ell ii}_{1i} & M_{11}^{\ell ii} & M_{11}^{\ell id} & B_{11}^{id} & M^{\ell ii}_{12} & M^{\ell id}_{12} & & \color{red}{M^{\ell ii}_{13} - M_{13}^{\ell di}B_{33}^{di}}\\
  \hdashline
  M^{\ell di}_{1i} & M_{11}^{\ell di} & M_{11}^{\ell dd} & I & M^{\ell di}_{12} & M^{\ell dd}_{12} & & \color{red}{M^{\ell di}_{13} -M_{13}^{\ell dd}B_{33}^{di}}\\
  & B^{di}_{11} & I & & & & & \\
  \hdashline
  M^{\ell ii}_{2i} & M^{\ell ii}_{21} & M^{\ell id}_{21} & & M_{22}^{\ell ii} & M_{22}^{\ell id} & B^{id}_{22} & \color{red}{M^{\ell ii}_{23} - M_{23}^{\ell di}B_{33}^{di}}\\
  \hdashline
  M^{\ell di}_{2i} & M^{\ell di}_{21} & M_{21}^{\ell dd} & & M_{22}^{\ell di} & M_{22}^{\ell dd} & I & \color{red}{M^{\ell di}_{23} - M_{23}^{\ell dd}B_{33}^{di}} \\
  & & & & B^{di}_{22} & I & & \\
  \hdashline
  \color{red}{M^{\ell ii}_{3i} - B_{33}^{id}M_{3i}^{\ell}} & \color{red}{M^{\ell ii}_{31} - B_{33}^{id}M_{31}^{\ell di}} & \color{red}{M^{\ell id}_{31} - B_{33}^{id}M_{31}^{\ell dd}} & & \color{red}{M_{32}^{\ell ii} - B_{33}^{id}M_{32}^{\ell di}} & \color{red}{M_{32}^{\ell id} - B_{33}^{id}M_{32}^{\ell dd}} & & \color{red}{M_{33}^{\ell ii} - (B_{33}^{id}M_{33}^{dd} + M_{23}^{dd}B_{33}^{di} - B_{33}^{id}M_{33}^{\ell dd}B_{33}^{di})}\\
  \end{array}
  \right]
  \end{array}


Note that the block sparsity pattern of the residual matrix is unchanged after the first application of the Schur complement.

Next, consider the next application of the Schur complement. Let:

.. math::
  :label: eq:a_prime

  \scriptsize
  A' =
  \left[
  \begin{array}
  {c:c:cc:c:cc}
  M^{\ell ii}_{ii} & M^{\ell ii}_{i1} & M^{\ell id}_{i1} & & M^{\ell ii}_{i2} & M^{\ell id}_{i2} & \\
  \hdashline
  M^{\ell ii}_{1i} & M_{11}^{\ell ii} & M_{11}^{\ell id} & B_{11}^{id} & M^{\ell ii}_{12} & M^{\ell id}_{12} &\\
  \hdashline
  M^{\ell di}_{1i} & M_{11}^{\ell di} & M_{11}^{\ell dd} & I & M^{\ell di}_{12} & M^{\ell dd}_{12} &\\
  & B^{di}_{11} & I & & & & \\
  \hdashline
  M^{\ell ii}_{2i} & M^{\ell ii}_{21} & M^{\ell id}_{21} & & M_{22}^{\ell ii} & M_{22}^{\ell id} & B^{id}_{22}\\
  \hdashline
  M^{\ell di}_{2i} & M^{\ell di}_{21} & M_{21}^{\ell dd} & & M_{22}^{\ell di} & M_{22}^{\ell dd} & I\\
  & & & & B^{di}_{22} & I &
  \end{array}
  \right]


.. math::
  :label: eq:d_prime

  D' = M_{33}^{\ell ii} - (B_{33}^{id}M_{33}^{dd} + M_{23}^{dd}B_{33}^{di} - B_{33}^{id}M_{33}^{\ell dd}B_{33}^{di})


.. math::
  :label: eq:c_prime

  C' =
  \begin{bmatrix}
    M^{\ell ii}_{3i} - B_{33}^{id}M_{3i}^{\ell} &
    M^{\ell ii}_{31} - B_{33}^{id}M_{31}^{\ell di} &
    M^{\ell id}_{31} - B_{33}^{id}M_{31}^{\ell dd} & 0 &
    M_{32}^{\ell ii} - B_{33}^{id}M_{32}^{\ell di} &
    M_{32}^{\ell id} - B_{33}^{id}M_{32}^{\ell dd} & 0
  \end{bmatrix}


.. math::
  :label: eq_c_prime2

  C' =
  \begin{bmatrix}
    M'^{\ell ii}_{3i} &
    M'^{\ell ii}_{31} &
    M'^{\ell id}_{31} & 0 &
    M'^{\ell ii}_{32} &
    M'^{\ell id}_{32} & 0
  \end{bmatrix}


.. math::
  :label: eq:a_double_prime

  \scriptsize
  C'^TD'^{-1}C' =
  \begin{bmatrix}
    M'^{\ell ii}_{i3}D'^{-1}M'^{\ell ii}_{3i} &
    M'^{\ell ii}_{i3}D'^{-1}M'^{\ell ii}_{31} &
    M'^{\ell ii}_{i3}D'^{-1}M'^{\ell id}_{31} & &
    M'^{\ell ii}_{i3}D'^{-1}M'^{\ell ii}_{32} &
    M'^{\ell ii}_{i3}D'^{-1}M'^{\ell id}_{32} & &
    \\
    M'^{\ell ii}_{13}D'^{-1}M'^{\ell ii}_{3i} &
    M'^{\ell ii}_{13}D'^{-1}M'^{\ell ii}_{31} &
    M'^{\ell ii}_{13}D'^{-1}M'^{\ell id}_{31} & &
    M'^{\ell ii}_{13}D'^{-1}M'^{\ell ii}_{32} &
    M'^{\ell ii}_{13}D'^{-1}M'^{\ell id}_{32} & &
    \\
    M'^{\ell di}_{13}D'^{-1}M'^{\ell ii}_{3i} &
    M'^{\ell di}_{13}D'^{-1}M'^{\ell ii}_{31} &
    M'^{\ell di}_{13}D'^{-1}M'^{\ell id}_{31} & &
    M'^{\ell di}_{13}D'^{-1}M'^{\ell ii}_{32} &
    M'^{\ell di}_{13}D'^{-1}M'^{\ell id}_{32} & &
    \\
    & & & & & & & \\
    M'^{\ell ii}_{23}D'^{-1}M'^{\ell ii}_{3i} &
    M'^{\ell ii}_{23}D'^{-1}M'^{\ell ii}_{31} &
    M'^{\ell ii}_{23}D'^{-1}M'^{\ell id}_{31} & &
    M'^{\ell ii}_{23}D'^{-1}M'^{\ell ii}_{32} &
    M'^{\ell ii}_{23}D'^{-1}M'^{\ell id}_{32} & & \\
    M'^{\ell di}_{23}D'^{-1}M'^{\ell ii}_{3i} &
    M'^{\ell di}_{23}D'^{-1}M'^{\ell ii}_{31} &
    M'^{\ell di}_{23}D'^{-1}M'^{\ell id}_{31} & &
    M'^{\ell di}_{23}D'^{-1}M'^{\ell ii}_{32} &
    M'^{\ell di}_{23}D'^{-1}M'^{\ell id}_{32} & &
    \\
    & & & & & & &
  \end{bmatrix}


Projecting the quantity :math:`C'^TD'^{-1}C'` onto the residual matrix, :math:`A'`, yields:

.. math::
  :label: eq:second_projection

  \scriptsize
  A'-C'^TD'^{-1}C' =
  \left[
  \begin{array}
  {c:c:cc:c:cc:c}
  \color{red}{M'^{\ell ii}_{ii}} &
  \color{red}{M'^{\ell ii}_{i1}} &
  \color{red}{M'^{\ell id}_{i1}} & &
  \color{red}{M'^{\ell ii}_{i2}} &
  \color{red}{M'^{\ell id}_{i2}} & \\
  \hdashline
  \color{red}{M'^{\ell ii}_{1i}} &
  \color{red}{M'^{\ell ii}_{11}} &
  \color{red}{M'^{\ell id}_{11}} & B_{11}^{id} &
  \color{red}{M'^{\ell ii}_{12}} &
  \color{red}{M'^{\ell id}_{12}} & \\
  \hdashline
  \color{red}{M'^{\ell di}_{1i}} &
  \color{red}{M'^{\ell di}_{11}} &
  \color{red}{M'^{\ell dd}_{11}} & I &
  \color{red}{M'^{\ell di}_{12}} &
  \color{red}{M'^{\ell dd}_{12}} & \\
  & B^{di}_{11} & I & & & & \\
  \hdashline
  \color{red}{M'^{\ell ii}_{2i}} &
  \color{red}{M'^{\ell ii}_{21}} &
  \color{red}{M'^{\ell id}_{21}} & &
  \color{red}{M'^{\ell ii}_{22}} &
  \color{red}{M'^{\ell id}_{22}} & B^{id}_{22} \\
  \hdashline
  \color{red}{M'^{\ell di}_{2i}} &
  \color{red}{M'^{\ell di}_{21}} &
  \color{red}{M'^{\ell dd}_{21}} & &
  \color{red}{M'^{\ell di}_{22}} &
  \color{red}{M'^{\ell dd}_{22}} & I\\
  & & & & B^{di}_{22} & I &
  \end{array}
  \right]


The quantities that have been modified during the projection are highlighted in red and noted with primes (:math:`'`). In the second application of the Schur complement, note that no projections were made onto the preconditioned constraint terms. The lower right block of Eq. :eq:`eq:second_projection` has the same structure as Eq. :eq:`eq:D` and again has an inverse, even if :math:`M'^{\ell dd}_{22}` is singular. The process of backward elimination continues similarly for the residual matrix. Once backward elimination is complete, the generalized accelerations and lagrange multipliers can be found by forward substitution.


Sparse Matrix Methods and Matrix Fills
**************************************

Up to this point, the solution process was described using full--dense matrix algebra. Note, however that the coefficient matrix in Eq. :eq:`eq:37` is moderately sparse, and therefore the factorization algorithm benefits from sparse matrix methods. Additionally, the coefficient matrix is symmetric, and therefore only the terms on and below the diagonal need to be processed or stored. Furthermore, knowledge of where the projections illustrated in Eqs. :eq:`eq:schur_substructure` and :eq:`eq:second_projection` occur allows one to reduce the number of operations. A sparse matrix solution algorithm can be tailored to the unique structure of the preconditioned equations of motion.

In sparse matrix methods, a *matrix fill* indicates that during the factorization process, a position that was initially zero is *filled* with a non-zero value. When bodies and joints are arranged in monotonically level-increasing order, the Parent-Child list can be used to factor the generalized mass matrix with zero fills using :math:`L^TL` or :math:`L^TDL` decomposition :cite:`featherstone2005efficient`. Additionally, when joints are arranged in monotonically increasing level-order, the Parent-Child list can be used to update the topology of the mechanism or evaluate the generalized mass matrix.

Note that the joint ordering provided by the kinematic substructuring algorithm is not strictly monotonically, level--increasing. However, the joints are arranged in monotonically level--increasing order within each substructure. Therefore, the Parent-Child list can still be used to update quantities such as influence coefficient matrices and the generalized mass matrix.

Partitioning the generalized coordinates into independent and dependent sets typically requires permuting variables out of optimal, level--increasing order, which results in matrix fills. When GCP is carried out on each substructure independently, any permutations of variables dictated by the algorithm remain local to a given substructure. A primary advantage to permuting variables locally within each substructure is that matrix fills are minimized.


Permutation Orders
********************

The various permutations used to solve the preconditioned equations of motion are described in  this section. Let the order of bodies and joints specified by the user be *Permutation Order 0*, or :math:`PO(0)`. The user-specified order is not used in the solution process. When the kinematic substructuing process is performed, bodies and joints are arranged in monotonically level increasing order within each substructure. Let the joint ordering provided by the kinematic substructuring algorithm be called *Permutation Order 1*, or :math:`PO(1)`. Kinematic substructuring would only be performed once at the beginning of a simulation, unless the topology of the system changes, and therefore it is typically not necessary to work between :math:`PO(0)` and :math:`PO(1)`.

The Generalized Coordinate Partitioning algorithm provides a new joint ordering, with generalized coordinates ordered approximately by their mechanical advantage within each substructure. From the generalized coordinate partitioning algorithm, the dependent variables come first, followed by the independent variables. The order provided by the GCP algorithm takes the variables out of monotonically increasing level order. Let the order provided by GCP be called *Permutation Order 2*, or :math:`PO(2)`. To obviate the need to work between three levels during the solution process, when the matrix preconditioning process is applied, the results are stored in Permutation Order 3, which is described next.

The system of equations in Eq. :eq:`eq:37` has the independent coordinates first, followed by dependent coordinates within each substructure, which indicates a final reordering of variables. Let the order used in the solution process be called *Permutation Order 3*, or :math:`PO(3)`. To minimize matrix fills when factoring the coefficient matrix, within each substructure the independent variables are arranged in monotonically level-increasing order first, followed by the dependent variables in monotonically level increasing order. In the excavator example, each kinematic substructure has one independent variable, which for this particular example implies that, at most, one variable is taken out of its optimal level-increasing order.

The permutation orders of the various quantities used to solve the dynamics equations are indicated below:


Permutation Order 1
^^^^^^^^^^^^^^^^^^^^^^

Permutation Order 1, :math:`PO(1)`, is the order of variables determined by kinematic substructuring. Bodies and joints are arranged in monotonically increasing level order within each substructure. In :math:`PO(1)`, the parent-child list can be used to generate the generalized mass matrix. The values that are accessed in :math:`PO(1)` include:

    =============================     ===============    ===========
    variable name                     name in program    description
    =============================     ===============    ===========
    :math:`\underline{\ddot{q}}`      *ddq*              second time derivative of generalized coordinates
    :math:`\underline{\dot{q}}`       *dq*               first time derivative of generalized coordinates
    PC                                *PC*               parent child list
    :math:`\underline{\lambda}^p`     *lambda_p*         lagrange multipliers associated with primary constraint equations
    :math:`\underline{f}`             *f*                working forces applied in the joints
    =============================     ===============    ===========


Permutation Order 2
^^^^^^^^^^^^^^^^^^^

Permutation Order 2, :math:`PO(2)`, is the order of variables determined by generalized coordinate partitioning with dependent variables first and independent variables last within each substructure. To avoid working between three levels during the factorization algorithm, the results of *preconditioning* are saved in Permutation Order 3.

    ============================================================    ===============    ===========
    variable name                                                   name in program    description
    ============================================================    ===============    ===========
    :math:`\underline{\gamma}^d`, :math:`\underline{\gamma}_c^p`    *gamma_d*          relative quadratic velocity terms across constraints, note that both :math:`\gamma^d`, :math:`\gamma_c^p` are evaluated and stored in the same location in computer memory in :math:`PO(2)`
    ============================================================    ===============    ===========


Permutation Order 3
^^^^^^^^^^^^^^^^^^^^

Permutation Order 3, :math:`PO(3)`, is the order of variables used to solve the system of dynamics equations with independent variables first and dependent last. Within each independent and dependent grouping, variables are arranged in level increasing order to minimize fills. The following quantities are stored in Permutation Order 3:

    =============================     ===============    ===========
    variable name                     name in program    description
    =============================     ===============    ===========
    :math:`M^{\ell}`                  *M_l*              generalized mass matrix
    :math:`\underline{\lambda}^d`     *lambda_d*         *preconditioned* lagrange multipliers
    :math:`\underline{f}^{\ell}`      *f_l*              generalized forces
    =============================     ===============    ===========


Representing Permutations
**************************

In linear algebra, permutations are represented using square, orthogonal binary matrices  :cite:`strang1988linear`. A column vector with quantities in :math:`PO(1)` can be permuted to :math:`PO(3)` by left multiplication with a suitable permutation matrix, :math:`P_{31}`.

.. math::
  :label: eq:permutation_example

  \underline{f}^{\ell}_{PO(3)} = P_{31} \underline{f}^{\ell}_{PO(1)}


Note the annihilation property of the subscripts associated with the permutation matrix. The subscripts on :math:`P_{31}` indicate a permutation *from* :math:`PO(1)` *to* :math:`PO(3)`.

A permutation matrix contains a 1 on each row and zeros elsewhere. To eliminate the need to process the many zeros in permutation matrices, a more compact and efficient representation is used in the computer implementation of applying permutations. Consider the permutation of variables from :math:`PO(1)` to :math:`PO(2)` in the example below:

.. math::
  :label: eq:permutation_example_2

  \begin{Bmatrix}5\\ 1 \\3\end{Bmatrix}_{PO(2)} =
  \begin{bmatrix}0 & 0 & 1\\
  1 & 0 & 0\\
  0 & 1 & 0\end{bmatrix}_{21}
  \begin{Bmatrix}
  1 \\ 3 \\ 5
  \end{Bmatrix}_{PO(1)}


where :math:`\begin{Bmatrix}1 & 3 & 5\end{Bmatrix}^T` is a column matrix of quantities in :math:`PO(1)` and :math:`\begin{Bmatrix}5 & 1 & 3\end{Bmatrix}^T` is a column matrix of quantities in :math:`PO(2)`. Then :math:`P_{21} = \begin{bmatrix}0 & 0 &1\\ 1 & 0 & 0\\ 0 & 1 & 0\end{bmatrix}`, is the permutation from :math:`PO(1)` to :math:`PO(2)`. The information in :math:`P_{21}` can be represented more compactly using a linear permutation array, :math:`\wp_{12}`, where:

.. math::
  :label: eq:permutation_array

  \wp_{12} = \begin{Bmatrix}2 & 0 & 1\end{Bmatrix}


The index into :math:`\wp_{12}` corresponds to an index into the array :math:`PO(2)`. The value found at the index indicates where to find the data in :math:`PO(1)`. Consider the example of transforming generalized forces from :math:`PO(1)` to :math:`PO(2)` as shown below:

.. math::
  :label: eq:permutation_array_example

  f^{\ell}_{PO(2)}(i_{PO(2)}) = f^{\ell}_{PO(1)}(\wp_{12}(i_{PO(2)}))


where :math:`i` is an index into :math:`\underline{f}^{\ell}_{PO(2)}` and the operation is carried out for indices of :math:`f^{\ell}_{PO(2)}`, in this case for :math:`i=0:len(f^{\ell})`, using the C/Python indexing convention.

In Python code, Eq. :eq:`eq:permutation_array_example` is expressed as:

.. code:: python

  for i in range(len(f_l2)):
    f_l2[i] = f_l1[p12[i]]


Note that the subscripts of :math:`\wp_{12}` and :math:`P_{21}` are reversed. Reversing the indices of :math:`\wp_{12}` preserves the annihilation property of indices in Eq. :eq:`eq:permutation_array_example`. The index :math:`i` is in :math:`PO(2)`. :math:`\wp_{12}` effectively converts the index :math:`i` in :math:`PO(2)` to :math:`PO(1)`.


Inverse Permutations
^^^^^^^^^^^^^^^^^^^^^

Permutation matrices are orthogonal, and therefore the inverse of the permutation matrix, :math:`P_{21}`, can be found by taking its transpose:

.. math::
  :label: eq:inv_permutation

  P_{12} = P_{21}^{-1} = P_{21}^T


The ability to convert indices in one permutation order to another can be used to find an inverse permutation array. For example, :math:`\wp_{12}^{-1}` (i.e. :math:`\wp_{21}`) can be found as follows:

.. math::
  :label: eq:inv_permutation2

  \wp_{21}(\wp_{12}(i_{PO(2)})) = i_{PO(2)}


The annihilation property of indices provides a useful debugging aid. The Python equivalent to Eq. :eq:`eq:inv_permutation2` is:

.. code::

  for i in range(len(p12)):
    p21[p12[i]] = i

.. _evaluating_permutations:

Evaluating Permutations
^^^^^^^^^^^^^^^^^^^^^^^

In the following discussion, permutation arrays will be used to set up a solution method. The generalized coordinate partitioning algorithm yields the matrix rank of each block subjacobian and local row and column permutation arrays, :math:`\wp_{12}^r` and :math:`\wp_{12}^c`. The local column permutation array effectively permutes the generalized coordinates or variables, and must be converted to a global permutation array for the entire multibody system, denoted as :math:`\wp_{12^v}`. Here the superscript :math:`v` indicates that the permutation is applied to variables. Converting each local permutation array to a single global permutation array requires knowledge of the offset of each substructure in the global list of generalized coordinates, which is computed and stored when kinematic substructuring is initially performed.

Global *identity* permutation and :code:`is_independent` arrays can be initialized as:

.. code:: python

  is_independent = [True for i in range(n)]
  p_v_12 = [i for i in range(n)]


where :code:`n` is the number of variables in the system. For each substructure, update the global permutation array as:

.. code:: python

  for i in range(len(pc)):
    p_v_12[offset + i] = pc[i] + offset


Then for each substructure, update the :code:`is_independent` array as:

.. code:: python

  for i in range(rank):
    is_independent[p_v_12[offset + i]] = pc[i] + offset


Here, :code:`offset` is the offset for each substructure, :code:`pc` is the local column permutation array and :code:`rank` is the matrix rank of the local block Jacobian. Next, the permutation array, :math:`\wp_{13}` is initialied and set.

.. code:: python

  p_v_13 = [i for i in range(n)]


Then, for each substructure:

.. code:: python

  k = offset
  for i in range(offset, column_dimenstion + offset):
    # find independent variables in level increasing order
    if is_independent[i]:
      p_v_13[k] = i
      k += 1
  for i in range(offset, column_dimension + offset):
    # find dependent variables in level increasing order
    if not is_independent[i]:
      p_v_13[k] = i
      k += 1


:code:`column_dimension` refers to the column dimension of the block submatrix of the current kinematic substructure being processed. Now that :math:`\wp_{12}` and :math:`\wp_{13}` are known, it is possible to find all other permutation arrays necessary to work between the various permutation orders.

Initialize *identity* permutation arrays as:

.. code:: python

  p_v_31 = [i for i in range(n)]
  p_v_23 = [i for i in range(n)]
  p_v_32 = [i for i in range(n)]


:math:`\wp_{23}`, :math:`\wp_{32}`, :math:`\wp_{13}` can be found using the annihilation of indices properties discussed above:


.. code:: python

  for i in range(n):
    p_v_31[p_v_13[n]] = n

  for i in range(n):
    p_v_23[i] = p_v_21[p_v_13[i]]

  for i in range(n):
    p_v_32[p_v_23[i]] = i





Evaluating Equations
**********************

Matrix Storage
^^^^^^^^^^^^^^^^

As the coefficient matrix is symmetric, memory for only the lower triangular portion need be allocated. The lower triangular portion of the coefficient matrix is allocated as a linear array with a size of :math:`((n + 1) * (n + 2)) / 2`. An array of length :math:`(n(n+1))/2` is sufficient to hold the terms on and below the diagonal of a full, dense array. In the solution process, the extra row is used as a workspace.


Evaluating the Generalized Mass Matrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Recall that the generalized mass matrix, :math:`M^{\ell}` is found by:

.. math::
  :label: gen_mass_matrix

  M^{\ell} = H^T C^{-T} M C^{-1} H


Where :math:`M` is a block diagonal matrix with body inertias along the diagonal, :math:`H` is a matrix with influence coefficient matrices along its diagonal and C is a square, nonsingular connectivity matrix, found by element-wise replacement of 1s and -1s in the transpose of the incidence matrix :math:`\Gamma^T`, with positive or negative :math:`6\times6` identity matrices,. If :math:`\Gamma` is square and nonsingular, :math:`\Gamma^{-T}` is the *oriented path matrix*, where each row shows joints that appear in the *kinematic path* from the base body to the corresponding body :cite:`uicker2013`.

.. math::
  :label: eq:path_matrix

  \Gamma^{-T} (b,h) = \begin{cases}
  +1&\text{if joint h is contained in the path from the base body to body b,}\\& \text{and if the orientation of joint h is aligned with the orientation of the path}\\
  -1& \text{if joint h is contained in the path from the base body to body b,} \\ & \text{and if joint h is aligned opposite the orientation of the path}\\
  0&\text{if joint h is not contained in the path from the base body to body b}
  \end{cases}

If the sense of all joints are aligned with the root-oriented spanning tree, the terms of the path matrix will consist of only positive ones and zeros. Note that :math:`C^{-1}` could also be formed by element-wise replacement of :math:`\Gamma^{-T}` with :math:`6 \times 6` identity matrices.

Consider the example of a hydraulic clamp mechanism, shown below:

.. figure:: figures/Diagram1.png
    :align: center

    Hydraulic Clamp Model *@todo: Redraw this using Graphviz*


After applying the cut--body approach described in section :ref:`kinematic_substructuring`, and with the addition of two virtual, massless bodies, the mass matrix has the form:

.. math::
  :label: eq:mass_matrix

  M = \begin{bmatrix}
    M_1 &     &     & &   &     & &\\
        & M_2 &     & &   &     & &\\
        &     & M_3 & &   &     & &\\
        &     &     & M_4 &     & &\\
        &     &     &     & M_5 & &\\
        &     &     &     &     & &\\
        &     &     &     &     & &
      \end{bmatrix}


:math:`C^{-1}` is:

.. math::
  :label: path_matrix

  C^{-1} = \begin{bmatrix}
  I &   &   &   &   &   &  \\
    & I &   &   &   &   &  \\
    &   & I &   &   &   &  \\
    & I &   & I &   &   &  \\
  I &   &   &   & I &   &  \\
  I &   &   &   & I & I &  \\
  I &   &   &   & I &   & I
  \end{bmatrix}


The matrix product :math:`C^{-T}MC^{-1}` has the form:

.. math::
  :label: eq:path_mass_matrix_product

  C^{-T}MC^{-1} = \begin{bmatrix}
  M_1 + M_5 &   &   &   &  M_5 &   &  \\
    & M_2 + M_4 &   & M_4  &   &   &  \\
    &   & M_3 &   &   &   &  \\
    & M_4 &   & M_4 &   &   &  \\
  M_5 &   &   &   & M_5 &   &  \\
   &   &   &   &  &  &  \\
   &   &   &   &  &   &
  \end{bmatrix}


The terms on the diagonal of Eq. :eq:`eq:path_mass_matrix_product` are sometimes referred to as *accumulated inertia matrices*. Note that the sparsity pattern of Eq. :eq:`eq:path_mass_matrix_product` reproduces symmetrically the sparsity pattern of :math:`C^{-1}`, with the exception of the virtual massless bodies, which have no contribution.

The next step in evaluating the generalized mass matrix involves carrying out the matrix product :math:`H^TC^{-T}MC^{-1}H`, which for the hydraulic clamp example is:

.. math::
  :label: eq:HTC-TMC-1H

  H^TC^{-T}MC^{-1}H = \begin{bmatrix}
  \underline{h}_1^T(M_1 + M_5)\underline{h}_1 &   &   &   &  \underline{h}_1^TM_5\underline{h}_5 &   &  \\
    & \underline{h}_2^T (M_2 + M_4) \underline{h}_2 &   & \underline{h}_2^T M_4 \underline{h}_4  &   &   &  \\
    &   & \underline{h}_3^T M_3 \underline{h}_3 &   &   &   &  \\
    & \underline{h}_4^T M_4 \underline{h}_2 &   & \underline{h}_4^T M_4 \underline{h}_4&   &   &  \\
  \underline{h}_5^T M_5 \underline{h}_1 &   &   &   & \underline{h}_5^T M_5 \underline{h}_5 &   &  \\
   &   &   &   &  &  &  \\
   &   &   &   &  &   &
  \end{bmatrix}


While the connectivity matrix and its inverse contain all the information necessary to update the generalized mass matrix, processing its many zeros is inefficient. As discussed in :ref:`kinematic_substructuring`, a more compact representation of a mechanism's topology is the parent-child list, :math:`PC`. As noted in the tables above, the coefficient matrix is evaluated in :math:`PO(3)`, but :math:`PC` and  computational quantities related to bodies and joints are stored in :math:`PO(1)`. To take advantage of the parent-child list in evaluating the generalized mass matrix, all operations are carried out in :math:`PO(1)` and the final assignment is made in :math:`PO(3)`.

Note that virtual, massless bodies (as well as any user defined massless bodies) have no contribution to the generalized mass matrix, and therefore no additional operations are required to process those bodies. A boolean :code:`has_mass` array equal in length to the number of bodies is used to determine if a body requires processing. Additionally, the :math:`6 \times 6` inertia matrices (for example, :math:`M_1`-:math:`M_5` in Eq. :eq:`eq:mass_matrix`) are stored in a linear array called :code:`inertia` of length :math:`n * 13`, with thirteen entries for each body including the mass, :math:`m`, mass times a vector to the body's center of mass, :math:`m\underline{r}`, the rotational inertia terms (:math:`I_{xx}`, :math:`I_{yy}`, :math:`I_{zz}`, :math:`I_{xy}`, :math:`I_{yz}` and :math:`I_{zx}`) and the negative of mass times a vector to the body center of mass, :math:`-m*\underline{r}`. An array called  :code:`inertia_p` is used to store the accumulated inertia matrices, shown in Eq. :eq:`eq:path_mass_matrix_product`. A temporary array called :code:`M_H_product` contains quantities in Eq. :eq:`eq:HTC-TMC-1H` in :math:`PO(1)` order.

The following section describes the minimum operations required to evaluate the matrix product :math:`C^{-T}MC^{-1}H` in :math:`PO(1)`. First the generalized mass matrix, :math:`M^{\ell}` (:code:`M_l`) is initialized to zero and the accumulated inertia matrices are computed using the Parent Child list as:

.. code:: python

  M_l[:] = 0
  inertia_p = np.copy(inertia)
  for j in range(n - 1, -1, -1):
      i = PC[j]
      if i > -1:
          for k in range(10):
              inertia_p[i * 13 + k] += inertia_p[j * 13 + k]


An additional savings can be achieved by noting that processing influence coefficient matrices for translational joints in Eq. :eq:`eq:HTC-TMC-1H` requires fewer operations -- as the last three entries will be all zero. Therefore, a boolean array called :code:`is_translation`, equal in length to the number of variables, is used to specify whether a variable corresponds to a translational (true) or rotational (false) degree of freedom.

Note that any operation that makes use of the Parent Child list implies a traversal of the mechanism's spanning tree. The above operation *projects* inertia on to the parent of a given body, starting from the leaves of the spanning tree and working towards the root, or fixed body. Some authors :cite:`featherstone1999divide` have investigated divide-and-conquer type techniques to apply parallelism in solving rigid body dynamics equations. In the case of updating the accumulated inertia matrices, two branches could be processed in parallel. There is, however, an overhead associated with synchronizing activity of multiple cores, and stopping and waiting for the result at each kinematic branch requires a high degree of synchronization. Nonetheless, for large mechanisms with many branches in the spanning tree, divide-and-conquer techniques may offer a performance improvement. Divide-and-conquer type algorithms are not the focus of study in the present work.

There is, however, an opportunity for parallelism in setting up dynamics equations that requires a minimal synchronization between cores. The operations described below that do not make use the Parent Child list may be performed in parallel. The n bodies and joints may be distributed between multiple processors, requiring no communication during processing. The threads must be joined before proceeding to the next operation.

.. code:: python

  for j in range(n):
      for i in range(10, 13):
          inertia_p[13 * j + i] = -inertia_p[13 * j + i - 9]

  for j in range(n):
      if not is_rigid[j]:
          k = index_lookup[j]

          M_H_product[k * 6 + 0] += inertia_p[13 * j + 0] * h[j * 6 + 0]
          M_H_product[k * 6 + 4] += inertia_p[13 * j + 3] * h[j * 6 + 0]
          M_H_product[k * 6 + 5] += inertia_p[13 * j + 11] * h[j * 6 + 0]

          M_H_product[k * 6 + 1] += inertia_p[13 * j + 0] * h[j * 6 + 1]
          M_H_product[k * 6 + 3] += inertia_p[13 * j + 12] * h[j * 6 + 1]
          M_H_product[k * 6 + 5] += inertia_p[13 * j + 1] * h[j * 6 + 1]

          M_H_product[k * 6 + 2] += inertia_p[13 * j + 0] * h[j * 6 + 2]
          M_H_product[k * 6 + 3] += inertia_p[13 * j + 2] * h[j * 6 + 2]
          M_H_product[k * 6 + 4] += inertia_p[13 * j + 10] * h[j * 6 + 2]

          if not is_translation[j]:
              M_H_product[k * 6 + 1] += inertia_p[13 * j + 12] * h[j * 6 + 3]
              M_H_product[k * 6 + 2] += inertia_p[13 * j + 2] * h[j * 6 + 3]
              M_H_product[k * 6 + 3] += inertia_p[13 * j + 4] * h[j * 6 + 3]
              M_H_product[k * 6 + 4] += inertia_p[13 * j + 7] * h[j * 6 + 3]
              M_H_product[k * 6 + 5] += inertia_p[13 * j + 9] * h[j * 6 + 3]

              M_H_product[k * 6 + 0] += inertia_p[13 * j + 3] * h[j * 6 + 4]
              M_H_product[k * 6 + 2] += inertia_p[13 * j + 10] * h[j * 6 + 4]
              M_H_product[k * 6 + 3] += inertia_p[13 * j + 7] * h[j * 6 + 4]
              M_H_product[k * 6 + 4] += inertia_p[13 * j + 5] * h[j * 6 + 4]
              M_H_product[k * 6 + 5] += inertia_p[13 * j + 8] * h[j * 6 + 4]

              M_H_product[k * 6 + 0] += inertia_p[13 * j + 11] * h[j * 6 + 5]
              M_H_product[k * 6 + 1] += inertia_p[13 * j + 1] * h[j * 6 + 5]
              M_H_product[k * 6 + 3] += inertia_p[13 * j + 9] * h[j * 6 + 5]
              M_H_product[k * 6 + 4] += inertia_p[13 * j + 8] * h[j * 6 + 5]
              M_H_product[k * 6 + 5] += inertia_p[13 * j + 6] * h[j * 6 + 5]


The matrix product :math:`H^TC^{-T}MC^{-1}H` is evaluated in :math:`PO(1)` and stored in :math:`PO(3)` using the :math:`\wp_{31}` (:code:`p_v_31`) permutation array. This operation is also carried out in level decreasing order using the Parent Child list:

.. code:: python

  for j in range(n - 1, -1, -1):
      if not is_rigid[j]:
          if has_mass[j]:
              m_1 = index_lookup[j]
              m_3 = P_v_31[m_1]
              ks_3 = int((m_3 * (m_3 + 1)) / 2)
              k_3 = ks_3 + m_3
              M_l_Boolean[k_3] = 1
              for l in range(3):
                  M_l[k_3] += M_H_product[m_1 * 6 + l] * h[j * 6 + l]
              if not is_translation[j]:
                  for l in range(3, 6):
                      M_l[k_3] += M_H_product[m_1 * 6 + l] * h[j * 6 + l]

              i = PC[j]
              while i > -1:
                  if not is_rigid[i]:
                      i_1 = index_lookup[i]
                      i_3 = P_v_31[i_1]
                      if m_3 > i_3:
                          k_3 = ks_3 + P_v_31[i_1]
                      else:
                          ks_alt = int((i_3 * (i_3 + 1)) / 2)
                          k_3 = ks_alt + P_v_31[m_1]
                      M_l_Boolean[k_3] = 1
                      for l in range(3):
                          M_l[k_3] += M_H_product[m_1 * 6 + l] * h[i * 6 + l]
                      if not is_translation[i]:
                          for l in range(3, 6):
                              M_l[k_3] += M_H_product[m_1 * 6 + l] * h[i * 6 + l]
                  i = PC[i]


Note that the terms of the generalized mass matrix and the constraint equations are stored in separate datastructures.


Evaluating Right Hand Side
^^^^^^^^^^^^^^^^^^^^^^^^^^

Similarly, the right hand side of the equations of motion can be evaluated in :math:`PO(1)` and stored in :math:`PO(3)`. The generalized forces, :math:`\underline{f}^{\ell}`, are found as:

.. math::
  :label: eq:generalized_forces

  \underline{f}^{\ell} = \underline{f} + H^T C^{-T} (\underline{\textbf{g}} - [ad]^*M \underline{\textbf{v}} - M \underline{\gamma})


The operation in Eq. :eq:`eq:generalized_forces` can be broken down into the following steps. First, carry out:

.. math::
  :label: step1

  \underline{\textbf{g}} \leftarrow \underline{\textbf{g}} - M\underline{\gamma}


and initialize the generalized forces, :math:`\underline{f}^{\ell}` (:code:`f_l`) to zero using the following code:

.. code:: python

    f_l[:] = 0.

    for j in range(n):
        g[j * 6 + 0] -= inertia[13 * j + 0] * a[j * 6 + 0]
        g[j * 6 + 4] -= inertia[13 * j + 3] * a[j * 6 + 0]
        g[j * 6 + 5] -= inertia[13 * j + 11] * a[j * 6 + 0]

        g[j * 6 + 1] -= inertia[13 * j + 0] * a[j * 6 + 1]
        g[j * 6 + 3] -= inertia[13 * j + 12] * a[j * 6 + 1]
        g[j * 6 + 5] -= inertia[13 * j + 1] * a[j * 6 + 1]

        g[j * 6 + 2] -= inertia[13 * j + 0] * a[j * 6 + 2]
        g[j * 6 + 3] -= inertia[13 * j + 2] * a[j * 6 + 2]
        g[j * 6 + 4] -= inertia[13 * j + 10] * a[j * 6 + 2]

        g[j * 6 + 1] -= inertia[13 * j + 12] * a[j * 6 + 3]
        g[j * 6 + 2] -= inertia[13 * j + 2] * a[j * 6 + 3]
        g[j * 6 + 3] -= inertia[13 * j + 4] * a[j * 6 + 3]
        g[j * 6 + 4] -= inertia[13 * j + 7] * a[j * 6 + 3]
        g[j * 6 + 5] -= inertia[13 * j + 9] * a[j * 6 + 3]

        g[j * 6 + 0] -= inertia[13 * j + 3] * a[j * 6 + 4]
        g[j * 6 + 2] -= inertia[13 * j + 10] * a[j * 6 + 4]
        g[j * 6 + 3] -= inertia[13 * j + 7] * a[j * 6 + 4]
        g[j * 6 + 4] -= inertia[13 * j + 5] * a[j * 6 + 4]
        g[j * 6 + 5] -= inertia[13 * j + 8] * a[j * 6 + 4]

        g[j * 6 + 0] -= inertia[13 * j + 11] * a[j * 6 + 5]
        g[j * 6 + 1] -= inertia[13 * j + 1] * a[j * 6 + 5]
        g[j * 6 + 3] -= inertia[13 * j + 9] * a[j * 6 + 5]
        g[j * 6 + 4] -= inertia[13 * j + 8] * a[j * 6 + 5]
        g[j * 6 + 5] -= inertia[13 * j + 6] * a[j * 6 + 5]


Next, update the generalized momentum:

.. math::
  :label: gen_momentum

  \underline{\textbf{p}} \leftarrow M\underline{\textbf{v}}


This operation is also carried out in PO(1).

.. code:: python

    p[:] = 0
    for j in range(n):
        p[j * 6 + 0] += inertia[13 * j + 0] * v[j * 6 + 0]
        p[j * 6 + 4] += inertia[13 * j + 3] * v[j * 6 + 0]
        p[j * 6 + 5] += inertia[13 * j + 11] * v[j * 6 + 0]

        p[j * 6 + 1] += inertia[13 * j + 0] * v[j * 6 + 1]
        p[j * 6 + 3] += inertia[13 * j + 12] * v[j * 6 + 1]
        p[j * 6 + 5] += inertia[13 * j + 1] * v[j * 6 + 1]

        p[j * 6 + 2] += inertia[13 * j + 0] * v[j * 6 + 2]
        p[j * 6 + 3] += inertia[13 * j + 2] * v[j * 6 + 2]
        p[j * 6 + 4] += inertia[13 * j + 10] * v[j * 6 + 2]

        p[j * 6 + 1] += inertia[13 * j + 12] * v[j * 6 + 3]
        p[j * 6 + 2] += inertia[13 * j + 2] * v[j * 6 + 3]
        p[j * 6 + 3] += inertia[13 * j + 4] * v[j * 6 + 3]
        p[j * 6 + 4] += inertia[13 * j + 7] * v[j * 6 + 3]
        p[j * 6 + 5] += inertia[13 * j + 9] * v[j * 6 + 3]

        p[j * 6 + 0] += inertia[13 * j + 3] * v[j * 6 + 4]
        p[j * 6 + 2] += inertia[13 * j + 10] * v[j * 6 + 4]
        p[j * 6 + 3] += inertia[13 * j + 7] * v[j * 6 + 4]
        p[j * 6 + 4] += inertia[13 * j + 5] * v[j * 6 + 4]
        p[j * 6 + 5] += inertia[13 * j + 8] * v[j * 6 + 4]

        p[j * 6 + 0] += inertia[13 * j + 11] * v[j * 6 + 5]
        p[j * 6 + 1] += inertia[13 * j + 1] * v[j * 6 + 5]
        p[j * 6 + 3] += inertia[13 * j + 9] * v[j * 6 + 5]
        p[j * 6 + 4] += inertia[13 * j + 8] * v[j * 6 + 5]
        p[j * 6 + 5] += inertia[13 * j + 6] * v[j * 6 + 5]


Next perform the screw cross product operation denoted by: :math:`[ad]^*\underline{\textbf{p}}` and subtract the results from :math:`\underline{\textbf{g}}` as:

.. math::
  :label: subtractcrossproduct

  \underline{\textbf{g}} \leftarrow \underline{\textbf{g}} - [ad]^*\underline{\textbf{p}}


using the following code. Again this operation can be performed in PO(1) and may be parallelized with minimum synchronization.

.. code:: python

    for j in range(n):
        g[j * 6 + 0:j * 6 + 3] -= \
            np.cross(v[j * 6 + 3:j * 6 + 6], p[j * 6 + 0: j * 6 + 3])
        g[j * 6 + 3:j * 6 + 6] -= \
            np.cross(v[j * 6 + 0:j * 6 + 3], p[j * 6 + 0: j * 6 + 3])
        g[j * 6 + 3:j * 6 + 6] -= \
            np.cross(v[j * 6 + 3:j * 6 + 6], p[j * 6 + 3: j * 6 + 6])


Then perform the operation:


.. math::
  :label: operation1

  \underline{\textbf{g}} \leftarrow C^{-T} \underline{\textbf{g}}


.. code:: python

    for j in range(n - 1, -1, -1):
        i = PC[j]
        if i > -1:
            for k in range(0, 6):
                g[6 * i + k] += g[6 * j + k]


The final operation required to update the right hand side is stored in :math:`PO(3)` using the :math:`\wp_{31}` (:code:`p_v_31`) permutation array.


.. math::
  :label: final_operation

  \underline{f}^{\ell} \leftarrow \underline{f} + H^T \underline{\textbf{g}}


The accompanying code to carry out Eq. :eq:`final_operation` is:


.. code:: python

    for j in range(n):
        if not is_rigid[j]:
            k = p_v_31[index_lookup[j]]
            f_l[k] = f[j]
            for l in range(3):
                f_l[k] += h[6 * j + l] * g[6 * j + l]
            if not is_translation[j]:
                for l in range(3, 6):
                    f_l[k] += h[6 * j + l] * g[6 * j + l]


Evaluating Constraints
*************************

The next step in setting up the equations of motion involves evaluating the constraints for each substructure. As mentioned above, each Jacobian matrix is initially evaluated in :math:`PO(1)`. Generalized coordinate partitioning yields :math:`PO(2)` and the equations of motion are solved in :math:`PO(3)`. In the following discussion, it is assumed that a first call to the LU decomposition algorithm with complete row and column pivoting has been performed, and the various global permutation arrays have been
generated according to section :ref:`evaluating_permutations`.

As described in Section :ref:`kinematic_substructuring`, after an initial call to generalized coordinate partitioning, subsequent evaluations of the Jacobian matrix are performed in :math:`PO(2)`. Preconditioning of the constraint equations is performed according to Section :ref:`generalized_coordinate_partitioning`, and the preconditioned results are stored in :math:`PO(3)`. As noted in section :ref:`generalized_coordinate_partitioning`, each preconditioned constraint equation contains a :math:`B^{di}` matrix with a row dimension equal to the number of dependent variables and a column dimension equal to the number of independent variables. To take advantage of sparsity, only the nonzero terms of each substructure's :math:`B^{di}` matrix are stored in a linear array in row major order. In order to evaluate the :math:`B^{di}` matrix's sparsity pattern in :math:`PO(3)`, it is first necessary to determine the sparsity pattern of each block subjacobian in :math:`PO(1)`, which is performed using the following code for each substructure. In the below code, :code:`J` is the local block subjacobian corresponding to the given substructure, :code:`J_B` is a boolean matrix with :code:`True` (or a nonzero value) indicating a nonzero value and :code:`False` (or a zero value) indicating a zero value at the corresponding position in the block subjacobian. :code:`row_dim` and :code:`col_dim` indicate the block subjacobian's row and column dimension, respectively. :code:`rank`, :code:`pr_inv` and :code:`pc_inv` are the rank and inverse linear row and column arrays generated by an initial call to LU decomposition with complete row and column pivoting. Additionally, the influence coefficient matrices that comprise the elements of the Jacobian matrix are evaluated and stored in the :code:`positive_loop_members` and :code:`negative_loop_members` data structures and an offset into the global list of joint variables is stored. Note that :code:`pr_inv` and :code:`pc_inv` are used to map :code:`positive_loop_members` and :code:`negative_loop_members` (which are stored in :math:`PO(1)`) to :math:`J` and :code:`Jb`, which are stored in :math:`PO(2)`.


.. code:: python

  for substructure in substructures:
      substructure["J_B"] = np.zeros(substructure["J"].shape)

      for i in range(len(substructure["positive_loop_members"])):
          for j in range(len(substructure["positive_loop_members"][i])):
              for k in range(6):
                  joint_ind = substructure["positive_loop_members"][i][j]
                  J_B[substructure["pr_inv"][i * 6 + k],
                       substructure["pc_inv"][index_lookup[joint_ind] - \
                          substructure["offset"]]] = 1

      for i in range(len(substructure["negative_loop_members"])):
          for j in range(len(substructure["negative_loop_members"][i])):
              for k in range(6):
                  joint_ind = substructure["negative_loop_members"][i][j]
                  J_Bm[substructure["pr_inv"][i * 6 + k],
                       substructure["pc_inv"][index_lookup[joint_ind] - \
                          substruture["offset"]]] = 1


Once the Jacobian is evaluated in :math:`PO(2)`, it is factored without pivoting and matrix fills are recorded in :code:`J_B`.


.. code:: python

  for substructure in substructures:
      for i in range(substructure["rank"]):
          for j in range(i + 1, substructure["rank"]):
              if substructure["J_B"][j, i]:
                  for k in range(i + 1, substructure["col_dim"]):
                      if substructure["J_B"][i, k]:
                          substructure["J_B"][j, k] = 1  # record a fill

      for i in range(rank - 1, -1, -1):
          for j in range(rank - 1, i, -1):
              if substructure["J_B"][i, j]:
                  for k in range(substructure["rank"], substructure["col_dim"]):
                      if substructure["J_B"][j, k]:
                          substructure["J_B"][i, k] = 1


Once the sparsity pattern of a factored :code:`J_B` has been found for each substructure, the sparsity pattern of :math:`B^{di}` can be found as follows. The :math:`B^{di}` for each Jacobian submatrix is stored in a linear array :code:`b3`. Additionally, memory for the linear permutation arrays :code:`pb_23` and :code:`pb_32` are allocated.


.. code:: python

  b_offset = 0
  for substructure in substructures:
      substructure["b_offset"] = b_offset
      nnzb = 0
      for i in range(substructure["rank"]):
          for j in range(substructure["rank"], substructure["shape"][1]):
              if substructure["J_B"][i][j]:
                  nnzb += 1

      # record number of nonzero entries in B matrix
      substructure["nnzb"] = nnzb
      b_offset += (substructure["shape"][1] - substructure["rank"]) * \
          substructure["rank"]

      # record total number of entries in full B matrix
      substructure["nb"] = b_offset - substructure["b_offset"]
      substructure["ij_mapping"] = np.zeros(substructure["J"].shape)

  # linear array to store entries of b and permutations from linear b2 to b3
  # equal in length to total number of entries in all B submatrices
  model["b3"] = np.zeros(b_offset)
  model["pb_23"] = np.zeros(b_offset)
  model["pb_32"] = np.zeros(b_offset)


Additionally, memory is allocated for the arrays :code:`rb2`, :code:`rb3_temp`, :code:`rb3`, :code:`cb2`, :code:`cb3_temp` and :code:`cb3`. After the analysis below is complete, the entries of :math:`B^{di}` are stored in :code:`b3` in :math:`PO(3)`. Each row of :code:`rb3` corresponds to a range of values from the array :code:`b3` for a given row. The corresponding value in :code:`cb3` yields the column index. :code:`cb3_temp` and :code:`rb3_temp` are temporary


.. code:: python

  rb2 = np.zeros([M_l_dim, 2])
  rb3_temp = np.zeros([M_l_dim, 2])
  rb3 = np.zeros([M_l_dim, 2])
  cb2 = np.zeros(b_offset)
  cb3_temp = np.zeros(b_offset)
  cb3 = np.zeros(b_offset)

  for substructure in substructures:
      var_offset = substructure["offset"]

      k = substructure["b_offset"]

      for i in range(var_offset, var_offset + substructure["rank"]):
          rb2[i][0] = k
          for j in range(substructure["rank"], substructure["shape"][1]):
              if substructure["J_B"][i - var_offset][j]:
                  model["cb2"][k] = j - substructure["rank"]
                  model["cb3_temp"][k] = p_v_32[j + substructure["offset"]]
                  k += 1
          rb2[i][1] = k
          rb3_temp[p_v_32[i]][0] = rb2[i][0]
          rb3_temp[p_v_32[i]][1] = rb2[i][1]

      k = substructure["b_offset"]
      b_offset = substructure["b_offset"]
      independent_count = substructure["shape"][1] - substructure["rank"]
      indices = [i for i in range(substructure["nb"])]

      for i in range(substructure["offset"] + independent_count,
                     substructure["offset"] + substructure["shape"][1]):
          rb3[i][0] = b_offset
          b_offset += rb3_temp[i][1] - rb3_temp[i][0]
          rb3[i][1] = b_offset

          for j in range(rb3_temp[i][0], rb3_temp[i][1]):
              min_index = j  # in global
              min_val = cb3_temp[j]  # j in global
              for l in range(j + 1, rb3_temp[i][1]):
                  if cb3_temp[l] < min_val:
                      min_index = l
                      min_val = cb3_temp[l]
              if min_index is not j:
                  cb3_temp[j], cb3_temp[min_index] = \
                      cb3_temp[min_index], cb3_temp[j]
                  indices[j - substructure["b_offset"]], \
                      indices[min_index - substructure["b_offset"]] = \
                      indices[min_index - substructure["b_offset"]], \
                      indices[j - substructure["b_offset"]]

              pb_23[k] = \
                  indices[j - substructure["b_offset"]] + \
                  substructure["b_offset"]
              cb3[k] = min_val
              k += 1

      for i in range(
              substructure["b_offset"],
              substructure["b_offset"] + substructure["nnzb"]):
          pb_32[pb_23[i]] = i


Finally, once the permutations are set, :code:`b3` can be populated as:


.. code:: python

  for substructure in substructures:
      k = substructure["b_offset"]
      for i in range(substructure["rank"]):
          for j in range(substructure["rank"], substructure["shape"][1]):
              if substructure["J_B"][i][j]:
                  b3[pb_32[k]] = substructure["J"][i][j]
                  k += 1
