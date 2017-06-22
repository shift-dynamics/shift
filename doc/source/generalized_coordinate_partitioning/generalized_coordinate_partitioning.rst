.. title:: Generalized Coordinate Partitioning

.. _generalized_coordinate_partitioning:

Generalized Coordinate Partitioning
===================================

Dynamic Equations
-----------------

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

    \underline{\mathbf{g}}^* = M\underline{\dot{\mathbf{v}}}


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

    C\underline{\dot{\mathbf{v}}}_{abs} = H\ddot{\underline{q}} + \dot{H}\dot{\underline{q}} = H\ddot{\underline{q}} + \underline{\gamma}


where :math:`\underline{\gamma}=\dot{H}\dot{\underline{q}}`. Eq. :eq:`eq:mapping_der` can be expressed in factored matrix form as:

.. math::
    :label: eq:mapping_der_matrix

    \begin{bmatrix}C & -H\end{bmatrix}\begin{Bmatrix} \underline{\dot{\mathbf{v}}}_{abs} \\ \underline{\ddot{q}}\end{Bmatrix} = \underline{\gamma}


Additionally the equation:

.. math::
    :label: eq:map_force_to_joint_space

    H^T\underline{\mathbf{k}}^*=\underline{f}


maps constraint reaction wrenches, :math:`\underline{\mathbf{k}}^*` onto joint space. The column matrix :math:`\underline{f}` represents working generalized forces (such as from springs, dampers, actuators, friction, etc.) applied within a joint. If no such effects exist, :math:`\underline{f}` will be zero.

Equation :eq:`eq:mapping_der_matrix` and :eq:`eq:map_force_to_joint_space` can be appended to Eq. :eq:`eq:newton_euler2` as:

.. math::
    :label: eq:augmented_newton_euler

    \begin{bmatrix}M & 0 & C^T\\ 0 & 0 & -H^T\\ C & -H & 0\end{bmatrix}
    \begin{Bmatrix}\underline{\dot{\mathbf{v}}}_{abs}\\ \underline{\ddot{q}}\\ \underline{\mathbf{k}}^* \end{Bmatrix} =
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


The symmetric, positive semidefinite :math:`M^{\ell}` corresponds to the generalized mass matrix. It contains the same quantities that would have been obtained by expanding a Lagrangian, hence the superscript :math:`\ell` :cite:`pars1979`. :math:`\underline{f}^{\ell}` represents the generalized forces, and additionally contains quantities that would have been obtained from expanding a Lagrangian. While equation :eq:`eq:generalized_eqns_motion_no_constraints` is suitable to describe the dynamic behavior of mechanisms with star configurations, for mechanisms with multiple loops it is necessary to append constraints to the system of equations. Setting up constraint equations for a multibody system requires identifying a *spanning* tree, i.e. a root-oriented directed graph that provides a unique path between each body and the fixed inertial reference frame. The set of graph edges comprising the spanning tree are known as the arcs of the directed graph. The remaining edges are classified as chords. Once the arcs and chords of the system have been identified, Eq. :eq:`eq:mapping` can be block partitioned as:

.. math::
    :label: eq:partitioned_mapping

    \begin{bmatrix}C_{arc}\\C_{chord}\end{bmatrix}\underline{\dot{q}}_{abs} =
    \begin{bmatrix}H_{arc} & 0\\ 0 & H_{chord} \end{bmatrix}
    \begin{Bmatrix}\underline{\dot{q}}_{arc}\\ \underline{\dot{q}}_{chord} \end{Bmatrix}

which yields:

.. math::
    :label: eq:partmapping_yield1

    \underline{\mathbf{v}}_{abs} = C^{-1}_{arc}H_{arc}\underline{q}_{arc}


and

.. math::
    :label: eq:partmapping_yield2

    C_{chord}\underline{\mathbf{v}}_{abs} = H_{chord}\underline{\dot{q}}_{chord}


Substituting Eq. :eq:`eq:partmapping_yield1` into Eq. :eq:`eq:partmapping_yield2` and rearranging yields:

.. math::
    :label: eq:constraint_eq1

    -C_{chord}C^{-1}_{arc}H_{arc}\underline{\dot{q}}_{arc} + H_{chord}\underline{\dot{q}}_{chord} = 0


Equation :eq:`eq:constraint_eq1` represents the chord-loop velocity constraints for a mechanism, and from it the constraint Jacobian matrix is identifed as:

.. math::
    :label: eq:jacobian

    J = \begin{bmatrix}-C_{chord}C^{-1}_{arc}H_{arc} & H_{chord}\end{bmatrix}


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


Need for Generalized Coordinate Partitioning
--------------------------------------------

Despite the advantages outlined above, there are some challenges associated with the solution process. If the coefficient matrix in Eq. :eq:`eq:lagrange_eqn` is nonsingular, a unique solution can be obtained using basic linear algebra methods :cite:`nikravesh1984`. However, in practice, the Jacobian matrix almost never has full row rank, so the coefficient matrix in Eq. :eq:`eq:lagrange_eqn` is generally singular. In addition, bifurcations, end--of--stroke conditions, or instantaneous alignment of joints may occur as a system moves to different postures. Only the latter of these changes Jacobian matrix rank and overall mobility, but the solution processes remains complicated even for relatively simple mechanisms with few degrees of freedom. Robust solution methods must reliably handle these situations with minimal demands on the analyst, who may lack the training to intervene.

The various approaches to deal with rank--deficient Jacobian matrices can be classified into two groups. In the first group, all variables and equations are considered to contribute equally to the solution, and the entire set of equations is solved using a best root--mean--squared (RMS) approximation approach, such as the Moore-Penrose pseudo-inverse method described in :cite:`uicker1964`. Unfortunately, the RMS approach not only requires many additional floating--point operations, but it also may fail to converge in conditions of poor mechanical advantage (see the discussion in Section 6.5 of :cite:`uicker2013`).

In the second group, a *cutset*, or minimal set of independent variables (often referred to as generalized coordinates); a minimal set of constraint equations (often referred to as primary constraints); and a minimal set of equations of motion (often referred to as generalized equations of motion) are identified. Once the independent variables are computed from the equations of motion, the primary constraint equations are used to evaluate the dependent variables.

This second, minimal approach has demonstrated an order of magnitude speedup over the RMS method :cite:`wehage1981` and has therefore attracted attention in recent decades---although splitting the complete set of equations into primary and secondary sets was first proposed as early as the beginning of the 20th century: see, for example :cite:`maggi1896,maggi1901,woronetz1910`.

While the second approach is appealing from an efficiency standpoint, ensuring robust solutions requires care in selecting the primary equations and independent variables. Particularly, choosing variables as independent that would result in large changes in system configuration for small incremental inputs must be avoided, because this may cause numerical instability and inaccuracy problems. Therefore, the *optimal* independent generalized coordinates are those that induce the smallest changes in system configuration. It follows that this set of generalized coordinates will have roughly the greatest mechanical advantage.

Historically with the second approach, specialists choose the cutset of primary constraints and generalized coordinates based on their experience with similar classes of problems. Manual selection, however, has its limitations. For complex systems, the manual approach is prone to error and imposes a burden on the analyst. The optimal set of generalized coordinates often depends on a system's posture and may require periodic updating during an analysis. In addition, a mechanism's changing posture may periodically require a different set of primary constraint equations, or even a different number of primary equations when the Jacobian matrix rank changes.

In recent decades numerical methods have been devised to automatically identify optimal sets of independent generalized coordinates and primary constraint equations. Generalized Coordinate Partitioning (GCP), introduced by Uicker and Sheth in the context of kinematics :cite:`sheth1972digital,sheth1972imp` and extended by Wehage and Haug in the context of dynamics :cite:`wehage1982`, is a row and column permutation and partitioning method applied to the Jacobian matrix. The GCP method is a byproduct of Gaussian Elimination with Complete Pivoting (GECP). GECP factors a matrix into row--echelon form, reliably revealing its rank :cite:`turing1948`. At each successive factorization step, GECP permutes the largest remaining pivotal element to the upper--left position in the residual matrix, thereby selecting the next dependent variable with possibly the smallest mechanical advantage. After factorization is complete, rows of the largest nonsingular matrix identify the primary constraint equations, and columns identify the dependent variables. Refer to :ref:`appendices` for an illustrative, worked example of GECP/LU decomposition.

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
