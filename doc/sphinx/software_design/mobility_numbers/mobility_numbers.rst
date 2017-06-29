.. title:: Mobility Numbers

Introducing Mobility Numbers
============================


Computing the TWS from the Jacobian Matrix
------------------------------------------

The Transmission Wrench Screw (TWS), plays an important role in most measures of transmission performance. Most methods in the literature compute the TWS using Plucker line geometry or screw coordinates. Others have attempted to compute a measure of transmission quality from the Jacobian matrix. In this section, it will be shown how the TWS method can be computed directly from a mechanism's Jacobian matrix.

An example of formulating the TWS for a representative 7H linkage, shown in the figure below is provided.

.. _7h_linkage

.. figure:: 7h_spatial_linkage.png
    :align: center
    :width: 400px


.. The joint screws and TWS of a 7H spatial linkage :cite:`tsai1994transmissivity`



The definition of the TWS is given in :cite:`yuan1970kinematic,sutherland1973transmission,tsai1994transmissivity` and is summarized as follows. For the TWS to be uniquely determined in a three-dimensional, single-loop spatial mechanism, the system of wrenches must be a *5-system*. As discussed in Section \ref{sec:constraint_forces}, each joint effectively constrains relative velocities in the five directions orthogonal to the joint motion; relative motion between bodies is permitted only in the directions aligned with the joint (i.e. only one direction for primitive joints). Forces are dual to velocities; their behavior is similar but opposite that of velocities. If no internal working forces are applied within the joints, then no forces may be transmitted in the direction of the joint's motion. Therefore, the only forces that are imparted from one joint to the next are those forces in directions orthogonal (i.e. *reciprocal*) to the orientation of the joint, as shown in Eq. \ref{eq:joint_reaction_forces}. A system of five non-redundant joints has one unique velocity screw in which forces may be transmitted.

Let the input and output influence coefficient matrices be :math:`\underline{s}_i` and :math:`\underline{s}_o` respectively. Let the influence coefficient matrices associated with joints 1-5, be :math:`\underline{s}_1 - \underline{s}_5`. The jacobian for the entire single loop mechanism in Fig \ref{fig:7h_linkage} becomes

.. math::

    J = \begin{bmatrix}
    \underline{s}_i & \underline{s}_1 & \underline{s}_2 & \underline{s}_3 & \underline{s}_4 & \underline{s}_5 & \underline{s}_o
    \end{bmatrix}


The transmission performance is a measure of how well-aligned the output velocity screw, :math:`\underline{s}_o` is aligned with the mechanism. Therefore, let us look at the jacobian, :math:`J_{1-5}`, for each of the floating links separately

.. math::

    J_{1-5} = \begin{bmatrix}
    \underline{s}_1 & \underline{s}_2 & \underline{s}_3 & \underline{s}_4 & \underline{s}_5
    \end{bmatrix}
    \label{eq:J15}


In the present notation, the Transmission Wrench Screw (TWS) is actually a coscrew -- in older literature no distinction was made between the two spaces. Let the TWS *coscrew* be denoted as :math:`\underline{f}^*_{TWS}`. :math:`\underline{f}^*_{TWS}` is a linear operator on the unique velocity screw not spanned by joints 1--5. Only the coscrew, :math:`\underline{f}^*_{TWS}`, can do useful work on the system, applied through the velocity screw :math:`\underline{s}_{TWS}`. Therefore, :math:`\underline{f}^*_{TWS}` applied through every one of the influence coefficient matrices in :math:`J_{1-5}` should generate zero power. Another interpretation of this statement is that the transpose of coscrew :math:`\underline{f}^*_{TWS}` lies in the left null space of :math:`J_{1-5}`.

The left null space of :math:`J_{1-5}` can be computed using L(D)U decomposition. Recall that the L(D)U decomposition algorithm permutes the jacobian into

.. math::
    :label: eq:J_15

    J_{1-5} =
    P_R
    \begin{bmatrix}
    J^{pd} & J^{pi}\\
    J^{sd} & J^{si}
    \end{bmatrix}
    P_C^T


If the 5 joints span a 5-system, the joints are not redundant and :math:`J^{pd}` is full column rank. Equation \ref{eq:J_15} then reduces to:

.. math::

    J_{1-5} =
    P_R
    \begin{bmatrix}
    J^{pd}\\
    J^{sd}
    \end{bmatrix}
    P_C^T


where

.. math::

    J^{pd} = LDU


:math:`P_R` is an orthonormal row permutation matrix and :math:`P_C` is an orthonormal column permutation matrix.


A convenient representation of the column--space of :math:`J` is

.. math::

    \begin{bmatrix}
    J^{pd}\\J^{sd}
    \end{bmatrix}U^{-1}D^{-1}L^{-1}=
    \begin{bmatrix}
    I\\
    J^{sd}U^{-1}D^{-1}L^{-1}
    \end{bmatrix}=
    \begin{bmatrix}
    I\\
    -N
    \end{bmatrix}


which allows trivially computing the left null--space, :math:`X`, of :math:`J` as

.. math::

    X\begin{bmatrix}
    I\\-N
    \end{bmatrix}=0,
    X=\begin{bmatrix}N & I\end{bmatrix}


The matrix :math:`c\begin{bmatrix}N & I\end{bmatrix}` is the cokernel of :math:`J`. Therefore any multiple of :math:`c` will satisfy the left nullity of :math:`J`. As :math:`J_{1-5}` is a 5-system, the matrix :math:`X` is a row vector. The TWS is expressed as a normalized quantity, however care must be taken in normalizing screw and coscrew quantities as the :math:`[Ad]` and :math:`[Ad]^*` transformation matrices are not orthonormal. :math:`\underline{f}^*` is normalized by setting :math:`c` to :math:`\frac{1}{\left|\underline{f}\right|}` where :math:`\left|\underline{f}\right|` are the linear force components of the coscrew :math:`\underline{f}^*` (excluding torques).  The column permutation matrix, :math:`P_C`, is used to permute the columns back to original unpermuted order.

.. math::

    \underline{f}^{*T}_{TWS}=\frac{XP_C^T}{\left|\underline{f}_{TWS}\right|}


Ball's virtual coefficient, :math:`VC`, which forms the basis of most measures of transmission performance and is given as

.. math::
    :label: eq:VC

    VC=\frac{1}{2}\underline{f}^*_{TWS}\cdot\underline{s}_o


Note that, by definition, :math:`s_o` is a normalized quantity.

This is the original formulation for transmission performance proposed by Yuan in :cite:`yuan1970kinematic`. The primary difference between this expression and other transmission indices lies in how Equation :ref:`eq:VC` is normalized.


Mobility Numbers
----------------

In this section, a new measure of transmission performance is proposed. As opposed to a single quantity, this method gives a normalized quantity ranging from 0 to 1 for each joint of a mechanism.

Recall that generalized coordinate partitioning (LU or LDU factorization with complete row and column pivoting) permutes and partitions the Jacobian matrix :math:`J` in to the form:

.. math::
    \begin{bmatrix}
    J^{pd} & J^{pi}\\
    J^{sd} & J^{si}
    \end{bmatrix}


and factors the largest nonsingular submatrix of :math:`J`, :math:`J^{pd}`, in to :math:`LU` or :math:`LDU`.

.. math::

    J^{pd} = LDU


A convenient representation of the row space of :math:`J` is found by:

.. math::

    U^{-1}D^{-1}L^{-1}
    \begin{bmatrix}
    J^{pd} & J^{pi}
    \end{bmatrix}=
    \begin{bmatrix}
    I & -B^{di}
    \end{bmatrix}


The right null space of J is the matrix X, such that

.. math::

    \begin{bmatrix}
    I & -B^{di}
    \end{bmatrix}X = 0


In this form, the solution for X is trivial:

.. math::

    X = \begin{bmatrix}B^{di}\\I\end{bmatrix}


The "instantaneous mobility", :math:`M`, of the mechanism is found by:

.. math::
    :label: eq:rank_x

    M = rank(X)


whereas the "global mobility" of a mechanism is typically computed by the Chebyshev-Gruebler-Kutzback criteria.

The right null-space matrix, :math:`X=\begin{bmatrix}B^{di} \\ I\end{bmatrix}`, can be factored in to:

.. math::

    X = VR = \begin{bmatrix}
    V^{di}\\
    R^{-1}
    \end{bmatrix}R=
    \begin{bmatrix}
    V^{di}\\
    V^{ii}
    \end{bmatrix}R

using Gram-Schmidt orthonormalization, (i.e. QR decomposition) :cite:`golub1996matrix` where :math:`R` is a transformation matrix and :math:`V` is an orthonormal basis. Note that the :math:`R` matrix is effectively taking a linear combination of directions along the mechanism's manifold. The resulting matrix, :math:`V`, is analogous to a direction-cosine matrix on the mechanism's manifold. Each column corresponds to a direction along the manifold and each row corresponds to unique joint variable. Furthermore, :math:`V` is partitioned in to :math:`V_di`, where each row corresponds to a unique *dependent* variable, and :math:`V^{ii}`, where each row corresponds to a unique *independent* variable.

The sum of squares of each row (i.e. the L2 norm squared) yields a "mobility number", i.e., a normalized measure of a joints ability to induce a change in the mechanism's instantaneous configuration. For example, the mobility number of joint i is found by

.. math::

    m_i=\sum_{n=1}^nV_{in}^2


where *j* is the number of columns of the matrix :math:`V`. The sum of all mobility numbers is equal to the instantaneous mobility

.. math::

    M=\sum_{n=1}^km_i


where :math:`k` is the total number of joints.

A mobility number of zero indicates that the joint is orthogonal to the manifold. A mobility number of one indicates that the joint is perfectly aligned with the manifold. If more than one joint align with the mechanism manifold, the total mobility will be shared between the joints.

Connection to Singular Value Decomposition
------------------------------------------

Recall that SVD can also be used find the largest nonsingular submatrix of :math:`J` :cite:`yang2008simple`:

.. math::

    J = U\Sigma V^T =
    \begin{bmatrix}
    U^P & U^S
    \end{bmatrix}
    \begin{bmatrix}
    \Sigma^{pd} & 0\\
    0 & 0
    \end{bmatrix}
    \begin{bmatrix}
    V^{dT}\\
    V^{iT}
    \end{bmatrix}R


where :math:`\Sigma` is a square diagonal matrix containing monotonically decreasing singular values and U and V are orthonormal transformation matrices.

Therefore

.. math::

    U^T J V = \begin{bmatrix}U^{pT}\\U^{sT}\end{bmatrix}J\begin{bmatrix}V^d & V^i\end{bmatrix}=\begin{bmatrix}\Sigma^{pd} & 0 \\ 0 & 0\end{bmatrix}

and

.. math::

    U^{sT}JV^i=0, \therefore  JV^i=0


L(D)U decomposition method has the following advantages over SVD for this application.

    * Computing L(D)U decomposition (:math:`\mathcal{O}(r(6mn - 3(m + n)r + 2r^2 )/3)`) is numerically less expensive than SVD (:math:`\mathcal{O}(4m^2n+22n^3)`).
    * In L(D)U decomposition, the variables are simply permuted, whereas in SVD, the factorization involves a linear combination of variables. Therefore, after L(D)U, each row of the permuted matrix maps to a unique joint variable.


Computing Joint Reaction Forces using Manifold Projection Matrix
----------------------------------------------------------------

For a static force balance, start with Lagrange's equations in joint coordinate space, canceling dynamic terms.

.. math::

    \require{cancel}
    \begin{bmatrix}\cancel{M^{\ell}} & J^T \\ J & 0\end{bmatrix}\begin{Bmatrix}\cancel{\underline{\ddot{ \phi }}}\\-\underline{\lambda}\end{Bmatrix}
    =\begin{Bmatrix}\underline{f}^{\ell} \\ \cancel{-\dot{J}\underline{\dot{\phi}}+\underline{\dot{e}}_t}\end{Bmatrix}


leaving the first block row:

.. math::
    :label: eq:first_block_row

    J^T\underline{\lambda} = \underline{f}^{\ell}= \underline{f} + H^TC^{-T}(\underline{g}-\cancel{MC^{-1}(C\underline{\ddot{q}}-H\underline{\ddot{\phi}})})

:math:`\underline{f}` represents joint "working forces": :math:`\underline{f}^*=H^*\underline{k}`, where :math:`H^*` is a block diagonal matrix of each joint's :math:`h^*` projector.

When no unique solution for :math:`\underline{f}`, exists, some authors find the set of Lagrange multipliers, :math:`\underline{\lambda}`, that minimizes the objective function :math:`\Phi`:

.. math::
    :label: eq:Phi

    \Phi=f^Tf=(J^T\underline{\lambda}-H^TC^-T\underline{g})^T(J^T\underline{\lambda}-H^TC^{-T}\underline{g})


An alternative approach makes use of the right null space matrix, :math:`V` for this purpose. Rearranging Eq. :eq:`eq:first_block_row` yields

.. math::
    :label: eq:15

    \underline{f}=J^T\underline{\lambda}-H^TC^{-T}\underline{g}


Recall that:

.. math::

    JV=0, \therefore V^TJ^T=0


Allowing us eliminate the Lagrange multipliers from Eq. :eq:`eq:15`.

.. math::

    V^T\underline{f} = \cancel{V^TJ^T\underline{\lambda}}-V^TH^TC^{-T}\underline{g}


:math:`V^T` effectively projects the working constraint forces, :math:`\underline{f}`, on to the mechanism's manifold. Left-multiplying by V again projects the forces back in to joint-space.

.. math::

    \underline{\bar{f}}=VV^T\underline{f}=-VV^TH^TC^{-T}\underline{g}



Mechanical Advantage from Mobility Numbers
------------------------------------------

For a mechanism with a one--dimensional manifold, the :math:`VV^T` projection matrix allows one to compute the mechanical advantage of any joint, *k*, with respect to any other joint, *j*.

Recall that the matrix :math:`V` effectively projects velocities expressed on the manifold back to joint space, and therefore

.. math::

    \dot{\underline{\phi}}=V\dot{p}


where :math:`\dot{p}` is the scalar velocity tangent to the manifold. Note that :math:`\dot{p}` is independent and therefore can be set to any value. Setting :math:`\dot{p}` to :math:`V_j^T` allows the computation of velocity ratios, :math:`\frac{\underline{\dot{\phi}}_k}{\underline{\dot{\phi}}_j}`, in terms of known quantities.

.. math::

    \dot{\phi}_j=V_j{V_j}^T=[VV^T]_{jj}=m_{jj}

where :math:`m_j` is the mobility number of the *jth* joint variable which appears on the diagonal of the projection matrix :math:`VV^T` and

.. math::


    \dot{\phi}_k=V_k{V_j}^T=[VV^T]_{kj}=m_{kj}

:math:`m_{jk}` is found from the off-diagonal element :math:`VV^T` corresponding to the row index of joint *k* and column index of joint *j*.

The ratio of :math:`\dot{\phi}_j` over :math:`\dot{\phi}_k` gives the mechanical advantage of joint *j* with respect to joint *k*.

.. math::
    :label: eq:mechanical_advantage

    MA = \frac{\dot{\phi}_j}{\dot{\phi}_k}=\frac{m_{jj}}{m_{kj}}


Note that *this technique does not generalize to mechanisms with manifolds having two or more dimensions*. In a mechanism with multiple redundant degrees of freedom and multiple possible configurations that it can be moved to, the velocity input from one joint may be distributed across multiple joints, and therefore, the concept of mechanical advantage becomes less clear.

Nonetheless, note that there is a particular direction along an n--dimensional manifold where Eq. :ref:`eq:mechanical_advantage` still holds. The mechanical advantage of joint *j* with respect to any other variable can only be computed using Eq. :ref:`eq:mechanical_advantage` when :math:`\dot{\underline{p}}=V_j^T`. If the joint rates are known, then it is possible to compute a direction cosine matrix that effectively transforms the present direction along the manifold (as specified by the joint rates) to a direction along the manifold such that :ref:`eq:mechanical_advantage` holds. This observation is not particularly helpful, however; if joint rates were already known, then velocity ratios could be computed using those joint rates directly without having to resort to performing an analysis on the manifold.

The relationship between diagonal and off-diagonal elements of :math:`VV^T` is directly related to mechanical advantage for mechanisms with 1-dimensional manifolds. However for mechanisms with higher-dimensional manifolds, the relationship is less clear. It is believed that the off-diagonal terms of :math:`VV^T` in these cases may be related to the power-path of a mechanism and requires further investigation.

It is typically of interest to calculate the MA for a given joint with respect to a specified output. For example, in the case of multiple redundantly actuated robots cooperating to carry a workpiece, the velocity of the workpiece may be specified. In this example, the workpiece may be constrained by adding an influence coefficient matrix corresponding to the target trajectory of the workpiece to the total Jacobian matrix. Due to the redundant actuation in the robots, the robots may assume multiple configurations while maintaining the specified trajectory. However, there is one unique direction along the manifold, i.e. the reconfiguration of the robots, that maximizes the mechanical advantage of each joint with respect to the output load. Finding a closed--form solution for a direction along the manifold that maximizes MA is another area of future work.

Note, however, that maximizing the MA of each of the joints with respect to an output trajectory as an objection function for path planning has some potential problems. There may arise a singular configuration where the output velocity goes to zero, while one or more of the inputs may have freedom to move (indicating an infinite MA). There is also the issue of dealing with mixed coordinates. For example the MA of a rotation over a translation is less meaningful.

Mobility Numbers for Path Planning
----------------------------------

In numerical path planning of robotic manipulators, it is common to introduce an artificial rigid connection between the end-effector frame and the fixed inertial reference frame, which effectively introduces one independent kinematic loop. For example, consider the 7 DOF robotic manipulator as shown below. Because of their increased dexterity and ability to avoid bifurcations and singular configurations, 7 and 8 DOF robotic manipulators are finding increased applications in industrial applications.

.. figure:: robot_arm.png
    :align: center

    A 7DOF Robot Arm


With one independent loop, the Jacobian matrix for the robot takes the form:

.. math::
    :label: eq:jacobian_7dof_robot

    J = \begin{bmatrix}
        \underline{s}_A & \underline{s}_B & \underline{s}_C & \underline{s}_D & \underline{s}_E & \underline{s}_F & \underline{s}_G
        \end{bmatrix}


One may then solve for an incremental change in joint angles by solving the equation:

.. math::
    :label: eq:incremental_change

    \hat{D} = Jd\phi


where :math:`\hat{D}` is an incremental change in configuration and :math:`d\phi` is an incremental change in joint angles. Refer to :cite:`uicker2013`, Chapter 6, for more details. Note that because the Jacobian is not full column rank, there is not a unique solution to the change in joint angles, :math:`d\phi` to achieve the desired incremental change in end-effector pose.

After the mechanism topology is traversed to update the position and orientation of the various frames, the Jacobian matrix for the robot in the configuration shown is given as:

.. math::
    :label: eq:robot_jacobian_numbers1

    J = \begin{bmatrix}
    0.0000 & -0.0134 & -0.1356 & -0.0256 & 0.1756 & -0.0123 & -0.0936\\
    0.0000 & 0.0200 & -0.0000 & 0.1821 & 0.0073 & -0.3956&  -0.0074 \\
    0.0000 & -0.1343 & 0.0136 & 0.1766 & -0.0106 & -0.1770&  0.0040  \\
    0.0000 & 0.9950 & -0.0978 & -0.9791 & 0.0604 & 0.9981&  -0.0584 \\
    1.0000 & 0.0000 & 0.1986 & -0.1947 & -0.0038 & -0.0040&  0.2024  \\
    0.0000 & -0.0998 & -0.9751 & 0.0585 & 0.9981 & -0.0604&  -0.9775 \\
    \end{bmatrix}


With the artificial constraint in place at the end-effector, this Jacobian matrix has a maximum possible rank of 6 and an instantaneous mobility of 1.  The implication of this is that the robot arm has one extra degree of motion available to it, the robot arm can move a linear combination of joints and maintain the pose set by the artificial constraint. Computing mobility numbers for the robot with the artificial constraint imposed is not particularly useful, as the numbers indicate how well a joint is aligned with a direction orthogonal to a possible end-effector motion.

To account for a desired end-effector motion when computing mechanism mobility, it is necessary to introduce an artificial 1DOF constraint aligned with the desired end-effector trajectory. For this discussion, let us assume that the motion path is parameterized in terms of orientation, position and time. Various interpolation schemes may be used for the purpose, such as spherical interpolation of quaternions :cite:`shoemake1985animating` (SLERP) or smooth invariant interpolation of rotations :cite:`park1997smooth`.

Let the end-effector position be determined by the :math:`SE(3)` matrix :math:`D_{10}`, which indicates an active transformation from the inertial reference frame, 0, to the end-effector frame at time :math:`t_1`. An interpolation scheme will yield an incremental change in end-effector orientation, position and time, described by a second :math:`SE(3)` matrix, :math:`D_{02}`, and time :math:`t_2`. An incremental change in configuration, in the end effector frame, :math:`D_{21}` is found as:

.. math::
    :label: eq:end_effector_frame

    D_{21} = D_{20}D_{10}^{-1}


The rotation component of the matrix :math:`D_{21}` may be parameterized in terms of an axis and an angle of rotation, :math:`\theta`.


To convert the incremental rotation to a velocity *twist*, divide the rotation and translation terms by change in time, :math:`\Delta time = time_2 - time_1`. The quantity may be expressed as a unit twist by dividing through by the magnitude of the translation.

.. math::
    :label: eq:path_screw

    \underline{s}_{path} = \begin{Bmatrix}
    \frac{\Delta t_x}{\Delta time}\\
    \frac{\Delta t_y}{\Delta time}\\
    \frac{\Delta t_z}{\Delta time}\\
    \frac{\Delta r_x}{\Delta time}\\
    \frac{\Delta r_y}{\Delta time}\\
    \frac{\Delta r_z}{\Delta time}\\
    \end{Bmatrix}\frac{1}{{\lVert t\rVert}}


To find the influence coefficient matrix associated with the unit screw in Eq. :eq:`eq:path_screw`, it is necessary to transform this unit screw to the fixed inertial frame using the :math:`[Ad]` operator, as described in :ref:`geometric_preliminaries`.  For the present example, let us assume that :math:`s_{path}` in the end-effector frame is given as:

.. math::
    :label: eq:unit_screw_ef_frame

    \underline{s}_{path}^1 = \begin{Bmatrix}
    0.1958 \\
    0.9789 \\
    0.0587 \\
    0.5873 \\
    0.0196 \\
    0.3916
    \end{Bmatrix}

Where the superscript 1 indicates that the screw is expressed in the end-effector frame. After applying the :math:`[Ad]` transformation, the influence coefficient matrix is found as:

.. math::
    :label: eq:path_influence_coeff

    \underline{s}_{path}^0 = \begin{Bmatrix}
    0.2853\\
    0.6905\\
    -0.0611\\
    0.2548\\
    -0.6369\\
    -0.1671
    \end{Bmatrix}


Here, the superscript 0 indicates that the quantity is expressed in the inertial reference frame. Using :math:`s_{path}^0`, it is possible to form the augmented Jacobian matrix:

.. math::
    :label: eq:augmented_jacobian

    J = \left[\begin{array}{ccccccc|c}
    \underline{s}_A & \underline{s}_B & \underline{s}_C & \underline{s}_D & \underline{s}_E & \underline{s}_F & \underline{s}_G & \underline{s}_{path}
    \end{array}\right]


Inserting numerical values yields:

.. math::
    :label: eq:robot_jacobian_numbers2

    J = \left[\begin{array}{ccccccc|c}
    0.0000 & -0.0134 & -0.1356 & -0.0256 & 0.1756 & -0.0123 & -0.0936 & 0.2853\\
    0.0000 & 0.0200 & -0.0000 & 0.1821 & 0.0073 & -0.3956&  -0.0074 & 0.6905 \\
    0.0000 & -0.1343 & 0.0136 & 0.1766 & -0.0106 & -0.1770&  0.0040 & -0.0611 \\
    0.0000 & 0.9950 & -0.0978 & -0.9791 & 0.0604 & 0.9981&  -0.0584 & 0.2548 \\
    1.0000 & 0.0000 & 0.1986 & -0.1947 & -0.0038 & -0.0040&  0.2024 & -0.6369 \\
    0.0000 & -0.0998 & -0.9751 & 0.0585 & 0.9981 & -0.0604&  -0.9775 & -0.1671 \\
    \end{array}\right]


Applying LU decomposition with complete row and column pivoting to find the Null space matrix, then applying Gram-Schmidt orthonormalization yields:

.. math::
    :label: eq:null_space

    V = \begin{array}{r}
    A\\
    B\\
    C\\
    D\\
    E\\
    F\\
    G\\
    path
    \end{array}
    \left[\begin{array}{rr}
    0.1849 & -0.2038 \\
    -0.1673 & -0.2237 \\
    0.6898 & 0.0460 \\
    0.0000 & 0.7862 \\
    -0.6642 & -0.0574 \\
    0.1192 & -0.5263 \\
    0.0059 & 0.0104 \\
    0.0803 & -0.0870
    \end{array}\right]


The two columns correspond to two orthogonal directions along the manifold. In the example above, where the end-effector was rigidly constrained, one direction along the manifold was identified as the end-effector's ability to move without causing a change in the overall mechanism configuration. A second, orthogonal direction corresponds to the unit screw introduced in :eq:`eq:augmented_jacobian`.

Note that while the two columns of :eq:`eq:null_space` correspond to orthogonal directions along the manifold, they do not necessarily align with the two directions of interest. That is, there is not a unique basis on the constraint manifold, and an orthonormal transformation applied to V will still lie on the manifold. The key to aligning the two columns to the two directions of interest is found in the last row of Eq. :eq:`eq:null_space`.

The matrix :math:`V` can be right multiplied by an orthonormal transformation matrix (i.e. a planar rotation matrix) to align the first column of :eq:`eq:null_space` to the direction of :math:`\underline{s}_{path}`.

.. math::
    :label: eq:aligned_null_space

    VR = V \begin{bmatrix}
    -0.6780 & -0.7351\\
    0.7351 & -0.6780\end{bmatrix} =
    \begin{array}{r}
    A\\
    B\\
    C\\
    D\\
    E\\
    F\\
    G\\
    path
    \end{array}
    \left[\begin{array}{rr}
    -0.2752 & 0.0023\\
    -0.0510 & 0.2747\\
    -0.4339 & -0.5383\\
    0.5779  & -0.5331\\
    0.4081  & 0.5272\\
    -0.4678 & 0.2692\\
    0.0036  & -0.0115\\
    -0.1184 & 0.0000
    \end{array}\right]


The matrix V has now been aligned to the directions of interest. The first column is aligned to the target path. The second, orthogonal direction corresponds to the robot's ability to move while holding the configuration of the end-effector fixed. The sum of the squares of the columns sum to 1. The square of each term in a given term indicates each joint's ability to effect a change in a given direction.

As mentioned above, with a 7DOF end-effector, there is not a unique choice for :math:`d\phi` to achieve a desired incremental displacement. The (square of the) numbers in column 1 may be used as an objective function to maximize the alignment of the joints with manifold at the next time step. The norm of column 1 is always 1 and is therefore not a suitable objective function. A more suitable objective function might be product of the squares of the first column for the entries associated with joints A-G.
