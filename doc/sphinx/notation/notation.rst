.. _notation:
.. title:: Notation

Notation
--------

Conventions
***********

===================================== =============================== =========================
example                               description                     reserved for
===================================== =============================== =========================
:math:`A`                             capital symbols                 Matrices
:math:`a`                             lowercase symbols               Scalar quantities
:math:`\underline{a}`                 underlined symbols              Vector quantities, or a column matrix of stacked vector quantities
:math:`\dot{v}`                       overhead dot                    First time derivative
:math:`\ddot{v}`                      overhead double dot             Second time derivative
:math:`\hat{e}`                       overhead hat                    Unit quantities, such as unit quaternions and unit screws
:math:`\begin{bmatrix} \end{bmatrix}` square brackets                 :math:`m \times n` matrices where :math:`m` and :math:`n` are typically greater than 1
:math:`\begin{Bmatrix} \end{Bmatrix}` *curly* brackets or braces      :math:`m \times 1` column matrix
:math:`\mathbf{\underline{v}}`        bold face/underlined            :math:`6 \times 1` *screw* quantities, such as velocities and accelerations
:math:`\mathbf{\underline{f}}^*`      bold face, underlined, asterisk :math:`6 \times 1` *coscrew* quantities, such as forces.
===================================== =============================== =========================

Symbols
*******

The second column is the name of the quantity in Shift's code. A trailing
underscore marks a class member.

================================ =================== ===========
symbol                           name in program     description
================================ =================== ===========
:math:`\underline{q}`            *q*                 generalized coordinates
:math:`\underline{\dot{q}}`      *dq*                first time derivative of generalized coordinates
:math:`\underline{\ddot{q}}`     *ddq*               second time derivative of generalized coordinates
PC                               *pcl_*              parent child list
:math:`\underline{e}`            *e_*                constraint errors
:math:`J`                        *J_*                constraint Jacobian matrix, :math:`J(\underline{q})`, a mapping of generalized velocities to the velocities of the constraints
:math:`B^{di}`                   *b3_*               the matrix giving the dependent velocities from the independent ones, :math:`\underline{\dot{q}}^d = -B^{di}\underline{\dot{q}}^i`; nonzeros in PO(3), indices in *rb3_* and *cb3_*
:math:`N`                        *right_null_space_* right null space of :math:`J`
:math:`Z`                        *right_null_space_* orthonormalized right null space of :math:`J`, :math:`N` orthonormalized in place
:math:`\mu_i`                    *mobility_numbers_* mobility number of generalized coordinate :math:`i`: the squared norm of row :math:`i` of :math:`Z`, the diagonal of the projector :math:`ZZ^T`
:math:`\underline{\lambda}^p`    *la_*               lagrange multipliers
:math:`\underline{\lambda}^d`    *la_*               *preconditioned* lagrange multipliers
:math:`\underline{\gamma}^p`     *gamma_d_*          quadratic velocity terms (``gamma_d_`` holds both :math:`\underline{\gamma}^p` and :math:`\underline{\gamma}^d`)
:math:`\underline{\gamma}^d`     *gamma_d_*          *preconditioned* quadratic velocity terms
:math:`M^{\ell}`                 *Ml_*               generalized mass matrix
:math:`\underline{f}`            *f_*                working forces applied in the joints
:math:`\underline{f}^{\ell}`     *fl_*               generalized forces
:math:`\underline{\mathbf{p}}^*` *p_*                momentum
:math:`\underline{\mathbf{h}}`   *h*                 influence coefficient matrix
:math:`\underline{\mathbf{v}}`   *v*                 velocity screw :math:`\left(\underline{\mathbf{v}} = \begin{Bmatrix}\underline{v}\\\underline{\omega}\end{Bmatrix}\right)`
:math:`\underline{\mathbf{g}}^*` *body_loads_*       externally applied forces, such as from gravity
================================ =================== ===========

Transformations
***************

========================== =============================================================================== ======
symbol                     operation                                                                       description
========================== =============================================================================== ======
:math:`R`                                                                                                  :math:`R \in \mathbb{R}^{3x3}`, an SO(3) rotation matrix
:math:`R^{-1}`             :math:`R^T`
:math:`\hat{e}`            :math:`\{\underline{e};\ e_3\}`                                                 a unit quaternion, scalar part last
:math:`\hat{e}^{-1}`       :math:`\hat{e}^* = \{-\underline{e};\ e_3\}`
:math:`\underline{r}`                                                                                      :math:`r \in \mathbb{R}^{3}`, a translation vector
:math:`\underline{r}^{-1}` :math:`- \underline{r}`
:math:`\tilde{r}`          :math:`\begin{bmatrix}0&-r_z&r_y&\\r_z&0&-r_x\\-r_y&r_x&0\end{bmatrix}`         :math:`\tilde{r} \in \mathbb{R}^{3x3}`, a skew-symmetric matrix
:math:`\Phi`                  :math:`\begin{bmatrix}R&\underline{r}\\0 & 1\end{bmatrix}`                      :math:`\Phi(R,r) \in SE(3)`, a general rigid body displacement consisting of rotation (:math:`R`) and translation (:math:`r`).
:math:`\Phi^{-1}`             :math:`\begin{bmatrix}R^T&-R^T\underline{r}\\0&1\end{bmatrix}`
:math:`[Ad]`               :math:`\begin{bmatrix}R & \tilde{r}R\\0&R\end{bmatrix}`                         adjoint operator for transforming screws
:math:`[Ad]^{-1}`          :math:`\begin{bmatrix}R^T&-R^T\tilde{r}\\0&R^T\end{bmatrix}`                         inverse of adjoint operator for transforming screws
:math:`[Ad]^*`             :math:`\begin{bmatrix}R & 0\\\tilde{r}R&R\end{bmatrix}`                         coadjoint operator for transforming coscrews
:math:`[ad]`               :math:`\begin{bmatrix}\tilde{\omega}&\tilde{v}\\0&\tilde{\omega}\end{bmatrix}`  time derivative of :math:`[Ad]`
:math:`[ad]^*`             :math:`\begin{bmatrix}\tilde{\omega}&0\\\tilde{v}&\tilde{\omega}\end{bmatrix}`  time derivative of :math:`[Ad]^*`
========================== =============================================================================== ======
