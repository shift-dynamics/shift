# Developer guide

## Permutation orders

| Permutation | Description |
--- | ---
| \f$ \wp_0 \f$ | Original order specified in input file |
| \f$ \wp_1 \f$ | Order dictated by kinematic substructuring |
| \f$ \wp_2 \f$ | Order dictated by Generalized Coordinate Partitioning |
| \f$ \wp_3 \f$ | Follows \f$\wp_1\f$ with independent joint variables moved first within each substructure |


## Notation

| example                               | description                     | reserved for |
| ------------------------------------- | ------------------------------- | ------------------------- |
| \f$ A \f$                             | capital symbols                 | Matrices |
| \f$ a \f$                             | lowercase symbols               | Scalar quantities |
| \f$ \underline{a} \f$                 | underlined symbols              | Vector quantities, or a column matrix of stacked vector quantities |
| \f$ \dot{v} \f$                       | overhead dot                    | First time derivative |
| \f$ \ddot{v} \f$                      | overhead double dot             | Second time derivative |
| \f$ \begin{bmatrix} \end{bmatrix} \f$ | square brackets                 | \f$ m \times n \f$ matrices where \f$ m \f$ and \f$ n \f$ are typically greater than 1 |
| \f$ \begin{Bmatrix} \end{Bmatrix} \f$ | *curly* brackets or braces      | \f$ m \times 1 \f$ column matrix |
| \f$ \mathbf{\underline{v}} \f$        | bold face/underlined            | \f$ 6 \times 1 \f$ *screw* quantities, such as velocities and accelerations |
| \f$ \mathbf{\underline{f}}^* \f$      | bold face, underlined, asterisk | \f$ 6 \times 1 \f$ *coscrew* quantities, such as forces. |


## Symbols

| symbol                           | permutation order | name in program    | description |
  -------------------------------- | ----------------- | ---------------    | ----------- 
| \f$ q \f$                        | \f$ \wp_1 \f$     | `shift::core::Context::q_`            | generalized coordinates, typically the relative joint coordinates between articulated bodies |
| \f$ \dot{q} \f$                  | \f$ \wp_1 \f$     | `shift::core::Context::dq_`           | first time derivative of generalized coordinates, generalized velocities |
| \f$ \ddot{q} \f$                 | \f$ \wp_1 \f$     | `shift::core::Context::ddq_`          | second time derivative of generalized coordinates |
| PC                               | \f$ \wp_1 \f$     | `shift::core::Context::pcl_`          | parent child list |
| \f$ f \f$                        | \f$ \wp_1 \f$     | `shift::core::Context::f_`            | working forces applied in the joints |
| \f$ \underline{\mathbf{g}}^* \f$ | \f$ \wp_1 \f$     | `shift::core::solver::RNCA::g_`       | spatial inertial forces and force due to gravity |
| \f$ \lambda_p \f$                | \f$ \wp_3 \f$     | `shift::core::solver::RNCA::la_`      | lagrange multipliers |
| \f$ M^{\ell} \f$                 | \f$ \wp_3 \f$     | `shift::core::solver::RNCA::Ml_`      | generalized mass matrix |
| \f$ \underline{\mathbf{p}}^* \f$ | \f$ \wp_1 \f$     | `shift::core::solver::RNCA::p_`       | spatial momentum | 
| \f$ \gamma^p \f$                 | \f$ \wp_2 \f$     | `shift::core::solver::RNCA::gamma_d_` | quadratic velocity terms |
| \f$ \gamma^d \f$                 | \f$ \wp_2 \f$     | `shift::core::solver::RNCA::gamma_d_` | *preconditioned* quadratic velocity terms |
| \f$ f^{\ell} \f$                 | \f$ \wp_1 \f$     | `shift::core::solver::RNCA::fl_`      | generalized forces |
| \f$ J \f$                        | \f$ \wp_1 \f$ , \f$\wp_2\f$     | `shift::core::Jacobian` | Jacobian matrix, \f$ J(q) \f$, a mapping of body velocities to generalized velocities |
| \f$ \Phi_{fb} \f$                | \f$ \wp_1 \f$     | `shift::core::topology::DHVA::d` | absolute displacement from fixed inertial frame \f$f\f$ to body reference frame \f$b\f$ in the active sense or from \f$b\f$ to \f$f\f$ in the passive sense |
| \f$ \underline{\mathbf{h}} \f$   | \f$ \wp_1 \f$     | `shift::core::topology::DHVA::h` | influence coefficient matrix, which forms 6x1 columns of the Jacobian matrix |
| \f$ \underline{\mathbf{v}} \f$   | \f$ \wp_1 \f$     | `shift::core::topology::DHVA::v` | velocity screw \f$ \left(\underline{\mathbf{v}} = \begin{Bmatrix}\underline{v}\\\underline{\omega}\end{Bmatrix}\right) \f$ |
|                                  | \f$ \wp_1 \f$     | `shift::core::topology::DHVA::aq` | quadratic velocity terms |


## Transformations

| symbol                     | operation                                                                       | description |
  -------------------------- | ------------------------------------------------------------------------------- | ------ 
| \f$ R \f$                  |                                                                                 | \f$ R \in \mathbb{R}^{3x3} \f$, an SO(3) rotation matrix |
| \f$ R^{-1} \f$             | \f$ R^T \f$                                                                     |                                                          |
| \f$ \underline{r} \f$      |                                                                                 | \f$ r \in \mathbb{R}^{3} \f$, a translation vector      |
| \f$ \underline{r}^{-1} \f$ | \f$ - \underline{r} \f$                                                         |                                                           |
| \f$ \tilde{r} \f$          | \f$ \begin{bmatrix}0&-r_z&r_y&\\r_z&0&-r_x\\-r_y&r_x&0\end{bmatrix} \f$         | \f$ \tilde{r} \in \mathbb{R}^{3x3} \f$, a skew-symmetric matrix |
| \f$ \Phi \f$               |    \f$ \begin{bmatrix}R&\underline{r}\\0 & 1\end{bmatrix} \f$                   |    \f$ \Phi(R,r) \in SE(3) \f$, a general rigid body displacement consisting of rotation (\f$ R \f$) and translation (\f$ r \f$) |
| \f$ \Phi^{-1} \f$          |    \f$ \begin{bmatrix}R^T&-R^T\underline{r}\\0&1\end{bmatrix} \f$               |                                                                   |
| \f$ [Ad] \f$               | \f$ \begin{bmatrix}R & \tilde{r}R\\0&R\end{bmatrix} \f$                         | adjoint operator for transforming screws |
| \f$ [Ad]^{-1} \f$          | \f$ \begin{bmatrix}R^T&-\tilde{r}R^T\\0&R^T\end{bmatrix} \f$                    | inverse of adjoint operator for transforming screws |
| \f$ [Ad]^* \f$             | \f$ \begin{bmatrix}R & 0\\\tilde{r}R&R\end{bmatrix} \f$                         | coadjoint operator for transforming coscrews |
| \f$ [ad] \f$               | \f$ \begin{bmatrix}\tilde{\omega}&\tilde{v}\\0&\tilde{\omega}\end{bmatrix} \f$  | time derivative of \f$ [Ad] \f$ |
| \f$ [ad]^* \f$             | \f$ \begin{bmatrix}\tilde{\omega}&0\\\tilde{v}&\tilde{\omega}\end{bmatrix} \f$  | time derivative of \f$ [Ad]^* \f$ |
