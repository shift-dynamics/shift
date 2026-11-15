#ifndef SHIFT_BODY_FREE_H_
#define SHIFT_BODY_FREE_H_

#include "shift/core/body/base.h"

#include <memory>

namespace shift {
namespace core {
namespace body {

/*!
@brief a rigid body outside the kinematic tree, model type "free_body"

A free body has no joints: none may connect to it. It moves under the loads
applied to it (e.g. contact forces) and gravity, by the spatial equations of
motion in the fixed frame,

\f[
M_{b,ff}^{ff} \dot{\mathbf{\underline{v}}}_{fb,f}^{f}
  = \mathbf{\underline{f}}_{b,f}^{*f}
  - [ad]_{fb,f}^{*f} M_{b,ff}^{ff} \mathbf{\underline{v}}_{fb,f}^{f},
\f]

with \f$\mathbf{\underline{v}}_{fb,f}^{f}\f$ the spatial velocity of body
\f$b\f$ relative to the fixed frame \f$f\f$ and
\f$\dot{\mathbf{\underline{v}}}_{fb,f}^{f}\f$ its spatial acceleration, both
at the origin of \f$f\f$ and in \f$f\f$ coordinates ([linear, angular], as
ksl_screw_t), \f$\mathbf{\underline{f}}_{b,f}^{*f}\f$ the load on the body at
the origin of \f$f\f$ ([force, moment], as ksl_coscrew_t),
\f$[ad]_{fb,f}^{*f}\f$ the coscrew cross product operator formed from
\f$\mathbf{\underline{v}}_{fb,f}^{f}\f$, and \f$M_{b,ff}^{ff}\f$ the spatial
inertia at the origin of \f$f\f$ (ksl_inertia_transformed of the centroidal
inertia). The linear part \f$\underline{v}_{fb,f}^{f}\f$ of
\f$\mathbf{\underline{v}}_{fb,f}^{f}\f$ is the velocity of the material point
of the body at the fixed origin: a point \f$\underline{p}^{f}\f$ of the body
moves with \f$\underline{v}_{fb,f}^{f} + \tilde{\omega}_{fb}^{f}
\underline{p}^{f}\f$, with \f$\underline{\omega}_{fb}^{f}\f$ the angular
part.

\f$M_{b,ff}^{ff}\f$ is never factored. It is the centroidal inertia shifted
from the center of mass \f$c\f$, at \f$\underline{r}_{fc}^{f}\f$, to the fixed
origin, and the centroidal inertia is block diagonal, \f$m I\f$ (\f$I\f$ the
identity) and the \f$3 \times 3\f$ moment of inertia \f$\bar{I}_{b,cc}^{ff}\f$,
so \f$\dot{\mathbf{\underline{v}}}_{fb,f}^{f} = (M_{b,ff}^{ff})^{-1}
\mathbf{\underline{f}}^{*f}_{b,f}\f$ is
(Featherstone, 2008, 2.13), with the force \f$\underline{f}_{b}^{f}\f$ and moment
\f$\underline{n}_{b,f}^{f}\f$ of the load, and \f$\underline{a}_{fb,f}^{f}\f$ and
\f$\underline{\alpha}_{fb}^{f}\f$ the linear and angular parts of the acceleration:

\f[
\begin{aligned}
\text{shift the load to } c: &\quad
  \underline{n}_{b,c}^{f} = \underline{n}_{b,f}^{f}
  - \tilde{r}_{fc}^{f} \underline{f}_{b}^{f} \\
\text{centroidal inverse}: &\quad
  \underline{a}_{fb,c}^{f} = \underline{f}_{b}^{f} / m, \quad
  \underline{\alpha}_{fb}^{f} = (\bar{I}_{b,cc}^{ff})^{-1}
  \underline{n}_{b,c}^{f} \\
\text{shift the motion to the origin}: &\quad
  \underline{a}_{fb,f}^{f} = \underline{a}_{fb,c}^{f}
  + \tilde{r}_{fc}^{f} \underline{\alpha}_{fb}^{f}
\end{aligned}
\f]

with \f$(\bar{I}_{b,cc}^{ff})^{-1} = R^{fb} (\bar{I}_{b,cc}^{bb})^{-1}
R^{fbT}\f$ from the inverse of the body frame centroidal inertia
\f$\bar{I}_{b,cc}^{bb}\f$, inverted once. Gravity \f$\underline{g}^{f}\f$ adds
\f$\begin{Bmatrix}\underline{g}^{f}\\\underline{0}\end{Bmatrix}\f$ to
\f$\dot{\mathbf{\underline{v}}}_{fb,f}^{f}\f$.

This is where a free body differs from a body connected by joints. A connected
body has no equations of its own: the influence coefficients express its motion
in the joint coordinates, so its inertia is projected onto them and enters the
generalized mass matrix \f$M^{\ell}\f$ with the other bodies of the model,
closed loops add constraints and the multipliers \f$\underline{\lambda}\f$, and
solver::RNCA factors and solves the resulting system at every evaluation. A free
body enters none of it. With no joint, nothing couples its acceleration to that
of another body; with no constraint, it has no row in the Jacobian, no
multiplier and no split into independent and dependent coordinates. Its
equations stay in spatial form, one \f$6 \times 6\f$ system per body, solved in
closed form as above with nothing assembled or factored, so the cost grows
linearly with the number of free bodies and they are evaluated in parallel
(Context::SetForces).

Free bodies therefore interact with the rest of the model only through
loads. Force modules add them with Context::AddBodyLoad, computed from the
current state, and the free bodies are evaluated after the force modules have
run. A contact force reaches a free body this way; a joint or constraint cannot.
To attach a body that moves in six degrees of freedom to a mechanism, or to
include it in a closed loop, give it a joint::Float instead: its coordinates are
then joint coordinates, solved with the rest of the model by solver::RNCA.

Its displacement d (of the origin), v and a are kept in the free part of
the DHVA (topology::DVA); a free body has no joint, so no influence
coefficients. Its generalized coordinates in the state vector, after q and
dq (see Context::get_state_count), are kStateCount values: the position and
orientation (unit quaternion [x, y, z, w]) of the origin and v, with time
derivatives v.lin + v.ang x t, the quaternion rate of v.ang, and a.

Featherstone, R. (2008), Rigid Body Dynamics Algorithms, Springer.
*/
class Free : public Base {
public:
  /*! generalized coordinates: [t (3), quaternion (4), v.lin (3),
   * v.ang (3)] */
  static constexpr int kStateCount = 13;

  Free()
    : Base() {
    DeclareParameters();
    is_articulated_ = false;
  }

  Free(std::string name)
    : Base(name) {
    DeclareParameters();
    is_articulated_ = false;
  }

  virtual ~Free() override {}

  static std::unique_ptr<Free> Create();

  void DeclareParameters() override;

  static bool is_registered() {
    return registered_;
  }

  /*! \brief check the inertia (positive mass, positive definite centroidal
   * moment of inertia) and invert it */
  void SetupInertia();

  /*! \brief the initial displacement of the origin and spatial velocity */
  const ksl_SE3_t& get_initial_d() const {
    return *frame_.get_d_wrt_origin();
  }
  ksl_screw_t get_initial_v() const;
  void set_initial_dv(const ksl_SE3_t& d, const ksl_screw_t& v);

  /*! \brief index among the free bodies of the context, -1 if inactive */
  int get_free_index() const {
    return free_index_;
  }
  void set_free_index(int index) {
    free_index_ = index;
  }

  /*!
  @brief the spatial acceleration, from the Newton-Euler equations
  @param d [in] displacement of the origin in the fixed frame
  @param v [in] spatial velocity
  @param load [in] load on the body, referred to the fixed origin
  @param gravity [in] acceleration of gravity in the fixed frame
  @param a [out] spatial acceleration
  */
  void Evaluate(const ksl_SE3_t& d, const ksl_screw_t& v,
                const ksl_coscrew_t& load, const ksl_vec3_t& gravity,
                ksl_screw_t& a) const;

private:
  static bool registered_;
  /*! initial displacement of the origin with respect to the fixed frame */
  shift::core::Frame frame_;
  /*! initial velocity [v, omega] of the origin, in fixed coordinates */
  ksl_screw_t initial_velocity_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  /*! centroidal inertia in body (origin) coordinates and m times the center
   * of mass from the origin, the input of ksl_inertia_transformed */
  ksl_inertia_t inertia_wrt_origin_ = {};
  /*! inverse of the centroidal moment of inertia in body coordinates, the
   * six terms of a symmetric matrix as in ksl_inertia_t (m and mt unused) */
  ksl_inertia_t inverse_inertia_ = {};
  int free_index_ = -1;
};

} // namespace body
} // namespace core
} // namespace shift

#endif
