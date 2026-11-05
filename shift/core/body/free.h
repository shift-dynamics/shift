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
applied to it (e.g. contact forces) and gravity, by the Newton-Euler
equations in the fixed frame,

\f[
\mathbf{\underline{f}}^* = M \dot{\mathbf{\underline{v}}} + [ad]^* M \mathbf{\underline{v}},
\f]

with \f$\mathbf{\underline{v}}\f$ the spatial velocity of the body and
\f$\dot{\mathbf{\underline{v}}}\f$ its spatial acceleration, both referred
to the fixed origin and in fixed coordinates ([linear, angular], as
ksl_screw_t), \f$\mathbf{\underline{f}}^*\f$ the load on the body referred
to the fixed origin ([force, moment], as ksl_coscrew_t) and \f$M\f$ the
spatial inertia about the fixed origin (ksl_inertia_transformed of the
centroidal inertia). The linear part \f$\underline{v}\f$ of
\f$\mathbf{\underline{v}}\f$ is the velocity of the material point of the
body at the fixed origin: a point \f$\underline{p}\f$ of the body moves with
\f$\underline{v} + \underline{\omega} \times \underline{p}\f$, with
\f$\underline{\omega}\f$ the angular part.

\f$M\f$ is never factored. It is the centroidal inertia shifted from the
center of mass \f$\underline{c}\f$ to the fixed origin, and the centroidal
inertia is block diagonal, \f$m E\f$ and \f$I_c\f$, so
\f$\dot{\mathbf{\underline{v}}} = M^{-1} \mathbf{\underline{f}}^*\f$ is
(Featherstone, 2008, 2.13), with the force \f$\underline{f}\f$ and moment
\f$\underline{n}\f$ of the load, and \f$\underline{a}\f$ and
\f$\underline{\alpha}\f$ the linear and angular parts of the acceleration:

\f[
\begin{aligned}
\text{shift the load to } \underline{c}: &\quad
  \underline{f}_c = \underline{f}, \quad
  \underline{n}_c = \underline{n} - \underline{c} \times \underline{f} \\
\text{centroidal inverse}: &\quad
  \underline{a}_c = \underline{f}_c / m, \quad
  \underline{\alpha} = I_c^{-1} \underline{n}_c \\
\text{shift the motion to the origin}: &\quad
  \underline{a} = \underline{a}_c - \underline{\alpha} \times \underline{c}
\end{aligned}
\f]

with \f$I_c^{-1} = R I_b^{-1} R^T\f$ from the inverse of the body frame
centroidal inertia \f$I_b\f$, inverted once. Gravity \f$\underline{g}\f$
adds \f$\begin{Bmatrix}\underline{g}\\\underline{0}\end{Bmatrix}\f$ to
\f$\dot{\mathbf{\underline{v}}}\f$.

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
