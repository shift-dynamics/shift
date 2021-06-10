#include "shift/core/joint/Prismatic.h"
#include "shift/core/Context.h"
#include "shift/core/Factory.h"

#include "ksl/matrix.h"
#include "ksl/print.h"

#include <math.h>

bool shift::core::joint::Prismatic::registered_ =
  shift::core::Factory<shift::core::joint::Base>::get()->RegisterModule(
    "prismatic", shift::core::joint::Prismatic::Create);

std::unique_ptr<shift::core::joint::Prismatic> shift::core::joint::Prismatic::Create() {
  return std::make_unique<shift::core::joint::Prismatic>();
}

/*!
@brief Prismatic joint along x-axis (dh)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tx_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                        const shift::core::topology::Offsets& offsets,
                        const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_product_ddtx(d_parent, *d_primitive, d_child);
}

/*!
@brief Prismatic Joint aligned with x-axis - inv (dh)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::txinv_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_product_ddtxinv(d_parent, *d_primitive, d_child);
}

/*!
@brief Prismatic Joint aligned with y-axis (dh)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ty_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                        const shift::core::topology::Offsets& offsets,
                        const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_product_ddty(d_parent, *d_primitive, d_child);
}

/*!
@brief Prismatic Joint aligned with y-axis - inv (dh)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tyinv_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_product_ddtyinv(d_parent, *d_primitive, d_child);
}

/*!
@brief Prismatic Joint aligned with z-axis (dh)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tz_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                        const shift::core::topology::Offsets& offsets,
                        const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_product_ddtz(d_parent, *d_primitive, d_child);
}

/*!
@brief Prismatic Joint aligned with z-axis - inv (dh)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tzinv_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_product_ddtzinv(d_parent, *d_primitive, d_child);
}

/*!
@brief Prismatic Joint aligned with arbitrary axis (dh)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ta_d(const shift::core::Context& context,
                        shift::core::topology::DHVA& dhva,
                        const shift::core::topology::Offsets& offsets,
                        const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.get_joint_data()[offsets.data]);
  ksl_vec3_t temp;
  ksl_product_av(*d_primitive, axis, &temp);
  ksl_product_ddt(d_parent, &temp, d_child);
}

/*!
@brief Prismatic Joint aligned with z-axis - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tainv_d(const shift::core::Context& context,
                           shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.get_joint_data()[offsets.data]);
  ksl_vec3_t temp;
  ksl_product_av(*d_primitive, axis, &temp);
  ksl_product_ddtinv(d_parent, &temp, d_child);
}

/*!
@brief Prismatic joint along x-axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tx_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                         const shift::core::topology::Offsets& offsets,
                         const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_screw_t* h = &dhva.h[offsets.dq];

  ksl_product_ddtx(d_parent, *d_primitive, d_child);
  ksl_hstx(d_child, h);
}

/*!
@brief @brief Prismatic joint along x-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tx_dhva(const shift::core::Context& context,
                           shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  shift::core::joint::tx_dh(context, dhva, offsets, q);

  ksl_screw_t* h = &dhva.h[offsets.dq];
  const double* v_primitive = dq + offsets.dq;
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];

  ksl_product_as(*v_primitive, h, &v_relative);
  ksl_add_sst(v_parent, &v_relative, v_child);
  ksl_cross_sst(v_parent, &v_relative, &aq_relative);
  ksl_add_sst(aq_parent, &aq_relative, aq_child);
}

/*!
@brief Prismatic Joint aligned with x-axis - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::txinv_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                            const shift::core::topology::Offsets& offsets,
                            const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_screw_t* h = &dhva.h[offsets.dq];

  ksl_product_ddtxinv(d_parent, *d_primitive, d_child);
  ksl_hstxinv(d_child, h);
}

/*!
@brief Prismatic joint along x-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::txinv_dhva(const shift::core::Context& context,
                              shift::core::topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  shift::core::joint::txinv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = &dhva.h[offsets.dq];
  const double* v_primitive = dq + offsets.dq;
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];

  ksl_product_as(*v_primitive, h, &v_relative);
  ksl_add_sst(v_parent, &v_relative, v_child);
  ksl_cross_sst(v_parent, &v_relative, &aq_relative);
  ksl_add_sst(aq_parent, &aq_relative, aq_child);
}

/*!
@brief Prismatic Joint aligned with y-axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ty_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                         const shift::core::topology::Offsets& offsets,
                         const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_screw_t* h = &dhva.h[offsets.dq];

  ksl_product_ddty(d_parent, *d_primitive, d_child);
  ksl_hsty(d_child, h);
}

/*!
@brief Prismatic joint along y-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ty_dhva(const shift::core::Context& context,
                           shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  shift::core::joint::ty_dh(context, dhva, offsets, q);

  ksl_screw_t* h = &dhva.h[offsets.dq];
  const double* v_primitive = dq + offsets.dq;
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];

  ksl_product_as(*v_primitive, h, &v_relative);
  ksl_add_sst(v_parent, &v_relative, v_child);
  ksl_cross_sst(v_parent, &v_relative, &aq_relative);
  ksl_add_sst(aq_parent, &aq_relative, aq_child);
}

/*!
@brief Prismatic Joint aligned with y-axis - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tyinv_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                            const topology::Offsets& offsets,
                            const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_screw_t* h = &dhva.h[offsets.dq];

  ksl_product_ddtyinv(d_parent, *d_primitive, d_child);
  ksl_hstyinv(d_child, h);
}

/*!
@brief @brief Prismatic joint along y-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tyinv_dhva(const shift::core::Context& context,
                              topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  shift::core::joint::tyinv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = &dhva.h[offsets.dq];
  const double* v_primitive = dq + offsets.dq;
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];

  ksl_product_as(*v_primitive, h, &v_relative);
  ksl_add_sst(v_parent, &v_relative, v_child);
  ksl_cross_sst(v_parent, &v_relative, &aq_relative);
  ksl_add_sst(aq_parent, &aq_relative, aq_child);
}

/*!
@brief Prismatic Joint aligned with z-axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tz_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                         const shift::core::topology::Offsets& offsets,
                         const double* __restrict__ q) {

  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];

  const double* d_primitive = &q[offsets.q];

  ksl_screw_t* h = &dhva.h[offsets.dq];

  ksl_product_ddtz(d_parent, *d_primitive, d_child);

  ksl_hstz(d_child, h);

#if 0
  std::cout << "tz_dh" << std::endl;
  ksl_SE3_print(stdout, d_parent, "d_parent: ");
  ksl_SE3_print(stdout, d_child, "d_child: ");
  std::cout << "d_primitive: " << *d_primitive << std::endl;
  ksl_screw_print(stdout, h, "h: ");
#endif
}

/*!
@brief Prismatic joint along z-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tz_dhva(const shift::core::Context& context,
                           shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  shift::core::joint::tz_dh(context, dhva, offsets, q);

  ksl_screw_t* h = &dhva.h[offsets.dq];
  const double* v_primitive = dq + offsets.dq;
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];

  ksl_product_as(*v_primitive, h, &v_relative);
  ksl_add_sst(v_parent, &v_relative, v_child);
  ksl_cross_sst(v_parent, &v_relative, &aq_relative);
  ksl_add_sst(aq_parent, &aq_relative, aq_child);

#if 0
  std::cout << "v_primitive: " << *v_primitive << std::endl;
  std::cout << "tz_dhva" << std::endl;
  ksl_screw_print(stdout, h, "h: ");
  ksl_screw_print(stdout, v_parent, "v_parent: ");
  ksl_screw_print(stdout, v_child, "v_child: ");
  ksl_screw_print(stdout, aq_parent, "aq_parent: ");
  ksl_screw_print(stdout, aq_child, "aq_child: ");
#endif
}

/*!
@brief Prismatic Joint aligned with z-axis - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tzinv_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                            const shift::core::topology::Offsets& offsets,
                            const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_screw_t* h = &dhva.h[offsets.dq];

  ksl_product_ddtzinv(d_parent, *d_primitive, d_child);
  ksl_hstzinv(d_child, h);
}

/*!
@brief Prismatic joint along z-axis - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tzinv_dhva(const shift::core::Context& context,
                              shift::core::topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  shift::core::joint::tzinv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = &dhva.h[offsets.dq];
  const double* v_primitive = dq + offsets.dq;
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];

  ksl_product_as(*v_primitive, h, &v_relative);
  ksl_add_sst(v_parent, &v_relative, v_child);
  ksl_cross_sst(v_parent, &v_relative, &aq_relative);
  ksl_add_sst(aq_parent, &aq_relative, aq_child);
}

/*!
@brief Prismatic Joint aligned with arbitrary axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ta_dh(const shift::core::Context& context,
                         shift::core::topology::DHVA& dhva,
                         const shift::core::topology::Offsets& offsets,
                         const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_screw_t* h = &dhva.h[offsets.dq];
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.get_joint_data()[offsets.data]);
  ksl_vec3_t temp;
  ksl_product_av(*d_primitive, axis, &temp);
  ksl_product_ddt(d_parent, &temp, d_child);
  ksl_hsta(d_child, axis, h);
}

/*!
@brief Prismatic joint along z-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ta_dhva(const shift::core::Context& context,
                           shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  shift::core::joint::tz_dh(context, dhva, offsets, q);

  ksl_screw_t* h = &dhva.h[offsets.dq];
  const double* v_primitive = dq + offsets.dq;
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];

  ksl_product_as(*v_primitive, h, &v_relative);
  ksl_add_sst(v_parent, &v_relative, v_child);
  ksl_cross_sst(v_parent, &v_relative, &aq_relative);
  ksl_add_sst(aq_parent, &aq_relative, aq_child);
}

/*!
@brief Prismatic Joint aligned with z-axis - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tainv_dh(const shift::core::Context& context,
                            shift::core::topology::DHVA& dhva,
                            const shift::core::topology::Offsets& offsets,
                            const double* __restrict__ q) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];
  const double* d_primitive = &q[offsets.q];
  ksl_screw_t* h = &dhva.h[offsets.dq];
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.get_joint_data()[offsets.data]);
  ksl_vec3_t temp;
  ksl_product_av(*d_primitive, axis, &temp);
  ksl_product_ddtinv(d_parent, &temp, d_child);
  ksl_hstainv(d_child, axis, h);
}

/*!
@brief Prismatic joint along z-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::tainv_dhva(const shift::core::Context& context,
                              shift::core::topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  shift::core::joint::tzinv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = &dhva.h[offsets.dq];
  const double* v_primitive = &dq[offsets.dq];
  ksl_screw_t* v_parent = &dhva.v[offsets.parent.v];
  ksl_screw_t v_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* v_child = &dhva.v[offsets.child.v];
  ksl_screw_t* aq_parent = &dhva.aq[offsets.parent.v];
  ksl_screw_t aq_relative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aq_child = &dhva.aq[offsets.child.v];

  ksl_product_as(*v_primitive, h, &v_relative);
  ksl_add_sst(v_parent, &v_relative, v_child);
  ksl_cross_sst(v_parent, &v_relative, &aq_relative);
  ksl_add_sst(aq_parent, &aq_relative, aq_child);
}

void shift::core::joint::Prismatic::DeclareParameters() {

  DeclareParameter("axis", &axis_,
               "Translation axis expressed in parent body frame, "
               "defaults to z-axis if no axis provided.",
               false);

  DeclareParameter(
    "initial_displacement", q_0_,
    "Initial joint displacement, defaults to zero if no value provided.",
    false);

  DeclareParameter("initial_velocity", dq_0_,
               "Initial joint velocity, defaults to zero if no value provided.",
               false);

  DeclareParameter(
    "damping_coefficients", &damping_coefficients_,
    "specify  polynomial damping coefficients a, where force due to damping is "
    "computed as: a_0 + a_1 * \\dot{x}^1 + a_2 * \\dot{x}^2 + ... + a_n * "
    "\\dot{x}^n\\",
    false)
    .set_row_dim(-1);

  DeclareParameter("stiffness_coefficients", &stiffness_coefficients_,
               "specify polynomial stiffness coefficients a, where force due "
               "to joint position is computed as: "
               "a_0 + a_1 * x^1 + a_2 * x^2 + ... + a_n * x^n\\",
               false)
    .set_row_dim(-1);
}

shift::core::joint::Prismatic::AxisType shift::core::joint::Prismatic::get_axis_type() {
  ksl_vec3_t x_axis = ksl_vec3(1.0, 0.0, 0.0);
  ksl_vec3_t y_axis = ksl_vec3(0.0, 1.0, 0.0);
  ksl_vec3_t z_axis = ksl_vec3(0.0, 0.0, 1.0);
  ksl_vec3_normalize(&axis_);
  if(std::abs(ksl_dot_vv(&x_axis, &axis_) - 1.0) < 1.0e-9) {
    return X_AXIS;
  } else if(std::abs(ksl_dot_vv(&y_axis, &axis_) - 1.0) < 1.0e-9) {
    return Y_AXIS;
  } else if(std::abs(ksl_dot_vv(&z_axis, &axis_) - 1.0) < 1.0e-9) {
    return Z_AXIS;
  } else {
    return ARBITRARY_AXIS;
  }
}

const double* shift::core::joint::Prismatic::get_data() {
  if(get_axis_type() == ARBITRARY_AXIS) {
    return (double*)&axis_;
  };
  return nullptr;
}

unsigned int shift::core::joint::Prismatic::get_data_count() {
  if(get_axis_type() == ARBITRARY_AXIS) {
    return 3;
  };
  return 0;
}

shift::core::topology::dh_fptr shift::core::joint::Prismatic::Dfptr() {
  AxisType axis = get_axis_type();
  switch(axis) {
    case X_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::tx_d;
      } else {
        return shift::core::joint::txinv_d;
      }
    }
    case Y_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::ty_d;
      } else {
        return shift::core::joint::tyinv_d;
      }
    }
    case Z_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::tz_d;
      } else {
        return shift::core::joint::tzinv_d;
      }
    }
    case ARBITRARY_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::ta_d;
      } else {
        return shift::core::joint::tainv_d;
      }
    }
  }
  return nullptr;
}

shift::core::topology::dh_fptr shift::core::joint::Prismatic::DHfptr() {
  AxisType axis = get_axis_type();
  switch(axis) {
    case X_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::tx_dh;
      } else {
        return shift::core::joint::txinv_dh;
      }
    }
    case Y_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::ty_dh;
      } else {
        return shift::core::joint::tyinv_dh;
      }
    }
    case Z_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::tz_dh;
      } else {
        return shift::core::joint::tzinv_dh;
      }
    }
    case ARBITRARY_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::ta_dh;
      } else {
        return shift::core::joint::tainv_dh;
      }
    }
  }
  return nullptr;
}

shift::core::topology::dhva_fptr shift::core::joint::Prismatic::DHVAfptr() {
  AxisType axis = get_axis_type();
  switch(axis) {
    case X_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::tx_dhva;
      } else {
        return shift::core::joint::txinv_dhva;
      }
    }
    case Y_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::ty_dhva;
      } else {
        return shift::core::joint::tyinv_dhva;
      }
    }
    case Z_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::tz_dhva;
      } else {
        return shift::core::joint::tzinv_dhva;
      }
    }
    case ARBITRARY_AXIS: {
      if(sense_ >= 0) {
        return shift::core::joint::ta_dhva;
      } else {
        return shift::core::joint::tainv_dhva;
      }
    }
  }
  return nullptr;
}

void shift::core::joint::Prismatic::Service(const std::vector<double>& y) {

  auto& f = context_->get_joint_working_forces();

  // stiffness
  const double q = y[q_index_];
  for(unsigned int i = 0; i < stiffness_coefficients_.size(); i++) {
    // f[dq_index_] -=
    //   stiffness_coefficients_[i] * copysign(fabs(std::pow(q, i)), q);

    f[dq_index_] -= stiffness_coefficients_[i] * pow(q, i);
  }

  // std::cout << "q: " << q << ", f: " << f[dq_index_] << std::endl;

  // damping
  const double dq = y[context_->get_q_count() + dq_index_];
  for(unsigned int i = 0; i < damping_coefficients_.size(); i++) {
    // f[dq_index_] -=
    //   damping_coefficients_[i] * copysign(fabs(std::pow(dq, i)), dq);
    f[dq_index_] -= damping_coefficients_[i] * pow(dq, i);
  }

  // std::cout << "dq: " << dq << ", f: " << f[dq_index_] << std::endl;
}
