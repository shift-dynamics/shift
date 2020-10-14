#include "shift/core/joint/prismatic.h"
#include "shift/core/context.h"
#include "shift/core/factory.h"

#include "ksl/matrix.h"
#include "ksl/print.h"

#include <math.h>

bool shift::core::joint::Prismatic::registered_ =
  shift::core::Factory<shift::core::joint::Base>::get()->registerModule(
    "prismatic", shift::core::joint::Prismatic::create);

std::unique_ptr<shift::core::joint::Prismatic> shift::core::joint::Prismatic::create() {
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_product_ddtx(dParent, *dPrimitive, dChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_product_ddtxinv(dParent, *dPrimitive, dChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_product_ddty(dParent, *dPrimitive, dChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_product_ddtyinv(dParent, *dPrimitive, dChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_product_ddtz(dParent, *dPrimitive, dChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_product_ddtzinv(dParent, *dPrimitive, dChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.jointData()[offsets.data]);
  ksl_vec3_t temp;
  ksl_product_av(*dPrimitive, axis, &temp);
  ksl_product_ddt(dParent, &temp, dChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.jointData()[offsets.data]);
  ksl_vec3_t temp;
  ksl_product_av(*dPrimitive, axis, &temp);
  ksl_product_ddtinv(dParent, &temp, dChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddtx(dParent, *dPrimitive, dChild);
  ksl_hstx(dChild, h);
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

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_sst(vParent, &vRelative, vChild);
  ksl_cross_sst(vParent, &vRelative, &aqRelative);
  ksl_add_sst(aqParent, &aqRelative, aqChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddtxinv(dParent, *dPrimitive, dChild);
  ksl_hstxinv(dChild, h);
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

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_sst(vParent, &vRelative, vChild);
  ksl_cross_sst(vParent, &vRelative, &aqRelative);
  ksl_add_sst(aqParent, &aqRelative, aqChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddty(dParent, *dPrimitive, dChild);
  ksl_hsty(dChild, h);
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

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_sst(vParent, &vRelative, vChild);
  ksl_cross_sst(vParent, &vRelative, &aqRelative);
  ksl_add_sst(aqParent, &aqRelative, aqChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddtyinv(dParent, *dPrimitive, dChild);
  ksl_hstyinv(dChild, h);
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

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_sst(vParent, &vRelative, vChild);
  ksl_cross_sst(vParent, &vRelative, &aqRelative);
  ksl_add_sst(aqParent, &aqRelative, aqChild);
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

  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;

  const double* dPrimitive = q + offsets.q;

  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddtz(dParent, *dPrimitive, dChild);

  ksl_hstz(dChild, h);

#if 0
  std::cout << "tz_dh" << std::endl;
  ksl_SE3_print(stdout, dParent, "dParent: ");
  ksl_SE3_print(stdout, dChild, "dChild: ");
  std::cout << "dPrimitive: " << *dPrimitive << std::endl;
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

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_sst(vParent, &vRelative, vChild);
  ksl_cross_sst(vParent, &vRelative, &aqRelative);
  ksl_add_sst(aqParent, &aqRelative, aqChild);

#if 0
  std::cout << "vPrimitive: " << *vPrimitive << std::endl;
  std::cout << "tz_dhva" << std::endl;
  ksl_screw_print(stdout, h, "h: ");
  ksl_screw_print(stdout, vParent, "vParent: ");
  ksl_screw_print(stdout, vChild, "vChild: ");
  ksl_screw_print(stdout, aqParent, "aqParent: ");
  ksl_screw_print(stdout, aqChild, "aqChild: ");
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddtzinv(dParent, *dPrimitive, dChild);
  ksl_hstzinv(dChild, h);
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

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_sst(vParent, &vRelative, vChild);
  ksl_cross_sst(vParent, &vRelative, &aqRelative);
  ksl_add_sst(aqParent, &aqRelative, aqChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_screw_t* h = dhva.h + offsets.dq;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.jointData()[offsets.data]);
  ksl_vec3_t temp;
  ksl_product_av(*dPrimitive, axis, &temp);
  ksl_product_ddt(dParent, &temp, dChild);
  ksl_hsta(dChild, axis, h);
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

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_sst(vParent, &vRelative, vChild);
  ksl_cross_sst(vParent, &vRelative, &aqRelative);
  ksl_add_sst(aqParent, &aqRelative, aqChild);
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
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_screw_t* h = dhva.h + offsets.dq;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.jointData()[offsets.data]);
  ksl_vec3_t temp;
  ksl_product_av(*dPrimitive, axis, &temp);
  ksl_product_ddtinv(dParent, &temp, dChild);
  ksl_hstainv(dChild, axis, h);
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

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_sst(vParent, &vRelative, vChild);
  ksl_cross_sst(vParent, &vRelative, &aqRelative);
  ksl_add_sst(aqParent, &aqRelative, aqChild);
}

void shift::core::joint::Prismatic::addParameters() {

  addParameter("axis", &axis_,
               "Translation axis expressed in parent body frame, "
               "defaults to z-axis if no axis provided.",
               false);

  addParameter(
    "initial_displacement", q_0_,
    "Initial joint displacement, defaults to zero if no value provided.",
    false);

  addParameter("initial_velocity", dq_0_,
               "Initial joint velocity, defaults to zero if no value provided.",
               false);

  addParameter(
    "damping_coefficients", &damping_coefficients_,
    "specify  polynomial damping coefficients a, where force due to damping is "
    "computed as: a_0 + a_1 * \\dot{x}^1 + a_2 * \\dot{x}^2 + ... + a_n * "
    "\\dot{x}^n\\",
    false)
    .setRowDim(-1);

  addParameter("stiffness_coefficients", &stiffness_coefficients_,
               "specify polynomial stiffness coefficients a, where force due "
               "to joint position is computed as: "
               "a_0 + a_1 * x^1 + a_2 * x^2 + ... + a_n * x^n\\",
               false)
    .setRowDim(-1);
}

shift::core::joint::Prismatic::AxisType shift::core::joint::Prismatic::axisType() {
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

const double* shift::core::joint::Prismatic::data() {
  if(axisType() == ARBITRARY_AXIS) {
    return (double*)&axis_;
  };
  return nullptr;
}

unsigned int shift::core::joint::Prismatic::dataCount() {
  if(axisType() == ARBITRARY_AXIS) {
    return 3;
  };
  return 0;
}

shift::core::topology::dh_fptr shift::core::joint::Prismatic::Dfptr() {
  AxisType axis = axisType();
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
  AxisType axis = axisType();
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
  AxisType axis = axisType();
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

void shift::core::joint::Prismatic::service(const std::vector<double>& y) {

  auto& f = context_->jointWorkingForces();

  // stiffness
  const double q = y[q_index_];
  for(unsigned int i = 0; i < stiffness_coefficients_.size(); i++) {
    // f[dq_index_] -=
    //   stiffness_coefficients_[i] * copysign(fabs(std::pow(q, i)), q);

    f[dq_index_] -= stiffness_coefficients_[i] * std::pow(q, i);
  }

  std::cout << "q: " << q << ", f: " << f[dq_index_] << std::endl;

  // damping
  const double dq = y[context_->qCount() + dq_index_];
  for(unsigned int i = 0; i < damping_coefficients_.size(); i++) {
    // f[dq_index_] -=
    //   damping_coefficients_[i] * copysign(fabs(std::pow(dq, i)), dq);
    f[dq_index_] -= damping_coefficients_[i] * std::pow(dq, i);
  }

  // std::cout << "dq: " << dq << ", f: " << f[dq_index_] << std::endl;
}
