#include "shift-core/joint/prismatic.h"
#include "shift-core/context.h"
#include "shift-core/factory.h"

#include "ksl/matrix.h"
#include "ksl/print.h"

bool Shift::Joint::Prismatic::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "prismatic", Shift::Joint::Prismatic::create);

std::unique_ptr<Shift::Joint::Prismatic> Shift::Joint::Prismatic::create() {
  return std::make_unique<Shift::Joint::Prismatic>();
}

/*!
@brief Prismatic joint along x-axis (dh)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::tx_d(const Shift::Context&, Shift::Topology::DHVA& dhva,
                        const Shift::Topology::Offsets& offsets,
                        const double* __restrict__ q) {
  std::cout << "tx_d" << std::endl;
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
void Shift::Joint::txinv_d(const Shift::Context&, Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__ q) {
  std::cout << "tx_d" << std::endl;
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
void Shift::Joint::ty_d(const Shift::Context&, Shift::Topology::DHVA& dhva,
                        const Shift::Topology::Offsets& offsets,
                        const double* __restrict__ q) {
  std::cout << "ty_d" << std::endl;
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
void Shift::Joint::tyinv_d(const Shift::Context&, Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__ q) {
  std::cout << "tyinv_d" << std::endl;
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
void Shift::Joint::tz_d(const Shift::Context&, Shift::Topology::DHVA& dhva,
                        const Shift::Topology::Offsets& offsets,
                        const double* __restrict__ q) {
  std::cout << "tz_d" << std::endl;
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
void Shift::Joint::tzinv_d(const Shift::Context&, Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__ q) {
  std::cout << "tzinv_d" << std::endl;
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
void Shift::Joint::ta_d(const Shift::Context& context,
                        Shift::Topology::DHVA& dhva,
                        const Shift::Topology::Offsets& offsets,
                        const double* __restrict__ q) {
  std::cout << "ta_d" << std::endl;
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(context.jointData() + offsets.data);
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
void Shift::Joint::tainv_d(const Shift::Context& context,
                           Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__ q) {
  std::cout << "tainv_d" << std::endl;
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(context.jointData() + offsets.data);
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
void Shift::Joint::tx_dh(const Shift::Context&, Shift::Topology::DHVA& dhva,
                         const Shift::Topology::Offsets& offsets,
                         const double* __restrict__ q) {
  std::cout << "tx_dh" << std::endl;
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
void Shift::Joint::tx_dhva(const Shift::Context& context,
                           Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  std::cout << "tx_dhva, ";
  Shift::Joint::tx_dh(context, dhva, offsets, q);

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
void Shift::Joint::txinv_dh(const Shift::Context&, Shift::Topology::DHVA& dhva,
                            const Shift::Topology::Offsets& offsets,
                            const double* __restrict__ q) {
  std::cout << "txinv_dh" << std::endl;
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
void Shift::Joint::txinv_dhva(const Shift::Context& context,
                              Shift::Topology::DHVA& dhva,
                              const Shift::Topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  std::cout << "txinv_dhva, ";
  Shift::Joint::txinv_dh(context, dhva, offsets, q);

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
void Shift::Joint::ty_dh(const Shift::Context&, Shift::Topology::DHVA& dhva,
                         const Shift::Topology::Offsets& offsets,
                         const double* __restrict__ q) {
  std::cout << "ty_dh" << std::endl;
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
void Shift::Joint::ty_dhva(const Shift::Context& context,
                           Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  std::cout << "ty_dhva, ";
  Shift::Joint::ty_dh(context, dhva, offsets, q);

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
void Shift::Joint::tyinv_dh(const Shift::Context&, Shift::Topology::DHVA& dhva,
                            const Topology::Offsets& offsets,
                            const double* __restrict__ q) {
  std::cout << "tyinv_dh" << std::endl;
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
void Shift::Joint::tyinv_dhva(const Shift::Context& context,
                              Topology::DHVA& dhva,
                              const Shift::Topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  std::cout << "tyinv_dhva, ";

  Shift::Joint::tyinv_dh(context, dhva, offsets, q);

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
void Shift::Joint::tz_dh(const Shift::Context&, Shift::Topology::DHVA& dhva,
                         const Shift::Topology::Offsets& offsets,
                         const double* __restrict__ q) {
  std::cout << "tz_dh" << std::endl;

  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_screw_t* h = dhva.h + offsets.dq;
  ksl_product_ddtz(dParent, *dPrimitive, dChild);

  std::cout << "  qOffset: " << offsets.q << std::endl;
  std::cout << "  dPrimitive: " << *dPrimitive << std::endl;
  ksl_SE3_print(stdout, dhva.d + offsets.parent.d, "  parent: ");
  ksl_SE3_print(stdout, dhva.d + offsets.child.d, "  child: ");

  ksl_hstz(dChild, h);
}

/*!
@brief Prismatic joint along z-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::tz_dhva(const Shift::Context& context,
                           Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  std::cout << "tz_dhva, ";

  Shift::Joint::tz_dh(context, dhva, offsets, q);

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
void Shift::Joint::tzinv_dh(const Shift::Context&, Shift::Topology::DHVA& dhva,
                            const Shift::Topology::Offsets& offsets,
                            const double* __restrict__ q) {
  std::cout << "tzinv_dh" << std::endl;

  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddtzinv(dParent, *dPrimitive, dChild);
  ksl_hstzinv(dChild, h);
}

/*!
@brief Prismatic joint along z-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::tzinv_dhva(const Shift::Context& context,
                              Shift::Topology::DHVA& dhva,
                              const Shift::Topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  std::cout << "tzinv_dhva, ";

  Shift::Joint::tzinv_dh(context, dhva, offsets, q);

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
void Shift::Joint::ta_dh(const Shift::Context& context,
                         Shift::Topology::DHVA& dhva,
                         const Shift::Topology::Offsets& offsets,
                         const double* __restrict__ q) {

  std::cout << "ta_dh" << std::endl;

  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_screw_t* h = dhva.h + offsets.dq;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(context.jointData() + offsets.data);
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
void Shift::Joint::ta_dhva(const Shift::Context& context,
                           Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {

  std::cout << "ta_dhva, ";

  Shift::Joint::tz_dh(context, dhva, offsets, q);

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
void Shift::Joint::tainv_dh(const Shift::Context& context,
                            Shift::Topology::DHVA& dhva,
                            const Shift::Topology::Offsets& offsets,
                            const double* __restrict__ q) {

  std::cout << "tainv_dh" << std::endl;

  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  const double* dPrimitive = q + offsets.q;
  ksl_screw_t* h = dhva.h + offsets.dq;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(context.jointData() + offsets.data);
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
void Shift::Joint::tainv_dhva(const Shift::Context& context,
                              Shift::Topology::DHVA& dhva,
                              const Shift::Topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  std::cout << "tainv_dhva, ";

  Shift::Joint::tzinv_dh(context, dhva, offsets, q);

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

void Shift::Joint::Prismatic::addKeys() {

  SerialEntry axis("axis", SerialType::KSL_VEC3, &axis_);
  axis.setRequired(false);
  axis.setDescription("Translation axis expressed in parent body frame, "
                      "defaults to z-axis if no axis provided.");
  addSerialEntry(axis);

  SerialEntry q0("initial_displacement", SerialType::DOUBLE, q_0_);
  q0.setRequired(false);
  q0.setDescription(
    "Initial joint displacement, defaults to zero if no value provided.");
  addSerialEntry(q0);

  SerialEntry dq0("initial_velocity", SerialType::DOUBLE, dq_0_);
  dq0.setRequired(false);
  dq0.setDescription(
    "Initial joint velocity, defaults to zero if no value provided.");
  addSerialEntry(dq0);
}

Shift::Joint::Prismatic::AxisType Shift::Joint::Prismatic::axisType() {
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

const double* Shift::Joint::Prismatic::data() {
  if(axisType() == ARBITRARY_AXIS) {
    return (double*)&axis_;
  };
  return NULL;
}

unsigned int Shift::Joint::Prismatic::dataCount() {
  if(axisType() == ARBITRARY_AXIS) {
    return 3;
  };
  return 0;
}

Shift::Topology::dh_fptr Shift::Joint::Prismatic::Dfptr() {
  AxisType axis = axisType();
  switch(axis) {
    case X_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::tx_d;
      } else {
        return Shift::Joint::txinv_d;
      }
    }
    case Y_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::ty_d;
      } else {
        return Shift::Joint::tyinv_d;
      }
    }
    case Z_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::tz_d;
      } else {
        return Shift::Joint::tzinv_d;
      }
    }
    case ARBITRARY_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::ta_d;
      } else {
        return Shift::Joint::tainv_d;
      }
    }
  }
  return NULL;
}

Shift::Topology::dh_fptr Shift::Joint::Prismatic::DHfptr() {
  AxisType axis = axisType();
  switch(axis) {
    case X_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::tx_dh;
      } else {
        return Shift::Joint::txinv_dh;
      }
    }
    case Y_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::ty_dh;
      } else {
        return Shift::Joint::tyinv_dh;
      }
    }
    case Z_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::tz_dh;
      } else {
        return Shift::Joint::tzinv_dh;
      }
    }
    case ARBITRARY_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::ta_dh;
      } else {
        return Shift::Joint::tainv_dh;
      }
    }
  }
  return NULL;
}

Shift::Topology::dhva_fptr Shift::Joint::Prismatic::DHVAfptr() {
  AxisType axis = axisType();
  switch(axis) {
    case X_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::tx_dhva;
      } else {
        return Shift::Joint::txinv_dhva;
      }
    }
    case Y_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::ty_dhva;
      } else {
        return Shift::Joint::tyinv_dhva;
      }
    }
    case Z_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::tz_dhva;
      } else {
        return Shift::Joint::tzinv_dhva;
      }
    }
    case ARBITRARY_AXIS: {
      if(sense_ >= 0) {
        return Shift::Joint::ta_dhva;
      } else {
        return Shift::Joint::tainv_dhva;
      }
    }
  }
  return NULL;
}
