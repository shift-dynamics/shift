#include "shift-core/joint/prismatic.h"
#include "shift-core/factory.h"

#include "ksl/matrix.h"

bool Shift::Joint::Prismatic::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "prismatic", Shift::Joint::Prismatic::create);

std::unique_ptr<Shift::Joint::Prismatic> Shift::Joint::Prismatic::create() {
  return std::make_unique<Shift::Joint::Prismatic>();
}

/*!
@brief Prismatic joint along x-axis (dh)

Updates displacement and influence coefficient matrices

    ksl_SE3_t* dParent = dhva[0];
    double* dPrimitive = dhva[1];
    ksl_SE3_t* dChild = dhva[2];
    ksl_screw_t* h = dhva[3];

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::tx_dh(const Shift::Context&,
                         Shift::Joint::DHVA& dhvaPointers,
                         const int* __restrict__ dhvaOffsets,
                         const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  const double* dPrimitive = q + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

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
void tx_dhva(const Shift::Context& context, Shift::Joint::DHVA& dhvaPointers,
             const int* __restrict__ dhvaOffsets, const double* __restrict__ q,
             const double* __restrict__ dq) {
  Shift::Joint::tx_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

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
void Shift::Joint::txinv_dh(const Shift::Context&,
                            Shift::Joint::DHVA& dhvaPointers,
                            const int* __restrict__ dhvaOffsets,
                            const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  const double* dPrimitive = q + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

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
                              Shift::Joint::DHVA& dhvaPointers,
                              const int* __restrict__ dhvaOffsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::txinv_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

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
void Shift::Joint::ty_dh(const Shift::Context&,
                         Shift::Joint::DHVA& dhvaPointers,
                         const int* __restrict__ dhvaOffsets,
                         const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  const double* dPrimitive = q + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

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
                           Shift::Joint::DHVA& dhvaPointers,
                           const int* __restrict__ dhvaOffsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {

  Shift::Joint::ty_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

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
void Shift::Joint::tyinv_dh(const Shift::Context&,
                            Shift::Joint::DHVA& dhvaPointers,
                            const int* __restrict__ dhvaOffsets,
                            const double* __restrict__ q) {

  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  const double* dPrimitive = q + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

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
                              Shift::Joint::DHVA& dhvaPointers,
                              const int* __restrict__ dhvaOffsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::tyinv_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

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
void Shift::Joint::tz_dh(const Shift::Context&,
                         Shift::Joint::DHVA& dhvaPointers,
                         const int* __restrict__ dhvaOffsets,
                         const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  const double* dPrimitive = q + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

  ksl_product_ddtz(dParent, *dPrimitive, dChild);
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
                           Shift::Joint::DHVA& dhvaPointers,
                           const int* __restrict__ dhvaOffsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {

  Shift::Joint::tz_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

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
void Shift::Joint::tzinv_dh(const Shift::Context&,
                            Shift::Joint::DHVA& dhvaPointers,
                            const int* __restrict__ dhvaOffsets,
                            const double* __restrict__ q) {

  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];
  const double* dPrimitive = q + dhvaOffsets[2];
  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];

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
                              Shift::Joint::DHVA& dhvaPointers,
                              const int* __restrict__ dhvaOffsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::tzinv_dh(context, dhvaPointers, dhvaOffsets, q);

  ksl_screw_t* h = dhvaPointers.h + dhvaOffsets[3];
  const double* vPrimitive = dq + dhvaOffsets[3];
  ksl_screw_t* vParent = dhvaPointers.v + dhvaOffsets[4];
  ksl_screw_t vRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* vChild = dhvaPointers.v + dhvaOffsets[5];
  ksl_screw_t* aqParent = dhvaPointers.aq + dhvaOffsets[4];
  ksl_screw_t aqRelative = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ksl_screw_t* aqChild = dhvaPointers.aq + dhvaOffsets[5];

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_sst(vParent, &vRelative, vChild);
  ksl_cross_sst(vParent, &vRelative, &aqRelative);
  ksl_add_sst(aqParent, &aqRelative, aqChild);
}

void Shift::Joint::Prismatic::addKeys() {
  Shift::Joint::Base::addKeys();
}

void Shift::Joint::Prismatic::updateD() {}

void Shift::Joint::Prismatic::updateDH() {}

void Shift::Joint::Prismatic::updateDHVA() {}
