#include "shift-core/joint/rigid.h"
#include "shift-core/context.h"
#include "shift-core/factory.h"

bool Shift::Joint::Rigid::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "rigid", Shift::Joint::Rigid::create);

std::unique_ptr<Shift::Joint::Rigid> Shift::Joint::Rigid::create() {
  return std::make_unique<Shift::Joint::Rigid>();
}

/*!
@brief Rigid Joint (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rigid_dh(const Shift::Context& context,
                            Shift::Joint::DHVA& dhvaPointers,
                            const int* __restrict__ dhvaOffsets,
                            const double* __restrict__) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  const ksl_SE3_t* dPrimitive = context.getDWRTReference() + dhvaOffsets[1];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[1];

  ksl_product_dd(dParent, dPrimitive, dChild);
}

/*!
@brief Rigid Joint (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rigid_dhva(const Shift::Context& context,
                              Shift::Joint::DHVA& dhvaPointers,
                              const int* __restrict__ dhvaOffsets,
                              const double* __restrict__ q,
                              const double* __restrict__) {

  Shift::Joint::rigid_dh(context, dhvaPointers, dhvaOffsets, q);
}

/*!
@brief Rigid Joint - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rigidinv_dh(const Shift::Context& context,
                               Shift::Joint::DHVA& dhvaPointers,
                               const int* __restrict__ dhvaOffsets,
                               const double* __restrict__) {
  ksl_SE3_t* dParent = dhvaPointers.d + dhvaOffsets[0];
  const ksl_SE3_t* dPrimitive = context.getDWRTReference() + dhvaOffsets[1];
  ksl_SE3_t* dChild = dhvaPointers.d + dhvaOffsets[2];

  ksl_product_ddinv(dParent, dPrimitive, dChild);
}

/*!
@brief Rigid Joint - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rigidinv_dhva(const Shift::Context& context,
                                 Shift::Joint::DHVA& dhvaPointers,
                                 const int* __restrict__ dhvaOffsets,
                                 const double* __restrict__ q,
                                 const double* __restrict__) {
  Shift::Joint::rigidinv_dh(context, dhvaPointers, dhvaOffsets, q);
}

void Shift::Joint::Rigid::addKeys() {
  Shift::Joint::Base::addKeys();
}

void Shift::Joint::Rigid::updateD() {}

void Shift::Joint::Rigid::updateDH() {}

void Shift::Joint::Rigid::updateDHVA() {}
