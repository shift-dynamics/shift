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
                            Shift::Joint::DHVA& dhva,
                            const Shift::Joint::Offsets& offsets,
                            const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  const ksl_SE3_t* dPrimitive = context.getDWRTReference() + offsets.q;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;

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
                              Shift::Joint::DHVA& dhva,
                              const Shift::Joint::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__) {

  Shift::Joint::rigid_dh(context, dhva, offsets, q);
}

/*!
@brief Rigid Joint - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rigidinv_dh(const Shift::Context& context,
                               Shift::Joint::DHVA& dhva,
                               const Shift::Joint::Offsets& offsets,
                               const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  const ksl_SE3_t* dPrimitive = context.getDWRTReference() + offsets.q;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;

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
                                 Shift::Joint::DHVA& dhva,
                                 const Shift::Joint::Offsets& offsets,
                                 const double* __restrict__ q,
                                 const double* __restrict__) {
  Shift::Joint::rigidinv_dh(context, dhva, offsets, q);
}

void Shift::Joint::Rigid::addKeys() {
  Shift::Joint::Base::addKeys();
}

void Shift::Joint::Rigid::readJSON(Json::Value& value) {
  Shift::Joint::Base::readJSON(value);
  frame_.setNameRequired(false);
  if(value["initial_displacement"]) {
    frame_.readJSON(value["initial_displacement"]);
  }
}

Shift::Joint::dh_fptr Shift::Joint::Rigid::getDH() {
  if(sense_ >= 0) {
    return rigid_dh;
  } else {
    return rigidinv_dh;
  }
}

Shift::Joint::dhva_fptr Shift::Joint::Rigid::getDHVA() {
  if(sense_ >= 0) {
    return rigid_dhva;
  } else {
    return rigidinv_dhva;
  }
}
