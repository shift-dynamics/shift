#include "shift/core/joint/rigid.h"
#include "shift/core/context.h"
#include "shift/core/factory.h"

bool shift::Joint::Rigid::registered_ =
  shift::Factory<shift::Joint::Base>::get()->registerModule(
    "rigid", shift::Joint::Rigid::create);

std::unique_ptr<shift::Joint::Rigid> shift::Joint::Rigid::create() {
  return std::make_unique<shift::Joint::Rigid>();
}

/*!
@brief Rigid Joint (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::Joint::rigid_dh(const shift::Context& context,
                            shift::Topology::DHVA& dhva,
                            const shift::Topology::Offsets& offsets,
                            const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  const ksl_SE3_t* dPrimitive = &context.DWRTReference()[offsets.q];
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
void shift::Joint::rigid_dhva(const shift::Context& context,
                              shift::Topology::DHVA& dhva,
                              const shift::Topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__) {

  shift::Joint::rigid_dh(context, dhva, offsets, q);
}

/*!
@brief Rigid Joint - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::Joint::rigidinv_dh(const shift::Context& context,
                               shift::Topology::DHVA& dhva,
                               const shift::Topology::Offsets& offsets,
                               const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  const ksl_SE3_t* dPrimitive = &context.DWRTReference()[offsets.q];
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
void shift::Joint::rigidinv_dhva(const shift::Context& context,
                                 shift::Topology::DHVA& dhva,
                                 const shift::Topology::Offsets& offsets,
                                 const double* __restrict__ q,
                                 const double* __restrict__) {
  shift::Joint::rigidinv_dh(context, dhva, offsets, q);
}

void shift::Joint::Rigid::addParameters() {}

// void shift::Joint::Rigid::readJSON(Json::Value& value) {
//   shift::Joint::Base::readJSON(value);
//   // frame_.setNameRequired(false);
//   if(value["initial_displacement"]) {
//     frame_.readJSON(value["initial_displacement"]);
//   }
// }

shift::Topology::dh_fptr shift::Joint::Rigid::Dfptr() {
  if(sense_ >= 0) {
    return rigid_dh;
  } else {
    return rigidinv_dh;
  }
}

shift::Topology::dh_fptr shift::Joint::Rigid::DHfptr() {
  if(sense_ >= 0) {
    return rigid_dh;
  } else {
    return rigidinv_dh;
  }
}

shift::Topology::dhva_fptr shift::Joint::Rigid::DHVAfptr() {
  if(sense_ >= 0) {
    return rigid_dhva;
  } else {
    return rigidinv_dhva;
  }
}
