#include "shift/core/joint/Rigid.h"
#include "shift/core/Context.h"
#include "shift/core/Factory.h"

bool shift::core::joint::Rigid::registered_ =
  shift::core::Factory<shift::core::joint::Base>::get()->RegisterModule(
    "rigid", shift::core::joint::Rigid::Create);

std::unique_ptr<shift::core::joint::Rigid> shift::core::joint::Rigid::Create() {
  return std::make_unique<shift::core::joint::Rigid>();
}

/*!
@brief Rigid Joint (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rigid_dh(const shift::core::Context& context,
                            shift::core::topology::DHVA& dhva,
                            const shift::core::topology::Offsets& offsets,
                            const double* __restrict__) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  const ksl_SE3_t* d_primitive = &context.get_d_wrt_reference()[offsets.q];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];

  ksl_product_dd(d_parent, d_primitive, d_child);
}

/*!
@brief Rigid Joint (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rigid_dhva(const shift::core::Context& context,
                              shift::core::topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__) {

  shift::core::joint::rigid_dh(context, dhva, offsets, q);
}

/*!
@brief Rigid Joint - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rigidinv_dh(const shift::core::Context& context,
                               shift::core::topology::DHVA& dhva,
                               const shift::core::topology::Offsets& offsets,
                               const double* __restrict__) {
  ksl_SE3_t* d_parent = &dhva.d[offsets.parent.d];
  const ksl_SE3_t* d_primitive = &context.get_d_wrt_reference()[offsets.q];
  ksl_SE3_t* d_child = &dhva.d[offsets.child.d];

  ksl_product_ddinv(d_parent, d_primitive, d_child);
}

/*!
@brief Rigid Joint - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rigidinv_dhva(const shift::core::Context& context,
                                 shift::core::topology::DHVA& dhva,
                                 const shift::core::topology::Offsets& offsets,
                                 const double* __restrict__ q,
                                 const double* __restrict__) {
  shift::core::joint::rigidinv_dh(context, dhva, offsets, q);
}

void shift::core::joint::Rigid::DeclareParameters() {}

shift::core::topology::dh_fptr shift::core::joint::Rigid::Dfptr() {
  if(sense_ >= 0) {
    return rigid_dh;
  } else {
    return rigidinv_dh;
  }
}

shift::core::topology::dh_fptr shift::core::joint::Rigid::DHfptr() {
  if(sense_ >= 0) {
    return rigid_dh;
  } else {
    return rigidinv_dh;
  }
}

shift::core::topology::dhva_fptr shift::core::joint::Rigid::DHVAfptr() {
  if(sense_ >= 0) {
    return rigid_dhva;
  } else {
    return rigidinv_dhva;
  }
}
