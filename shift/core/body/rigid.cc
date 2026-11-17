#include "shift/core/body/rigid.h"
#include "shift/core/factory.h"

#include "ksl/matrix.h"

bool shift::core::body::Rigid::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "rigid", shift::core::body::Rigid::Create);

void shift::core::body::Rigid::DeclareParameters() {
  shift::core::body::Base::DeclareParameters();
}

std::unique_ptr<shift::core::body::Rigid> shift::core::body::Rigid::Create() {
  return std::make_unique<shift::core::body::Rigid>();
}

ksl_inertia_t shift::core::body::Rigid::get_inertia_wrt_reference() {
  ksl_inertia_t inertia_temp{};

  if(!(inertia_.m < 1e-9 || is_virtual_)) {
    /* the reference frame (r) in the origin's coordinates (o) is D_{or}; the
    centroidal moments, in the origin's axes, are turned into the reference
    frame's by R^{ro} = (R^{or})^T, and the first moment is the mass times
    the center of mass in the reference frame's coordinates,
    D_{ro} t_{oc}^o */
    ksl_mat3x3_t r_ro;
    ksl_mat3x3_transposed(&get_reference_frame()->get_d_wrt_origin()->R,
                          &r_ro);
    memcpy(&inertia_temp, &inertia_, sizeof(ksl_inertia_t));
    ksl_inertia_rotated(&inertia_, &r_ro, &inertia_temp);
    const ksl_vec3_t t_rc_r = get_center_of_mass_wrt_reference();
    ksl_product_av(inertia_temp.m, &t_rc_r, &(inertia_temp.mt));
  }
  return inertia_temp;
}
