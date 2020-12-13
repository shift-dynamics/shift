#include "shift/core/body/Rigid.h"
#include "shift/core/Factory.h"

#include "ksl/matrix.h"

bool shift::core::body::Rigid::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->registerModule(
    "rigid", shift::core::body::Rigid::create);

void shift::core::body::Rigid::addParameters() {
  shift::core::body::Base::addParameters();
}

std::unique_ptr<shift::core::body::Rigid> shift::core::body::Rigid::create() {
  return std::make_unique<shift::core::body::Rigid>();
}

ksl_inertia_t shift::core::body::Rigid::inertiaWRTReference() {
  ksl_inertia_t inertia_temp{};

  if(!(inertia_.m < 1e-9 || is_virtual_)) {
    ksl_SE3_t d_rc;
    ksl_SE3_t d_cr;
    /* compute displacement from center of mass (c) to reference frame (r)
    D_{ro} * (D(t)_{co})^-1 -> D_{rc} */
    ksl_product_ddtinv(referenceFrame()->dWRTOrigin(),
                       &center_of_mass_wrt_origin_, &d_rc);
    /* find displacement from reference frame (r) to center of mass (c)
      D_{rc}^-1 -> D_{cr} */
    ksl_SE3_inverted(&d_rc, &d_cr);
#if 0
    printf("  [%i] ", i);
    ksl_inertia_print(stdout, body->inertia);
#endif

    memcpy(&inertia_temp, &inertia_, sizeof(ksl_inertia_t));
    ksl_inertia_rotated(&inertia_, &(referenceFrame()->dWRTOrigin()->R),
                        &inertia_temp);
    ksl_product_av(inertia_temp.m, &(d_cr.t), &(inertia_temp.mt));
  }
  return inertia_temp;
}
