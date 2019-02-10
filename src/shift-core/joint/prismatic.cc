#include "shift-core/joint/prismatic.h"
#include "shift-core/context.h"
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
void Shift::Joint::tx_dh(const Shift::Context&, Shift::Joint::DHVA& dhva,
                         const int* __restrict__ offsets,
                         const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhva.d + offsets[0];
  ksl_SE3_t* dChild = dhva.d + offsets[1];
  const double* dPrimitive = q + offsets[2];
  ksl_screw_t* h = dhva.h + offsets[3];

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
                           Shift::Joint::DHVA& dhva,
                           const int* __restrict__ offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  Shift::Joint::tx_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets[3];
  const double* vPrimitive = dq + offsets[3];
  ksl_screw_t* vParent = dhva.v + offsets[4];
  ksl_screw_t vRelative = {ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
  ksl_screw_t* vChild = dhva.v + offsets[5];
  ksl_screw_t* aqParent = dhva.aq + offsets[4];
  ksl_screw_t aqRelative = {ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
  ksl_screw_t* aqChild = dhva.aq + offsets[5];

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
void Shift::Joint::txinv_dh(const Shift::Context&, Shift::Joint::DHVA& dhva,
                            const int* __restrict__ offsets,
                            const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhva.d + offsets[0];
  ksl_SE3_t* dChild = dhva.d + offsets[1];
  const double* dPrimitive = q + offsets[2];
  ksl_screw_t* h = dhva.h + offsets[3];

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
                              Shift::Joint::DHVA& dhva,
                              const int* __restrict__ offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::txinv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets[3];
  const double* vPrimitive = dq + offsets[3];
  ksl_screw_t* vParent = dhva.v + offsets[4];
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets[5];
  ksl_screw_t* aqParent = dhva.aq + offsets[4];
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets[5];

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
void Shift::Joint::ty_dh(const Shift::Context&, Shift::Joint::DHVA& dhva,
                         const int* __restrict__ offsets,
                         const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhva.d + offsets[0];
  ksl_SE3_t* dChild = dhva.d + offsets[1];
  const double* dPrimitive = q + offsets[2];
  ksl_screw_t* h = dhva.h + offsets[3];

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
                           Shift::Joint::DHVA& dhva,
                           const int* __restrict__ offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {

  Shift::Joint::ty_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets[3];
  const double* vPrimitive = dq + offsets[3];
  ksl_screw_t* vParent = dhva.v + offsets[4];
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets[5];
  ksl_screw_t* aqParent = dhva.aq + offsets[4];
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets[5];

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
void Shift::Joint::tyinv_dh(const Shift::Context&, Shift::Joint::DHVA& dhva,
                            const int* __restrict__ offsets,
                            const double* __restrict__ q) {

  ksl_SE3_t* dParent = dhva.d + offsets[0];
  ksl_SE3_t* dChild = dhva.d + offsets[1];
  const double* dPrimitive = q + offsets[2];
  ksl_screw_t* h = dhva.h + offsets[3];

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
                              Shift::Joint::DHVA& dhva,
                              const int* __restrict__ offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::tyinv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets[3];
  const double* vPrimitive = dq + offsets[3];
  ksl_screw_t* vParent = dhva.v + offsets[4];
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets[5];
  ksl_screw_t* aqParent = dhva.aq + offsets[4];
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets[5];

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
void Shift::Joint::tz_dh(const Shift::Context&, Shift::Joint::DHVA& dhva,
                         const int* __restrict__ offsets,
                         const double* __restrict__ q) {
  ksl_SE3_t* dParent = dhva.d + offsets[0];
  ksl_SE3_t* dChild = dhva.d + offsets[1];
  const double* dPrimitive = q + offsets[2];
  ksl_screw_t* h = dhva.h + offsets[3];

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
                           Shift::Joint::DHVA& dhva,
                           const int* __restrict__ offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {

  Shift::Joint::tz_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets[3];
  const double* vPrimitive = dq + offsets[3];
  ksl_screw_t* vParent = dhva.v + offsets[4];
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets[5];
  ksl_screw_t* aqParent = dhva.aq + offsets[4];
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets[5];

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
void Shift::Joint::tzinv_dh(const Shift::Context&, Shift::Joint::DHVA& dhva,
                            const int* __restrict__ offsets,
                            const double* __restrict__ q) {

  ksl_SE3_t* dParent = dhva.d + offsets[0];
  ksl_SE3_t* dChild = dhva.d + offsets[1];
  const double* dPrimitive = q + offsets[2];
  ksl_screw_t* h = dhva.h + offsets[3];

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
                              Shift::Joint::DHVA& dhva,
                              const int* __restrict__ offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::tzinv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets[3];
  const double* vPrimitive = dq + offsets[3];
  ksl_screw_t* vParent = dhva.v + offsets[4];
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets[5];
  ksl_screw_t* aqParent = dhva.aq + offsets[4];
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets[5];

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
                         Shift::Joint::DHVA& dhva,
                         const int* __restrict__ offsets,
                         const double* __restrict__ q) {

  ksl_SE3_t* dParent = dhva.d + offsets[0];
  ksl_SE3_t* dChild = dhva.d + offsets[1];
  const double* dPrimitive = q + offsets[2];
  ksl_screw_t* h = dhva.h + offsets[3];
  const ksl_vec3_t* axis = (ksl_vec3_t*)(context.getJointData() + offsets[6]);
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
                           Shift::Joint::DHVA& dhva,
                           const int* __restrict__ offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {

  Shift::Joint::tz_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets[3];
  const double* vPrimitive = dq + offsets[3];
  ksl_screw_t* vParent = dhva.v + offsets[4];
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets[5];
  ksl_screw_t* aqParent = dhva.aq + offsets[4];
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets[5];

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
                            Shift::Joint::DHVA& dhva,
                            const int* __restrict__ offsets,
                            const double* __restrict__ q) {

  ksl_SE3_t* dParent = dhva.d + offsets[0];
  ksl_SE3_t* dChild = dhva.d + offsets[1];
  const double* dPrimitive = q + offsets[2];
  ksl_screw_t* h = dhva.h + offsets[3];

  const ksl_vec3_t* axis = (ksl_vec3_t*)(context.getJointData() + offsets[6]);
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
                              Shift::Joint::DHVA& dhva,
                              const int* __restrict__ offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::tzinv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets[3];
  const double* vPrimitive = dq + offsets[3];
  ksl_screw_t* vParent = dhva.v + offsets[4];
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets[5];
  ksl_screw_t* aqParent = dhva.aq + offsets[4];
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets[5];

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_sst(vParent, &vRelative, vChild);
  ksl_cross_sst(vParent, &vRelative, &aqRelative);
  ksl_add_sst(aqParent, &aqRelative, aqChild);
}

void Shift::Joint::Prismatic::addKeys() {
  Shift::Joint::Base::addKeys();

  SerialEntry axis("axis", SerialType::KSL_VEC3, &axis_);
  axis.setRequired(false);
  axis.setDescription(
    "Axis of translation expressed in parent body frame, defaults to z-axis if "
    "no axis is provided.");
  addSerialEntry(axis);

  SerialEntry q0("initial_displacement", SerialType::DOUBLE, &q_0_);
  q0.setRequired(false);
  q0.setDescription("Initial joint displacement.");
  addSerialEntry(q0);

  SerialEntry dq0("initial_velocity", SerialType::DOUBLE, &dq_0_);
  dq0.setRequired(false);
  dq0.setDescription("Initial joint velocity.");
  addSerialEntry(dq0);
}

Shift::Joint::Prismatic::AxisType Shift::Joint::Prismatic::getAxisType() {
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

const double* Shift::Joint::Prismatic::getData() {
  if(getAxisType() == ARBITRARY_AXIS) {
    return (double*)&axis_;
  };
  return NULL;
}

unsigned int Shift::Joint::Prismatic::getDataCount() {
  if(getAxisType() == ARBITRARY_AXIS) {
    return 3;
  };
  return 0;
}

Shift::Joint::dh_fptr Shift::Joint::Prismatic::getDH() {
  AxisType axis = getAxisType();
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

Shift::Joint::dhva_fptr Shift::Joint::Prismatic::getDHVA() {
  AxisType axis = getAxisType();
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
