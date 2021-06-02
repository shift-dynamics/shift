#include "shift/core/joint/Revolute.h"
#include "shift/core/Context.h"
#include "shift/core/Factory.h"

#include "ksl/print.h"

#include <math.h>

bool shift::core::joint::Revolute::registered_ =
  shift::core::Factory<shift::core::joint::Base>::get()->RegisterModule(
    "revolute", shift::core::joint::Revolute::Create);

std::unique_ptr<shift::core::joint::Revolute> shift::core::joint::Revolute::Create() {
  return std::make_unique<shift::core::joint::Revolute>();
}

/*!
@brief Revolute Joint aligned with x-axis (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rx_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                        const shift::core::topology::Offsets& offsets,
                        const double* __restrict__) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;

  ksl_product_ddrx(d_parent, dc, d_child);
}

/*!
@brief Revolute Joint aligned with x-axis - inv (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rxinv_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_product_ddrxinv(d_parent, dc, d_child);
}

/*!
@brief Revolute Joint aligned with y-axis (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ry_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                        const shift::core::topology::Offsets& offsets,
                        const double* __restrict__) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_product_ddry(d_parent, dc, d_child);
}

/*!
@brief Revolute Joint aligned with y-axis - inv (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ryinv_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_product_ddryinv(d_parent, dc, d_child);
}

/*!
@brief Revolute Joint aligned with z-axis (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rz_d(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                        const shift::core::topology::Offsets& offsets,
                        const double* __restrict__) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_product_ddrz(d_parent, dc, d_child);
}

/*!
@brief Revolute Joint aligned with z-axis - inv (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rzinv_d(const shift::core::Context&, topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_product_ddrzinv(d_parent, dc, d_child);
}

/*!
@brief Revolute joint aligned with arbitrary axis (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ra_d(const shift::core::Context& context, topology::DHVA& dhva,
                        const shift::core::topology::Offsets& offsets,
                        const double*) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.get_joint_data()[offsets.data]);
  ksl_mat3x3_t temp;
  ksl_mat3x3_setFromAxisDC(&temp, axis, dc);
  ksl_product_ddr(d_parent, &temp, d_child);
  ksl_vec3_copy(&d_parent->t, &d_child->t);
}

/*!
@brief Revolute joint aligned with arbitrary axis - inverse (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rainv_d(const shift::core::Context& context, topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double*) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.get_joint_data()[offsets.data]);
  ksl_vec3_t neg_axis = ksl_vec3(-axis->x, -axis->y, -axis->z);
  ksl_mat3x3_t temp;
  ksl_mat3x3_setFromAxisDC(&temp, &neg_axis, dc);
  ksl_product_ddr(d_parent, &temp, d_child);
  ksl_vec3_copy(&d_parent->t, &d_child->t);
}

/*!
@brief Revolute Joint aligned with x-axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rx_dh(const shift::core::Context&, topology::DHVA& dhva,
                         const shift::core::topology::Offsets& offsets,
                         const double* __restrict__) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddrx(d_parent, dc, d_child);
  ksl_hsrx(d_child, h);
}

/*!
@brief Revolute joint aligned with x-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rx_dhva(const shift::core::Context& context, topology::DHVA& dhva,
                           const topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  shift::core::joint::rx_dh(context, dhva, offsets, q);
  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with x-axis - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rxinv_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                            const shift::core::topology::Offsets& offsets,
                            const double* __restrict__) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddrxinv(d_parent, dc, d_child);
  ksl_hsrxinv(d_child, h);
}

/*!
@brief Revolute joint aligned with x-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rxinv_dhva(const shift::core::Context& context,
                              shift::core::topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  shift::core::joint::rxinv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with y-axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ry_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                         const shift::core::topology::Offsets& offsets,
                         const double* __restrict__) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddry(d_parent, dc, d_child);
  ksl_hsry(d_child, h);
}

/*!
@brief Revolute joint aligned with y-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ry_dhva(const shift::core::Context& context,
                           shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,

                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  shift::core::joint::ry_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with y-axis - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ryinv_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                            const shift::core::topology::Offsets& offsets,
                            const double* __restrict__) {

  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddryinv(d_parent, dc, d_child);
  ksl_hsryinv(d_child, h);
}

/*!
@brief Revolute joint aligned with y-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ryinv_dhva(const shift::core::Context& context,
                              shift::core::topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  shift::core::joint::ryinv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with z-axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rz_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                         const shift::core::topology::Offsets& offsets,
                         const double* __restrict__) {

  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddrz(d_parent, dc, d_child);
  ksl_hsrz(d_child, h);
}

/*!
@brief Revolute joint aligned with z-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rz_dhva(const shift::core::Context& context,
                           shift::core::topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  shift::core::joint::rz_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute Joint aligned with z-axis - inv (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rzinv_dh(const shift::core::Context&, shift::core::topology::DHVA& dhva,
                            const shift::core::topology::Offsets& offsets,
                            const double* __restrict__) {

  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddrzinv(d_parent, dc, d_child);
  ksl_hsrzinv(d_child, h);
}

/*!
@brief Revolute joint aligned with z-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rzinv_dhva(const shift::core::Context& context,
                              shift::core::topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  shift::core::joint::rzinv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute joint aligned with arbitrary axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ra_dh(const shift::core::Context& context, topology::DHVA& dhva,
                         const shift::core::topology::Offsets& offsets,
                         const double*) {
  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.get_joint_data()[offsets.data]);
  ksl_mat3x3_t temp;

  ksl_mat3x3_setFromAxisDC(&temp, axis, dc);
  ksl_product_ddr(d_parent, &temp, d_child);
  ksl_vec3_copy(&d_parent->t, &d_child->t);
  ksl_hsra(d_child, axis, h);
}

/*!
@brief Revolute joint aligned with arbitrary axis - inverse (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rainv_dh(const shift::core::Context& context, topology::DHVA& dhva,
                            const shift::core::topology::Offsets& offsets,
                            const double*) {

  ksl_SE3_t* d_parent = dhva.d + offsets.parent.d;
  ksl_SE3_t* d_child = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.get_joint_data()[offsets.data]);
  ksl_vec3_t neg_axis = ksl_vec3(-axis->x, -axis->y, -axis->z);
  ksl_mat3x3_t temp;

  ksl_mat3x3_setFromAxisDC(&temp, &neg_axis, dc);
  ksl_product_ddr(d_parent, &temp, d_child);
  ksl_vec3_copy(&d_parent->t, &d_child->t);
  ksl_hsrainv(d_child, axis, h);
}

/*!
@brief Revolute joint aligned with arbitrary axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::ra_dhva(const shift::core::Context& context, topology::DHVA& dhva,
                           const shift::core::topology::Offsets& offsets,
                           const double* q, const double* dq) {
  shift::core::joint::ra_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

/*!
@brief Revolute joint aligned with arbitrary axis - inverse (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void shift::core::joint::rainv_dhva(const shift::core::Context& context,
                              topology::DHVA& dhva,
                              const shift::core::topology::Offsets& offsets,
                              const double* q, const double* dq) {
  shift::core::joint::rainv_dh(context, dhva, offsets, q);

  ksl_screw_t* h = dhva.h + offsets.dq;
  const double* vPrimitive = dq + offsets.dq;
  ksl_screw_t* vParent = dhva.v + offsets.parent.v;
  ksl_screw_t vRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* vChild = dhva.v + offsets.child.v;
  ksl_screw_t* aqParent = dhva.aq + offsets.parent.v;
  ksl_screw_t aqRelative = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t* aqChild = dhva.aq + offsets.child.v;

  ksl_product_as(*vPrimitive, h, &vRelative);
  ksl_add_ss(vParent, &vRelative, vChild);
  ksl_cross_ss(vChild, &vRelative, &aqRelative);
  ksl_add_ss(aqParent, &aqRelative, aqChild);
}

void shift::core::joint::Revolute::DeclareParameters() {
  DeclareParameter(
    "axis", &axis_,
    "Axis of rotation expressed in parent body frame, defaults to z-axis if "
    "no axis is provided.",
    false);
  DeclareParameter("initial_displacement", q_0_, "Initial joint displacement.",
               false);
  DeclareParameter("initial_velocity", dq_0_, "Initial joint velocity.", false);
  DeclareParameter(
    "damping_coefficients", &damping_coefficients_,
    "specify damping coefficients a, where force due to damping is computed as "
    "a_0 + a_1 * \\dot{x}^1 + a_2 * \\dot{x}^2 + .. a_n * \\dot{x}^n\\",
    false);
  DeclareParameter("stiffness_coefficients", &stiffness_coefficients_,
               "specify damping coefficients a, where force due to stiffness "
               "is computed as "
               "a_0 + a_1 * x^1 + a_2 * x^2 + .. a_n * x^n\\",
               false);
}

shift::core::joint::Revolute::AxisType shift::core::joint::Revolute::get_axis_type() {
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

const double* shift::core::joint::Revolute::get_data() {
  if(get_axis_type() == ARBITRARY_AXIS) {
    return (double*)&axis_;
  };
  return nullptr;
}

unsigned int shift::core::joint::Revolute::get_data_count() {
  if(get_axis_type() == ARBITRARY_AXIS) {
    return 3;
  };
  return 0;
}

shift::core::topology::dh_fptr shift::core::joint::Revolute::Dfptr() {
  AxisType axis = get_axis_type();
  switch(axis) {
    case X_AXIS: {
      if(sense_ >= 0) {
        return rx_d;
      } else {
        return rxinv_d;
      }
    }
    case Y_AXIS: {
      if(sense_ >= 0) {
        return ry_d;
      } else {
        return ryinv_d;
      }
    }
    case Z_AXIS: {
      if(sense_ >= 0) {
        return rz_d;
      } else {
        return rzinv_d;
      }
    }
    case ARBITRARY_AXIS: {
      if(sense_ >= 0) {
        return ra_d;
      } else {
        return rainv_d;
      }
    }
  }
  assert(0);
  return rz_d;
}

shift::core::topology::dh_fptr shift::core::joint::Revolute::DHfptr() {
  AxisType axis = get_axis_type();
  switch(axis) {
    case X_AXIS: {
      if(sense_ >= 0) {
        return rx_dh;
      } else {
        return rxinv_dh;
      }
    }
    case Y_AXIS: {
      if(sense_ >= 0) {
        return ry_dh;
      } else {
        return ryinv_dh;
      }
    }
    case Z_AXIS: {
      if(sense_ >= 0) {
        return rz_dh;
      } else {
        return rzinv_dh;
      }
    }
    case ARBITRARY_AXIS: {
      if(sense_ >= 0) {
        return ra_dh;
      } else {
        return rainv_dh;
      }
    }
  }
  assert(0);
  return rz_dh;
}

shift::core::topology::dhva_fptr shift::core::joint::Revolute::DHVAfptr() {
  AxisType axis = get_axis_type();
  switch(axis) {
    case X_AXIS: {
      if(sense_ >= 0) {
        return rx_dhva;
      } else {
        return rxinv_dhva;
      }
    }
    case Y_AXIS: {
      if(sense_ >= 0) {
        return ry_dhva;
      } else {
        return ryinv_dhva;
      }
    }
    case Z_AXIS: {
      if(sense_ >= 0) {
        return rz_dhva;
      } else {
        return rzinv_dhva;
      }
    }
    case ARBITRARY_AXIS: {
      if(sense_ >= 0) {
        return ra_dhva;
      } else {
        return rainv_dhva;
      }
    }
  }
  assert(0);
  return rz_dhva;
}

void shift::core::joint::Revolute::Service(const std::vector<double>& y) {

  std::vector<double>& f = context_->get_joint_working_forces();

  // stiffness
  const double q = y[q_index_];
  for(unsigned int i = 0; i < stiffness_coefficients_.size(); i++) {
    // f[dq_index_] -= stiffness_coefficients_[i] * copysign(pow(q, i), q);
    f[dq_index_] -= stiffness_coefficients_[i] * pow(q, i);
  }

  // damping
  const double dq = y[context_->get_q_count() + dq_index_];
  for(unsigned int i = 0; i < damping_coefficients_.size(); i++) {
    // f[dq_index_] -= damping_coefficients_[i] * copysign(pow(dq, i), dq);
    f[dq_index_] -= damping_coefficients_[i] * pow(dq, i);
  }
}
