#include "shift-core/joint/revolute.h"
#include "shift-core/context.h"
#include "shift-core/factory.h"

#include "ksl/print.h"

bool Shift::Joint::Revolute::registered_ =
  Shift::Factory<Shift::Joint::Base>::get()->registerModule(
    "revolute", Shift::Joint::Revolute::create);

std::unique_ptr<Shift::Joint::Revolute> Shift::Joint::Revolute::create() {
  return std::make_unique<Shift::Joint::Revolute>();
}

/*!
@brief Revolute Joint aligned with x-axis (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rx_d(const Shift::Context&, Shift::Topology::DHVA& dhva,
                        const Shift::Topology::Offsets& offsets,
                        const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;

  ksl_product_ddrx(dParent, dc, dChild);
}

/*!
@brief Revolute Joint aligned with x-axis - inv (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rxinv_d(const Shift::Context&, Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_product_ddrxinv(dParent, dc, dChild);
}

/*!
@brief Revolute Joint aligned with y-axis (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ry_d(const Shift::Context&, Shift::Topology::DHVA& dhva,
                        const Shift::Topology::Offsets& offsets,
                        const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_product_ddry(dParent, dc, dChild);
}

/*!
@brief Revolute Joint aligned with y-axis - inv (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ryinv_d(const Shift::Context&, Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_product_ddryinv(dParent, dc, dChild);
}

/*!
@brief Revolute Joint aligned with z-axis (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rz_d(const Shift::Context&, Shift::Topology::DHVA& dhva,
                        const Shift::Topology::Offsets& offsets,
                        const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_product_ddrz(dParent, dc, dChild);
}

/*!
@brief Revolute Joint aligned with z-axis - inv (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rzinv_d(const Shift::Context&, Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_product_ddrzinv(dParent, dc, dChild);
}

/*!
@brief Revolute joint aligned with arbitrary axis (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ra_d(const Shift::Context& context, Topology::DHVA& dhva,
                        const Shift::Topology::Offsets& offsets,
                        const double*) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.jointData()[offsets.data]);
  ksl_mat3x3_t temp;
  ksl_mat3x3_setFromAxisDC(&temp, axis, dc);
  ksl_product_ddr(dParent, &temp, dChild);
  ksl_vec3_copy(&dParent->t, &dChild->t);
}

/*!
@brief Revolute joint aligned with arbitrary axis - inverse (d)

Updates displacements only

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rainv_d(const Shift::Context& context, Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double*) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.jointData()[offsets.data]);
  ksl_vec3_t neg_axis = ksl_vec3(-axis->x, -axis->y, -axis->z);
  ksl_mat3x3_t temp;
  ksl_mat3x3_setFromAxisDC(&temp, &neg_axis, dc);
  ksl_product_ddr(dParent, &temp, dChild);
  ksl_vec3_copy(&dParent->t, &dChild->t);
}

/*!
@brief Revolute Joint aligned with x-axis (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rx_dh(const Shift::Context&, Topology::DHVA& dhva,
                         const Shift::Topology::Offsets& offsets,
                         const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddrx(dParent, dc, dChild);
  ksl_hsrx(dChild, h);
}

/*!
@brief Revolute joint aligned with x-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rx_dhva(const Shift::Context& context, Topology::DHVA& dhva,
                           const Topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  Shift::Joint::rx_dh(context, dhva, offsets, q);
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
void Shift::Joint::rxinv_dh(const Shift::Context&, Shift::Topology::DHVA& dhva,
                            const Shift::Topology::Offsets& offsets,
                            const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddrxinv(dParent, dc, dChild);
  ksl_hsrxinv(dChild, h);
}

/*!
@brief Revolute joint aligned with x-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rxinv_dhva(const Shift::Context& context,
                              Shift::Topology::DHVA& dhva,
                              const Shift::Topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  Shift::Joint::rxinv_dh(context, dhva, offsets, q);

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
void Shift::Joint::ry_dh(const Shift::Context&, Shift::Topology::DHVA& dhva,
                         const Shift::Topology::Offsets& offsets,
                         const double* __restrict__) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddry(dParent, dc, dChild);
  ksl_hsry(dChild, h);
}

/*!
@brief Revolute joint aligned with y-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ry_dhva(const Shift::Context& context,
                           Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,

                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  Shift::Joint::ry_dh(context, dhva, offsets, q);

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
void Shift::Joint::ryinv_dh(const Shift::Context&, Shift::Topology::DHVA& dhva,
                            const Shift::Topology::Offsets& offsets,
                            const double* __restrict__) {

  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddryinv(dParent, dc, dChild);
  ksl_hsryinv(dChild, h);
}

/*!
@brief Revolute joint aligned with y-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ryinv_dhva(const Shift::Context& context,
                              Shift::Topology::DHVA& dhva,
                              const Shift::Topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {

  Shift::Joint::ryinv_dh(context, dhva, offsets, q);

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
void Shift::Joint::rz_dh(const Shift::Context&, Shift::Topology::DHVA& dhva,
                         const Shift::Topology::Offsets& offsets,
                         const double* __restrict__) {

  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddrz(dParent, dc, dChild);
  ksl_hsrz(dChild, h);
}

/*!
@brief Revolute joint aligned with z-axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rz_dhva(const Shift::Context& context,
                           Shift::Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* __restrict__ q,
                           const double* __restrict__ dq) {
  Shift::Joint::rz_dh(context, dhva, offsets, q);

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
void Shift::Joint::rzinv_dh(const Shift::Context&, Shift::Topology::DHVA& dhva,
                            const Shift::Topology::Offsets& offsets,
                            const double* __restrict__) {

  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;

  ksl_product_ddrzinv(dParent, dc, dChild);
  ksl_hsrzinv(dChild, h);
}

/*!
@brief Revolute joint aligned with z-axis - inv (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rzinv_dhva(const Shift::Context& context,
                              Shift::Topology::DHVA& dhva,
                              const Shift::Topology::Offsets& offsets,
                              const double* __restrict__ q,
                              const double* __restrict__ dq) {
  Shift::Joint::rzinv_dh(context, dhva, offsets, q);

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
void Shift::Joint::ra_dh(const Shift::Context& context, Topology::DHVA& dhva,
                         const Shift::Topology::Offsets& offsets,
                         const double*) {
  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.jointData()[offsets.data]);
  ksl_mat3x3_t temp;

  ksl_mat3x3_setFromAxisDC(&temp, axis, dc);
  ksl_product_ddr(dParent, &temp, dChild);
  ksl_vec3_copy(&dParent->t, &dChild->t);
  ksl_hsra(dChild, axis, h);
}

/*!
@brief Revolute joint aligned with arbitrary axis - inverse (dh)

Updates displacement and influence coefficient matrices

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::rainv_dh(const Shift::Context& context, Topology::DHVA& dhva,
                            const Shift::Topology::Offsets& offsets,
                            const double*) {

  ksl_SE3_t* dParent = dhva.d + offsets.parent.d;
  ksl_SE3_t* dChild = dhva.d + offsets.child.d;
  double* dc = dhva.dc + offsets.dc;
  ksl_screw_t* h = dhva.h + offsets.dq;
  const ksl_vec3_t* axis = (ksl_vec3_t*)(&context.jointData()[offsets.data]);
  ksl_vec3_t neg_axis = ksl_vec3(-axis->x, -axis->y, -axis->z);
  ksl_mat3x3_t temp;

  ksl_mat3x3_setFromAxisDC(&temp, &neg_axis, dc);
  ksl_product_ddr(dParent, &temp, dChild);
  ksl_vec3_copy(&dParent->t, &dChild->t);
  ksl_hsrainv(dChild, axis, h);
}

/*!
@brief Revolute joint aligned with arbitrary axis (dhva)

Updates displacement and influence coefficient matrices,
velocities and accelerations

@param dhva an array of pointers to properties related to
       displacement, influence coefficient matrices, velocities,
       and accelerations.
*/
void Shift::Joint::ra_dhva(const Shift::Context& context, Topology::DHVA& dhva,
                           const Shift::Topology::Offsets& offsets,
                           const double* q, const double* dq) {
  Shift::Joint::ra_dh(context, dhva, offsets, q);

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
void Shift::Joint::rainv_dhva(const Shift::Context& context,
                              Topology::DHVA& dhva,
                              const Shift::Topology::Offsets& offsets,
                              const double* q, const double* dq) {
  Shift::Joint::rainv_dh(context, dhva, offsets, q);

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

void Shift::Joint::Revolute::addParameters() {
  addParameter(
    "axis", &axis_,
    "Axis of rotation expressed in parent body frame, defaults to z-axis if "
    "no axis is provided.",
    false);
  addParameter("initial_displacement", q_0_, "Initial joint displacement.",
               false);
  addParameter("initial_velocity", dq_0_, "Initial joint velocity.", false);
}

Shift::Joint::Revolute::AxisType Shift::Joint::Revolute::axisType() {
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

const double* Shift::Joint::Revolute::data() {
  if(axisType() == ARBITRARY_AXIS) {
    return (double*)&axis_;
  };
  return NULL;
}

unsigned int Shift::Joint::Revolute::dataCount() {
  if(axisType() == ARBITRARY_AXIS) {
    return 3;
  };
  return 0;
}

Shift::Topology::dh_fptr Shift::Joint::Revolute::Dfptr() {
  AxisType axis = axisType();
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

Shift::Topology::dh_fptr Shift::Joint::Revolute::DHfptr() {
  AxisType axis = axisType();
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

Shift::Topology::dhva_fptr Shift::Joint::Revolute::DHVAfptr() {
  AxisType axis = axisType();
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
