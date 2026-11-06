#include "shift/core/collision_geometry/cylinder.h"

#include <algorithm>
#include <cmath>

#include "shift/core/collision_geometry/fitting.h"
#include "shift/core/exception.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/factory.h"

bool shift::core::collision_geometry::Cylinder::registered_ =
  shift::core::Factory<shift::core::collision_geometry::Base>::get()
    ->RegisterModule("cylinder",
                     shift::core::collision_geometry::Cylinder::Create);

std::unique_ptr<shift::core::collision_geometry::Cylinder>
shift::core::collision_geometry::Cylinder::Create() {
  return std::make_unique<shift::core::collision_geometry::Cylinder>();
}

void shift::core::collision_geometry::Cylinder::DeclareParameters() {
  DeclareParameter("radius", &radius_,
                   "radius of the cylinder; if not given, the cylinder is "
                   "fitted to the body's mesh",
                   false);
  DeclareParameter("half_length", &half_length_,
                   "half of the length of the cylinder along its axis, the z "
                   "axis of the pose; centered at the origin of the pose "
                   "(the body origin by default)",
                   false);
  DeclareParameter("edge_radius", &edge_radius_,
                   "radius to which the rims are taken to be rounded, for "
                   "their Hertz contact with CAD geometry; defaults to 0.2% "
                   "of the size of the cylinder (the diagonal of its bounds)",
                   false);
}

bool shift::core::collision_geometry::Cylinder::HasDimensions() const {
  if(radius_ == 0.0 && half_length_ == 0.0) {
    return false;
  }
  if(!(radius_ > 0.0 && half_length_ > 0.0)) {
    throw shift::core::Exception("Cylinder collision geometry '" + name_ +
                                 "' needs a positive radius and half_length");
  }
  return true;
}

void shift::core::collision_geometry::Cylinder::SetFromDimensions(
  const ksl_SE3_t& pose) {
  d_wrt_body_ = pose;
  bounding_radius_ = std::hypot(radius_, half_length_);
}

void shift::core::collision_geometry::Cylinder::Fit(
  const std::vector<ksl_vec3_t>& vertices, const std::vector<unsigned int>&) {
  double low, high;
  FitAxis(vertices, d_wrt_body_, low, high, radius_);
  half_length_ = 0.5 * (high - low);
  /* a flat or thin set of points still gets a cylinder of some size */
  radius_ = std::max(radius_, 1e-6);
  half_length_ = std::max(half_length_, 1e-6);
  bounding_radius_ = std::hypot(radius_, half_length_);
}

void shift::core::collision_geometry::Cylinder::CopyFit(const Base& other) {
  const auto& cylinder = dynamic_cast<const Cylinder&>(other);
  radius_ = cylinder.radius_;
  half_length_ = cylinder.half_length_;
  d_wrt_body_ = cylinder.d_wrt_body_;
  bounding_radius_ = cylinder.bounding_radius_;
}

void shift::core::collision_geometry::Cylinder::Support(
  const ksl_vec3_t& direction, ksl_vec3_t& local, ksl_vec3_t& global,
  int&) const {
  /* the direction in the frame of the cylinder, R^T * direction */
  ksl_vec3_t u;
  for(int i = 0; i < 3; i++) {
    u.at[i] = ksl_dot_vv(&d_absolute_.R.v[i], &direction);
  }
  const double radial = std::hypot(u.x, u.y);
  const double k = radial > 0.0 ? radius_ / radial : 0.0;
  local = {{k * u.x, k * u.y, u.z >= 0.0 ? half_length_ : -half_length_}};
  ksl_product_dv(&d_absolute_, &local, &global);
}

std::shared_ptr<const shift::core::collision_geometry::CadContactShape>
shift::core::collision_geometry::Cylinder::MakeExactShape() const {
  return MakeCylinderShape(radius_, half_length_, true);
}
