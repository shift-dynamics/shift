#include "shift/core/collision_geometry/capsule.h"

#include <algorithm>
#include <cmath>

#include "shift/core/collision_geometry/fitting.h"
#include "shift/core/exception.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/factory.h"

bool shift::core::collision_geometry::Capsule::registered_ =
  shift::core::Factory<shift::core::collision_geometry::Base>::get()
    ->RegisterModule("capsule",
                     shift::core::collision_geometry::Capsule::Create);

std::unique_ptr<shift::core::collision_geometry::Capsule>
shift::core::collision_geometry::Capsule::Create() {
  return std::make_unique<shift::core::collision_geometry::Capsule>();
}

void shift::core::collision_geometry::Capsule::DeclareParameters() {
  DeclareParameter("radius", &radius_,
                   "radius of the capsule, of its cylinder and of its hemispherical "
                   "end caps; if not given, the capsule is fitted to the "
                   "body's mesh",
                   false);
  DeclareParameter("half_length", &half_length_,
                   "half of the length of the segment of the capsule along its "
                   "axis, the z axis of the pose, without the end caps; "
                   "centered at the origin of the pose (the body origin by "
                   "default)",
                   false);
}

bool shift::core::collision_geometry::Capsule::HasDimensions() const {
  if(radius_ == 0.0 && half_length_ == 0.0) {
    return false;
  }
  if(!(radius_ > 0.0 && half_length_ >= 0.0)) {
    throw shift::core::Exception("Capsule collision geometry '" + name_ +
                                 "' needs a positive radius and a nonnegative "
                                 "half_length");
  }
  return true;
}

void shift::core::collision_geometry::Capsule::SetFromDimensions(
  const ksl_SE3_t& pose) {
  d_wrt_body_ = pose;
  bounding_radius_ = half_length_ + radius_;
}

void shift::core::collision_geometry::Capsule::Fit(
  const std::vector<ksl_vec3_t>& vertices, const std::vector<unsigned int>&) {
  double low, high;
  FitAxis(vertices, d_wrt_body_, low, high, radius_);
  /* the segment is the extent along the axis less the radius of the caps,
  and the radius grows to enclose the points near the ends */
  half_length_ = std::max(0.5 * (high - low) - radius_, 0.0);
  radius_ = 1e-6;
  for(const auto& p : vertices) {
    const double z = ksl_dot_vv(&d_wrt_body_.R.v[2], &p) -
                     ksl_dot_vv(&d_wrt_body_.R.v[2], &d_wrt_body_.t);
    const double u = ksl_dot_vv(&d_wrt_body_.R.v[0], &p) -
                     ksl_dot_vv(&d_wrt_body_.R.v[0], &d_wrt_body_.t);
    const double v = ksl_dot_vv(&d_wrt_body_.R.v[1], &p) -
                     ksl_dot_vv(&d_wrt_body_.R.v[1], &d_wrt_body_.t);
    const double beyond = std::max(std::fabs(z) - half_length_, 0.0);
    radius_ = std::max(radius_, std::sqrt(u * u + v * v + beyond * beyond));
  }
  bounding_radius_ = half_length_ + radius_;
}

void shift::core::collision_geometry::Capsule::CopyFit(const Base& other) {
  const auto& capsule = dynamic_cast<const Capsule&>(other);
  radius_ = capsule.radius_;
  half_length_ = capsule.half_length_;
  d_wrt_body_ = capsule.d_wrt_body_;
  bounding_radius_ = capsule.bounding_radius_;
}

void shift::core::collision_geometry::Capsule::Support(
  const ksl_vec3_t& direction, ksl_vec3_t& local, ksl_vec3_t& global,
  int&) const {
  /* the direction in the frame of the capsule, R^T * direction */
  ksl_vec3_t u;
  for(int i = 0; i < 3; i++) {
    u.at[i] = ksl_dot_vv(&d_absolute_.R.v[i], &direction);
  }
  /* the end of the segment toward the direction, and the radius along it */
  const double norm = std::sqrt(ksl_dot_vv(&u, &u));
  const double k = norm > 0.0 ? radius_ / norm : 0.0;
  local = {{k * u.x, k * u.y,
            (u.z >= 0.0 ? half_length_ : -half_length_) + k * u.z}};
  ksl_product_dv(&d_absolute_, &local, &global);
}

std::shared_ptr<const shift::core::collision_geometry::CadContactShape>
shift::core::collision_geometry::Capsule::MakeExactShape() const {
  /* a capsule without a length is a sphere at each end, with no side */
  if(half_length_ <= 0.0) {
    return std::make_shared<CadContactShape>();
  }
  return MakeCylinderShape(radius_, half_length_, false);
}
