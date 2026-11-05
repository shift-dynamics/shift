#include "shift/core/collision_geometry/box.h"

#include <algorithm>
#include <cmath>

#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::collision_geometry::Box::registered_ =
  shift::core::Factory<shift::core::collision_geometry::Base>::get()
    ->RegisterModule("box", shift::core::collision_geometry::Box::Create);

std::unique_ptr<shift::core::collision_geometry::Box>
shift::core::collision_geometry::Box::Create() {
  return std::make_unique<shift::core::collision_geometry::Box>();
}

bool shift::core::collision_geometry::Box::HasDimensions() const {
  const auto& h = half_extents_;
  if(h.x == 0.0 && h.y == 0.0 && h.z == 0.0) {
    return false;
  }
  if(!(h.x > 0.0 && h.y > 0.0 && h.z > 0.0)) {
    throw shift::core::Exception("Box collision geometry '" + name_ +
                                 "' needs three positive half extents");
  }
  return true;
}

void shift::core::collision_geometry::Box::Fit(
  const std::vector<ksl_vec3_t>& vertices, const std::vector<unsigned int>&) {
  /* the bounding box along the principal axes of the vertices */
  ksl_mat3x3_t covariance = Covariance(vertices);
  ksl_mat3x3_t axes;
  Jacobi(covariance, axes);
  ksl_vec3_t low = {{HUGE_VAL, HUGE_VAL, HUGE_VAL}};
  ksl_vec3_t high = {{-HUGE_VAL, -HUGE_VAL, -HUGE_VAL}};
  for(const auto& p : vertices) {
    for(int i = 0; i < 3; i++) {
      const double u = ksl_dot_vv(&axes.v[i], &p);
      low.at[i] = std::min(low.at[i], u);
      high.at[i] = std::max(high.at[i], u);
    }
  }
  ksl_SE3_t pose = {{{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}}};
  pose.R = axes;
  pose.t = {{0.0, 0.0, 0.0}};
  for(int i = 0; i < 3; i++) {
    /* a flat set of points still gets a box of some thickness */
    half_extents_.at[i] = std::max(0.5 * (high.at[i] - low.at[i]), 1e-6);
    const double middle = 0.5 * (high.at[i] + low.at[i]);
    for(int k = 0; k < 3; k++) {
      pose.t.at[k] += middle * axes.v[i].at[k];
    }
  }
  SetFromDimensions(pose);
}

void shift::core::collision_geometry::Box::CopyFit(const Base& other) {
  ConvexHull::CopyFit(other);
  half_extents_ = dynamic_cast<const Box&>(other).half_extents_;
}
