#include "shift/core/collision_geometry/sphere.h"

#include <cmath>

#include "shift/core/collision_geometry/fitting.h"
#include "shift/core/factory.h"

bool shift::core::collision_geometry::Sphere::registered_ =
  shift::core::Factory<shift::core::collision_geometry::Base>::get()
    ->RegisterModule("sphere", shift::core::collision_geometry::Sphere::Create);

std::unique_ptr<shift::core::collision_geometry::Sphere>
shift::core::collision_geometry::Sphere::Create() {
  return std::make_unique<shift::core::collision_geometry::Sphere>();
}

void shift::core::collision_geometry::Sphere::Fit(
  const std::vector<ksl_vec3_t>& vertices, const std::vector<unsigned int>&) {
  ksl_vec3_t& center = d_wrt_body_.t;
  if(vertices.size() == 1) {
    radius_ = 0.0;
    center = vertices[0];
    bounding_radius_ = 0.0;
    return;
  }

  /* initial sphere spanning the extreme points along the direction of
  greatest spread, the eigenvector of the covariance matrix with the largest
  eigenvalue */
  ksl_mat3x3_t covariance = Covariance(vertices);
  ksl_mat3x3_t eigenvectors;
  Jacobi(covariance, eigenvectors);
  int axis = 0;
  for(int i = 1; i < 3; i++) {
    if(std::fabs(covariance.at[i][i]) > std::fabs(covariance.at[axis][axis])) {
      axis = i;
    }
  }
  const ksl_vec3_t& e = eigenvectors.v[axis];
  size_t imin = 0;
  size_t imax = 0;
  double pmin = HUGE_VAL;
  double pmax = -HUGE_VAL;
  for(size_t i = 0; i < vertices.size(); i++) {
    const double p = ksl_dot_vv(&e, &vertices[i]);
    if(p < pmin) {
      pmin = p;
      imin = i;
    }
    if(p > pmax) {
      pmax = p;
      imax = i;
    }
  }
  double dist2 = 0.0;
  for(int k = 0; k < 3; k++) {
    const double d = vertices[imax].at[k] - vertices[imin].at[k];
    dist2 += d * d;
    center.at[k] = 0.5 * (vertices[imax].at[k] + vertices[imin].at[k]);
  }
  radius_ = 0.5 * std::sqrt(dist2);

  /* grow the sphere to just encompass any point outside of it */
  for(const auto& p : vertices) {
    ksl_vec3_t d;
    for(int k = 0; k < 3; k++) {
      d.at[k] = p.at[k] - center.at[k];
    }
    const double d2 = ksl_dot_vv(&d, &d);
    if(d2 > radius_ * radius_) {
      const double dist = std::sqrt(d2);
      const double new_radius = 0.5 * (radius_ + dist);
      const double k = (new_radius - radius_) / dist;
      radius_ = new_radius;
      for(int j = 0; j < 3; j++) {
        center.at[j] += k * d.at[j];
      }
    }
  }
  bounding_radius_ = radius_;
}

void shift::core::collision_geometry::Sphere::CopyFit(const Base& other) {
  const auto& sphere = dynamic_cast<const Sphere&>(other);
  radius_ = sphere.radius_;
  d_wrt_body_ = sphere.d_wrt_body_;
  bounding_radius_ = sphere.bounding_radius_;
}

void shift::core::collision_geometry::Sphere::Support(
  const ksl_vec3_t& direction, ksl_vec3_t& local, ksl_vec3_t& global,
  int&) const {
  const double norm = ksl_vec3_l2norm(&direction);
  const double k = norm > 0.0 ? radius_ / norm : 0.0;
  ksl_vec3_t offset;
  for(int i = 0; i < 3; i++) {
    offset.at[i] = k * direction.at[i];
    global.at[i] = d_absolute_.t.at[i] + offset.at[i];
  }
  /* local = R^T * offset */
  for(int i = 0; i < 3; i++) {
    local.at[i] = ksl_dot_vv(&d_absolute_.R.v[i], &offset);
  }
}
