#include "shift/core/collision_geometry/plane.h"

#include <cmath>

#include "shift/core/body/base.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::collision_geometry::Plane::registered_ =
  shift::core::Factory<shift::core::collision_geometry::Base>::get()
    ->RegisterModule("plane", shift::core::collision_geometry::Plane::Create);

std::unique_ptr<shift::core::collision_geometry::Plane>
shift::core::collision_geometry::Plane::Create() {
  return std::make_unique<shift::core::collision_geometry::Plane>();
}

void shift::core::collision_geometry::Plane::DeclareParameters() {
  DeclareParameter("normal", &normal_,
                   "normal of the plane in the pose (the body frame by "
                   "default), out of the solid "
                   "into free space; defaults to [0, 0, -1], up with shift's "
                   "z axis down",
                   false);
  DeclareParameter("offset", &offset_,
                   "distance of the plane from the pose origin along the "
                   "normal, defaults to 0",
                   false);
}

void shift::core::collision_geometry::Plane::Load() {
  if(name_.empty() && get_parent_body()) {
    name_ = get_parent_body()->get_name();
  }
  SetPlane(normal_, offset_);
}

void shift::core::collision_geometry::Plane::SetPlane(const ksl_vec3_t& normal,
                                                      double offset) {
  const double length = ksl_vec3_l2norm(&normal);
  if(!(length > 0.0)) {
    throw shift::core::Exception("Plane collision geometry '" + name_ +
                                 "' needs a nonzero normal");
  }
  normal_ = normal;
  offset_ = offset;
  for(int k = 0; k < 3; k++) {
    unit_normal_.at[k] = normal.at[k] / length;
  }
  /* the frame has the orientation of the pose, its origin on the plane at
  offset * normal in the pose */
  const ksl_SE3_t& pose = *pose_.get_d_wrt_origin();
  d_wrt_body_.R = pose.R;
  for(int k = 0; k < 3; k++) {
    d_wrt_body_.t.at[k] = pose.t.at[k];
    for(int i = 0; i < 3; i++) {
      d_wrt_body_.t.at[k] += offset * unit_normal_.at[i] * pose.R.v[i].at[k];
    }
  }
  bounding_radius_ = HUGE_VAL;
  loaded_ = true;
}

void shift::core::collision_geometry::Plane::set_pose(const ksl_SE3_t& pose) {
  pose_.set_d_wrt_origin(pose);
  SetPlane(normal_, offset_);
  /* contact detection updates the absolute displacement from the body pose;
  until then it is that of a plane of the fixed body, which does not move */
  d_absolute_ = d_wrt_body_;
}

ksl_vec3_t shift::core::collision_geometry::Plane::get_normal_wrt_body() const {
  ksl_vec3_t n = {{0.0, 0.0, 0.0}};
  for(int i = 0; i < 3; i++) {
    for(int k = 0; k < 3; k++) {
      n.at[k] += unit_normal_.at[i] * d_wrt_body_.R.v[i].at[k];
    }
  }
  return n;
}

ksl_vec3_t shift::core::collision_geometry::Plane::get_normal() const {
  /* R * n: the frame has the body's orientation */
  ksl_vec3_t n = {{0.0, 0.0, 0.0}};
  for(int i = 0; i < 3; i++) {
    for(int k = 0; k < 3; k++) {
      n.at[k] += unit_normal_.at[i] * d_absolute_.R.v[i].at[k];
    }
  }
  return n;
}

void shift::core::collision_geometry::Plane::Support(const ksl_vec3_t&,
                                                     ksl_vec3_t& local,
                                                     ksl_vec3_t& global,
                                                     int&) const {
  local = {{0.0, 0.0, 0.0}};
  global = d_absolute_.t;
}

void shift::core::collision_geometry::Plane::Fit(
  const std::vector<ksl_vec3_t>&, const std::vector<unsigned int>&) {}

void shift::core::collision_geometry::Plane::CopyFit(const Base& other) {
  const auto& plane = dynamic_cast<const Plane&>(other);
  SetPlane(plane.normal_, plane.offset_);
}
