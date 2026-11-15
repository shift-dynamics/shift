#include "shift/gui/camera.h"

#include <algorithm>
#include <cmath>

static const ksl_vec3f_t kWorldUp = {{0.f, 0.f, -1.f}}; // +z is down
static constexpr float kElevationLimit = 1.55f;  // just short of +-pi/2

ksl_vec3f_t shift::gui::Camera::TurntableForward() const {
  // unit vector from the camera to the target
  const float c = std::cos(elevation_);
  return ksl_vec3f(-c * std::cos(azimuth_), -c * std::sin(azimuth_),
                   std::sin(elevation_));
}

ksl_vec3f_t shift::gui::Camera::TurntableRight() const {
  const ksl_vec3f_t forward = TurntableForward();
  ksl_vec3f_t right;
  ksl_cross_vvf(&forward, &kWorldUp, &right);
  ksl_vec3f_normalize(&right);
  return right;
}

ksl_vec3f_t shift::gui::Camera::TurntableUp() const {
  const ksl_vec3f_t right = TurntableRight();
  const ksl_vec3f_t forward = TurntableForward();
  ksl_vec3f_t up;
  ksl_cross_vvf(&right, &forward, &up);
  return up;
}

ksl_vec3f_t shift::gui::Camera::TurntablePosition() const {
  const ksl_vec3f_t forward = TurntableForward();
  ksl_vec3f_t position = target_;
  ksl_axpy_vvf(-distance_, &forward, &position);
  return position;
}

ksl_vec3f_t shift::gui::Camera::get_forward() const {
  const ksl_vec3f_t v = TurntableForward();
  ksl_vec3f_t world;
  ksl_product_drvf(&follow_.R, &v, &world);
  return world;
}

ksl_vec3f_t shift::gui::Camera::get_position() const {
  const ksl_vec3f_t p = TurntablePosition();
  ksl_vec3f_t world;
  ksl_product_dvf(&follow_, &p, &world);
  return world;
}

ksl_vec3f_t shift::gui::Camera::get_right() const {
  const ksl_vec3f_t v = TurntableRight();
  ksl_vec3f_t world;
  ksl_product_drvf(&follow_.R, &v, &world);
  return world;
}

ksl_vec3f_t shift::gui::Camera::get_up() const {
  const ksl_vec3f_t v = TurntableUp();
  ksl_vec3f_t world;
  ksl_product_drvf(&follow_.R, &v, &world);
  return world;
}

ksl_SE3f_t shift::gui::Camera::get_view_matrix() const {
  // looking from the position at the target: the rows of the rotation are
  // the right, up and backward directions, in the turntable's coordinates
  const ksl_vec3f_t right = TurntableRight();
  const ksl_vec3f_t up = TurntableUp();
  const ksl_vec3f_t forward = TurntableForward();
  ksl_SE3f_t look;
  for(int k = 0; k < 3; k++) {
    look.R.at[k][0] = right.at[k];
    look.R.at[k][1] = up.at[k];
    look.R.at[k][2] = -forward.at[k];
  }
  const ksl_vec3f_t position = TurntablePosition();
  ksl_vec3f_t rotated;
  ksl_product_drvf(&look.R, &position, &rotated);
  ksl_product_avf(-1.f, &rotated, &look.t);
  // then from world coordinates to the turntable's
  ksl_SE3f_t view;
  ksl_product_ddinvf(&look, &follow_, &view);
  return view;
}

ksl_SE3f_t shift::gui::Camera::GroundFrame(const ksl_SE3f_t& pose,
                                           float ground_z) {
  // the heading: the body's x axis seen from above
  const float heading = std::atan2(pose.R.at[0][1], pose.R.at[0][0]);
  const float c = std::cos(heading), s = std::sin(heading);
  ksl_SE3f_t frame;
  ksl_SE3f_setIdentity(&frame);
  frame.R.at[0][0] = c;
  frame.R.at[0][1] = s;
  frame.R.at[1][0] = -s;
  frame.R.at[1][1] = c;
  frame.t = ksl_vec3f(pose.t.x, pose.t.y, ground_z);
  return frame;
}

void shift::gui::Camera::ClearFollow() {
  const ksl_vec3f_t forward = get_forward();
  const ksl_vec3f_t target = target_;
  ksl_product_dvf(&follow_, &target, &target_);
  ksl_SE3f_setIdentity(&follow_);
  azimuth_ = std::atan2(-forward.y, -forward.x);
  elevation_ = std::clamp(std::asin(std::clamp(forward.z, -1.f, 1.f)),
                          -kElevationLimit, kElevationLimit);
}

QMatrix4x4 shift::gui::Camera::get_projection_matrix(float aspect) const {
  QMatrix4x4 projection;
  // the near plane follows the distance to the target, which zooming keeps
  // on the surface in view (ZoomTowards); the far plane reaches past the
  // framed box, so that the rest of the model stays in view up close
  const float near_plane = distance_ * 0.01f;
  const float far_plane =
    std::max(distance_ * 100.f, distance_ + 4.f * radius_);
  projection.perspective(fov_y_, aspect, near_plane, far_plane);
  return projection;
}

void shift::gui::Camera::set_direction(float azimuth, float elevation) {
  azimuth_ = azimuth;
  elevation_ = std::clamp(elevation, -kElevationLimit, kElevationLimit);
}

void shift::gui::Camera::Orbit(float delta_azimuth, float delta_elevation) {
  azimuth_ += delta_azimuth;
  elevation_ = std::clamp(elevation_ + delta_elevation, -kElevationLimit,
                          kElevationLimit);
}

void shift::gui::Camera::Pan(float dx, float dy) {
  // scale so a drag across the viewport height moves the target across the
  // visible height at the target distance
  const float view_height =
    2.f * distance_ * std::tan(0.5f * fov_y_ * float(M_PI) / 180.f);
  const ksl_vec3f_t right = TurntableRight();
  const ksl_vec3f_t up = TurntableUp();
  ksl_axpy_vvf(-dx * view_height, &right, &target_);
  ksl_axpy_vvf(dy * view_height, &up, &target_);
}

void shift::gui::Camera::Zoom(float factor) {
  distance_ = std::clamp(distance_ * factor, distance_min_, distance_max_);
}

void shift::gui::Camera::ZoomTowards(const ksl_vec3f_t& world_point,
                                     float factor) {
  ksl_vec3f_t point;
  ksl_product_dinvvf(&follow_, &world_point, &point);
  const ksl_vec3f_t forward = TurntableForward();
  const ksl_vec3f_t position = TurntablePosition();
  // depth of the point along the view direction, after the move
  ksl_vec3f_t to_point;
  ksl_subtract_vvf(&point, &position, &to_point);
  const float depth = ksl_dot_vvf(&to_point, &forward);
  if(!(depth > 0.f)) {
    Zoom(factor);
    return;
  }
  const float new_depth =
    std::clamp(depth * factor, distance_min_, distance_max_);
  // the new position, along the line from the point to the old one
  ksl_vec3f_t new_position = point;
  ksl_axpy_vvf(-new_depth / depth, &to_point, &new_position);
  distance_ = new_depth;
  target_ = new_position;
  ksl_axpy_vvf(new_depth, &forward, &target_);
}

void shift::gui::Camera::Frame(const ksl_vec3f_t& box_min,
                               const ksl_vec3f_t& box_max) {
  ksl_vec3f_t center, size;
  ksl_add_vvf(&box_min, &box_max, &center);
  ksl_vec3f_scale(&center, 0.5f);
  ksl_product_dinvvf(&follow_, &center, &target_);
  ksl_subtract_vvf(&box_max, &box_min, &size);
  const float radius = std::max(0.5f * ksl_vec3f_l2norm(&size), 1e-3f);
  distance_ = radius / std::sin(0.5f * fov_y_ * float(M_PI) / 180.f);
  distance_min_ = radius * 1e-4f;
  distance_max_ = radius * 1e3f;
  radius_ = radius;
}

QMatrix4x4 shift::gui::ToQMatrix4x4(const ksl_SE3f_t& d) {
  // QMatrix4x4 takes its values row by row; KSL's R.at[column][row]
  return QMatrix4x4(d.R.at[0][0], d.R.at[1][0], d.R.at[2][0], d.t.x,
                    d.R.at[0][1], d.R.at[1][1], d.R.at[2][1], d.t.y,
                    d.R.at[0][2], d.R.at[1][2], d.R.at[2][2], d.t.z,
                    0.f, 0.f, 0.f, 1.f);
}
