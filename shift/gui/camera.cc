#include "shift/gui/camera.h"

#include <algorithm>
#include <cmath>

static const QVector3D kWorldUp(0.f, 0.f, -1.f); // +z is down
static constexpr float kElevationLimit = 1.55f;  // just short of +-pi/2

QVector3D shift::gui::Camera::TurntableForward() const {
  // unit vector from the camera to the target
  const float c = std::cos(elevation_);
  return -QVector3D(c * std::cos(azimuth_), c * std::sin(azimuth_),
                    -std::sin(elevation_));
}

QVector3D shift::gui::Camera::TurntableRight() const {
  return QVector3D::crossProduct(TurntableForward(), kWorldUp).normalized();
}

QVector3D shift::gui::Camera::TurntableUp() const {
  return QVector3D::crossProduct(TurntableRight(), TurntableForward());
}

QVector3D shift::gui::Camera::get_forward() const {
  return follow_.mapVector(TurntableForward());
}

QVector3D shift::gui::Camera::get_position() const {
  return follow_.map(target_ - TurntableForward() * distance_);
}

QVector3D shift::gui::Camera::get_right() const {
  return follow_.mapVector(TurntableRight());
}

QVector3D shift::gui::Camera::get_up() const {
  return follow_.mapVector(TurntableUp());
}

QMatrix4x4 shift::gui::Camera::get_view_matrix() const {
  QMatrix4x4 view;
  view.lookAt(target_ - TurntableForward() * distance_, target_, kWorldUp);
  return view * follow_.inverted();
}

QMatrix4x4 shift::gui::Camera::GroundFrame(const QMatrix4x4& pose,
                                           float ground_z) {
  // the heading: the body's x axis seen from above
  const float heading = std::atan2(pose(1, 0), pose(0, 0));
  const float c = std::cos(heading), s = std::sin(heading);
  return QMatrix4x4(c, -s, 0.f, pose(0, 3), s, c, 0.f, pose(1, 3), 0.f, 0.f,
                    1.f, ground_z, 0.f, 0.f, 0.f, 1.f);
}

void shift::gui::Camera::ClearFollow() {
  const QVector3D forward = get_forward();
  target_ = follow_.map(target_);
  follow_.setToIdentity();
  azimuth_ = std::atan2(-forward.y(), -forward.x());
  elevation_ = std::clamp(std::asin(std::clamp(forward.z(), -1.f, 1.f)),
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
  target_ += (-TurntableRight() * dx + TurntableUp() * dy) * view_height;
}

void shift::gui::Camera::Zoom(float factor) {
  distance_ = std::clamp(distance_ * factor, distance_min_, distance_max_);
}

void shift::gui::Camera::ZoomTowards(const QVector3D& world_point,
                                     float factor) {
  const QVector3D point = follow_.inverted().map(world_point);
  const QVector3D forward = TurntableForward();
  const QVector3D position = target_ - forward * distance_;
  // depth of the point along the view direction, after the move
  const float depth = QVector3D::dotProduct(point - position, forward);
  if(!(depth > 0.f)) {
    Zoom(factor);
    return;
  }
  const float new_depth =
    std::clamp(depth * factor, distance_min_, distance_max_);
  const QVector3D new_position =
    point + (position - point) * (new_depth / depth);
  distance_ = new_depth;
  target_ = new_position + forward * new_depth;
}

void shift::gui::Camera::Frame(const QVector3D& box_min,
                               const QVector3D& box_max) {
  target_ = follow_.inverted().map(0.5f * (box_min + box_max));
  const float radius = std::max(0.5f * (box_max - box_min).length(), 1e-3f);
  distance_ = radius / std::sin(0.5f * fov_y_ * float(M_PI) / 180.f);
  distance_min_ = radius * 1e-4f;
  distance_max_ = radius * 1e3f;
  radius_ = radius;
}
