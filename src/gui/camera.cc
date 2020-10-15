#include "shift/gui/camera.h"

Camera::Camera(const QVector3D& pos)
  : forward_(0.0f, 0.0f, -1.0f)
  , right_(1.0f, 0.0f, 0.0f)
  , up_(0.0f, 1.0f, 0.0f)
  , pos_(pos)
  , yaw_(0.0f)
  , pitch_(0.0f) {}

static inline void clamp360(float* v) {
  if(*v > 360.0f)
    *v -= 360.0f;
  if(*v < -360.0f)
    *v += 360.0f;
}

void Camera::yaw(float degrees) {
  yaw_ += degrees;
  clamp360(&yaw_);
  yawMatrix_.setToIdentity();
  yawMatrix_.rotate(yaw_, 0, 1, 0);

  QMatrix4x4 rotMat = pitchMatrix_ * yawMatrix_;
  forward_ = (QVector4D(0.0f, 0.0f, -1.0f, 0.0f) * rotMat).toVector3D();
  right_ = (QVector4D(1.0f, 0.0f, 0.0f, 0.0f) * rotMat).toVector3D();
}

void Camera::pitch(float degrees) {
  pitch_ += degrees;
  clamp360(&pitch_);
  pitchMatrix_.setToIdentity();
  pitchMatrix_.rotate(pitch_, 1, 0, 0);

  QMatrix4x4 rotMat = pitchMatrix_ * yawMatrix_;
  forward_ = (QVector4D(0.0f, 0.0f, -1.0f, 0.0f) * rotMat).toVector3D();
  up_ = (QVector4D(0.0f, 1.0f, 0.0f, 0.0f) * rotMat).toVector3D();
}

void Camera::walk(float amount) {
  pos_[0] += amount * forward_.x();
  pos_[2] += amount * forward_.z();
}

void Camera::strafe(float amount) {
  pos_[0] += amount * right_.x();
  pos_[2] += amount * right_.z();
}

QMatrix4x4 Camera::viewMatrix() const {
  QMatrix4x4 m = pitchMatrix_ * yawMatrix_;
  m.translate(-pos_);
  return m;
}
