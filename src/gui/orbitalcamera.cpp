#include "orbitalcamera.hpp"
#include "renderwindow.hpp"


void s2_OrbitalCamera::wheelEvent(QWheelEvent *ev) {
  m_translate +=
    m_lookMat.inverted().column(2).toVector3D() * (ev->delta() / 1000.f);
  // setScroll(scroll() + ev->delta() / 1000.0);
  update();
}


void s2_OrbitalCamera::update() {
    switch (m_mode) {
      case OrbitalCameraMode::Translate: {
        updateTranslate();
      } break;
      case OrbitalCameraMode::Rotate: {
        updateRotate();
      } break;
      case OrbitalCameraMode::Scale: {
        updateScale();
      }break;
      default: {
      } break;
    }

    m_modelMat = m_rotMat;
    // m_modelMat.scale(1.0 - m_scroll * 0.1);

    m_viewMat = m_lookMat;
    m_viewMat.translate(m_translate);
}


void s2_OrbitalCamera::updateScale() {
    const double dy = 20.0 * (m_newPoint.y() - m_oldPoint.y()) / m_renderWindow->height();
    m_scroll += dy;
}


void s2_OrbitalCamera::updateTranslate() {
  const QMatrix4x4 camera2objMat = m_lookMat.inverted();

  const double dx = 10.0 * (m_newPoint.x() - m_oldPoint.x()) / m_renderWindow->width();
  const double dy = -10.0 * (m_newPoint.y() - m_oldPoint.y()) / m_renderWindow->height();

  m_translate += camera2objMat * QVector3D(dx, dy, 0.0);
}


void s2_OrbitalCamera::updateRotate() {
  static const double Pi = 4.0 * std::atan(1.0);

  const QVector3D u =
    getVector(m_newPoint.x(), m_newPoint.y());
  const QVector3D v =
    getVector(m_oldPoint.x(), m_oldPoint.y());

  const double angle = std::acos(std::min(1.0f, QVector3D::dotProduct(u, v)));

  const QVector3D rotAxis = QVector3D::crossProduct(v, u);
  const QMatrix4x4 camera2objMat = m_rotMat.inverted();

  const QVector3D objSpaceRotAxis = camera2objMat * rotAxis;

  QMatrix4x4 temp;
  double angleByDegree = 180.0 * angle / Pi;
  temp.rotate(4.0 * angleByDegree, objSpaceRotAxis);

  m_rotMat = m_rotMat * temp;
}


QVector3D s2_OrbitalCamera::getVector(int x, int y) {
  QVector3D pt( 2.0 * x / m_renderWindow->width()  - 1.0,
               -2.0 * y / m_renderWindow->height() + 1.0,
                0.0);

  const double xySquared = pt.x() * pt.x() + pt.y() * pt.y();
  if (xySquared) {
    pt.setZ(std::sqrt(1.0 - xySquared));
  } else {
    pt.normalized();
  }

  return pt;
}
