#include "orbitalcamera.hpp"
#include "renderwindow.hpp"

QMatrix4x4 getRotation(QMatrix4x4& mat) {
  QMatrix4x4 rotation = mat;
  rotation(0, 3) = 0.f;
  rotation(1, 3) = 0.f;
  rotation(2, 3) = 0.f;
  return rotation;
}

void Shift::OrbitalCamera::wheelEvent(QWheelEvent* ev) {
  QVector3D temp =
    m_view.transposed().column(2).toVector3D() * (ev->delta() / 1000.f);
  m_view.translate(temp);
  update();
}

void Shift::OrbitalCamera::setLookAt(const QVector3D& pos, const QVector3D& to,
                                     const QVector3D& up) {
  m_view.setToIdentity();
  m_view.lookAt(pos, to, up);
  update();
}

void Shift::OrbitalCamera::mousePressEvent(QMouseEvent* ev) {
  setOldPoint(ev->pos());
  setNewPoint(ev->pos());
  switch(ev->button()) {
    case Qt::LeftButton: {
      setMode(OrbitalCameraMode::Rotate);
      break;
    }
    case Qt::RightButton: {
      setMode(OrbitalCameraMode::Translate);
      break;
    }
    case Qt::MiddleButton: {
      setMode(OrbitalCameraMode::Scale);
      break;
    }
    default:
      break;
  }
}

void Shift::OrbitalCamera::update() {
  switch(m_mode) {
    case OrbitalCameraMode::Translate: {
      updateTranslate();
      break;
    }
    case OrbitalCameraMode::Rotate: {
      updateRotate();
      break;
    }
    case OrbitalCameraMode::Scale: {
      updateScale();
      break;
    }
    default:
      break;
  }
}

void Shift::OrbitalCamera::mouseMoveEvent(QMouseEvent* ev) {
  setNewPoint(ev->pos());
  update();
  setOldPoint(ev->pos());
}

void Shift::OrbitalCamera::mouseReleaseEvent(QMouseEvent*) {
  setMode(OrbitalCameraMode::None);
}

void Shift::OrbitalCamera::updateScale() {
  const double dy =
    20.0 * (m_newPoint.y() - m_oldPoint.y()) / m_renderWindow->height();
  m_scroll += dy;
}

void Shift::OrbitalCamera::updateTranslate() {
  QMatrix4x4 camera2objMat = getRotation(m_view).transposed();
  const double dx =
    10.0 * (m_newPoint.x() - m_oldPoint.x()) / m_renderWindow->width();
  const double dy =
    -10.0 * (m_newPoint.y() - m_oldPoint.y()) / m_renderWindow->height();
  m_view.translate(camera2objMat * QVector3D(dx, dy, 0.0));
}

void Shift::OrbitalCamera::updateRotate() {
  const QVector3D va = getVector(m_oldPoint.x(), m_oldPoint.y());
  const QVector3D vb = getVector(m_newPoint.x(), m_newPoint.y());
  const double angle = std::acos(std::min(1.0f, QVector3D::dotProduct(va, vb)));
  const QVector3D rotAxis = QVector3D::crossProduct(va, vb);
  QMatrix4x4 temp;
  double angleByDegree = 180.0 * angle / M_PI;
  temp.rotate(4.0 * angleByDegree, rotAxis);

  /*! @TODO if user has clicked on a vertex in model, find rotation center to
  apply rotation about user-defined position, otherwise, rotate camera about a
  point 1 m in front of camera */
  QMatrix4x4 viewToRotCenter;
  viewToRotCenter.translate(0, 0, 1);
  QMatrix4x4 rotCenter = viewToRotCenter * m_view;

  QMatrix4x4 rotCenterToView;
  rotCenterToView.translate(0, 0, -1);
  m_view = rotCenterToView * temp * rotCenter;
}

QVector3D Shift::OrbitalCamera::getVector(int x, int y) {
  QVector3D pt(2.0 * x / m_renderWindow->width() - 1.0,
               -2.0 * y / m_renderWindow->height() + 1.0, 0.0);

  const double xySquared = pt.x() * pt.x() + pt.y() * pt.y();
  if(xySquared) {
    pt.setZ(std::sqrt(1.0 - xySquared));
  } else {
    pt.normalized();
  }
  return pt;
}
