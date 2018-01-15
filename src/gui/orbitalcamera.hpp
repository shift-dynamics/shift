#ifndef _S2_ORBITALCAMERA_H_
#define _S2_ORBITALCAMERA_H_

#include <cmath>

#include <QtGui/qevent.h>
#include <QtGui/qmatrix4x4.h>
#include <QtGui/qvector3d.h>
#include <QtWidgets/qwidget.h>

namespace Shift {

class RenderWindow;

enum class OrbitalCameraMode : int {
  None = 0x00,
  Translate = 0x01,
  Rotate = 0x02,
  Scale = 0x04
};

class OrbitalCamera {

public:
  OrbitalCamera(RenderWindow* renderWindow) {
    m_renderWindow = renderWindow;
    m_view.setToIdentity();
    setLookAt(QVector3D(-2, -2, -2), QVector3D(0.0, 0.0, 0.0),
              QVector3D(0.0, 0.0, -1.0));
  }

  void setLookAt(const QVector3D& pos, const QVector3D& to,
                 const QVector3D& up);

  void setPerspective(float fov, float aspect, float nearClip, float farClip);

  void setOrtho(float left, float right, float bottom, float top,
                float nearClip, float farClip);

  void update();

  inline QMatrix4x4 getViewMatrix() const {
    return m_view;
  }

  inline double scroll() const {
    return m_scroll;
  }

  inline void setMode(OrbitalCameraMode mode) {
    m_mode = mode;
  }

  inline void setOldPoint(const QPoint& pos) {
    m_oldPoint = pos;
  }

  inline void setNewPoint(const QPoint& pos) {
    m_newPoint = pos;
  }

  inline void setScroll(double scroll) {
    m_scroll = scroll;
  }

  void mousePressEvent(QMouseEvent* ev);

  void mouseMoveEvent(QMouseEvent* ev);

  void mouseReleaseEvent(QMouseEvent* ev);

  void wheelEvent(QWheelEvent* ev);

private:
  QVector3D getVector(int x, int y);

  void updateTranslate();

  void updateRotate();

  void updateScale();

  QMatrix4x4 m_view;

  double m_scroll = 0.0;

  QPoint m_oldPoint = QPoint(0, 0);

  QPoint m_newPoint = QPoint(0, 0);

  OrbitalCameraMode m_mode = OrbitalCameraMode::None;

  RenderWindow* m_renderWindow;
};

} // namespace Shift

#endif
