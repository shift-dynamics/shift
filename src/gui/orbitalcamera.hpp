#ifndef _S2_ORBITALCAMERA_H_
#define _S2_ORBITALCAMERA_H_

#include <cmath>

#include <QtWidgets/qwidget.h>
#include <QtGui/qvector3d.h>
#include <QtGui/qmatrix4x4.h>
#include <QtGui/qevent.h>
class RenderWindow;



enum class OrbitalCameraMode : int {
    None = 0x00,
    Translate = 0x01,
    Rotate = 0x02,
    Scale = 0x04
};

class s2_OrbitalCamera {

public:

    s2_OrbitalCamera(RenderWindow* renderWindow) {
      m_renderWindow = renderWindow;
      m_modelMat.setToIdentity();
      m_viewMat.setToIdentity();
      m_projMat.setToIdentity();
      m_rotMat.setToIdentity();
      m_lookMat.setToIdentity();
    }

    void setLookAt(const QVector3D &pos, const QVector3D& to, const QVector3D &up) {
        m_lookMat.setToIdentity();
        m_lookMat.lookAt(pos, to, up);
        update();
    }

    void setPerspective(float fov, float aspect, float nearClip, float farClip) {
        m_projMat.setToIdentity();
        m_projMat.perspective(fov, aspect, nearClip, farClip);
        update();
    }

    void setOrtho(
      float left, float right, float bottom, float top,
                  float nearClip, float farClip) {
        m_projMat.setToIdentity();
        m_projMat.ortho(left, right, bottom, top, nearClip, farClip);
        update();
    }

    void update();

    inline QMatrix4x4 getModelMatrix() const {
      return m_modelMat;
    }

    inline QMatrix4x4 getViewMatrix() const {
      return m_viewMat;
    }

    inline QMatrix4x4 getProjectionMatrix() const {
      return m_projMat;
    }

    inline QMatrix4x4 getMVMatrix() const {
      return m_viewMat * m_modelMat;
    }

    inline QMatrix4x4 getMVPMatrix() const {
      return m_projMat * m_viewMat * m_modelMat;
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

    void mousePressEvent(QMouseEvent *ev) {
      setOldPoint(ev->pos());
      setNewPoint(ev->pos());
      if (ev->button() == Qt::LeftButton) {
        setMode(OrbitalCameraMode::Rotate);
      } else if (ev->button() == Qt::RightButton) {
        setMode(OrbitalCameraMode::Translate);
      } else if (ev->button() == Qt::MiddleButton) {
        setMode(OrbitalCameraMode::Scale);
      }
    }

    void mouseMoveEvent(QMouseEvent *ev) {
      setNewPoint(ev->pos());
      update();
      setOldPoint(ev->pos());
    }

    void mouseReleaseEvent(QMouseEvent *ev) {
      setMode(OrbitalCameraMode::None);
    }

    void wheelEvent(QWheelEvent *ev);

private:
    QVector3D getVector(int x, int y);

    void updateTranslate();

    void updateRotate();

    void updateScale();

    QMatrix4x4 m_modelMat;
    QMatrix4x4 m_viewMat;
    QMatrix4x4 m_projMat;
    double m_scroll = 0.0;
    QPoint m_oldPoint = QPoint(0, 0);
    QPoint m_newPoint = QPoint(0, 0);

    OrbitalCameraMode m_mode = OrbitalCameraMode::None;
    QVector3D m_translate = QVector3D(0.0f, 0.0f, 0.0f);
    QMatrix4x4 m_lookMat;
    QMatrix4x4 m_rotMat;
    RenderWindow* m_renderWindow;
};

#endif
