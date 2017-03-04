#ifndef _STAR2_GLWIDGET_H_
#define _STAR2_GLWIDGET_H_

#include "../star2.h"
#include <QGLWidget>
#include <QString>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMatrix4x4>

class GLWidget : public QGLWidget {
  Q_OBJECT

public:
  explicit GLWidget(QWidget* parent = 0);
  // void initializeGL();
  void updateGL();
  void paintGL();
  void resizeGL(int width, int height);
  void initializeGL();
  // bool event(QEvent* event);
  void loadContext(s2_context_t* context);
  QString getCursorPositionText();
  void checkError(const QString &prefix);

public slots:
  void initialize();
  void setXRotation(int angle);
  void setYRotation(int angle);
  void setZRotation(int angle);
  // void setPerspective();
  // void setOrthogonal();
  // void toggleVertexNormalDisplay();

signals:
  void mouseLeftPressEvent();
  void mouseRightPressEvent();
  void mouseReleaseEvent();
  void mouseMove();
  void mouseOut();
  void xRotationChanged(int angle);
  void yRotationChanged(int angle);
  void zRotationChanged(int angle);

protected:
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void wheelEvent(QWheelEvent* event);

private:
  s2_context_t* m_context;

  QMatrix4x4 m_projection;
  QMatrix4x4 m_view;

  int m_frame;
  int m_x;
  int m_y;

  int m_xRot;
  int m_yRot;
  int m_zRot;

  float m_xPan;
  float m_yPan;
  float m_zPan;

  QPoint m_lastPos;
  float m_camDistance;

  bool m_perspective;
  bool m_drawVertexNormals;

  GLfloat m_backgroundColor[4];
  GLfloat m_normalColor[4];

  GLuint m_programLine;
  GLuint m_programGouraud;
  GLuint m_programPhong;
  GLuint m_programFlat;

  LightUniforms m_lightGouraud;
  LightUniforms m_lightPhong;
  LightUniforms m_lightFlat;

  const char* getShaderSource(
    const char* shaderSource);

  void checkShader(
    GLuint shader);

  bool m_initialized;

};

#endif
