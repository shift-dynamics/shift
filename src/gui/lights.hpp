#ifndef _STAR2_LIGHTS_HPP_
#define _STAR2_LIGHTS_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "geometry.hpp"

#include <QString>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QtOpenGL/QGLWidget>
#include <QHBoxLayout>
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QWidget>
#include <QString>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QResizeEvent>
#include <QTimer>
#include <QOpenGLTexture>

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;
class RenderWindow;

class s2_Lights : public s2_Geometry {

  Q_OBJECT

public:
  s2_Lights(RenderWindow* renderWindow)
  : s2_Geometry(renderWindow)
  , m_lightCount(0) {

    memset(m_lightPosition, 0, 30 * sizeof(GLfloat));
    memset(m_KaLight, 0, 30 * sizeof(GLfloat));
    memset(m_KdLight, 0, 30 * sizeof(GLfloat));
    memset(m_KsLight, 0, 30 * sizeof(GLfloat));
    memset(m_directional, 0, 10 * sizeof(GLfloat));

  }
  ~s2_Lights();

  int getLightCount() {
    return m_lightCount;
  }

  GLfloat* getLightPosition() {
    return m_lightPosition;
  }

  GLfloat* getKa() {
    return m_KaLight;
  }

  GLfloat* getKd() {
    return m_KdLight;
  }

  GLfloat* getKs() {
    return m_KsLight;
  }

  GLint* getDirectional() {
    return m_directional;
  }

  virtual void initialize();

  virtual void draw();

private:

  int m_lightCount;
  GLfloat m_lightPosition[30];
  GLfloat m_KaLight[30];
  GLfloat m_KdLight[30];
  GLfloat m_KsLight[30];
  GLint m_directional[10];
};


#endif
