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
  , m_count(0) {

    memset(m_position, 0, 30 * sizeof(GLfloat));
    memset(m_Ka, 0, 30 * sizeof(GLfloat));
    memset(m_Kd, 0, 30 * sizeof(GLfloat));
    memset(m_Ks, 0, 30 * sizeof(GLfloat));
    memset(m_directional, 0, 10 * sizeof(GLfloat));

  }
  ~s2_Lights();

  int getCount() {
    return m_count;
  }

  GLfloat* getPosition() {
    return m_position;
  }

  GLfloat* getKa() {
    return m_Ka;
  }

  GLfloat* getKd() {
    return m_Kd;
  }

  GLfloat* getKs() {
    return m_Ks;
  }

  GLint* getDirectional() {
    return m_directional;
  }

  void initialize();
  void draw();
  void loadContext();




private:

  int m_count;
  GLfloat m_position[30];
  GLfloat m_Ka[30];
  GLfloat m_Kd[30];
  GLfloat m_Ks[30];
  GLint m_directional[10];
};


#endif
