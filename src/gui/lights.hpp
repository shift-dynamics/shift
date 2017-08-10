#ifndef _STAR2_LIGHTS_HPP_
#define _STAR2_LIGHTS_HPP_

#include "../star2.h"
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

struct s2_lights_t {
  GLfloat position[30];
  GLfloat Ka[30];
  GLfloat Kd[30];
  GLfloat Ks[30];
  GLint directional[10];
  int count;
};

class s2_Lights : public s2_Geometry {

  Q_OBJECT

public:
  s2_Lights(RenderWindow* renderWindow);
  ~s2_Lights() {

  };

  int getCount() {
    return m_lights.count;
  };

  GLfloat* getPosition() {
    return m_lights.position;
  }

  GLfloat* getKa() {
    return m_lights.Ka;
  }

  GLfloat* getKd() {
    return m_lights.Kd;
  }

  GLfloat* getKs() {
    return m_lights.Ks;
  }

  GLint* getDirectional() {
    return m_lights.directional;
  }

  void initialize();
  void draw();
  void loadContext();

private:

  s2_lights_t m_lights;

};


#endif
