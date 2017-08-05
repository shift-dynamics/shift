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

class s2_Lights : public s2_Geometry {

public:
  s2_Lights();
  ~s2_Lights();
  void initialize();
  void draw();

private:
  QVector<s2_displacement_t*> m_frames;
};




#endif
