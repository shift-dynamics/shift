#ifndef _STAR2_SHADOWSHADERPROGRAM_HPP_
#define _STAR2_SHADOWSHADERPROGRAM_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "shaderprogram.hpp"

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
class s2_Lights;


class ShadowShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  ShadowShaderProgram(RenderWindow* renderWindow)
  : ShaderProgram(renderWindow) {
    initialize();
  };

  ~ShadowShaderProgram() {};

  void initialize();

  void setTransform(s2_displacement_t*);

signals:


protected:


private:

  GLuint m_mvp_u;
  GLuint m_scale_u;
  GLint m_position_a;

};

#endif
