#ifndef _SHIFT_SHADOWSHADERPROGRAM_HPP_
#define _SHIFT_SHADOWSHADERPROGRAM_HPP_

#include "../shift.h"
#include "openglwindow.hpp"
#include "shaderprogram.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLDebugLogger>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>
#include <QResizeEvent>
#include <QString>
#include <QTimer>
#include <QWheelEvent>
#include <QWidget>
#include <QtOpenGL/QGLWidget>

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

namespace Shift {

class RenderWindow;
class Lights;

class ShadowShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  ShadowShaderProgram(RenderWindow* renderWindow)
    : ShaderProgram(renderWindow) {
    initialize();
  };

  ~ShadowShaderProgram(){};

  void initialize();

  void setTransform(s2_displacement_t*);

signals:

protected:
private:
  GLuint m_mvp_u;
  GLuint m_scale_u;
  GLint m_position_a;
};
} // namespace Shift
#endif
