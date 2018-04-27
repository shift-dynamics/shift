#ifndef _SHIFT_SKYBOXSHADERPROGRAM_HPP_
#define _SHIFT_SKYBOXSHADERPROGRAM_HPP_

#include "openglwindow.h"
#include "shaderprogram.h"
#include "shift.h"

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

class SkyboxShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  SkyboxShaderProgram(RenderWindow* renderWindow)
    : ShaderProgram(renderWindow) {
    initialize();
  };

  ~SkyboxShaderProgram(){};

  void initialize();
  void setTransform();

private:
  GLint m_matrices_v_u;
  GLint m_texture_cube_u;
};

} // namespace Shift

#endif
