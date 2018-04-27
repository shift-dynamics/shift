#ifndef _SHIFT_FLOORSHADOWSHADERPROGRAM_HPP_
#define _SHIFT_FLOORSHADOWSHADERPROGRAM_HPP_

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

class FloorShadowShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  FloorShadowShaderProgram(RenderWindow* renderWindow)
    : ShaderProgram(renderWindow) {
    initialize();
  };

  ~FloorShadowShaderProgram(){};

  void initialize();

  void draw();

  void setTransform(QMatrix4x4&);

  void setTransform(s2_displacement_t*);

  void bindVao() {
    m_vao->bind();
  }

  void releaseVao() {
    m_vao->release();
  }

signals:

protected:
private:
  /* matrix uniform locations */
  GLint m_matrices_mvp_u;
  QOpenGLVertexArrayObject* m_vao;
};
} // namespace Shift
#endif
