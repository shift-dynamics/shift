#ifndef _SHIFT_SIMPLESHADERPROGRAM_HPP_
#define _SHIFT_SIMPLESHADERPROGRAM_HPP_

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

class SimpleShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  SimpleShaderProgram(RenderWindow* renderWindow)
    : ShaderProgram(renderWindow) {
    initialize();
  };

  ~SimpleShaderProgram(){};

  void initialize();
  void draw();
  void setTransform(s2_displacement_t*);
  void setTransform(QMatrix4x4&);
  void setColor(GLfloat, GLfloat, GLfloat, GLfloat);
  void setScale(GLfloat);

  GLint getPositionAttribute() {
    return m_position_a;
  };

signals:

protected:
private:
  /* uniforms */
  GLint m_mvp_u;
  GLint m_color_u;
  GLint m_scale_u;

  /* attributes */
  GLint m_position_a;
};

} // namespace Shift
#endif
