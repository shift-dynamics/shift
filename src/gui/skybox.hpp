#ifndef _SHIFT_SKYBOX_HPP_
#define _SHIFT_SKYBOX_HPP_

#include "../shift.h"
#include "geometry.hpp"
#include "lights.hpp"
#include "openglwindow.hpp"

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

struct s2_lights_t;
struct s2_collisionobject_t;

namespace Shift {

class RenderWindow;
class SkyboxShaderProgram;

/*!
@brief used to draw a skybox
*/
class Skybox : public Geometry {

  Q_OBJECT

public:
  Skybox(RenderWindow* renderWindow)
    : Geometry(renderWindow)
    , m_initialized(false)
    , m_drawEnabled(false){};

  ~Skybox(){

  };

  void initialize();
  void setShader(SkyboxShaderProgram* program) {
    m_program = program;
  }
  void draw();

private:
  QOpenGLTexture* m_cubeTexture;
  QOpenGLVertexArrayObject* m_vao;
  SkyboxShaderProgram* m_program;
  bool m_initialized;
  bool m_drawEnabled;
};

} // namespace Shift

#endif
