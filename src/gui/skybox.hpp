#ifndef _SHIFT_SKYBOX_HPP_
#define _SHIFT_SKYBOX_HPP_

#include "geometry.hpp"
#include "openglwindow.hpp"
#include "renderwindow.hpp"

#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>

namespace Shift {

class SkyboxShaderProgram;

/*!
@brief interface to skybox geometry
*/
class Skybox : public Geometry {

  Q_OBJECT

public:
  Skybox(RenderWindow* renderWindow)
    : Geometry(renderWindow)
    , m_program(NULL)
    , m_cubeTexture(NULL)
    , m_vao(NULL) {
    m_drawEnabled = false;
  };

  ~Skybox() {
    delete m_cubeTexture;
    delete m_vao;
  };

  void draw();
  void initialize();
  void setShader(SkyboxShaderProgram* program) {
    m_program = program;
  }

private:
  SkyboxShaderProgram* m_program;
  QOpenGLTexture* m_cubeTexture;
  QOpenGLVertexArrayObject* m_vao;
};

} // namespace Shift

#endif
