#ifndef _SHIFT_FLOOR_HPP_
#define _SHIFT_FLOOR_HPP_

#include "../shift.h"

#include "geometry.hpp"
#include "openglwindow.hpp"
#include "renderwindow.hpp"

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

namespace Shift {

class FloorShaderProgram;
class FloorShadowShaderProgram;

class Floor : public Geometry {

  Q_OBJECT

public:
  Floor(RenderWindow* renderWindow)
    : Geometry(renderWindow) {

    m_scaleFactor = 100.0;
  }

  ~Floor() {
    // s2_displacement_t* origin = m_frames.at(0);
    // free(origin);
  }

  void draw();

  void drawShadows();

  void initialize();

  void setShader(FloorShaderProgram* program) {
    m_program = program;
  };

  void setShadowShader(FloorShadowShaderProgram* program) {
    m_programShadow = program;
  };

private:
  QOpenGLTexture* m_texture_diffuse = NULL;
  QOpenGLTexture* m_texture_bump = NULL;
  QOpenGLTexture* m_texture_specular = NULL;
  FloorShaderProgram* m_program = NULL;
  FloorShadowShaderProgram* m_programShadow = NULL;
};
} // namespace Shift

#endif
