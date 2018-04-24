#ifndef _SHIFT_FLOOR_HPP_
#define _SHIFT_FLOOR_HPP_

#include "../shift.h"
#include "floorshaderprogram.hpp"
#include "floorshadowshaderprogram.hpp"
#include "geometry.hpp"
#include "openglwindow.hpp"
#include "renderwindow.hpp"

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

namespace Shift {

class FrameShaderProgram;

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

  void initialize();

private:
  QOpenGLTexture* m_texture_diffuse;
  QOpenGLTexture* m_texture_bump;
  QOpenGLTexture* m_texture_specular;
  FloorShaderProgram* m_program;
  FloorShadowShaderProgram* m_programShadow;
};
} // namespace Shift

#endif
