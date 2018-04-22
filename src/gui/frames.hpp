#ifndef _SHIFT_FRAMES_HPP_
#define _SHIFT_FRAMES_HPP_

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

class FrameShaderProgram;

class Frames : public Geometry {

  Q_OBJECT

public:
  Frames(RenderWindow* renderWindow)
    : Geometry(renderWindow) {

    m_scaleFactor = 0.5;
  }

  ~Frames() {
    // s2_displacement_t* origin = m_frames.at(0);
    // free(origin);
  }

  void draw();
  void addFrames(s2_assembly_t* assembly);
  void drawWFrames();
  void loadContext();
  void clearContext();
  void setShader(FrameShaderProgram* program) {
    m_program = program;
  }

private:
  FrameShaderProgram* m_program;
  QVector<s2_frame_t*> m_frames;
};
} // namespace Shift

#endif
