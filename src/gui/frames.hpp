#ifndef _STAR2_FRAMES_HPP_
#define _STAR2_FRAMES_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "geometry.hpp"
#include "renderwindow.hpp"

#include <QString>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMatrix4x4>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QtOpenGL/QGLWidget>
#include <QHBoxLayout>
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>


class FrameShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

class s2_Frames : public s2_Geometry {

    Q_OBJECT

  public:
    s2_Frames(RenderWindow* renderWindow)
      : s2_Geometry(renderWindow) {

      m_scaleFactor = 0.5;
      // m_frames.append(s2_displacement());
    }

    ~s2_Frames() {
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


#endif
