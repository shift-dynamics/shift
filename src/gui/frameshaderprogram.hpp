#ifndef _SHIFT_FRAMESHADERPROGRAM_HPP_
#define _SHIFT_FRAMESHADERPROGRAM_HPP_

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

class FrameShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  FrameShaderProgram(RenderWindow* renderWindow)
    : ShaderProgram(renderWindow) {
    m_vertices[0] = 0.f;
    m_vertices[1] = 0.f;
    m_vertices[2] = 0.f;

    m_vertices[3] = 1.f;
    m_vertices[4] = 0.f;
    m_vertices[5] = 0.f;

    m_vertices[6] = 0.f;
    m_vertices[7] = 1.f;
    m_vertices[8] = 0.f;

    m_vertices[9] = 0.f;
    m_vertices[10] = 0.f;
    m_vertices[11] = 1.f;

    m_indices[0] = 0;
    m_indices[1] = 1;
    m_indices[2] = 0;
    m_indices[3] = 2;
    m_indices[4] = 0;
    m_indices[5] = 3;

    initialize();
  };

  ~FrameShaderProgram(){};

  void initialize();
  void draw();
  void setTransform(s2_displacement_t*);
  void setMVP(QMatrix4x4&);
  void setScale(GLfloat);
  void bindVao();
  void releaseVao();

signals:

protected:
private:
  GLint m_mvp_u;
  GLint m_scale_u;

  GLint m_position_a;

  QOpenGLVertexArrayObject* m_vao;
  QOpenGLBuffer* m_vbo;
  QOpenGLBuffer* m_ibo;

  GLfloat m_vertices[12];
  GLuint m_indices[6];
};
} // namespace Shift
#endif
