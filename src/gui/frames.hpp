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
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QtOpenGL/QGLWidget>
#include <QHBoxLayout>
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QWidget>
#include <QString>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QResizeEvent>
#include <QTimer>
#include <QOpenGLTexture>

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

class s2_Frames : public s2_Geometry {
public:
  s2_Frames(RenderWindow* renderWindow) {
    m_renderWindow = renderWindow;
    initialize();
  };
  ~s2_Frames();
  void initialize();
  void draw();
  void setContext(s2_context_t*);

private:
  s2_context_t* m_context;
  GLfloat m_vertices[18];
  GLfloat m_colors[18];
  GLuint m_indices[6];

  RenderWindow* m_renderWindow;

  /* uniform locations */
  GLuint m_position;
  GLuint m_mvp;
  GLuint m_color;

  /* vertex'face buffers */
  QOpenGLVertexArrayObject* m_vao;
  QOpenGLBuffer* m_vbo;
  QOpenGLBuffer* m_ibo;

};




#endif
