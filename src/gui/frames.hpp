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

  Q_OBJECT

public:
  s2_Frames(RenderWindow* renderWindow)
    : s2_Geometry(renderWindow) {

    m_scaleFactor = 0.5;
    initialize();
  };

  ~s2_Frames();

  virtual void initialize();
  virtual void draw();
  void drawFrames();
  void drawWFrames();
  void loadContext();

private:

  QOpenGLShaderProgram* m_program;
  GLfloat m_vertices[18];
  GLfloat m_colors[18];
  GLuint m_indices[6];

  /* uniform locations */
  GLuint m_mvp;
  GLuint m_posAttr;
  GLuint m_colorAttr;

  /* vertex, face and index buffers */
  QOpenGLVertexArrayObject* m_vao;
  QOpenGLBuffer* m_vbo;
  QOpenGLBuffer* m_ibo;

  QVector<s2_displacement_t*> m_frames;

};




#endif
