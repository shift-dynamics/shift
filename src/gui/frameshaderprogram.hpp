#ifndef _STAR2_FRAMESHADERPROGRAM_HPP_
#define _STAR2_FRAMESHADERPROGRAM_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "shaderprogram.hpp"

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
class RenderWindow;
class s2_Lights;


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

    ~FrameShaderProgram() {};

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

#endif
