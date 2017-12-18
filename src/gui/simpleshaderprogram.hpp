#ifndef _SHIFT_SIMPLESHADERPROGRAM_HPP_
#define _SHIFT_SIMPLESHADERPROGRAM_HPP_

#include "../shift.h"
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


class SimpleShaderProgram : public ShaderProgram {

    Q_OBJECT

  public:
    SimpleShaderProgram(RenderWindow* renderWindow)
      : ShaderProgram(renderWindow) {
      initialize();
    };

    ~SimpleShaderProgram() {};

    void initialize();
    void draw();
    void setTransform(s2_displacement_t*);
    void setTransform(QMatrix4x4&);
    void setColor(GLfloat, GLfloat, GLfloat, GLfloat);
    void setScale(GLfloat);

    GLint getPositionAttribute() {
      return m_position_a;
    };

  signals:


  protected:


  private:

    /* uniforms */
    GLint m_mvp_u;
    GLint m_color_u;
    GLint m_scale_u;

    /* attributes */
    GLint m_position_a;

};

#endif
