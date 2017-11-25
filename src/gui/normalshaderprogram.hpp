#ifndef _STAR2_NORMALSHADERPROGRAM_HPP_
#define _STAR2_NORMALSHADERPROGRAM_HPP_

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


class NormalShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  NormalShaderProgram(RenderWindow* renderWindow)
  : ShaderProgram(renderWindow) {
    initialize();
  };

  ~NormalShaderProgram() {};

  void initialize();
  void draw();
  void setTransform(s2_displacement_t*);
  void setColor(float&, float&, float&);
  void setScale(float&);
  GLint getPositionAttribute() {
    return m_position_a;
  };
  GLint getNormalAttribute() {
    return m_normal_a;
  };

signals:


protected:


private:

  GLint m_matrices_u;
  GLint m_matrices_mvp_u;
  GLint m_matrices_mv_u;
  GLint m_matrices_v_u;
  GLint m_matrices_nm_u;
  
  GLint m_color_u;
  GLint m_scale_u;

  GLint m_position_a;
  GLint m_normal_a;

};



#endif
