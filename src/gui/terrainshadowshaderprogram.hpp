#ifndef _STAR2_TERRAINSHADOWSHADERPROGRAM_HPP_
#define _STAR2_TERRAINSHADOWSHADERPROGRAM_HPP_

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


class TerrainShadowShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  TerrainShadowShaderProgram(RenderWindow* renderWindow)
  : ShaderProgram(renderWindow) {
    initialize();
  };

  ~TerrainShadowShaderProgram() {};

  void initialize();

  void draw();

  void setTransform(QMatrix4x4&);

  void setTransform(s2_displacement_t*);

  void bindVao() {
    m_vao->bind();
  }

  void releaseVao() {
    m_vao->release();
  }

  void setZExtents(GLfloat, GLfloat);

  void setViewPort(GLfloat, GLfloat);

  void setTextureDisplacement(QOpenGLTexture*);

  void setUniforms(s2_terrain_t*);

signals:


protected:


private:

  GLint m_matrices_mvp_u;
  GLint m_matrices_mv_u;
  GLint m_matrices_v_u;
  GLint m_matrices_p_u;

  GLint m_terrain_zMin_u;
  GLint m_terrain_zMax_u;
  GLint m_terrain_xScale_u;
  GLint m_terrain_yScale_u;
  GLint m_terrain_xRange_u;
  GLint m_terrain_yRange_u;
  GLint m_terrain_viewport_u;

  GLint m_texture_displacement_u;

  QOpenGLVertexArrayObject* m_vao;

};

#endif
