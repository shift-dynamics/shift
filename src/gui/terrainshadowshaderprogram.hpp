#ifndef _SHIFT_TERRAINSHADOWSHADERPROGRAM_HPP_
#define _SHIFT_TERRAINSHADOWSHADERPROGRAM_HPP_

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

class TerrainShadowShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  TerrainShadowShaderProgram(RenderWindow* renderWindow)
    : ShaderProgram(renderWindow) {
    initialize();
  };

  ~TerrainShadowShaderProgram(){};

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

  void setScale(GLfloat, GLfloat);

  void setViewPort(GLfloat, GLfloat);

  void setTextureElevation(QOpenGLTexture*);

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
  GLint m_terrain_scale_u;
  GLint m_terrain_viewport_u;

  GLint m_texture_elevation_u;

  QOpenGLVertexArrayObject* m_vao;
};
} // namespace Shift
#endif
