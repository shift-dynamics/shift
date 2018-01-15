#ifndef _SHIFT_TERRAINSHADERPROGRAM_HPP_
#define _SHIFT_TERRAINSHADERPROGRAM_HPP_

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

class TerrainShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  TerrainShaderProgram(RenderWindow* renderWindow)
    : ShaderProgram(renderWindow) {
    initialize();
  };

  ~TerrainShaderProgram(){};

  void initialize();
  void setTransform(s2_displacement_t*);
  void setTransform(QMatrix4x4&);

  void setTextureElevation(QOpenGLTexture*);
  void setTextureDiffuse(QOpenGLTexture*);
  void setTextureDisplacement(QOpenGLTexture*);
  void setTextureNormal(QOpenGLTexture*);
  void setTextureSpecular(QOpenGLTexture*);
  void setTextureBump(QOpenGLTexture*);

  void setLights(Lights*);

  void setKd(GLfloat[4]);
  void setKaOverKd(GLfloat);
  void setKsOverKd(GLfloat);
  void setNs(GLint);
  void setUniforms(s2_terrain_t*);

  void setZExtents(GLfloat, GLfloat);
  void setScale(GLfloat, GLfloat);
  void setTcScale(GLfloat, GLfloat);
  void setViewPort(GLfloat, GLfloat);

  void bindVao() {
    m_vao->bind();
  }

  void releaseVao() {
    m_vao->release();
  }

  void bindDepthBuffer();

signals:

protected:
private:
  /* texture uniform locations */
  GLint m_texture_shadow_u;
  GLint m_texture_shadowCube_u;
  GLint m_texture_diffuse_u;
  GLint m_texture_elevation_u;
  GLint m_texture_normal_u;
  GLint m_texture_specular_u;
  GLint m_texture_bump_u;

  /* matrix uniform locations */
  // GLint m_matrices_u;
  GLint m_matrices_mvp_u;
  GLint m_matrices_mv_u;
  GLint m_matrices_v_u;
  GLint m_matrices_p_u;
  GLint m_matrices_nm_u;

  /* material uniform locations */
  // GLint m_material_u;
  GLint m_material_Kd_u;
  GLint m_material_KaOverKd_u;
  GLint m_material_KsOverKd_u;
  GLint m_material_Ns_u;

  /* light uniform locations */
  // GLint m_lights_u;
  GLint m_lights_position_u;
  GLint m_lights_Ka_u;
  GLint m_lights_Kd_u;
  GLint m_lights_Ks_u;
  GLint m_lights_type_u;
  GLint m_lights_count_u;
  GLint m_lights_drawShadows_u;
  GLint m_lights_shadowMatrix_u;

  /* terrain uniform locations */
  // GLint m_terrain_u;
  GLint m_terrain_zMin_u;
  GLint m_terrain_zMax_u;
  GLint m_terrain_scale_u;
  GLint m_terrain_tcScale_u;
  GLint m_terrain_viewport_u;
  GLint m_terrain_enableDiffuseMap_u;
  GLint m_terrain_enableNormalMap_u;
  GLint m_terrain_enableSpecularMap_u;
  GLint m_terrain_enableBumpMap_u;

  QOpenGLVertexArrayObject* m_vao;
};
} // namespace Shift
#endif
