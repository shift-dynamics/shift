#ifndef _STAR2_TERRAINSHADERPROGRAM_HPP_
#define _STAR2_TERRAINSHADERPROGRAM_HPP_

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


class TerrainShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  TerrainShaderProgram(RenderWindow* renderWindow)
  : ShaderProgram(renderWindow) {
    initialize();
  };

  ~TerrainShaderProgram() {};

  void initialize();
  void setTransform(s2_displacement_t*);
  void setTransform(QMatrix4x4&);

  void setTextureDisplacement(QOpenGLTexture*);
  void setTextureColor(QOpenGLTexture*);
  void setTextureBump(QOpenGLTexture*);
  void setTextureNormal(QOpenGLTexture*);
  void setTextureSpecular(QOpenGLTexture*);

  void setLights(s2_Lights*);

  void setKd(GLfloat[4]);
  void setKaOverKd(GLfloat);
  void setKsOverKd(GLfloat);
  void setNs(GLint);
  void setUniforms(s2_elevationmap_t*);

  void setZExtents(GLfloat, GLfloat);
  void setScale(GLfloat, GLfloat);
  void setRange(GLfloat, GLfloat);
  void setViewPort(GLfloat, GLfloat);

signals:


protected:


private:

  /* texture uniform locations */
  GLint m_texture_color_u;
  GLint m_texture_displacement_u;
  // GLint m_texture_bump_u;
  // GLint m_texture_normal_u;
  // GLint m_texture_specular_u;

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
  GLint m_lights_directional_u;
  GLint m_lights_count_u;

  /* terrain uniform locations */
  // GLint m_terrain_u;
  GLint m_terrain_zMin_u;
  GLint m_terrain_zMax_u;
  // GLint m_terrain_xScale_u;
  // GLint m_terrain_yScale_u;
  // GLint m_terrain_xRange_u;
  // GLint m_terrain_yRange_u;
  GLint m_terrain_viewport_u;

};

#endif
