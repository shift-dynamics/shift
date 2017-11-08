#ifndef _STAR2_POINTCLOUDSHADERPROGRAM_HPP_
#define _STAR2_POINTCLOUDSHADERPROGRAM_HPP_

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

enum s2_pointshader_enumt_t {
  S2_POINTSHADER_PHONG,
  S2_POINTSHADER_SIMPLE,
  S2_POINTSHADER_SPRITE
};


class PointCloudShaderProgram {

  Q_OBJECT

public:
  PointCloudShaderProgram(RenderWindow* renderWindow)
  : m_initialized(false)
  , m_phongShader(NULL)
  , m_simpleShader(NULL)
  , m_spriteShader(NULL)
  , m_shaderType(S2_POINTSHADER_SIMPLE) {
    m_renderWindow = renderWindow;
    initialize();
  };

  ~PointCloudShaderProgram() {};

  void initialize();
  void draw();
  void setTransform(s2_displacement_t*);
  void setMaterial(s2_material_t*);
  void setLights(s2_Lights*);

  GLint getPositionAttribute();
  GLint getColorAttribute();
  void bindDepthBuffer();

signals:


protected:


private:

  bool m_initialized;
  RenderWindow* m_renderWindow;
  ShaderProgram* m_phongShader;
  ShaderProgram* m_simpleShader;
  ShaderProgram* m_spriteShader;
  s2_pointshader_enumt_t m_shaderType;

  QOpenGLTexture* m_texture_normal;
  QOpenGLTexture* m_texture_sprite;

  void initializePhongShaderProgram();
  void initializeSimpleShaderProgram();
  void initializeSpriteShaderProgram();

  struct phongUniforms {
    GLint m_matrices_mvp_u;
    GLint m_matrices_mv_u;
    GLint m_matrices_v_u;

    GLint m_lights_position_u;
    GLint m_lights_Ka_u;
    GLint m_lights_Kd_u;
    GLint m_lights_Ks_u;
    GLint m_lights_directional_u;
    GLint m_lights_count_u;
    GLint m_lights_drawShadows_u;
    GLint m_lights_shadowMatrix_u;

    GLint m_texture_normal_u;

    GLint m_position_a;
    GLint m_color_a;
  } m_phongUniforms;

  struct simpleUniforms {
    GLint m_matrices_mvp_u;
    GLint m_matrices_mv_u;
    GLint m_position_a;
    GLint m_color_a;
  } m_simpleUniforms;

  struct spriteUniforms {
    GLint m_matrices_mvp_u;
    GLint m_matrices_mv_u;

    GLint m_texture_sprite_u;

    GLint m_position_a;
    GLint m_color_a;
  } m_spriteUniforms;

};



#endif
