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

enum s2_point_shading_enum_t {
  S2_POINT_SHADING_PHONG,
  S2_POINT_SHADING_SIMPLE,
  S2_POINT_SHADING_SPRITE
};


class PointCloudShaderProgram : QObject {

    Q_OBJECT

  public:
    PointCloudShaderProgram(RenderWindow* renderWindow)
      : m_initialized(false)
      , m_phongShader(NULL)
      , m_simpleShader(NULL)
      , m_spriteShader(NULL)
      , m_shadingType(S2_POINT_SHADING_SIMPLE) {
      m_renderWindow = renderWindow;
      initialize();
    };

    ~PointCloudShaderProgram() {};

    void initialize();
    void draw();
    void setTransform(s2_displacement_t*);
    void setMaterial(s2_material_t*);
    void setLights(s2_Lights*);
    void setShadingType(s2_point_shading_enum_t shadingType) {
      m_shadingType = shadingType;
    }

    GLint getPositionAttribute();
    GLint getColorAttribute();
    GLint getPositionAttribute(s2_point_shading_enum_t);
    GLint getColorAttribute(s2_point_shading_enum_t);
    void bindDepthBuffer();
    void bindTextures();

    ShaderProgram* getShaderProgram(s2_point_shading_enum_t shading_type) {
      switch(shading_type) {
        case S2_POINT_SHADING_PHONG: {
          return m_phongShader;
        }
        break;
        case S2_POINT_SHADING_SIMPLE: {
          return m_simpleShader;
        }
        break;
        case S2_POINT_SHADING_SPRITE: {
          return m_spriteShader;
        }
        break;
      }
    }

    void bind() {
      switch(m_shadingType) {
        case S2_POINT_SHADING_PHONG: {
          m_phongShader->bind();
        }
        break;
        case S2_POINT_SHADING_SIMPLE: {
          m_simpleShader->bind();
        }
        break;
        case S2_POINT_SHADING_SPRITE: {
          m_spriteShader->bind();
        }
        break;
      }
    }

    void release() {
      switch(m_shadingType) {
        case S2_POINT_SHADING_PHONG: {
          m_phongShader->release();
        }
        break;
        case S2_POINT_SHADING_SIMPLE: {
          m_simpleShader->release();
        }
        break;
        case S2_POINT_SHADING_SPRITE: {
          m_spriteShader->release();
        }
        break;
      }
    }


  signals:


  protected:


  private:

    bool m_initialized;
    RenderWindow* m_renderWindow;
    ShaderProgram* m_phongShader;
    ShaderProgram* m_simpleShader;
    ShaderProgram* m_spriteShader;
    s2_point_shading_enum_t m_shadingType;

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

      GLint m_texture_shadow_u;
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
