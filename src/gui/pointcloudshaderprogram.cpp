#include "pointcloudshaderprogram.hpp"
#include "renderwindow.hpp"
#include "lights.hpp"

void PointCloudShaderProgram::initialize() {
  if(!m_initialized) {
    initializeSimpleShaderProgram();
    initializeSpriteShaderProgram();
    initializePhongShaderProgram();
    m_initialized = true;
  }
}


void PointCloudShaderProgram::initializeSimpleShaderProgram() {
  m_simpleShader = new ShaderProgram(m_renderWindow);
  m_simpleShader->addProgram();
  m_simpleShader->addVertexShader(":/shaders/point_simple.vert");
  m_simpleShader->addFragmentShader(":/shaders/point_simple.frag");
  m_simpleShader->link();
  m_simpleShader->bind();

  /* set uniform locations */
  m_simpleUniforms.m_matrices_mvp_u =
    m_simpleShader->getProgram()->uniformLocation("matrices_u.mvp");
  m_simpleUniforms.m_matrices_mv_u =
    m_simpleShader->getProgram()->uniformLocation("matrices_u.mv");
  m_simpleShader->checkError("after set simple uniforms");

  /* set attribute locations */
  m_simpleUniforms.m_position_a =
    m_simpleShader->getProgram()->attributeLocation("position_a");
  m_simpleUniforms.m_color_a =
    m_simpleShader->getProgram()->attributeLocation("color_a");
  m_simpleShader->checkError("after set simple attrinutes");

  m_simpleShader->release();
}


void PointCloudShaderProgram::initializeSpriteShaderProgram() {
  m_spriteShader = new ShaderProgram(m_renderWindow);
  m_spriteShader->addProgram();
  m_spriteShader->addVertexShader(":/shaders/point_sprite.vert");
  m_spriteShader->addFragmentShader(":/shaders/point_sprite.frag");
  m_spriteShader->link();
  m_spriteShader->bind();

  /* set uniform locations */
  m_spriteUniforms.m_matrices_mvp_u =
    m_spriteShader->getProgram()->uniformLocation("matrices_u.mvp");
  m_spriteUniforms.m_matrices_mv_u =
    m_spriteShader->getProgram()->uniformLocation("matrices_u.mv");
  m_spriteUniforms.m_texture_sprite_u =
    m_spriteShader->getProgram()->uniformLocation("texture_sprite_u");
  m_spriteShader->checkError("after set simple uniforms");

  m_texture_sprite = new QOpenGLTexture(QImage(":/textures/sphere.png"));
  m_texture_sprite->setWrapMode(QOpenGLTexture::ClampToEdge);
  m_texture_sprite->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);

  /* set attribute locations */
  m_spriteUniforms.m_position_a =
    m_spriteShader->getProgram()->attributeLocation("position_a");
  m_spriteUniforms.m_color_a =
    m_spriteShader->getProgram()->attributeLocation("color_a");
  m_spriteShader->checkError("after set simple attrinutes");

  m_simpleShader->release();
}


void PointCloudShaderProgram::initializePhongShaderProgram() {
  m_phongShader = new ShaderProgram(m_renderWindow);
  m_phongShader->addProgram();
  m_phongShader->addVertexShader(":/shaders/phong_sprite.vert");
  m_phongShader->addFragmentShader(":/shaders/phong_sprite.frag");
  m_phongShader->link();
  m_phongShader->bind();

  /* set uniform locations */
  m_phongUniforms.m_matrices_mvp_u =
    m_phongShader->getProgram()->uniformLocation("matrices_u.mvp");
  m_phongUniforms.m_matrices_mv_u =
    m_phongShader->getProgram()->uniformLocation("matrices_u.mv");
  m_phongUniforms.m_matrices_v_u =
    m_phongShader->getProgram()->uniformLocation("matrices_u.v");

  m_phongUniforms.m_lights_position_u =
    m_phongShader->getProgram()->uniformLocation("lights_u.position");
  m_phongUniforms.m_lights_Ka_u =
    m_phongShader->getProgram()->uniformLocation("lights_u.Ka");
  m_phongUniforms.m_lights_Kd_u =
    m_phongShader->getProgram()->uniformLocation("lights_u.Kd");
  m_phongUniforms.m_lights_Ks_u =
    m_phongShader->getProgram()->uniformLocation("lights_u.Ks");
  m_phongUniforms.m_lights_directional_u =
    m_phongShader->getProgram()->uniformLocation("lights_u.directional");
  m_phongUniforms.m_lights_count_u =
    m_phongShader->getProgram()->uniformLocation("lights_u.count");
  m_phongUniforms.m_lights_drawShadows_u =
    m_phongShader->getProgram()->uniformLocation("lights_u.drawShadows");
  m_phongUniforms.m_lights_shadowMatrix_u =
    m_phongShader->getProgram()->uniformLocation("lights_u.shadowMatrix");

  m_phongUniforms.m_texture_normal_u =
    m_phongShader->getProgram()->uniformLocation("texture_normal_u");
  m_phongShader->checkError("after set simple uniforms");

  m_texture_normal = new QOpenGLTexture(QImage(":/textures/sphere_normal.png"));
  m_texture_normal->setWrapMode(QOpenGLTexture::ClampToEdge);
  m_texture_normal->setMinMagFilters(
    QOpenGLTexture::Linear, QOpenGLTexture::Linear);



  /// bind textures
  m_phongShader->bind();
  m_texture_normal->bind();
  m_phongShader->getProgram()->setUniformValue("texture_normal_u", 0);
  m_texture_normal->release();
  m_phongShader->release();

  m_programPhong->bind();
  m_normalTexture->bind();
  m_programPhong->setUniformValue("texture_normal_u", 0);
  m_normalTexture->release();
  m_programPhong->release();

  /* set attribute locations */
  m_phongUniforms.m_position_a =
    m_phongShader->getProgram()->attributeLocation("position_a");
  m_phongUniforms.m_color_a =
    m_phongShader->getProgram()->attributeLocation("color_a");
  m_phongShader->checkError("after set simple attrinutes");

  m_simpleShader->release();
}


GLint PointCloudShaderProgram::getPositionAttribute() {
  switch(m_shaderType) {
    case S2_POINTSHADER_PHONG: {
      return m_phongUniforms.m_position_a;
    } break;
    case S2_POINTSHADER_SIMPLE: {
      return m_simpleUniforms.m_position_a;
    } break;
    case S2_POINTSHADER_SPRITE: {
      return m_spriteUniforms.m_position_a;
    } break;
  }
};


GLint PointCloudShaderProgram::getColorAttribute() {
  switch(m_shaderType) {
    case S2_POINTSHADER_PHONG: {
      return m_phongUniforms.m_color_a;
    } break;
    case S2_POINTSHADER_SIMPLE: {
      return m_simpleUniforms.m_color_a;
    } break;
    case S2_POINTSHADER_SPRITE: {
      return m_spriteUniforms.m_color_a;
    } break;
  }
};


void PointCloudShaderProgram::setTransform(s2_displacement_t* transform) {

  QMatrix4x4 model = s2_displacementToQMatrix4x4(transform);
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  QMatrix4x4 mv = view * model;
  QMatrix4x4 mvp = projection * mv;
  QMatrix3x3 nm = mv.normalMatrix();

  switch(m_shaderType) {
    case S2_POINTSHADER_PHONG: {
      m_phongShader->getProgram()->setUniformValue(
        m_phongUniforms.m_matrices_mvp_u, mvp);
      m_phongShader->getProgram()->setUniformValue(
        m_phongUniforms.m_matrices_mv_u, mv);
      m_phongShader->getProgram()->setUniformValue(
        m_phongUniforms.m_matrices_v_u, view);

      if(m_renderWindow->getLights()->isEnabled()) {
        m_phongShader->getProgram()->setUniformValueArray(
          m_phongUniforms.m_lights_shadowMatrix_u,
          m_renderWindow->getLights()->setShadowMatrix(transform),
          m_renderWindow->getLights()->getMaxCount());
        m_phongShader->checkError("phong: after set shadow matrix array");
      }
    } break;
    case S2_POINTSHADER_SIMPLE: {

    } break;
    case S2_POINTSHADER_SPRITE: {

    } break;
  }
    m_program->setUniformValue(m_matrices_mvp_u, mvp);
    checkError("phong: after set mvp");

    m_program->setUniformValue(m_matrices_mv_u, mv);
    checkError("phong: after set mv");

    m_program->setUniformValue(m_matrices_v_u, view);
    checkError("phong: after set view");

    m_program->setUniformValue(m_matrices_nm_u, nm);
    checkError("phong: after set nm");


    }
  }

}


void PointCloudShaderProgram::setLights(s2_Lights* lights) {
  m_program->setUniformValueArray(m_lights_position_u, lights->getPosition(), 10, 3);
  m_program->setUniformValueArray(m_lights_Ka_u, lights->getKa(), 10, 3);
  m_program->setUniformValueArray(m_lights_Kd_u, lights->getKd(), 10, 3);
  m_program->setUniformValueArray(m_lights_Ks_u, lights->getKs(), 10, 3);
  m_program->setUniformValueArray(
    m_lights_directional_u, lights->getDirectional(), 10);
  m_program->setUniformValue(m_lights_count_u, lights->getCount());
  if(m_renderWindow->getLights()->isEnabled()) {
    m_program->setUniformValue(m_lights_drawShadows_u, 1);
  } else {
    m_program->setUniformValue(m_lights_drawShadows_u, 0);
  }
  checkError("after set lights");
}


/*!
@brief bind depth buffer to index 0
*/
void PointCloudShaderProgram::bindDepthBuffer() {
  m_program->setUniformValue(m_texture_shadow_u, 0);
}
