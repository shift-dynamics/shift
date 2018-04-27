#include "phongshaderprogram.h"
#include "lights.h"
#include "renderwindow.h"

void Shift::PhongShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/phong.glsl");
    addFragmentShader(":/shaders/phong.glsl");
    link();
    bind();

    /* set uniform locations */
    m_matrices_u = m_program->uniformLocation("matrices_u");
    m_matrices_mvp_u = m_program->uniformLocation("matrices_u.mvp");
    m_matrices_mv_u = m_program->uniformLocation("matrices_u.mv");
    m_matrices_v_u = m_program->uniformLocation("matrices_u.v");
    m_matrices_nm_u = m_program->uniformLocation("matrices_u.nm");

    m_material_u = m_program->uniformLocation("material_u");
    m_material_Ka_u = m_program->uniformLocation("material_u.Ka");
    m_material_Kd_u = m_program->uniformLocation("material_u.Kd");
    m_material_Ks_u = m_program->uniformLocation("material_u.Ks");
    m_material_Ke_u = m_program->uniformLocation("material_u.Ke");
    m_material_Ns_u = m_program->uniformLocation("material_u.Ns");

    m_lights_u = m_program->uniformLocation("lights_u");
    m_lights_position_u = m_program->uniformLocation("lights_u.position");
    m_lights_Ka_u = m_program->uniformLocation("lights_u.Ka");
    m_lights_Kd_u = m_program->uniformLocation("lights_u.Kd");
    m_lights_Ks_u = m_program->uniformLocation("lights_u.Ks");
    m_lights_type_u = m_program->uniformLocation("lights_u.type");
    m_lights_count_u = m_program->uniformLocation("lights_u.count");
    m_lights_drawShadows_u = m_program->uniformLocation("lights_u.drawShadows");
    m_lights_shadowMatrix_u =
      m_program->uniformLocation("lights_u.shadowMatrix");
    m_texture_shadow_u = m_program->uniformLocation("texture_shadow_u");

    checkError("after set phong uniforms");

    /* set attribute locations */
    m_position_a = m_program->attributeLocation("position_a");
    m_normal_a = m_program->attributeLocation("normal_a");

    release();
    checkError("after initialize phong shader");
    m_initialized = true;
  }
}

void Shift::PhongShaderProgram::setTransform(s2_displacement_t* transform) {

  QMatrix4x4 model = displacementToQMatrix4x4(transform);
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  QMatrix4x4 mv = view * model;
  QMatrix4x4 mvp = projection * mv;
  QMatrix3x3 nm = mv.normalMatrix();

  m_program->setUniformValue(m_matrices_mvp_u, mvp);
  checkError("phong: after set mvp");

  m_program->setUniformValue(m_matrices_mv_u, mv);
  checkError("phong: after set mv");

  m_program->setUniformValue(m_matrices_v_u, view);
  checkError("phong: after set view");

  m_program->setUniformValue(m_matrices_nm_u, nm);
  checkError("phong: after set nm");

  if(m_renderWindow->getLights()->isEnabled()) {
    m_program->setUniformValueArray(
      m_lights_shadowMatrix_u,
      m_renderWindow->getLights()->setShadowMatrix(transform),
      m_renderWindow->getLights()->getMaxCount());
    checkError("phong: after set shadow matrix array");
  }
}

void Shift::PhongShaderProgram::setKa(GLfloat Ka[4]) {
  m_program->setUniformValue(m_material_Ka_u, Ka[0], Ka[1], Ka[2], Ka[3]);
}

void Shift::PhongShaderProgram::setKd(GLfloat Kd[4]) {
  m_program->setUniformValue(m_material_Kd_u, Kd[0], Kd[1], Kd[2], Kd[3]);
}

void Shift::PhongShaderProgram::setKs(GLfloat Ks[4]) {
  m_program->setUniformValue(m_material_Ks_u, Ks[0], Ks[1], Ks[2], Ks[3]);
}

void Shift::PhongShaderProgram::setNs(GLint Ns) {
  m_program->setUniformValue(m_material_Ns_u, Ns);
}

void Shift::PhongShaderProgram::setMaterial(s2_material_t* material) {
  setKa(material->GLuniforms.Ka);
  setKd(material->GLuniforms.Kd);
  setKs(material->GLuniforms.Ks);
  setNs(material->GLuniforms.Ns);
  checkError("after set material");
}

void Shift::PhongShaderProgram::setLights(Lights* lights) {
  m_program->setUniformValueArray(m_lights_position_u, lights->getPosition(),
                                  10, 3);
  m_program->setUniformValueArray(m_lights_Ka_u, lights->getKa(), 10, 3);
  m_program->setUniformValueArray(m_lights_Kd_u, lights->getKd(), 10, 3);
  m_program->setUniformValueArray(m_lights_Ks_u, lights->getKs(), 10, 3);
  m_program->setUniformValueArray(m_lights_type_u, lights->getType(), 10);
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
void Shift::PhongShaderProgram::bindDepthBuffer() {
  m_program->setUniformValue(m_texture_shadow_u, 0);
}
