#include "floorshaderprogram.h"
#include "lights.h"
#include "renderwindow.h"

/*!
@brief initialize frame shader and check for errors
*/
void Shift::FloorShaderProgram::initialize() {
  if(!m_initialized) {
    checkError("FloorShaderProgram::initialize(): before add vertex shader");
    addProgram();
    addVertexShader(":/shaders/floor.glsl");
    checkError("FloorShaderProgram::initialize(): after add vertex shader");
    addFragmentShader(":/shaders/floor.glsl");
    checkError("FloorShaderProgram::initialize(): after add fragment shader");
    link();
    bind();
    checkError("FloorShaderProgram::initialize(): after link program");

    m_vao = new QOpenGLVertexArrayObject(m_renderWindow);
    m_vao->create();
    m_vao->bind();

    /* texture uniform locations */
    m_texture_shadow_u = m_program->uniformLocation("texture_shadow_u");
    m_texture_shadowCube_u = m_program->uniformLocation("texture_shadowCube_u");
    m_texture_diffuse_u = m_program->uniformLocation("texture_diffuse_u");
    m_texture_normal_u = m_program->uniformLocation("texture_normal_u");
    m_texture_specular_u = m_program->uniformLocation("texture_specular_u");
    m_texture_bump_u = m_program->uniformLocation("texture_bump_u");

    qDebug() << "m_texture_shadow_u:" << m_texture_shadow_u;
    qDebug() << "m_texture_diffuse_u:" << m_texture_diffuse_u;
    qDebug() << "m_texture_normal_u:" << m_texture_normal_u;
    qDebug() << "m_texture_specular_u:" << m_texture_specular_u;
    qDebug() << "m_texture_bump_u:" << m_texture_bump_u;

    /* matrix uniform locations */
    m_matrices_mvp_u = m_program->uniformLocation("matrices_u.mvp");
    m_matrices_mv_u = m_program->uniformLocation("matrices_u.mv");
    m_matrices_v_u = m_program->uniformLocation("matrices_u.v");
    m_matrices_p_u = m_program->uniformLocation("matrices_u.p");
    m_matrices_nm_u = m_program->uniformLocation("matrices_u.nm");

    qDebug() << "matrices_u.mvp:" << m_matrices_mvp_u;
    qDebug() << "matrices_u.mv:" << m_matrices_mv_u;
    qDebug() << "matrices_u.v:" << m_matrices_v_u;
    qDebug() << "matrices_u.p:" << m_matrices_p_u;
    qDebug() << "matrices_u.nm:" << m_matrices_nm_u;

    /* material uniform locations */
    m_material_Kd_u = m_program->uniformLocation("material_u.Kd");
    m_material_KaOverKd_u = m_program->uniformLocation("material_u.KaOverKd");
    m_material_KsOverKd_u = m_program->uniformLocation("material_u.KsOverKd");
    m_material_Ns_u = m_program->uniformLocation("material_u.Ns");

    qDebug() << "m_material_Kd_u:" << m_material_Kd_u;
    qDebug() << "m_material_KaOverKd_u:" << m_material_KaOverKd_u;
    qDebug() << "m_material_KsOverKd_u:" << m_material_KsOverKd_u;
    qDebug() << "m_material_Ns_u:" << m_material_Ns_u;

    /* light uniform locations */
    m_lights_position_u = m_program->uniformLocation("lights_u.position");
    m_lights_direction_u = m_program->uniformLocation("lights_u.direction");
    m_lights_type_u = m_program->uniformLocation("lights_u.type");
    m_lights_Ka_u = m_program->uniformLocation("lights_u.Ka");
    m_lights_Kd_u = m_program->uniformLocation("lights_u.Kd");
    m_lights_Ks_u = m_program->uniformLocation("lights_u.Ks");
    m_lights_count_u = m_program->uniformLocation("lights_u.count");
    m_lights_drawShadows_u = m_program->uniformLocation("lights_u.drawShadows");
    m_lights_shadowMatrix_u =
      m_program->uniformLocation("lights_u.shadowMatrix");

    qDebug() << "lights_u.position: " << m_lights_position_u;
    qDebug() << "lights_u.direction: " << m_lights_direction_u;
    qDebug() << "lights_u.Ka: " << m_lights_Ka_u;
    qDebug() << "lights_u.Kd: " << m_lights_Kd_u;
    qDebug() << "lights_u.Ks: " << m_lights_Ks_u;
    qDebug() << "lights_u.count: " << m_lights_count_u;

    /* floor specific uniform locations */
    m_floor_tcScale_u = m_program->uniformLocation("floor_u.tcScale");
    m_floor_enableNormalMap_u =
      m_program->uniformLocation("floor_u.enableNormalMap");
    m_floor_enableSpecularMap_u =
      m_program->uniformLocation("floor_u.enableSpecularMap");
    m_floor_enableBumpMap_u =
      m_program->uniformLocation("floor_u.enableBumpMap");

    qDebug() << "floor_u.tcScale: " << m_floor_tcScale_u;
    qDebug() << "floor_u.enableNormalMap: " << m_floor_enableNormalMap_u;
    qDebug() << "floor_u.enableSpecularMap: " << m_floor_enableSpecularMap_u;
    qDebug() << "floor_u.enableBumpMap: " << m_floor_enableBumpMap_u;

    m_vao->release();
    release();
    checkError("FloorShaderProgram::initialize(): after initialize uniforms");
    m_initialized = true;
  }
}

/*!
@brief set mvp matrix by specifying a model matrix. The model and view
matrices are obtained from the current render window.
*/
void Shift::FloorShaderProgram::setTransform(s2_displacement_t* transform) {
  QMatrix4x4 model = displacementToQMatrix4x4(transform);
  setTransform(model);
}

/*!
@brief set mvp matrix by specifying a model matrix. The model and view
matrices are obtained from the current render window.
*/
void Shift::FloorShaderProgram::setTransform(QMatrix4x4& model) {
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  QMatrix4x4 mv = view * model;
  QMatrix4x4 mvp = projection * mv;
  m_program->setUniformValue(m_matrices_mvp_u, mvp);
  m_program->setUniformValue(m_matrices_mv_u, mv);
  m_program->setUniformValue(m_matrices_v_u, view);
  m_program->setUniformValue(m_matrices_p_u, projection);
  m_program->setUniformValue(m_matrices_nm_u, view.normalMatrix());
  if(m_renderWindow->getLights()->isEnabled()) {
    m_program->setUniformValueArray(
      m_lights_shadowMatrix_u,
      m_renderWindow->getLights()->setShadowMatrix(model),
      m_renderWindow->getLights()->getMaxCount());

    checkError(
      "FloorShaderProgram::setTransform(): after set shadow matrix array");
  }
}

/*!
@brief set scale factor
*/
void Shift::FloorShaderProgram::setScale(GLfloat scale) {
  m_program->setUniformValue(m_floor_tcScale_u, scale);
}

/*!
@brief bind dummy vertex array object
*/
void Shift::FloorShaderProgram::bindVao() {
  m_vao->bind();
}

/*!
@brief release vao object
*/
void Shift::FloorShaderProgram::releaseVao() {
  m_vao->release();
}

/*!
@brief bind diffuse texture
*/
void Shift::FloorShaderProgram::bindDiffuseTexture(QOpenGLTexture* tex) {
  tex->bind(2);
  m_program->setUniformValue("texture_diffuse_u", 2);
}

/*!
@brief bind normal texture
*/
void Shift::FloorShaderProgram::bindNormalTexture(QOpenGLTexture* tex) {
  if(tex) {
    tex->bind(3);
    m_program->setUniformValue("texture_normal_u", 3);
    m_program->setUniformValue(m_floor_enableNormalMap_u, 1);
  } else {
    m_program->setUniformValue(m_floor_enableNormalMap_u, 0);
  }
}

/*!
@brief bind specular texture
*/
void Shift::FloorShaderProgram::bindSpecularTexture(QOpenGLTexture* tex) {
  if(tex) {
    tex->bind(4);
    m_program->setUniformValue("texture_specular_u", 4);
    m_program->setUniformValue(m_floor_enableSpecularMap_u, 1);
  } else {
    m_program->setUniformValue(m_floor_enableSpecularMap_u, 0);
  }
}

/*!
@brief bind texture for bump mapping
*/
void Shift::FloorShaderProgram::bindBumpTexture(QOpenGLTexture* tex) {
  if(tex) {
    tex->bind(5);
    m_program->setUniformValue("texture_bump_u", 5);
    m_program->setUniformValue(m_floor_enableSpecularMap_u, 1);
  } else {
    m_program->setUniformValue(m_floor_enableSpecularMap_u, 0);
  }
}

/*!
@brief set light uniforms
*/
void Shift::FloorShaderProgram::setLights(Lights* lights) {
  m_program->setUniformValueArray(m_lights_position_u, lights->getPosition(),
                                  10, 3);
  checkError("after FloorShaderProgram::setLights - m_lights_position_u");
  m_program->setUniformValueArray(m_lights_Ka_u, lights->getKa(), 10, 3);
  checkError("after FloorShaderProgram::setLights - m_lights_Ka_u");
  m_program->setUniformValueArray(m_lights_Kd_u, lights->getKd(), 10, 3);
  checkError("after FloorShaderProgram::setLights - m_lights_Kd_u");
  m_program->setUniformValueArray(m_lights_Ks_u, lights->getKs(), 10, 3);
  checkError("after FloorShaderProgram::setLights - m_lights_Ks_u");
  m_program->setUniformValueArray(m_lights_type_u, lights->getType(), 10);
  checkError("after FloorShaderProgram::setLights - m_lights_direction_u");
  m_program->setUniformValue(m_lights_count_u, lights->getCount());
  checkError("after FloorShaderProgram::setLights - m_lights_count_u");

  if(m_renderWindow->getLights()->isEnabled()) {
    m_program->setUniformValue(m_lights_drawShadows_u, 1);
  } else {
    m_program->setUniformValue(m_lights_drawShadows_u, 0);
  }
}
