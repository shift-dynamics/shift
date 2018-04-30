#include "terrainshaderprogram.h"
#include "lights.h"
#include "renderwindow.h"

/*!
@brief initialize frame shader and check for errors
*/
void Shift::TerrainShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/terrain.glsl");
    addTessellationControlShader(":/shaders/terrain.glsl");
    addTessellationEvaluationShader(":/shaders/terrain.glsl");
    addFragmentShader(":/shaders/terrain.glsl");
    link();
    bind();

    /* texture uniform locations */
    m_texture_shadow_u = m_program->uniformLocation("texture_shadow_u");
    m_texture_shadowCube_u = m_program->uniformLocation("texture_shadowCube_u");
    m_texture_elevation_u = m_program->uniformLocation("texture_elevation_u");
    m_texture_diffuse_u = m_program->uniformLocation("texture_diffuse_u");
    m_texture_normal_u = m_program->uniformLocation("texture_normal_u");
    m_texture_specular_u = m_program->uniformLocation("texture_specular_u");
    m_texture_bump_u = m_program->uniformLocation("texture_bump_u");

    qDebug() << "m_texture_shadow_u:" << m_texture_shadow_u;
    qDebug() << "m_texture_elevation_u:" << m_texture_elevation_u;
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
    m_lights_Ka_u = m_program->uniformLocation("lights_u.Ka");
    m_lights_Kd_u = m_program->uniformLocation("lights_u.Kd");
    m_lights_Ks_u = m_program->uniformLocation("lights_u.Ks");
    m_lights_type_u = m_program->uniformLocation("lights_u.type");
    m_lights_count_u = m_program->uniformLocation("lights_u.count");
    m_lights_drawShadows_u = m_program->uniformLocation("lights_u.drawShadows");
    m_lights_shadowMatrix_u =
      m_program->uniformLocation("lights_u.shadowMatrix");

    qDebug() << "lights_u.position: " << m_lights_position_u;
    qDebug() << "lights_u.Ka: " << m_lights_Ka_u;
    qDebug() << "lights_u.Kd: " << m_lights_Kd_u;
    qDebug() << "lights_u.Ks: " << m_lights_Ks_u;
    qDebug() << "lights_u.directional: " << m_lights_type_u;
    qDebug() << "lights_u.count: " << m_lights_count_u;

    /* terrain uniform locations */
    m_terrain_zMin_u = m_program->uniformLocation("terrain_u.zMin");
    m_terrain_zMax_u = m_program->uniformLocation("terrain_u.zMax");
    m_terrain_scale_u = m_program->uniformLocation("terrain_u.scale");
    m_terrain_tcScale_u = m_program->uniformLocation("terrain_u.tcScale");
    m_terrain_viewport_u = m_program->uniformLocation("terrain_u.viewport");
    m_terrain_enableDiffuseMap_u =
      m_program->uniformLocation("terrain_u.enableDiffuseMap");
    m_terrain_enableNormalMap_u =
      m_program->uniformLocation("terrain_u.enableNormalMap");
    m_terrain_enableSpecularMap_u =
      m_program->uniformLocation("terrain_u.enableSpecularMap");
    m_terrain_enableBumpMap_u =
      m_program->uniformLocation("terrain_u.enableBumpMap");

    qDebug() << "terrain_u.zMin: " << m_terrain_zMin_u;
    qDebug() << "terrain_u.zMax: " << m_terrain_zMax_u;
    qDebug() << "terrain_u.scale: " << m_terrain_scale_u;
    qDebug() << "terrain_u.tcScale: " << m_terrain_tcScale_u;
    qDebug() << "terrain_u.viewport: " << m_terrain_viewport_u;
    qDebug() << "terrain_u.enableDiffuseMap: " << m_terrain_enableDiffuseMap_u;
    qDebug() << "terrain_u.enableNormalMap: " << m_terrain_enableNormalMap_u;
    qDebug() << "terrain_u.enableSpecularMap: "
             << m_terrain_enableSpecularMap_u;
    qDebug() << "terrain_u.enableBumpMap: " << m_terrain_enableBumpMap_u;

    /* create emtpy vertex array object */
    m_vao = new QOpenGLVertexArrayObject(m_renderWindow);
    m_vao->create();

    m_program->release();

    checkError("after initialize terrain");
    m_initialized = true;
  }
}

/*!
@brief set mvp using only a transform data structure. The model and view
matrices are obtained from the current render window.
*/
void Shift::TerrainShaderProgram::setTransform(s2_displacement_t* transform) {
  QMatrix4x4 model = displacementToQMatrix4x4(transform);
  setTransform(model);
}

/*!
@brief set mvp using only a transform data structure. The model and view
matrices are obtained from the current render window.
*/
void Shift::TerrainShaderProgram::setTransform(QMatrix4x4& model) {
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  QMatrix4x4 mv = view * model;
  QMatrix4x4 mvp = projection * mv;
  m_program->setUniformValue(m_matrices_mvp_u, mvp);
  m_program->setUniformValue(m_matrices_mv_u, mv);
  m_program->setUniformValue(m_matrices_v_u, view);
  m_program->setUniformValue(m_matrices_p_u, projection);
  m_program->setUniformValue(m_matrices_nm_u, mv.normalMatrix());
  if(m_renderWindow->getLights()->isEnabled()) {
    m_program->setUniformValueArray(
      m_lights_shadowMatrix_u,
      m_renderWindow->getLights()->setShadowMatrix(model),
      m_renderWindow->getLights()->getMaxCount());
    checkError(
      "TerrainShaderProgram::setTransform(): after set shadow matrix array");
  }
}

/*!
@brief set scale factor
*/
void Shift::TerrainShaderProgram::setScale(GLfloat xScale, GLfloat yScale) {
  m_program->setUniformValue(m_terrain_scale_u, xScale, yScale);
}

/*!
@brief set view port size
*/
void Shift::TerrainShaderProgram::setViewPort(GLfloat vp1, GLfloat vp2) {
  m_program->setUniformValue(m_terrain_viewport_u, vp1, vp2);
}

/*!
@brief bind displacement texture
*/
void Shift::TerrainShaderProgram::setTextureElevation(QOpenGLTexture* tex) {
  if(tex) {
    tex->bind(2);
    m_program->setUniformValue(m_texture_elevation_u, 2);
    checkError("terrain: after set uniform value elevation");
  } else {
    qDebug() << "no elevation texture specified";
  }
}

/*!
@brief bind color texture
*/
void Shift::TerrainShaderProgram::setTextureDiffuse(QOpenGLTexture* tex) {
  if(tex) {
    m_program->setUniformValue(m_terrain_enableDiffuseMap_u, 1);
    tex->bind(3);
    m_program->setUniformValue(m_texture_diffuse_u, 3);
    checkError("terrain: after set uniform value color");
  } else {
    m_program->setUniformValue(m_terrain_enableDiffuseMap_u, 0);
  }
}

/*!
@brief bind bump map / displacement texture
*/
void Shift::TerrainShaderProgram::setTextureNormal(QOpenGLTexture* tex) {
  if(tex) {
    m_program->setUniformValue(m_terrain_enableNormalMap_u, 1);
    tex->bind(4);
    m_program->setUniformValue(m_texture_normal_u, 4);
  } else {
    m_program->setUniformValue(m_terrain_enableNormalMap_u, 0);
  }
}

/*!
@brief bind specular map texture
*/
void Shift::TerrainShaderProgram::setTextureSpecular(QOpenGLTexture* tex) {
  if(tex) {
    m_program->setUniformValue(m_terrain_enableSpecularMap_u, 1);
    tex->bind(5);
    m_program->setUniformValue(m_texture_specular_u, 5);
  } else {
    m_program->setUniformValue(m_terrain_enableSpecularMap_u, 0);
  }
}

/*!
@brief bind bump map texture
*/
void Shift::TerrainShaderProgram::setTextureBump(QOpenGLTexture* tex) {
  if(tex) {
    m_program->setUniformValue(m_terrain_enableBumpMap_u, 1);
    tex->bind(6);
    m_program->setUniformValue(m_texture_bump_u, 6);
  } else {
    m_program->setUniformValue(m_terrain_enableBumpMap_u, 0);
  }
}

/*!
@brief set light uniforms
*/
void Shift::TerrainShaderProgram::setLights(Lights* lights) {
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
}

/*!
@brief set diffuse color, only used if no texture bound
*/
void Shift::TerrainShaderProgram::setKd(GLfloat Kd[4]) {
  m_program->setUniformValue(m_material_Kd_u, Kd[0], Kd[1], Kd[2], Kd[3]);
}

/*!
@brief sets ratio of Ka over Kd
*/
void Shift::TerrainShaderProgram::setKaOverKd(GLfloat KaOverKd) {
  m_program->setUniformValue(m_material_KaOverKd_u, KaOverKd);
}

/*!
@brief sets ratio of Ks over Kd
*/
void Shift::TerrainShaderProgram::setKsOverKd(GLfloat KsOverKd) {
  m_program->setUniformValue(m_material_KsOverKd_u, KsOverKd);
}

/*!
@brief sets Ns, specular exponent term
*/
void Shift::TerrainShaderProgram::setNs(GLint Ns) {
  m_program->setUniformValue(m_material_Ns_u, Ns);
}

/*!
@brief set max and min z value
*/
void Shift::TerrainShaderProgram::setZExtents(GLfloat zMin, GLfloat zMax) {
  m_program->setUniformValue(m_terrain_zMin_u, zMin);
  m_program->setUniformValue(m_terrain_zMax_u, zMax);
}

/*!
@brief set max x and y values
*/
void Shift::TerrainShaderProgram::setTcScale(GLfloat tcXScale,
                                             GLfloat tcYScale) {
  m_program->setUniformValue(m_terrain_tcScale_u, tcXScale, tcYScale);
}

/*!
@brief convenience function that sets all uniforms located in elevationmap data
structure
*/
void Shift::TerrainShaderProgram::setUniforms(s2_terrain_t* terrain) {
  setKd(terrain->Kd);
  setKaOverKd(terrain->KaOverKd);
  setKsOverKd(terrain->KsOverKd);
  setNs(terrain->Ns);
  setZExtents(terrain->zMin, terrain->zMax);
  setScale(terrain->scale[0], terrain->scale[1]);
  setTcScale(terrain->textureScale[0], terrain->textureScale[1]);
}

/*!
@brief bind depth buffers to index 0 and 1
*/
void Shift::TerrainShaderProgram::bindDepthBuffer() {
  m_program->setUniformValue(m_texture_shadow_u, 0);
  m_program->setUniformValue(m_texture_shadowCube_u, 1);
}
