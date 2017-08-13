#include "terrainshaderprogram.hpp"
#include "renderwindow.hpp"
#include "lights.hpp"


/*!
@brief initialize frame shader and check for errors
*/
void TerrainShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/terrain.vert");
    addTessellationControlShader(":/shaders/terrain.tesc");
    addTessellationEvaluationShader(":/shaders/terrain.tese");
    addFragmentShader(":/shaders/terrain.frag");
    link();
    bind();

    /* texture uniform locations */
    m_texture_color_u = m_program->uniformLocation("texture_color_u");
    m_texture_displacement_u =
      m_program->uniformLocation("texture_displacement_u");
    m_texture_bump_u = m_program->uniformLocation("texture_bump_u");
    m_texture_normal_u = m_program->uniformLocation("texture_normal_u");
    m_texture_specular_u = m_program->uniformLocation("texture_specular_u");;

    qDebug() << "m_texture_color_u:" << m_texture_color_u;
    qDebug() << "m_texture_displacement_u:" << m_texture_displacement_u;
    qDebug() << "m_texture_bump_u:" << m_texture_bump_u;
    qDebug() << "m_texture_normal_u:" << m_texture_normal_u;
    qDebug() << "m_texture_specular_u:" << m_texture_specular_u;

    /* matrix uniform locations */
    m_matrices_u = m_program->uniformLocation("matrices_u");
    m_matrices_mvp_u = m_program->uniformLocation("matrices_u.mvp");
    m_matrices_mv_u = m_program->uniformLocation("matrices_u.mv");
    m_matrices_v_u = m_program->uniformLocation("matrices_u.v");
    m_matrices_p_u = m_program->uniformLocation("matrices_u.p");
    m_matrices_nm_u = m_program->uniformLocation("matrices_u.nm");

    qDebug() << "m_matrices_u:" << m_matrices_u;
    qDebug() << "m_matrices_mvp_u:" << m_matrices_mvp_u;
    qDebug() << "m_matrices_mv_u:" << m_matrices_mv_u;
    qDebug() << "m_matrices_v_u:" << m_matrices_v_u;
    qDebug() << "m_matrices_p_u:" << m_matrices_p_u;
    qDebug() << "m_matrices_nm_u:" << m_matrices_nm_u;

    /* material uniform locations */
    m_material_u = m_program->uniformLocation("material_u");
    m_material_Kd_u = m_program->uniformLocation("material_u.Kd");
    m_material_KaOverKd_u = m_program->uniformLocation("material_u.KaOverKd");
    m_material_KsOverKd_u = m_program->uniformLocation("material_u.KsOverKd");
    m_material_Ns_u = m_program->uniformLocation("material_u.Ns");

    qDebug() << "m_material_u:" << m_material_u;
    qDebug() << "m_material_Kd_u:" << m_material_Kd_u;
    qDebug() << "m_material_KaOverKd_u:" << m_material_KaOverKd_u;
    qDebug() << "m_material_KsOverKd_u:" << m_material_KsOverKd_u;
    qDebug() << "m_material_Ns_u:" << m_material_Ns_u;

    /* light uniform locations */
    m_lights_u = m_program->uniformLocation("lights_u");
    m_lights_position_u = m_program->uniformLocation("lights_u.position");
    m_lights_Ka_u = m_program->uniformLocation("lights_u.Ka");
    m_lights_Kd_u = m_program->uniformLocation("lights_u.Kd");
    m_lights_Ks_u = m_program->uniformLocation("lights_u.Ks");
    m_lights_directional_u = m_program->uniformLocation("lights_u.directional");
    m_lights_count_u = m_program->uniformLocation("lights_u.count");

    qDebug() << "lights_u: " << m_lights_u;
    qDebug() << "lights_u.position: " << m_lights_position_u;
    qDebug() << "lights_u.Ka: " << m_lights_Ka_u;
    qDebug() << "lights_u.Kd: " << m_lights_Kd_u;
    qDebug() << "lights_u.Ks: " << m_lights_Ks_u;
    qDebug() << "lights_u.directional: " << m_lights_directional_u;
    qDebug() << "lights_u.count: " << m_lights_count_u;

    /* terrain uniform locations */
    m_terrain_u = m_program->uniformLocation("terrain_u");
    m_terrain_zMin_u = m_program->uniformLocation("terrain_u.zMin");
    m_terrain_zMax_u = m_program->uniformLocation("terrain_u.zMax");
    m_terrain_xScale_u = m_program->uniformLocation("terrain_u.xScale");
    m_terrain_yScale_u = m_program->uniformLocation("terrain_u.yScale");
    m_terrain_xRange_u = m_program->uniformLocation("terrain_u.xRange");
    m_terrain_yRange_u = m_program->uniformLocation("terrain_u.yRange");
    m_terrain_viewport_u = m_program->uniformLocation("terrain_u.viewport");

    qDebug() << "terrain_u: " << m_terrain_u;
    qDebug() << "m_terrain_zMin_u: " << m_terrain_zMin_u;
    qDebug() << "m_terrain_zMax_u: " << m_terrain_zMax_u;
    qDebug() << "m_terrain_xScale_u: " << m_terrain_xScale_u;
    qDebug() << "m_terrain_yScale_u: " << m_terrain_yScale_u;
    qDebug() << "m_terrain_xRange_u: " << m_terrain_xRange_u;
    qDebug() << "m_terrain_yRange_u: " << m_terrain_yRange_u;
    qDebug() << "m_terrain_viewport_u: " << m_terrain_viewport_u;

    m_program->release();

    checkError("after initialize terrain");
    m_initialized = true;
  }
}


/*!
@brief set mvp using only a transform data structure. The model and view matrices are obtained from the current render window.
*/
void TerrainShaderProgram::setTransform(s2_displacement_t* transform) {
  QMatrix4x4 model = s2_displacementToQMatrix4x4(transform);
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  QMatrix4x4 mv = view * model;
  QMatrix4x4 mvp = projection * view * model;
  m_program->setUniformValue(m_matrices_mvp_u, mvp);
  m_program->setUniformValue(m_matrices_mv_u, mv);
  m_program->setUniformValue(m_matrices_v_u, view);
  m_program->setUniformValue(m_matrices_p_u, projection);
  m_program->setUniformValue(m_matrices_nm_u, mv.normalMatrix());
}


/*!
@brief set mvp using only a transform data structure. The model and view matrices are obtained from the current render window.
*/
void TerrainShaderProgram::setTransform(QMatrix4x4& model) {
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  QMatrix4x4 mv = view * model;
  QMatrix4x4 mvp = projection * view * model;
  m_program->setUniformValue(m_matrices_mvp_u, mvp);
  m_program->setUniformValue(m_matrices_mv_u, mv);
  m_program->setUniformValue(m_matrices_v_u, view);
  m_program->setUniformValue(m_matrices_p_u, projection);
  m_program->setUniformValue(m_matrices_nm_u, mv.normalMatrix());
}


/*!
@brief set scale factor
*/
void TerrainShaderProgram::setScale(GLfloat xScale, GLfloat yScale) {
  m_program->setUniformValue(m_terrain_xScale_u, xScale);
  m_program->setUniformValue(m_terrain_yScale_u, yScale);
}


/*!
@brief set view port size
*/
void TerrainShaderProgram::setViewPort(GLfloat vp1, GLfloat vp2) {
  m_program->setUniformValue(m_terrain_viewport_u, vp1, vp2);
}


/*!
@brief bind displacement texture
*/
void TerrainShaderProgram::setTextureDisplacement(QOpenGLTexture* tex) {
  tex->bind(0);
  m_program->setUniformValue(m_texture_displacement_u, 0);
}


/*!
@brief bind color texture
*/
void TerrainShaderProgram::setTextureColor(QOpenGLTexture* tex) {
  if(tex) {
    tex->bind(1);
    m_program->setUniformValue(m_texture_color_u, 1);
  }
}


/*!
@brief bind bump map texture
*/
void TerrainShaderProgram::setTextureBump(QOpenGLTexture* tex) {
  if(tex) {
    tex->bind(2);
    m_program->setUniformValue(m_texture_bump_u, 2);
  }
}


/*!
@brief bind normal map texture
*/
void TerrainShaderProgram::setTextureNormal(QOpenGLTexture* tex) {
  if(tex) {
    tex->bind(3);
    m_program->setUniformValue(m_texture_normal_u, 3);
  }
}


/*!
@brief bind specular map texture
*/
void TerrainShaderProgram::setTextureSpecular(QOpenGLTexture* tex) {
  if(tex) {
    tex->bind(4);
    m_program->setUniformValue(m_texture_specular_u, 4);
  }
}


/*!
@brief set light uniforms
*/
void TerrainShaderProgram::setLights(s2_Lights* lights) {
  m_program->setUniformValueArray(m_lights_position_u, lights->getPosition(), 10, 3);
  m_program->setUniformValueArray(m_lights_Ka_u, lights->getKa(), 10, 3);
  m_program->setUniformValueArray(m_lights_Kd_u, lights->getKd(), 10, 3);
  m_program->setUniformValueArray(m_lights_Ks_u, lights->getKs(), 10, 3);
  m_program->setUniformValueArray(
    m_lights_directional_u, lights->getDirectional(), 10);
  m_program->setUniformValue(m_lights_count_u, lights->getCount());
}


/*!
@brief set diffuse color, only used if no texture bound
*/
void TerrainShaderProgram::setKd(GLfloat Kd[4]) {
  m_program->setUniformValue(m_material_Kd_u, Kd[0], Kd[1], Kd[2], Kd[3]);
}


/*!
@brief sets ratio of Ka over Kd
*/
void TerrainShaderProgram::setKaOverKd(GLfloat KaOverKd) {
  m_program->setUniformValue(m_material_KaOverKd_u, KaOverKd);
}


/*!
@brief sets ratio of Ks over Kd
*/
void TerrainShaderProgram::setKsOverKd(GLfloat KsOverKd) {
  m_program->setUniformValue(m_material_KsOverKd_u, KsOverKd);
}


/*!
@brief sets Ns, specular exponent term
*/
void TerrainShaderProgram::setNs(GLint Ns) {
  m_program->setUniformValue(m_material_Ns_u, Ns);
}


/*!
@brief set max and min z value
*/
void TerrainShaderProgram::setZExtents(GLfloat zMin, GLfloat zMax) {
  m_program->setUniformValue(m_terrain_zMin_u, zMin);
  m_program->setUniformValue(m_terrain_zMax_u, zMax);
}


/*!
@brief set max x and y values
*/
void TerrainShaderProgram::setRange(GLfloat xRange, GLfloat yRange) {
  m_program->setUniformValue(m_terrain_xRange_u, xRange);
  m_program->setUniformValue(m_terrain_yRange_u, yRange);
}
