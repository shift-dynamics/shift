#include "terrainshadowshaderprogram.hpp"
#include "renderwindow.hpp"
#include "lights.hpp"


/*!
@brief initialize frame shader and check for errors
*/
void TerrainShadowShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/terrain_shadow.vert");
    addTessellationControlShader(":/shaders/terrain_shadow.tesc");
    addTessellationEvaluationShader(":/shaders/terrain_shadow.tese");
    addFragmentShader(":/shaders/terrain_shadow.frag");
    link();
    bind();

    /* set uniform locations */
    m_matrices_mvp_u = m_program->uniformLocation("matrices_u.mvp");
    m_matrices_mv_u = m_program->uniformLocation("matrices_u.mv");
    // m_matrices_v_u = m_program->uniformLocation("matrices_u.v");
    m_matrices_p_u = m_program->uniformLocation("matrices_u.p");

    // qDebug() << "m_matrices_u:" << m_matrices_u;
    qDebug() << "m_matrices_mvp_u:" << m_matrices_mvp_u;
    qDebug() << "m_matrices_mv_u:" << m_matrices_mv_u;
    // qDebug() << "m_matrices_v_u:" << m_matrices_v_u;
    qDebug() << "m_matrices_p_u:" << m_matrices_p_u;

    /* terrain uniform locations */
    // m_terrain_u = m_program->uniformLocation("terrain_u");
    m_terrain_zMin_u = m_program->uniformLocation("terrain_u.zMin");
    m_terrain_zMax_u = m_program->uniformLocation("terrain_u.zMax");
    m_terrain_xScale_u = m_program->uniformLocation("terrain_u.xScale");
    m_terrain_yScale_u = m_program->uniformLocation("terrain_u.yScale");
    m_terrain_xRange_u = m_program->uniformLocation("terrain_u.xRange");
    m_terrain_yRange_u = m_program->uniformLocation("terrain_u.yRange");
    m_terrain_viewport_u = m_program->uniformLocation("terrain_u.viewport");
    m_texture_displacement_u =
      m_program->uniformLocation("texture_displacement_u");

    // qDebug() << "terrain_u: " << m_terrain_u;
    qDebug() << "m_terrain_zMin_u: " << m_terrain_zMin_u;
    qDebug() << "m_terrain_zMax_u: " << m_terrain_zMax_u;
    qDebug() << "m_terrain_xScale_u: " << m_terrain_xScale_u;
    qDebug() << "m_terrain_yScale_u: " << m_terrain_yScale_u;
    qDebug() << "m_terrain_xRange_u: " << m_terrain_xRange_u;
    qDebug() << "m_terrain_yRange_u: " << m_terrain_yRange_u;
    qDebug() << "m_terrain_viewport_u: " << m_terrain_viewport_u;
    qDebug() << "m_texture_displacement_u:" << m_texture_displacement_u;

    m_vao = new QOpenGLVertexArrayObject(m_renderWindow);
    m_vao->create();

    release();
    checkError("terrain-shadow: after initialize shadow shader");
    m_initialized = true;
  }
}


/*!
@brief set mvp matrix
*/
void TerrainShadowShaderProgram::setMVP(
  QMatrix4x4& m,
  QMatrix4x4& v,
  QMatrix4x4& p) {
  QMatrix4x4 mv = v * m;
  QMatrix4x4 mvp = p * mv;
  m_program->setUniformValue(m_matrices_mvp_u, mvp);
  m_program->setUniformValue(m_matrices_mv_u, mv);
  m_program->setUniformValue(m_matrices_mv_u, v);
  m_program->setUniformValue(m_matrices_mv_u, p);

  checkError("terrain-shadow: after set mvp");
}


/*!
@brief set view port size
*/
void TerrainShadowShaderProgram::setViewPort(GLfloat vp1, GLfloat vp2) {
  m_program->setUniformValue(m_terrain_viewport_u, vp1, vp2);
}


/*!
@brief bind displacement texture
*/
void TerrainShadowShaderProgram::setTextureDisplacement(QOpenGLTexture* tex) {
  if(tex) {
    tex->bind(1);
    m_program->setUniformValue(m_texture_displacement_u, 1);
    checkError("terrain-shadow: after set uniform value displacement");
  } else {
    qDebug() << "no displacement texture specified";
  }
}


/*!
@brief set max and min z value
*/
void TerrainShadowShaderProgram::setZExtents(GLfloat zMin, GLfloat zMax) {
  m_program->setUniformValue(m_terrain_zMin_u, zMin);
  m_program->setUniformValue(m_terrain_zMax_u, zMax);
}


/*!
@brief convenience function that sets all uniforms located in terrain data structure
*/
void TerrainShadowShaderProgram::setUniforms(s2_terrain_t* terrain) {
  setZExtents(terrain->zMin, terrain->zMax);
}
