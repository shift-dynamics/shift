#include "terrainshaderprogram.hpp"
#include "renderwindow.hpp"
#include "lights.hpp"


/*!
@brief initialize frame shader and check for errors
*/
void TerrainShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/frame.vert");
    addFragmentShader(":/shaders/frame.frag");
    link();
    bind();

    m_projectionMatrix_u =
      m_program->uniformLocation("proj_matrix");
    m_textureDisplacementMap_u =
      m_program->uniformLocation("tex_displacement");
    m_textureColor_u = m_program->uniformLocation("tex_color");

    /* @TODO these values currently not used */
    m_textureBumpMap_u = -1;
    m_textureNormalMap_u = -1;
    m_textureSpecularMap_u = -1;

    m_zMax_u = m_program->uniformLocation("zMax");
    m_zMin_u = m_program->uniformLocation("zMin");
    m_viewport_u = m_program->uniformLocation("viewport");
    m_KaOverKd_u = m_program->uniformLocation("KaOverKd");
    m_KsOverKd_u = m_program->uniformLocation("KsOverKd");

    m_program->release();

    checkError("after initialize terrain uniforms");
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
  QMatrix4x4 mvp = projection * view * model;
  m_program->setUniformValue(m_mvp_u, mvp);
  checkError("after set transform");
}


/*!
@brief set mvp matrix
*/
void TerrainShaderProgram::setMVP(QMatrix4x4& mvp) {
  m_program->setUniformValue(m_mvp_u, mvp);
  checkError("after set transform");
}


/*!
@brief set scale factor
*/
void TerrainShaderProgram::setScale(GLfloat scale) {
  m_program->setUniformValue(m_scale_u, scale);
}
